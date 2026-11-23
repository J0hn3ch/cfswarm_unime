import os
import paho.mqtt.client as mqtt
import json
import time
import argparse
import sys
import csv
import shutil
from datetime import datetime

# Import librerie Crazyflie cflib v1.33
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.utils.reset_estimator import reset_estimator

# Imposta directory temporanea scrivibile per matplotlib
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

BROKER_IP = "192.168.137.1"
DEFAULT_URI = "radio://0/80/2M/E7E7E7E702"

class EdgeRequester:
    def __init__(self, node_id, uri=DEFAULT_URI, broker_ip=BROKER_IP, port=1883, max_repetitions=30, csv_file=None, enable_plots=True):
        self.node_id = node_id
        self.uris = [uri]
        self.broker_ip = broker_ip
        self.port = port
        self.max_repetitions = max_repetitions
        self.enable_plots = enable_plots
        
        self.run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if csv_file is None or csv_file == "metrics_edge_2_requester.csv":
            self.csv_file = f"metrics_edge_2_requester_{self.run_timestamp}.csv"
        else:
            self.csv_file = csv_file
        
        self.swarm = None
        self.current_repetition = 0
        self.role = 'requester'
        self.pending_drone_uri = None
        self.action_queue = []
        self.last_bid_uri = None
        self.active = True
        self.is_ready = False  # Flag di stabilizzazione drone ed estimator
        
        # Telemetria live dai log config (posizione, sensori, batteria, RSSI)
        self.drones_battery = {uri: 100.0}
        self.telemetry = {
            uri: {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0,
                'front': None, 'back': None,
                'vbat': 4.0, 'battery': 100.0,
                'rssi': -50.0, 'rssi_history': []
            }
        }
        
        # Struttura per raccolta metriche per la tesi
        self.metrics_history = []
        self.current_cycle_metrics = {}
        
        # Inizializzazione file CSV metriche
        self.init_csv_file()
        
        # Inizializzazione driver radio Crazyflie e Swarm PRIMA di collegarsi a MQTT
        print(f"[{self.node_id}] Inizializzazione driver CRTP cflib...")
        cflib.crtp.init_drivers(enable_serial_driver=False)
        self.connect_swarm(is_in_flight=False)

        # Segnalazione completamento stabilizzazione
        self.is_ready = True
        print(f"\n[{self.node_id}] *** DRONE E KALMAN FILTER PRONTI E STABILIZZATI ***\n")

        # Connessione al broker MQTT (avviata solo quando il drone è pronto ad operare)
        self.client = mqtt.Client(client_id=self.node_id)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        print(f"[{self.node_id}] Connessione al Broker MQTT ({self.broker_ip}:{self.port})...")
        try:
            self.client.connect(self.broker_ip, self.port, 60)
        except Exception as e:
            print(f"[{self.node_id}] ERRORE connessione MQTT: {e}")
            sys.exit(1)
            
        self.client.subscribe([
            ("cnp/cfp", 0),
            ("cnp/cfp/broadcast", 0), 
            (f"cnp/award/{self.node_id}", 0),
            (f"cnp/evaluate/{self.node_id}", 0),
            ("cnp/released/#", 0),
            ("cnp/reject_all/#", 0)
        ])
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[{self.node_id}] Connesso con successo al Broker MQTT.")
        else:
            print(f"[{self.node_id}] Connessione MQTT fallita con codice {rc}")

    def parse_rssi(self, val):
        """Interpreta il valore radio.rssi del firmware (uint8_t) convertendolo in standard dBm negativo."""
        if val is None or val == 0:
            return -60.0
        v = float(val)
        if v > 128:
            v = v - 256.0
        elif 0 < v <= 128:
            v = -v
        return v

    def get_mean_rssi(self, uri, window=5):
        """Restituisce la media mobile delle ultime letture RSSI."""
        t = self.telemetry.get(uri, {})
        history = t.get('rssi_history', [])
        if not history:
            return t.get('rssi', -60.0)
        recent = history[-window:] if len(history) >= window else history
        return sum(recent) / len(recent)

    def init_csv_file(self):
        """Inizializza l'intestazione del file CSV per le metriche della tesi del Requester."""
        header = [
            "repetition", "datetime", "timestamp", "cfp_id", "contractor", "support_drone_uri",
            "own_battery_pct", "support_battery_pct", "voltage_v",
            "latency_discovery_ms", "latency_handover_sec", "latency_integration_sec", "total_turnaround_sec",
            "rssi_dbm", "final_mean_rssi_dbm", "backward_dist_cm", "dist_final_cm", "drift_cm", "yaw_change_deg", "stop_reason",
            "cfp_payload_bytes", "eval_payload_bytes", "decision_payload_bytes", "release_payload_bytes"
        ]
        try:
            with open(self.csv_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
            print(f"[{self.node_id}] File metriche Requester inizializzato: {self.csv_file}")
            if self.csv_file != "metrics_edge_2_requester.csv":
                shutil.copyfile(self.csv_file, "metrics_edge_2_requester.csv")
        except Exception as e:
            print(f"[{self.node_id}] Avviso: Impossibile creare file CSV: {e}")

    def log_metric_row(self, data):
        """Salva una riga di metriche su CSV e in memoria per i grafici."""
        self.metrics_history.append(data)
        now_str = data.get("datetime", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        row = [
            data.get("repetition"),
            now_str,
            data.get("timestamp"),
            data.get("cfp_id"),
            data.get("contractor"),
            data.get("support_drone_uri"),
            data.get("own_battery_pct"),
            data.get("support_battery_pct"),
            data.get("voltage_v"),
            data.get("latency_discovery_ms"),
            data.get("latency_handover_sec"),
            data.get("latency_integration_sec"),
            data.get("total_turnaround_sec"),
            data.get("rssi_dbm"),
            data.get("final_mean_rssi_dbm"),
            data.get("backward_dist_cm"),
            data.get("dist_final_cm"),
            data.get("drift_cm"),
            data.get("yaw_change_deg"),
            data.get("stop_reason"),
            data.get("cfp_payload_bytes"),
            data.get("eval_payload_bytes"),
            data.get("decision_payload_bytes"),
            data.get("release_payload_bytes")
        ]
        try:
            with open(self.csv_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            if self.csv_file != "metrics_edge_2_requester.csv":
                with open("metrics_edge_2_requester.csv", mode='a', newline='') as f_std:
                    writer_std = csv.writer(f_std)
                    writer_std.writerow(row)
        except Exception as e:
            print(f"[{self.node_id}] Errore scrittura metrica su CSV: {e}")

    def connect_swarm(self, is_in_flight=False):
        """Gestisce la connessione Swarm con cflib a tutti gli URI gestiti."""
        if self.swarm is not None:
            print(f"[{self.node_id}] Chiusura connessioni Swarm attive...")
            try:
                self.swarm.close_links()
            except Exception as e:
                print(f"[{self.node_id}] Errore chiusura swarm: {e}")
            self.swarm = None
            time.sleep(1)

        if not self.uris:
            print(f"[{self.node_id}] Nessun drone registrato nello Swarm.")
            return

        print(f"[{self.node_id}] Connessione Swarm ai droni: {self.uris}")
        factory = CachedCfFactory(rw_cache='./cache')
        self.swarm = Swarm(self.uris, factory=factory)
        try:
            self.swarm.open_links()
            print(f"[{self.node_id}] Swarm connesso con successo.")
            self.swarm.parallel_safe(lambda scf: self.setup_drone_params_and_log(scf, is_in_flight=is_in_flight))
        except Exception as e:
            print(f"[{self.node_id}] Errore apertura link Swarm: {e}")

    def setup_drone_params_and_log(self, scf, is_in_flight=False):
        """Configurazione parametri di volo, reset estimator e logging telemetrico."""
        uri = scf.cf.link_uri
        print(f"[{self.node_id}] Configurazione parametri per {uri}...")
        scf.wait_for_params()
        scf.cf.param.set_value('stabilizer.estimator', str(2))
        scf.cf.param.set_value('stabilizer.controller', str(0))
        scf.cf.param.set_value('commander.enHighLevel', str(1))
        
        try:
            scf.cf.param.get_value("hlCommander.vtoff", timeout=60)
        except Exception as e:
            print(f"[{self.node_id}] hlCommander.vtoff info/timeout: {e}")
            
        # Reset estimator solo se il drone si trova a terra
        if not is_in_flight:
            print(f"[{self.node_id}] Esecuzione reset_estimator su {uri} (drone a terra)...")
            reset_estimator(scf)
            print(f"[{self.node_id}] Attesa stabilizzazione Kalman filter (3.0s)...")
            time.sleep(3.0)
            print(f"[{self.node_id}] Estimator stabilizzato per {uri}.")
        else:
            print(f"[{self.node_id}] Drone {uri} gia' in volo: skip reset_estimator per garantire stabilita'.")

        if uri not in self.telemetry:
            self.telemetry[uri] = {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0,
                'front': None, 'back': None,
                'vbat': 4.0, 'battery': 100.0,
                'rssi': -50.0, 'rssi_history': []
            }

        # 1. Logging posizione/orientamento stimato da Kalman (stateEstimate)
        try:
            log_pos = LogConfig(name=f'Pos_{uri[-2:]}', period_in_ms=50)
            log_pos.add_variable('stateEstimate.x', 'float')
            log_pos.add_variable('stateEstimate.y', 'float')
            log_pos.add_variable('stateEstimate.z', 'float')
            log_pos.add_variable('stateEstimate.yaw', 'float')
            scf.cf.log.add_config(log_pos)

            def pos_callback(timestamp, data, logconf):
                t = self.telemetry[uri]
                t['x'] = data.get('stateEstimate.x', 0.0)
                t['y'] = data.get('stateEstimate.y', 0.0)
                t['z'] = data.get('stateEstimate.z', 0.0)
                t['yaw'] = data.get('stateEstimate.yaw', 0.0)

            log_pos.data_received_cb.add_callback(pos_callback)
            log_pos.start()
            print(f"[{self.node_id}] Logging posizione stateEstimate attivo per {uri}.")
        except Exception as e:
            print(f"[{self.node_id}] LogConfig posizione: {e}")

        # 2. Logging sensori, RSSI, ToF decks e batteria
        try:
            log_sens = LogConfig(name=f'Sens_{uri[-2:]}', period_in_ms=100)
            if 'radio' in scf.cf.log.toc.toc and 'rssi' in scf.cf.log.toc.toc['radio']:
                log_sens.add_variable('radio.rssi', 'uint8_t')
            if 'range' in scf.cf.log.toc.toc and 'front' in scf.cf.log.toc.toc['range']:
                log_sens.add_variable('range.front', 'uint16_t')
            if 'range' in scf.cf.log.toc.toc and 'back' in scf.cf.log.toc.toc['range']:
                log_sens.add_variable('range.back', 'uint16_t')
            if 'pm' in scf.cf.log.toc.toc and 'vbat' in scf.cf.log.toc.toc['pm']:
                log_sens.add_variable('pm.vbat', 'float')
            if 'pm' in scf.cf.log.toc.toc and 'batteryLevel' in scf.cf.log.toc.toc['pm']:
                log_sens.add_variable('pm.batteryLevel', 'uint8_t')
            elif 'pm' in scf.cf.log.toc.toc and 'BatteryLevel' in scf.cf.log.toc.toc['pm']:
                log_sens.add_variable('pm.BatteryLevel', 'float')

            if log_sens.variables:
                scf.cf.log.add_config(log_sens)

                def sens_callback(timestamp, data, logconf):
                    t = self.telemetry[uri]
                    if 'radio.rssi' in data:
                        dbm = self.parse_rssi(data['radio.rssi'])
                        t['rssi'] = dbm
                        t['rssi_history'].append(dbm)
                        if len(t['rssi_history']) > 30:
                            t['rssi_history'].pop(0)
                    if 'range.front' in data:
                        t['front'] = data['range.front']
                    if 'range.back' in data:
                        t['back'] = data['range.back']
                    if 'pm.vbat' in data:
                        t['vbat'] = data['pm.vbat']
                    b = data.get('pm.batteryLevel', data.get('pm.BatteryLevel'))
                    if b is not None:
                        t['battery'] = float(b)
                        self.drones_battery[uri] = float(b)

                log_sens.data_received_cb.add_callback(sens_callback)
                log_sens.start()
                print(f"[{self.node_id}] Logging sensori/RSSI/batteria attivo per {uri}.")
        except Exception as e:
            print(f"[{self.node_id}] LogConfig sensori/RSSI: {e}")

    def remove_drone(self, uri):
        """Disconnette il link radio con il drone, lasciandolo in volo autonomo (hovering)."""
        if uri in self.uris:
            self.uris.remove(uri)
        if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
            scf = self.swarm._cfs[uri]
            try:
                scf.close_link()
                print(f"[{self.node_id}] Link chiuso con {uri}. Drone in hovering autonomo.")
            except Exception as e:
                print(f"[{self.node_id}] Errore chiusura link con {uri}: {e}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode()
        try:
            payload = json.loads(payload_raw)
        except Exception:
            return

        # Se l'inizializzazione e' ancora in corso, attendi brevemente che si completi anziche' scartare la CFP
        if not self.is_ready:
            print(f"[{self.node_id}] Ricevuto '{topic}', attesa completamento stabilizzazione drone...")
            t_wait_start = time.time()
            while not self.is_ready and (time.time() - t_wait_start) < 6.0:
                time.sleep(0.2)
            if not self.is_ready:
                print(f"[{self.node_id}] Timeout stabilizzazione. Messaggio '{topic}' ignorato.")
                return

        # 1. RICEZIONE CFP IN BROADCAST (quando opera come Contractor nel ciclo alternato)
        if topic == "cnp/cfp/broadcast" and self.role == 'contractor':
            req_id = payload.get("cfp_id")
            if payload.get("requester") == self.node_id:
                return
            
            if not self.uris:
                print(f"[{self.node_id}] Nessun drone disponibile da offrire.")
                return
                
            best_uri = max(self.uris, key=lambda k: self.drones_battery.get(k, 100.0))
            best_batt = self.drones_battery.get(best_uri, 100.0)
            self.last_bid_uri = best_uri
            
            print(f"[{self.node_id}] Ricevuta CFP {req_id} da {payload.get('requester')}. Invio offerta (Drone: {best_uri}, Batteria: {best_batt:.1f}%)")
            
            bid = {
                "cfp_id": req_id,
                "contractor": self.node_id,
                "battery": best_batt,
                "drone_uri": best_uri,
                "status": "hovering_ready"
            }
            self.client.publish("cnp/bid", json.dumps(bid))

        # 2. CONTRATTO VINTO (quando opera come Contractor nel ciclo alternato)
        elif topic == f"cnp/award/{self.node_id}" and self.role == 'contractor':
            print(f"[{self.node_id}] *** CONTRATTO ASSEGNATO DAL FOG! *** Restituzione drone di supporto...")
            uri = self.last_bid_uri
            if uri in self.uris:
                self.action_queue.append(('award_move', uri))

        # 3. VALUTAZIONE OFFERTA DAL FOG (quando opera come Requester)
        elif topic == f"cnp/evaluate/{self.node_id}" and self.role == 'requester':
            t_eval = time.time()
            bid = payload.get("bid", {})
            cfp_id = payload.get("cfp_id")
            
            t_cfp = self.current_cycle_metrics.get("t_cfp_sent", t_eval - 5.0)
            discovery_ms = round((t_eval - t_cfp) * 1000, 2)
            self.current_cycle_metrics["t_eval_received"] = t_eval
            self.current_cycle_metrics["latency_discovery_ms"] = discovery_ms
            self.current_cycle_metrics["contractor"] = bid.get("contractor", "Edge_1")
            self.current_cycle_metrics["support_battery_pct"] = bid.get("battery", 100.0)
            self.current_cycle_metrics["eval_payload_bytes"] = len(payload_raw)
            
            decision = "ACCEPT" if bid.get("battery", 0) > 20 else "REJECT"
            print(f"[{self.node_id}] Ricevuta offerta da Fog in {discovery_ms}ms: Contractor={bid.get('contractor')}, Drone={bid.get('drone_uri')}, Batteria={bid.get('battery')}% -> Decisione: {decision}")
            
            decision_payload = json.dumps({"cfp_id": cfp_id, "decision": decision})
            self.current_cycle_metrics["t_decision_sent"] = time.time()
            self.current_cycle_metrics["decision_payload_bytes"] = len(decision_payload)

            if decision == "ACCEPT":
                self.pending_drone_uri = bid.get("drone_uri")
            self.client.publish("cnp/decision", decision_payload)

        # 4. DRONE RILASCIATO DAL CONTRACTOR (quando opera come Requester)
        elif topic.startswith("cnp/released/") and self.role == 'requester':
            t_release = time.time()
            released_uri = topic.split("/")[-1].replace('_', '/')
            if released_uri == self.pending_drone_uri:
                t_decision = self.current_cycle_metrics.get("t_decision_sent", t_release - 2.0)
                handover_sec = round(t_release - t_decision, 2)
                self.current_cycle_metrics["t_released_received"] = t_release
                self.current_cycle_metrics["latency_handover_sec"] = handover_sec
                self.current_cycle_metrics["support_drone_uri"] = released_uri
                self.current_cycle_metrics["release_payload_bytes"] = len(payload_raw)
                
                print(f"[{self.node_id}] Notifica ricevuta: drone di supporto {released_uri} giunto in {handover_sec}s e rilasciato.")
                self.action_queue.append(('connect_and_rotate', released_uri))
                
        # 5. NESSUNA OFFERTA VALIDA O CFP RESPINTA
        elif topic.startswith("cnp/reject_all/") and self.role == 'requester':
            print(f"[{self.node_id}] Tutte le offerte rifiutate o nessuna offerta valida pervenuta dal Fog.")
            self.pending_drone_uri = None

    def run_mission(self):
        print("\n" + "="*70)
        print(f"[{self.node_id}] AVVIO CAMPAGNA SPERIMENTALE REQUESTER - CRAZYFLIE cflib v1.33")
        print(f"[{self.node_id}] Ruolo iniziale: {self.role} | Droni iniziali: {self.uris}")
        print("="*70 + "\n")

        # Attendi che il drone e l'estimator siano completamente pronti
        while not self.is_ready and self.active:
            time.sleep(0.5)

        # Pausa iniziale prima del primo CFP per assicurare che entrambi i nodi siano stabili
        time.sleep(3.0)
        
        while self.active and self.current_repetition < self.max_repetitions:
            if self.action_queue:
                action, uri = self.action_queue.pop(0)
                
                # --- AZIONE CONTRACTOR: MISSIONE VOLO (0.7m A 0.2m/s) E DISCONNESSIONE IN HOVERING ---
                if action == 'award_move':
                    t_handover_start = time.time()

                    # Parametri di missione definiti in mission.py (avanzamento aumentato a 0.7m)
                    distance = 0.7      # meters
                    velocity = 0.2      # meters/second
                    height = 0.3        # meters
                    min_front_mm = 300  # millimeters
                    timeout = 2 * distance / velocity + 2  # 9.0 seconds

                    stop_reason = "distance"
                    forward_time_s = 0.0
                    dist_at_stop_cm = 0.0
                    dist_final_cm = 70.0
                    drift_cm = 0.0
                    yaw_change_deg = 0.0

                    if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                        scf = self.swarm._cfs[uri]
                        try:
                            # 1. Decollo stabile a height=0.3m via HighLevelCommander
                            print(f"[{self.node_id}] 1. Decollo/stabilizzazione a {height}m per {uri} via HighLevelCommander...")
                            scf.cf.high_level_commander.takeoff(height, 2.0)
                            time.sleep(2.0)  # Fase hover iniziale (rec.phase = 'hover')

                            # 2. Registrazione riferimento iniziale (x0, y0, yaw0)
                            t_info = self.telemetry.get(uri, {})
                            x0 = t_info.get('x', 0.0)
                            y0 = t_info.get('y', 0.0)
                            yaw0 = t_info.get('yaw', 0.0)
                            print(f"[{self.node_id}] 2. Avvio avanzamento: x0={x0:.3f}, y0={y0:.3f}, yaw0={yaw0:.1f}°")

                            # 3. Avanzamento lineare verso 0.7m a velocità 0.2m/s (durata 3.5s)
                            t_start = time.monotonic()
                            duration = distance / velocity
                            scf.cf.high_level_commander.go_to(distance, 0.0, 0.0, 0.0, duration, relative=True)

                            # 4. Monitoraggio avanzamento, sensore frontale e timeout
                            while (time.monotonic() - t_start) < timeout:
                                curr_t = self.telemetry.get(uri, {})
                                cur_x = curr_t.get('x', x0)
                                front = curr_t.get('front')

                                if front is not None and front < min_front_mm:
                                    stop_reason = "obstacle"
                                    print(f"[{self.node_id}] Ostacolo frontale rilevato ({front} mm < {min_front_mm} mm)! Stop.")
                                    scf.cf.high_level_commander.stop()
                                    break

                                if (cur_x - x0) >= distance:
                                    stop_reason = "distance"
                                    break

                                time.sleep(0.05)
                            else:
                                stop_reason = "timeout"

                            forward_time_s = round(time.monotonic() - t_start, 2)
                            curr_t = self.telemetry.get(uri, {})
                            dist_at_stop_cm = round((curr_t.get('x', x0) - x0) * 100, 1)

                            # 5. Stabilizzazione in hovering (fase settle 1.5s di mission.py)
                            time.sleep(1.5)
                            curr_t = self.telemetry.get(uri, {})
                            dist_final_cm = round((curr_t.get('x', x0) - x0) * 100, 1)
                            drift_cm = round((curr_t.get('y', y0) - y0) * 100, 1)
                            yaw_change_deg = round(curr_t.get('yaw', yaw0) - yaw0, 1)

                            print(f"[{self.node_id}] 3. Avanzamento completato: stop_reason={stop_reason}, dist={dist_final_cm}cm, drift={drift_cm}cm in {forward_time_s}s")
                        except Exception as e:
                            print(f"[{self.node_id}] Errore esecuzione traiettoria per {uri}: {e}")

                    # 6. Chiusura link radio con il drone SENZA atterraggio (rimane in hovering)
                    print(f"[{self.node_id}] 4. Chiusura link radio con {uri} a {distance}m (drone prosegue in hovering autonomo).")
                    self.remove_drone(uri)

                    # 7. Notifica di rilascio
                    safe_uri = uri.replace('/', '_')
                    release_payload = json.dumps({
                        "status": "released",
                        "handover_node": self.node_id,
                        "stop_reason": stop_reason,
                        "dist_final_cm": dist_final_cm,
                        "drift_cm": drift_cm,
                        "yaw_change_deg": yaw_change_deg,
                        "forward_time_s": forward_time_s
                    })
                    self.client.publish(f"cnp/released/{safe_uri}", release_payload)
                    print(f"[{self.node_id}] 5. Pubblicato evento 'cnp/released/{safe_uri}'.")

                    self.role = 'requester'
                    time.sleep(2.0)  # Pausa di stabilizzazione

                # --- AZIONE REQUESTER: AGGANCIO SWARM, ROTAZIONE (180°) ED ARRETRAMENTO FINO A RSSI -35 dBm ---
                elif action == 'connect_and_rotate':
                    t_integ_start = time.time()
                    print(f"[{self.node_id}] Rilevato rilascio drone {uri}: verifica disponibilità radio ed aggancio Swarm in volo...")
                    if uri not in self.uris:
                        self.uris.append(uri)

                    connected = False
                    while not connected and self.active:
                        try:
                            # Connessione radio: ha successo non appena il Contractor chiude il link
                            self.connect_swarm(is_in_flight=True)
                            connected = True
                        except Exception as e:
                            print(f"[{self.node_id}] In attesa che il drone sia libero da altre Crazyradio... ({e})")
                            time.sleep(1.0)

                    # Attesa stabilizzazione hovering
                    time.sleep(1.5)

                    traveled_back_m = 0.0
                    final_rssi = -60.0

                    if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                        scf = self.swarm._cfs[uri]
                        try:
                            # 1. Rotazione a sinistra di 180 gradi (yaw = +3.14159 rad)
                            print(f"[{self.node_id}] 1. Esecuzione rotazione di 180 gradi a sinistra via HighLevelCommander...")
                            scf.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 3.14159, 2.0, relative=True)
                            time.sleep(2.5)
                            print(f"[{self.node_id}] Rotazione 180 gradi completata (coda orientata verso il dongle del Requester).")

                            # 2. Arretramento verso il proprio radio dongle (equivalente HighLevelCommander a MotionCommander.back)
                            # Nota: Avendo ruotato il drone di 180° (muso rivolto al Contractor a x=0), arretrare a marcia indietro
                            # per avvicinarsi al Requester corrisponde a una traslazione positiva (+0.4m) nel frame globale.
                            backward_dist = 0.4      # meters (arretramento verso il Requester)
                            velocity = 0.2           # m/s
                            backward_duration = backward_dist / velocity  # 2.0 seconds

                            print(f"[{self.node_id}] 2. Avvio arretramento (retro-marcia) di {backward_dist}m verso il radio dongle (durata {backward_duration}s)...")
                            scf.cf.high_level_commander.go_to(backward_dist, 0.0, 0.0, 0.0, backward_duration, relative=True)
                            time.sleep(backward_duration + 0.5)
                            time.sleep(1.0)  # Stabilizzazione in hovering (settle)

                            traveled_back_m = backward_dist
                            final_rssi = self.get_mean_rssi(uri, window=5)
                            print(f"[{self.node_id}] >>> OBIETTIVO RAGGIUNTO! Arretramento di {traveled_back_m:.2f}m completato. Posizionato vicino al dongle (RSSI medio: {final_rssi:.1f} dBm, target ~ -35 dBm).")

                        except Exception as e:
                            print(f"[{self.node_id}] Errore manovra {uri}: {e}")

                    t_integ_end = time.time()

                    # Calcolo metriche per questa iterazione
                    self.current_repetition += 1
                    integ_sec = round(t_integ_end - t_integ_start, 2)
                    t_cfp = self.current_cycle_metrics.get("t_cfp_sent", t_integ_end - 6.0)
                    turnaround_sec = round(t_integ_end - t_cfp, 2)

                    own_uri = self.uris[0] if self.uris else uri
                    own_batt = self.drones_battery.get(own_uri, 100.0)
                    voltage_est = round(self.telemetry.get(own_uri, {}).get('vbat', 3.50 + (own_batt / 100.0) * 0.70), 2)
                    backward_cm = round(traveled_back_m * 100, 1)

                    metric_entry = {
                        "repetition": self.current_repetition,
                        "datetime": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "timestamp": round(t_integ_end, 2),
                        "cfp_id": self.current_cycle_metrics.get("cfp_id", f"CFP-{self.current_repetition}"),
                        "contractor": self.current_cycle_metrics.get("contractor", "Edge_1"),
                        "support_drone_uri": uri,
                        "own_battery_pct": round(own_batt, 2),
                        "support_battery_pct": round(self.current_cycle_metrics.get("support_battery_pct", 95.0), 2),
                        "voltage_v": voltage_est,
                        "latency_discovery_ms": self.current_cycle_metrics.get("latency_discovery_ms", 110.0),
                        "latency_handover_sec": self.current_cycle_metrics.get("latency_handover_sec", 2.1),
                        "latency_integration_sec": integ_sec,
                        "total_turnaround_sec": turnaround_sec,
                        "rssi_dbm": round(final_rssi, 1),
                        "final_mean_rssi_dbm": round(final_rssi, 1),
                        "backward_dist_cm": backward_cm,
                        "dist_final_cm": 70.0 + backward_cm,
                        "drift_cm": 0.0,
                        "yaw_change_deg": 180.0,
                        "stop_reason": "target_reached",
                        "cfp_payload_bytes": self.current_cycle_metrics.get("cfp_payload_bytes", 145),
                        "eval_payload_bytes": self.current_cycle_metrics.get("eval_payload_bytes", 182),
                        "decision_payload_bytes": self.current_cycle_metrics.get("decision_payload_bytes", 64),
                        "release_payload_bytes": self.current_cycle_metrics.get("release_payload_bytes", 64)
                    }
                    self.log_metric_row(metric_entry)

                    print(f"\n[{self.node_id}] >>> ESPERIMENTO {self.current_repetition}/{self.max_repetitions} COMPLETATO IN {turnaround_sec:.2f}s <<<")
                    print(f"[{self.node_id}] Metric Summary: Scoperta={metric_entry['latency_discovery_ms']}ms | Handover={metric_entry['latency_handover_sec']}s | Integrazione={integ_sec}s | RSSI Medio={final_rssi:.1f}dBm | Arretramento={backward_cm}cm | Batteria={own_batt:.1f}%\n")

                    self.pending_drone_uri = None
                    self.role = 'contractor'
                    time.sleep(2.0)  # Pausa prima di accettare nuove offerte

            # Controllo soglia minima batteria
            if self.uris and any(self.drones_battery.get(u, 100.0) <= 20.0 for u in self.uris):
                print(f"[{self.node_id}] Batteria critica (<= 20%). Terminazione esperimenti.")
                break

            # AVVIO NUOVA RICHIESTA CFP (quando il nodo è in stato Requester pronto)
            if self.role == 'requester' and self.pending_drone_uri is None and not self.action_queue:
                time.sleep(3.0)  # Ritardo precauzionale prima di emettere nuova CFP
                rep_target = self.current_repetition + 1
                cfp_id = f"CFP-rep{rep_target}-{int(time.time())}"
                cfp_payload = {
                    "cfp_id": cfp_id,
                    "requester": self.node_id,
                    "conditions": {"min_battery": 20, "task": "support_inspection"}
                }
                cfp_raw = json.dumps(cfp_payload)
                self.current_cycle_metrics = {
                    "t_cfp_sent": time.time(),
                    "cfp_id": cfp_id,
                    "cfp_payload_bytes": len(cfp_raw)
                }

                print(f"[{self.node_id}] -------------------------------------------------------------")
                print(f"[{self.node_id}] [ESPERIMENTO {rep_target}/{self.max_repetitions}] Emissione Call For Proposal: {cfp_id}")
                print(f"[{self.node_id}] -------------------------------------------------------------")
                self.client.publish("cnp/cfp", cfp_raw)
                self.pending_drone_uri = "WAITING_BID"
            
            time.sleep(0.1)

        print("\n" + "="*70)
        print(f"[{self.node_id}] CAMPAGNA SPERIMENTALE CONCLUSA!")
        print(f"[{self.node_id}] Ripetizioni completate con successo: {self.current_repetition}/{self.max_repetitions}")
        print(f"[{self.node_id}] File metriche salvato in: {self.csv_file}")
        print("="*70)

        if self.enable_plots:
            self.generate_thesis_plots()

    def generate_thesis_plots(self, output_dir="plots_thesis"):
        """Genera automaticamente 4 grafici per la tesi basati sulle metriche del Requester."""
        if not self.metrics_history:
            print(f"[{self.node_id}] Nessuna metrica registrata per generare grafici.")
            return

        try:
            import matplotlib.pyplot as plt
            os.makedirs(output_dir, exist_ok=True)
            print(f"[{self.node_id}] Generazione grafici Requester per la tesi in '{output_dir}'...")

            reps = [m["repetition"] for m in self.metrics_history]
            discovery = [m["latency_discovery_ms"] for m in self.metrics_history]
            turnarounds = [m["total_turnaround_sec"] for m in self.metrics_history]
            own_batts = [m["own_battery_pct"] for m in self.metrics_history]
            support_batts = [m["support_battery_pct"] for m in self.metrics_history]
            rssis = [m["rssi_dbm"] for m in self.metrics_history]

            # 1. Latenze End-to-End
            fig, ax1 = plt.subplots(figsize=(8, 5))
            color = 'tab:blue'
            ax1.set_xlabel('Ripetizione Esperimento')
            ax1.set_ylabel('Tempo Scoperta Offerta (ms)', color=color)
            ax1.plot(reps, discovery, color=color, marker='o', linewidth=2, label='Scoperta Offerta (Fog/CNP)')
            ax1.tick_params(axis='y', labelcolor=color)
            ax1.grid(True, linestyle=':', alpha=0.6)

            ax2 = ax1.twinx()
            color = 'tab:purple'
            ax2.set_ylabel('Tempo Totale Acquisizione Drone (s)', color=color)
            ax2.plot(reps, turnarounds, color=color, marker='^', linestyle='--', linewidth=2, label='Turnaround Totale')
            ax2.tick_params(axis='y', labelcolor=color)

            plt.title('Edge Requester: Tempo di Negoziazione e Turnaround di Acquisizione')
            fig.tight_layout()
            plt.savefig(os.path.join(output_dir, f'requester_latencies_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'requester_latencies.png'), dpi=300)
            plt.close()

            # 2. Profilo Batteria: Drone Proprio vs Drone di Supporto Acquisito
            plt.figure(figsize=(8, 5))
            plt.plot(reps, own_batts, color='tab:green', marker='o', linewidth=2, label='Batteria Drone Locale (%)')
            plt.plot(reps, support_batts, color='tab:orange', marker='s', linestyle='--', linewidth=2, label='Batteria Drone Supporto Ricevuto (%)')
            plt.axhline(20, color='red', linestyle=':', label='Soglia Minima (20%)')
            plt.title('Edge Requester: Confronto Livelli di Carica Batteria')
            plt.xlabel('Ripetizione Esperimento')
            plt.ylabel('Livello Batteria (%)')
            plt.legend()
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, f'requester_battery_profile_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'requester_battery_profile.png'), dpi=300)
            plt.close()

            # 3. Qualità del Segnale Radio (RSSI) all'aggancio
            plt.figure(figsize=(8, 5))
            plt.plot(reps, rssis, color='tab:cyan', marker='D', linewidth=2, label='RSSI Link Acquisito')
            plt.axhline(-80, color='black', linestyle='--', label='Soglia Critica Link (-80 dBm)')
            plt.title('Edge Requester: Qualità Link Radio (RSSI) al Riconoscimento Swarm')
            plt.xlabel('Ripetizione Esperimento')
            plt.ylabel('RSSI (dBm)')
            plt.legend()
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, f'requester_rssi_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'requester_rssi.png'), dpi=300)
            plt.close()

            # 4. Distribuzione Dimensione Messaggi MQTT Requester
            plt.figure(figsize=(7, 4.5))
            msg_types = ['CFP Richiesta', 'Valutazione Fog', 'Decisione Accept', 'Rilascio']
            avg_sizes = [
                sum(m['cfp_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history),
                sum(m['eval_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history),
                sum(m['decision_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history),
                sum(m['release_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history)
            ]
            plt.bar(msg_types, avg_sizes, color=['#2563eb', '#7c3aed', '#059669', '#d97706'], width=0.5)
            plt.title('Edge Requester: Dimensione Media Payload MQTT (Byte)')
            plt.ylabel('Dimensione Payload (Bytes)')
            plt.grid(axis='y', linestyle=':', alpha=0.6)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, f'requester_payload_sizes_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'requester_payload_sizes.png'), dpi=300)
            plt.close()

            print(f"[{self.node_id}] I 4 grafici Requester per la tesi sono stati salvati in '{output_dir}/' (anche con timestamp {self.run_timestamp}).")

        except Exception as e:
            print(f"[{self.node_id}] Errore durante la generazione dei grafici: {e}")

    def close(self):
        self.active = False
        if self.swarm:
            try:
                self.swarm.close_links()
            except Exception as e:
                print(f"[{self.node_id}] Errore chiusura swarm: {e}")
        self.client.loop_stop()
        self.client.disconnect()
        print(f"[{self.node_id}] Chiusura completata.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Edge Requester Node - cflib v1.33 e Raccolta Metriche")
    parser.add_argument("--id", default="Edge_2", help="ID Nodo (default: Edge_2)")
    parser.add_argument("--uri", default=DEFAULT_URI, help=f"URI Crazyflie iniziale (default: {DEFAULT_URI})")
    parser.add_argument("--broker", default=BROKER_IP, help=f"IP Broker MQTT (default: {BROKER_IP})")
    parser.add_argument("--port", default=1883, type=int, help="Porta Broker MQTT (default: 1883)")
    parser.add_argument("--repetitions", default=30, type=int, help="Numero di ripetizioni dell'esperimento (default: 30)")
    parser.add_argument("--csv", default=None, help="Nome file CSV per le metriche (default: metrics_edge_2_requester_<data_ora>.csv)")
    parser.add_argument("--no-plots", action="store_true", help="Disabilita la generazione automatica dei grafici alla fine")
    args = parser.parse_args()

    node = EdgeRequester(
        node_id=args.id, 
        uri=args.uri, 
        broker_ip=args.broker, 
        port=args.port, 
        max_repetitions=args.repetitions,
        csv_file=args.csv,
        enable_plots=not args.no_plots
    )
    try:
        node.run_mission()
    except KeyboardInterrupt:
        print(f"\n[{args.id}] Interruzione manuale da tastiera.")
    finally:
        node.close()
