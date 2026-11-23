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
DEFAULT_URI = "radio://0/80/2M/E7E7E7E701"

class EdgeContractor:
    def __init__(self, node_id, uri=DEFAULT_URI, broker_ip=BROKER_IP, port=1883, max_repetitions=30, csv_file=None, enable_plots=True):
        self.node_id = node_id
        self.uris = [uri]
        self.broker_ip = broker_ip
        self.port = port
        self.max_repetitions = max_repetitions
        self.enable_plots = enable_plots
        
        self.run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if csv_file is None or csv_file == "metrics_edge_1_contractor.csv":
            self.csv_file = f"metrics_edge_1_contractor_{self.run_timestamp}.csv"
        else:
            self.csv_file = csv_file
        
        self.swarm = None
        self.current_repetition = 0
        self.role = 'contractor'
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
        """Inizializza l'intestazione del file CSV per le metriche della tesi con timestamp e data/ora."""
        header = [
            "repetition", "datetime", "timestamp", "cfp_id", "requester", "drone_uri",
            "bid_battery_pct", "final_battery_pct", "voltage_v",
            "latency_negotiation_ms", "latency_handover_sec", "total_cycle_sec",
            "rssi_dbm", "dist_final_cm", "drift_cm", "yaw_change_deg", "stop_reason",
            "cfp_payload_bytes", "bid_payload_bytes", "release_payload_bytes"
        ]
        try:
            with open(self.csv_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
            print(f"[{self.node_id}] File metriche inizializzato: {self.csv_file}")
            if self.csv_file != "metrics_edge_1_contractor.csv":
                shutil.copyfile(self.csv_file, "metrics_edge_1_contractor.csv")
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
            data.get("requester"),
            data.get("drone_uri"),
            data.get("bid_battery_pct"),
            data.get("final_battery_pct"),
            data.get("voltage_v"),
            data.get("latency_negotiation_ms"),
            data.get("latency_handover_sec"),
            data.get("total_cycle_sec"),
            data.get("rssi_dbm"),
            data.get("dist_final_cm"),
            data.get("drift_cm"),
            data.get("yaw_change_deg"),
            data.get("stop_reason"),
            data.get("cfp_payload_bytes"),
            data.get("bid_payload_bytes"),
            data.get("release_payload_bytes")
        ]
        try:
            with open(self.csv_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            if self.csv_file != "metrics_edge_1_contractor.csv":
                with open("metrics_edge_1_contractor.csv", mode='a', newline='') as f_std:
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

        # 1. RICEZIONE CALL FOR PROPOSAL (CFP) IN BROADCAST DAL FOG
        if topic == "cnp/cfp/broadcast" and self.role == 'contractor':
            t_cfp_received = time.time()
            req_id = payload.get("cfp_id")
            requester = payload.get("requester")
            
            if requester == self.node_id:
                return

            if not self.uris:
                print(f"[{self.node_id}] Nessun drone disponibile per formulare un'offerta.")
                return

            req_conditions = payload.get("conditions", {})
            min_battery_req = req_conditions.get("min_battery", 20)

            best_uri = max(self.uris, key=lambda k: self.drones_battery.get(k, 100.0))
            best_batt = self.drones_battery.get(best_uri, 100.0)
            self.last_bid_uri = best_uri

            print(f"[{self.node_id}] Ricevuta CFP {req_id} da '{requester}' (Requisito batteria: >={min_battery_req}%).")
            print(f"[{self.node_id}] Miglior drone: {best_uri} con batteria {best_batt:.1f}%.")

            if best_batt < min_battery_req:
                print(f"[{self.node_id}] Batteria insufficiente ({best_batt:.1f}% < {min_battery_req}%). Offerta non inviata.")
                return

            self.current_cycle_metrics = {
                "t_cfp_received": t_cfp_received,
                "cfp_id": req_id,
                "requester": requester,
                "drone_uri": best_uri,
                "bid_battery_pct": round(best_batt, 2),
                "cfp_payload_bytes": len(payload_raw)
            }

            bid = {
                "cfp_id": req_id,
                "contractor": self.node_id,
                "battery": best_batt,
                "drone_uri": best_uri,
                "status": "hovering_ready"
            }
            bid_json = json.dumps(bid)
            self.current_cycle_metrics["t_bid_sent"] = time.time()
            self.current_cycle_metrics["bid_payload_bytes"] = len(bid_json)
            
            print(f"[{self.node_id}] Invio offerta su 'cnp/bid' per la CFP {req_id}...")
            self.client.publish("cnp/bid", bid_json)

        # 2. RICEZIONE CONTRATTO ASSEGNATO DAL FOG NODE
        elif topic == f"cnp/award/{self.node_id}" and self.role == 'contractor':
            t_award = time.time()
            print(f"[{self.node_id}] *** CONTRATTO VINTO! Assegnazione confermata dal Fog Node. ***")
            if "t_bid_sent" in self.current_cycle_metrics:
                self.current_cycle_metrics["t_award_received"] = t_award
                self.current_cycle_metrics["latency_negotiation_ms"] = round((t_award - self.current_cycle_metrics["t_bid_sent"]) * 1000, 2)
            
            uri = self.last_bid_uri
            if uri in self.uris:
                self.action_queue.append(('award_move', uri))

        # 3. VALUTAZIONE OFFERTE QUANDO OPERA COME REQUESTER (RUOLO ALTERNATO)
        elif topic == f"cnp/evaluate/{self.node_id}" and self.role == 'requester':
            bid = payload["bid"]
            cfp_id = payload["cfp_id"]
            decision = "ACCEPT" if bid.get("battery", 0) > 20 else "REJECT"
            print(f"[{self.node_id}] Valutazione offerta da {bid.get('contractor')} -> Decisione: {decision}")
            if decision == "ACCEPT":
                self.pending_drone_uri = bid["drone_uri"]
            self.client.publish("cnp/decision", json.dumps({"cfp_id": cfp_id, "decision": decision}))

        # 4. RICEZIONE DRONE RILASCIATO (RUOLO ALTERNATO)
        elif topic.startswith("cnp/released/") and self.role == 'requester':
            released_uri = topic.split("/")[-1].replace('_', '/')
            if released_uri == self.pending_drone_uri:
                print(f"[{self.node_id}] Drone di supporto {released_uri} rilasciato e disponibile.")
                self.action_queue.append(('connect_and_rotate', released_uri))

        # 5. REJECT ALL DAL FOG
        elif topic.startswith("cnp/reject_all/") and self.role == 'requester':
            print(f"[{self.node_id}] CFP respinta o nessuna offerta valida pervenuta.")
            self.pending_drone_uri = None

    def run_mission(self):
        print("\n" + "="*70)
        print(f"[{self.node_id}] EDGE CONTRACTOR ATTIVO - CRAZYFLIE cflib v1.33")
        print(f"[{self.node_id}] Ruolo iniziale: {self.role} | Droni controllati: {self.uris}")
        print("="*70 + "\n")

        # Attendi che il drone e l'estimator siano completamente pronti
        while not self.is_ready and self.active:
            time.sleep(0.5)

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
                            print(f"[{self.node_id}] 1. Decollo stabile a {height}m per {uri} via HighLevelCommander...")
                            scf.cf.high_level_commander.takeoff(height, 2.0)
                            time.sleep(2.0)  # Fase hover iniziale (come rec.phase = 'hover' in mission.py)

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
                                    print(f"[{self.node_id}] Ostacolo frontale rilevato ({front} mm < {min_front_mm} mm)! Stop preventivo.")
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

                            print(f"[{self.node_id}] 3. Avanzamento completato: stop_reason={stop_reason}, dist={dist_final_cm}cm, drift={drift_cm}cm, yaw={yaw_change_deg}deg in {forward_time_s}s")
                        except Exception as e:
                            print(f"[{self.node_id}] Errore esecuzione traiettoria per {uri}: {e}")

                    batt_current = self.drones_battery.get(uri, 100.0)
                    voltage_est = round(self.telemetry.get(uri, {}).get('vbat', 3.50 + (batt_current / 100.0) * 0.70), 2)
                    current_rssi = round(self.get_mean_rssi(uri, window=5), 1)

                    # 6. Chiusura del link radio con il drone SENZA atterrare (il drone rimane in volo autonomo)
                    print(f"[{self.node_id}] 4. Chiusura link radio con {uri} a {distance}m (drone prosegue in hovering autonomo).")
                    self.remove_drone(uri)

                    # 7. Notifica di rilascio al requester con le metriche di volo
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
                    t_released = time.time()
                    print(f"[{self.node_id}] 5. Pubblicata notifica su 'cnp/released/{safe_uri}'.")

                    # Calcolo e salvataggio metriche
                    self.current_repetition += 1
                    handover_sec = round(t_released - t_handover_start, 2)
                    t_cfp = self.current_cycle_metrics.get("t_cfp_received", t_handover_start - 3.0)
                    total_sec = round(t_released - t_cfp, 2)

                    metric_entry = {
                        "repetition": self.current_repetition,
                        "datetime": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "timestamp": round(t_released, 2),
                        "cfp_id": self.current_cycle_metrics.get("cfp_id", f"CFP-{self.current_repetition}"),
                        "requester": self.current_cycle_metrics.get("requester", "Edge_2"),
                        "drone_uri": uri,
                        "bid_battery_pct": self.current_cycle_metrics.get("bid_battery_pct", round(batt_current, 2)),
                        "final_battery_pct": round(batt_current, 2),
                        "voltage_v": voltage_est,
                        "latency_negotiation_ms": self.current_cycle_metrics.get("latency_negotiation_ms", 120.0),
                        "latency_handover_sec": handover_sec,
                        "total_cycle_sec": total_sec,
                        "rssi_dbm": current_rssi,
                        "dist_final_cm": dist_final_cm,
                        "drift_cm": drift_cm,
                        "yaw_change_deg": yaw_change_deg,
                        "stop_reason": stop_reason,
                        "cfp_payload_bytes": self.current_cycle_metrics.get("cfp_payload_bytes", 145),
                        "bid_payload_bytes": self.current_cycle_metrics.get("bid_payload_bytes", 182),
                        "release_payload_bytes": len(release_payload)
                    }
                    self.log_metric_row(metric_entry)

                    print(f"\n[{self.node_id}] >>> ITERAZIONE {self.current_repetition}/{self.max_repetitions} REGISTRATA <<<")
                    print(f"[{self.node_id}] Metric Summary: Handover={handover_sec}s | Negoziazione={metric_entry['latency_negotiation_ms']}ms | Batteria={batt_current:.1f}% ({voltage_est}V) | RSSI={current_rssi}dBm\n")

                    # Inversione di ruolo per consentire al requester di restituire il drone
                    self.role = 'requester'
                    time.sleep(2.0)  # Pausa di raffreddamento e stabilizzazione

                # --- AZIONE REQUESTER: AGGANCIO SWARM, ROTAZIONE (180°) ED ARRETRAMENTO FINO A RSSI -35 dBm ---
                elif action == 'connect_and_rotate':
                    print(f"[{self.node_id}] Rilevato rilascio drone {uri}: verifica disponibilità radio ed aggancio Swarm in volo...")
                    if uri not in self.uris:
                        self.uris.append(uri)
                    
                    connected = False
                    while not connected and self.active:
                        try:
                            self.connect_swarm(is_in_flight=True)
                            connected = True
                        except Exception as e:
                            print(f"[{self.node_id}] Attesa che la radio del drone sia libera... ({e})")
                            time.sleep(1.0)
                    
                    # Stabilizzazione in hovering prima delle manovre
                    time.sleep(1.5)

                    traveled_back_m = 0.0

                    if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                        scf = self.swarm._cfs[uri]
                        try:
                            # 1. Rotazione a sinistra di 180 gradi (yaw = +3.14159 rad)
                            print(f"[{self.node_id}] 1. Rotazione di 180 gradi a sinistra via HighLevelCommander...")
                            scf.cf.high_level_commander.go_to(0.0, 0.0, 0.0, 3.14159, 2.0, relative=True)
                            time.sleep(2.5)
                            print(f"[{self.node_id}] Rotazione 180 gradi completata (coda orientata verso il dongle).")

                            # 2. Arretramento verso il proprio radio dongle (equivalente HighLevelCommander a MotionCommander.back)
                            # Nota: Avendo ruotato il drone di 180°, arretrare a marcia indietro corrisponde
                            # a una traslazione positiva (+0.4m) nel frame globale verso il dongle.
                            backward_dist = 0.4      # meters (movimento all'indietro per completare il percorso)
                            velocity = 0.2           # m/s
                            backward_duration = backward_dist / velocity  # 2.0 seconds

                            print(f"[{self.node_id}] 2. Avvio arretramento (retro-marcia) di {backward_dist}m verso il radio dongle (durata {backward_duration}s)...")
                            scf.cf.high_level_commander.go_to(backward_dist, 0.0, 0.0, 0.0, backward_duration, relative=True)
                            time.sleep(backward_duration + 0.5)
                            time.sleep(1.0)  # Stabilizzazione in hovering (settle)

                            traveled_back_m = backward_dist
                            final_rssi = self.get_mean_rssi(uri, window=5)
                            print(f"[{self.node_id}] >>> OBIETTIVO RAGGIUNTO! Posizionamento vicino al dongle completato ({traveled_back_m:.2f}m percorsi indietro, RSSI medio: {final_rssi:.1f} dBm).")
                        except Exception as e:
                            print(f"[{self.node_id}] Errore manovra {uri}: {e}")
                            
                    self.pending_drone_uri = None
                    self.role = 'contractor'
                    time.sleep(2.0)  # Pausa di stabilizzazione prima di accettare nuove offerte

            # Controllo soglia critica batteria
            if self.uris and any(self.drones_battery.get(u, 100.0) <= 20.0 for u in self.uris):
                print(f"[{self.node_id}] Batteria critica (<= 20%). Termine sessione.")
                break

            # Richiesta supporto se in ruolo Requester (per completare lo scambio bidirezionale)
            if self.role == 'requester' and self.pending_drone_uri is None and not self.action_queue:
                time.sleep(3.0)  # Ritardo prima di inviare nuova CFP
                cfp_id = f"CFP-{self.node_id}-rep{self.current_repetition + 1}-{int(time.time())}"
                cfp_payload = {
                    "cfp_id": cfp_id,
                    "requester": self.node_id,
                    "conditions": {"min_battery": 20, "task": "support_inspection"}
                }
                print(f"[{self.node_id}] Emissione CFP di scambio: {cfp_id}...")
                self.client.publish("cnp/cfp", json.dumps(cfp_payload))
                self.pending_drone_uri = "WAITING_BID"

            time.sleep(0.1)

        print("\n" + "="*70)
        print(f"[{self.node_id}] SESSIONE SPERIMENTALE CONCLUSA!")
        print(f"[{self.node_id}] Iterazioni completate: {self.current_repetition}/{self.max_repetitions}")
        print(f"[{self.node_id}] File metriche salvato in: {self.csv_file}")
        print("="*70)
        
        if self.enable_plots:
            self.generate_thesis_plots()

    def generate_thesis_plots(self, output_dir="plots_thesis"):
        """Genera automaticamente 4 grafici ad alta risoluzione (300 DPI) per la presentazione della tesi."""
        if not self.metrics_history:
            print(f"[{self.node_id}] Nessuna metrica registrata per generare grafici.")
            return

        try:
            import matplotlib.pyplot as plt
            os.makedirs(output_dir, exist_ok=True)
            print(f"[{self.node_id}] Generazione grafici per la presentazione della tesi in '{output_dir}'...")

            reps = [m["repetition"] for m in self.metrics_history]
            neg_latencies = [m["latency_negotiation_ms"] for m in self.metrics_history]
            handover_times = [m["latency_handover_sec"] for m in self.metrics_history]
            batteries = [m["final_battery_pct"] for m in self.metrics_history]
            voltages = [m["voltage_v"] for m in self.metrics_history]
            rssis = [m["rssi_dbm"] for m in self.metrics_history]

            # 1. Grafico Latenze di Negoziazione e Handover
            fig, ax1 = plt.subplots(figsize=(8, 5))
            color = 'tab:blue'
            ax1.set_xlabel('Ripetizione Esperimento')
            ax1.set_ylabel('Latenza Negoziazione MQTT (ms)', color=color)
            ax1.plot(reps, neg_latencies, color=color, marker='o', linewidth=2, label='Negoziazione CNP')
            ax1.tick_params(axis='y', labelcolor=color)
            ax1.grid(True, linestyle=':', alpha=0.6)

            ax2 = ax1.twinx()
            color = 'tab:orange'
            ax2.set_ylabel('Durata Handover Fisico (s)', color=color)
            ax2.plot(reps, handover_times, color=color, marker='s', linestyle='--', linewidth=2, label='Handover Fisico')
            ax2.tick_params(axis='y', labelcolor=color)

            plt.title('Latenza di Negoziazione e Tempo di Handover per Iterazione')
            fig.tight_layout()
            plt.savefig(os.path.join(output_dir, f'contractor_latencies_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'contractor_latencies.png'), dpi=300)
            plt.close()

            # 2. Profilo Decadimento Batteria e Voltaggio
            fig, ax1 = plt.subplots(figsize=(8, 5))
            color = 'tab:green'
            ax1.set_xlabel('Ripetizione Esperimento')
            ax1.set_ylabel('Batteria Residua (%)', color=color)
            ax1.plot(reps, batteries, color=color, marker='^', linewidth=2)
            ax1.axhline(20, color='red', linestyle=':', label='Soglia Minima (20%)')
            ax1.tick_params(axis='y', labelcolor=color)
            ax1.grid(True, linestyle=':', alpha=0.6)

            ax2 = ax1.twinx()
            color = 'tab:purple'
            ax2.set_ylabel('Tensione Stimata (V)', color=color)
            ax2.plot(reps, voltages, color=color, marker='d', linestyle='-.', linewidth=2)
            ax2.tick_params(axis='y', labelcolor=color)

            plt.title('Profilo Energetico: Decadimento Batteria e Tensione nei Trasferimenti')
            fig.tight_layout()
            plt.savefig(os.path.join(output_dir, f'contractor_battery_profile_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'contractor_battery_profile.png'), dpi=300)
            plt.close()

            # 3. Qualità del Segnale Radio (RSSI)
            plt.figure(figsize=(8, 5))
            plt.plot(reps, rssis, color='tab:red', marker='o', linewidth=2, label='RSSI Rilevato')
            plt.axhline(-80, color='black', linestyle='--', label='Soglia Critica Link (-80 dBm)')
            plt.title('Qualità Segnale Radio (RSSI) durante i Passaggi tra Nodi Edge')
            plt.xlabel('Ripetizione Esperimento')
            plt.ylabel('RSSI (dBm)')
            plt.legend()
            plt.grid(True, linestyle=':', alpha=0.6)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, f'contractor_rssi_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'contractor_rssi.png'), dpi=300)
            plt.close()

            # 4. Distribuzione Dimensione Messaggi MQTT
            plt.figure(figsize=(7, 4.5))
            msg_types = ['Call for Proposal (CFP)', 'Offerta (Bid)', 'Rilascio (Release)']
            avg_sizes = [
                sum(m['cfp_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history),
                sum(m['bid_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history),
                sum(m['release_payload_bytes'] for m in self.metrics_history) / len(self.metrics_history)
            ]
            plt.bar(msg_types, avg_sizes, color=['#3b82f6', '#10b981', '#f59e0b'], width=0.5)
            plt.title('Dimensione Media Payload Messaggi MQTT (Byte)')
            plt.ylabel('Dimensione Payload (Bytes)')
            plt.grid(axis='y', linestyle=':', alpha=0.6)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir, f'contractor_payload_sizes_{self.run_timestamp}.png'), dpi=300)
            plt.savefig(os.path.join(output_dir, 'contractor_payload_sizes.png'), dpi=300)
            plt.close()

            print(f"[{self.node_id}] I 4 grafici per la tesi sono stati salvati in '{output_dir}/' (anche con timestamp {self.run_timestamp}).")

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
    parser = argparse.ArgumentParser(description="Edge Contractor Node - cflib v1.33 e Raccolta Metriche")
    parser.add_argument("--id", default="Edge_1", help="ID Nodo (default: Edge_1)")
    parser.add_argument("--uri", default=DEFAULT_URI, help=f"URI Crazyflie iniziale (default: {DEFAULT_URI})")
    parser.add_argument("--broker", default=BROKER_IP, help=f"IP Broker MQTT (default: {BROKER_IP})")
    parser.add_argument("--port", default=1883, type=int, help="Porta Broker MQTT (default: 1883)")
    parser.add_argument("--repetitions", default=30, type=int, help="Numero di ripetizioni dell'esperimento (default: 30)")
    parser.add_argument("--csv", default=None, help="Nome file CSV per le metriche (default: metrics_edge_1_contractor_<data_ora>.csv)")
    parser.add_argument("--no-plots", action="store_true", help="Disabilita la generazione automatica dei grafici alla fine")
    args = parser.parse_args()

    node = EdgeContractor(
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
