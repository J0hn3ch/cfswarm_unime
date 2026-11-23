import paho.mqtt.client as mqtt
import json
import time
import argparse
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, CachedCfFactory

BROKER_IP = "192.168.137.1"

class EdgeContractor:
    def __init__(self, node_id, uri):
        self.node_id = node_id
        self.uris = [uri]
        self.swarm = None
        self.drones_battery = {uri: 100}
        
        self.role = 'contractor'
        self.pending_drone_uri = None
        self.action_queue = []
        self.last_bid_uri = None
        self.active = True
        
        self.client = mqtt.Client(client_id=self.node_id)
        self.client.on_message = self.on_message
        self.client.connect(BROKER_IP, 1883, 60)
        self.client.subscribe([
            ("cnp/cfp", 0),
            ("cnp/cfp/broadcast", 0), 
            (f"cnp/award/{self.node_id}", 0),
            (f"cnp/evaluate/{self.node_id}", 0),
            ("cnp/released/#", 0),
            ("cnp/reject_all/#", 0)
        ])
        self.client.loop_start()

        cflib.crtp.init_drivers(enable_serial_driver=False)
        self.connect_swarm()

    def connect_swarm(self):
        # 1. Chiudi la connessione con eventuali Crazyflie attivi
        if self.swarm is not None:
            print(f"[{self.node_id}] Chiusura connessioni attive...")
            try:
                self.swarm.close_links()
            except Exception as e:
                print(f"Errore chiusura swarm: {e}")
            self.swarm = None
            time.sleep(1)
            
        print(f"[{self.node_id}] Connessione Swarm ai droni: {self.uris}")
        factory = CachedCfFactory(rw_cache='./cache')
        # 2. Scrivi un altra funzione per la connessione ai droni che utilizzi la classe Swarm di CFlib.
        self.swarm = Swarm(self.uris, factory=factory)
        self.swarm.open_links()
        print(f"[{self.node_id}] Swarm connesso.")
        
        self.swarm.parallel_safe(self.setup_drone_params_and_log)

    def setup_drone_params_and_log(self, scf):
        uri = scf.cf.link_uri
        print(f"[{self.node_id}] Setup parametri per {uri}...")
        scf.cf.param.set_value('stabilizer.estimator', '2')
        scf.cf.param.set_value('stabilizer.controller', '0')
        scf.cf.param.set_value('commander.enHighLevel', '1')
        
        # Inizializziamo a 100 se non esiste
        if uri not in self.drones_battery:
            self.drones_battery[uri] = 100
            
        # Auto-detect variable name and type from TOC
        batt_var = 'pm.batteryLevel'
        if 'pm' in scf.cf.log.toc.toc and 'BatteryLevel' in scf.cf.log.toc.toc['pm']:
            batt_var = 'pm.BatteryLevel'
            
        log_conf = LogConfig(name=f'Batt_{uri[-2:]}', period_in_ms=1000)
        # Omettiamo il tipo per lasciare che cflib usi quello corretto dal TOC (evita unpacking errato se fosse float)
        log_conf.add_variable(batt_var)
        scf.cf.log.add_config(log_conf)
        
        def battery_callback(timestamp, data, logconf):
            val = data.get(batt_var, 100)
            # Assicuriamoci che sia un float per i confronti, nel caso in cui arrivasse come int o stringa
            self.drones_battery[uri] = float(val)
            
        log_conf.data_received_cb.add_callback(battery_callback)
        log_conf.start()

    def remove_drone(self, uri):
        if uri in self.uris:
            self.uris.remove(uri)
            if uri in self.drones_battery:
                del self.drones_battery[uri]
            
            if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                scf = self.swarm._cfs[uri]
                scf.close_link()

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = json.loads(msg.payload.decode())

        if topic == "cnp/cfp/broadcast" and self.role == 'contractor':
            req_id = payload["cfp_id"]
            if payload["requester"] == self.node_id: return
            
            if not self.uris:
                print(f"[{self.node_id}] Nessun drone disponibile per l'offerta.")
                return
                
            best_uri = max(self.uris, key=lambda k: self.drones_battery.get(k, 0))
            best_batt = self.drones_battery.get(best_uri, 0)
            self.last_bid_uri = best_uri
            
            print(f"[{self.node_id}] Ricevuta CFP {req_id}. Miglior batteria: {best_batt:.1f}%")
            
            bid = {
                "cfp_id": req_id,
                "contractor": self.node_id,
                "battery": best_batt,
                "drone_uri": best_uri,
                "status": "hovering_ready"
            }
            self.client.publish("cnp/bid", json.dumps(bid))
            
        elif topic == f"cnp/award/{self.node_id}" and self.role == 'contractor':
            print(f"[{self.node_id}] *** CONTRATTO VINTO! ***")
            uri = self.last_bid_uri
            if uri in self.uris:
                self.action_queue.append(('award_move', uri))
            
        elif topic == f"cnp/evaluate/{self.node_id}" and self.role == 'requester':
            bid = payload["bid"]
            cfp_id = payload["cfp_id"]
            
            decision = "ACCEPT" if bid["battery"] > 20 else "REJECT"
            print(f"[{self.node_id}] Decisione {decision} per l'offerta da {bid['contractor']}")
            if decision == "ACCEPT":
                self.pending_drone_uri = bid["drone_uri"]
            self.client.publish("cnp/decision", json.dumps({"cfp_id": cfp_id, "decision": decision}))

        elif topic.startswith("cnp/released/") and self.role == 'requester':
            released_uri = topic.split("/")[-1].replace('_', '/')
            if released_uri == self.pending_drone_uri:
                print(f"[{self.node_id}] Il drone di supporto {released_uri} rilasciato.")
                self.action_queue.append(('connect_and_rotate', released_uri))
                
        elif topic.startswith("cnp/reject_all/") and self.role == 'requester':
            print(f"[{self.node_id}] CFP fallita o nessuna offerta valida.")
            self.pending_drone_uri = None

    def run_mission(self):
        print(f"[{self.node_id}] Avvio simulazione. Ruolo iniziale: {self.role}")
        while self.active:
            if self.action_queue:
                action, uri = self.action_queue.pop(0)
                if action == 'award_move':
                    print(f"[{self.node_id}] Decollo e sposto avanti di 1.5m {uri} (senza atterrare)...")
                    if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                        scf = self.swarm._cfs[uri]
                        try:
                            # Utilizziamo l'HighLevelCommander in modo che il drone
                            # mantenga autonomamente l'hovering anche se cade il link radio.
                            scf.cf.high_level_commander.takeoff(0.3, 2.0)
                            time.sleep(2.0)
                            scf.cf.high_level_commander.go_to(0.9, 0, 0, 0, 2.0, relative=True)
                            time.sleep(2.0)
                        except Exception as e:
                            print(f"Errore movimento: {e}")
                        
                    print(f"[{self.node_id}] Chiudo connessione con {uri}")
                    self.remove_drone(uri)
                    safe_uri = uri.replace('/', '_')
                    self.client.publish(f"cnp/released/{safe_uri}", json.dumps({"status": "released"}))
                    self.role = 'requester'

                elif action == 'connect_and_rotate':
                    print(f"[{self.node_id}] Aggiungo {uri} alla lista e riconnetto lo Swarm...")
                    # 3. Lo URI del drone di supporto del contractor deve essere memorizzato in una lista di URI 
                    if uri not in self.uris:
                        self.uris.append(uri)
                    
                    connected = False
                    while not connected and self.active:
                        try:
                            # 1. Chiudi la connessione con eventuali Crazyflie attivi e 2. Scrivi un altra funzione...
                            self.connect_swarm()
                            connected = True
                        except Exception as e:
                            print(f"Attesa riconnessione Swarm... {e}")
                            time.sleep(2)
                    
                    print(f"[{self.node_id}] Swarm connesso. Ruoto di 180 gradi SOLO il drone {uri}...")
                    if self.swarm and self.swarm._cfs and uri in self.swarm._cfs:
                        scf = self.swarm._cfs[uri]
                        try:
                            # Fai attenzione perchè se si connette ai droni con Swarm, deve girare solo il drone di supporto
                            scf.cf.high_level_commander.go_to(0, 0, 0, 3.14159, 2.0, relative=True)
                            time.sleep(2.0)
                        except Exception as e:
                            print(f"Errore rotazione: {e}")
                            
                    self.pending_drone_uri = None
                    self.role = 'contractor'

            if self.uris and any(self.drones_battery.get(u, 100) <= 20 for u in self.uris):
                print(f"[{self.node_id}] Un drone ha la batteria <= 20%. Fine simulazione.")
                break

            if self.role == 'requester' and self.pending_drone_uri is None and not self.action_queue:
                cfp_id = f"CFP-{int(time.time())}"
                cfp_payload = {
                    "cfp_id": cfp_id,
                    "requester": self.node_id,
                    "conditions": {"min_battery": 20, "task": "support_inspection"}
                }
                print(f"[{self.node_id}] Richiesta supporto. Pubblico CFP {cfp_id}...")
                self.client.publish("cnp/cfp", json.dumps(cfp_payload))
                self.pending_drone_uri = "WAITING_BID"
            
            time.sleep(0.1)

    def close(self):
        self.active = False
        if self.swarm:
            self.swarm.close_links()
        self.client.loop_stop()
        self.client.disconnect()
        print(f"[{self.node_id}] Chiuso.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", required=True, help="ID Nodo (es. Edge_1)")
    parser.add_argument("--uri", required=True, help="URI Crazyflie")
    args = parser.parse_args()

    node = EdgeContractor(args.id, args.uri)
    try:
        node.run_mission()
    except KeyboardInterrupt:
        print("Interruzione.")
    finally:
        node.close()