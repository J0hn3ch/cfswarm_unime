# python 2026-10-05_node_edge1_rev1.py --uri radio://0/80/2M/E7E7E7E701 --role contractor
import paho.mqtt.client as mqtt
import json
import random
import time
import argparse
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils.reset_estimator import reset_estimator

import sys

BROKER_IP = "192.168.137.1" # IP del nodo Fog (WSL2 Laptop)

"""
# Step 1: Connessione al Crazyflie
cflib.crtp.init_drivers()
print(f"[{self.node_id}] Connessione al Crazyflie {self.uri} in corso...")
self.cf = Crazyflie(rw_cache='./cache')
self.scf = SyncCrazyflie(self.uri, cf=self.cf)
self.scf.open_link()

self.setup_battery_logging()
"""

class EdgeNode:
    def __init__(self, node_id, uri, role=None):
        self.node_id = node_id
        self.uri = uri
        self.role = role
        self.battery_level = 100
        self.scf = None
        self.mc = None
        self.edge_swarm = ['radio://0/80/2M/E7E7E7E701']
        self.scf = None
        self.winner = False

        # Inizializzazione MQTT
        self.client = mqtt.Client(client_id=self.node_id)
        self.client.on_message = self.on_message
        self.client.connect(BROKER_IP, 1883, 60)

        # Il Contractor ascolta i broadcast, le assegnazioni e i rifiuti globali
        self.client.subscribe([
            ("cnp/cfp/broadcast", 0),
            (f"cnp/award/{self.node_id}", 0),
            (f"cnp/reject_all/#", 0)
        ])
        self.client.loop_start()

        self.scf = self.crazyflie_initialization()
        
        # Logging configuration
        print("Logging configuration")
        self.setup_battery_logging()
        
    def crazyflie_initialization(self):
        # Communication protocol driver - Initialize the low-level drivers
        cflib.crtp.init_drivers(enable_serial_driver=False)
        # CrazyRadio - Interface scanning
        print(f"|- Wait for Crazyflie scanning...", end=' ')
        drones = dict() # Dictionary: { 'drone_uri': driverClass_uri}
        for uri in self.edge_swarm:
            drone_addr = uri.split('/')[-1]
            available = cflib.crtp.scan_interfaces(address=int(drone_addr, 16))
            if available:
                drones[available[0][0]] = 1
        drones_uri = list(drones.keys())
        drone_uri = drones_uri[0]
        cf_stats = Crazyflie(rw_cache='./cache')
        scf = SyncCrazyflie(drone_uri, cf=cf_stats)
        scf.open_link()
        print(f"Crazyflie {drone_uri} connected to {self.node_id}")
        time.sleep(3)  # give some time to establish connection; 3 sec seems to work
        print("|- Crazyflie - Wait for params..")
        scf.wait_for_params()
        scf.cf.param.set_value('stabilizer.estimator', str(2))
        scf.cf.param.set_value('stabilizer.controller', str(0))
        reset_estimator(scf) 
        time.sleep(1)
        return scf
        
    def setup_battery_logging(self):
        log_conf = LogConfig(name='Battery', period_in_ms=1000)
        log_conf.add_variable('pm.vbat', 'float')
        self.scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.battery_callback)
        log_conf.start()

    def battery_callback(self, timestamp, data, logconf):
        # Mappatura semplificata voltaggio (3.0V - 4.2V) a percentuale (0-100%)
        vbat = data['pm.vbat']
        self.battery_level = max(0, min(100, (vbat - 3.0) / (4.2 - 3.0) * 100))

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = json.loads(msg.payload.decode())

        if topic == "cnp/cfp/broadcast" and self.role == "contractor":
            # Step 5: Accoglimento CFP e interrogazione stato batteria
            req_id = payload["cfp_id"]
            if payload["requester"] == self.node_id: return
            
            print(f"[{self.node_id}] Ricevuta CFP {req_id}. Lettura batteria: {self.battery_level:.1f}%")
            
            # Step 6: Costruzione e invio offerta
            bid = {
                "cfp_id": req_id,
                "contractor": self.node_id,
                "battery": self.battery_level,
                "status": "hovering_ready"
            }
            self.client.publish("cnp/bid", json.dumps(bid))

        elif topic.startswith(f"cnp/evaluate/"):
            # Step 10: Il richiedente riceve l'offerta e decide
            bid = payload["bid"]
            cfp_id = payload["cfp_id"]
            print(f"[{self.node_id}] Ricevuta proposta dal Fog per nodo {bid['contractor']} (Batt:{bid['battery']:.1f}%).")
            
            # Logica di decisione (es. accetta se la batteria è > 60%)
            decision = "ACCEPT" if bid["battery"] > 60 else "REJECT"
            self.client.publish("cnp/decision", json.dumps({"cfp_id": cfp_id, "decision": decision}))

        elif topic.startswith(f"cnp/award/"):
            # Step 14: Il nodo vincitore invia il drone
            print(f"[{self.node_id}] *** CONTRATTO VINTO! *** Invio drone in supporto al nodo {payload['target_node']}")
            if self.mc:
                # Simulazione di invio spostando il drone in avanti
                print("DRONE FORWARD")
                self.mc.forward(1.0)
                time.sleep(3)
                #self.scf.open_link()
                print("DRONE CLOSE LINK") # self.mc.land()
                self.scf.close_link()

        elif topic.startswith(f"cnp/reject_all/"):
            # Step 19: Liberazione dalla CFP
            print(f"[{self.node_id}] CFP {payload.get('cfp_id')} rifiutata o scaduta. Riprendo missione originale.")

    def run_mission(self):
        # Attendo la negoziazione
        print("Waiting for negotiation...", end=' ')
        while not self.winner:
            try:
                print('*', end='')
                time.sleep(1)
            except KeyboardInterrupt:
                return
                
        # Step 2: Decollo e Hovering
        print(f"Winner! [{self.node_id}] Decollo e hovering in corso...")
        mc = MotionCommander(self.scf)
    	#with MotionCommander(self.scf) as mc:
        if mc:
            self.mc = mc
            self.mc.take_off(height=0.4, velocity=0.5)
            time.sleep(5) # Stabilizza l'hovering
            
            if self.role == "requester":
                # Step 3: Definizione richiesta di supporto
                cfp_id = f"CFP-{int(time.time())}"
                cfp_payload = {
                    "cfp_id": cfp_id,
                    "requester": self.node_id,
                    "conditions": {"min_battery": 50, "task": "support_inspection"}
                }
                print(f"[{self.node_id}] Necessito supporto. Invio CFP {cfp_id} al Fog...")
                self.client.publish("cnp/cfp", json.dumps(cfp_payload))
            
            
            print(f"[{self.node_id}] Fine missione. Atterraggio.")

    def close(self):
        if self.scf:
            self.scf.close_link()
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.add_argument("--id", required=True, help="ID Nodo (es. Edge_1)")
    parser.add_argument("--uri", required=True, help="URI Crazyflie (es. radio://0/80/2M/E7E7E7E7E7)")
    parser.add_argument("--role", choices=["requester", "contractor"], required=True, help="Ruolo nel test")
    
    args = parser.parse_args()
    client_id = f'EdgeNode1-{random.randint(0, 100)}'
    node = EdgeNode(client_id, args.uri)
    try:
        node.run_mission()
    except KeyboardInterrupt:
        print("Keyboard Interrupt Detected")
    finally:
        node.close()
        """
        self.scf.close_link()
        time.sleep(3)
        sys.exit()
        """
