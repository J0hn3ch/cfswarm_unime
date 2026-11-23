from paho.mqtt import client as mqtt_client
import json
import logging
import random
import sys
import time

BROKER_IP = "192.168.137.1" # Essendo in esecuzione su Windows stesso
TASK_ID = "task_001"
WAIT_TIME = 3 # Tempo di xattesa per le offerte

# Logging configuration
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s > %(message)s')

topic = "python/mqtt"
proposals = {}

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        logging.info("1.2 | FogNode connected to MQTT Broker!")
        client.subscribe("cnp/propose")
        logging.info("1.2.1 | \'cnp/propose\' created, subscribed")
        client.subscribe("cnp/inform")
        logging.info("1.2.2 |\'cnp/inform\' created, subscribed")
    else:
        logging.error(f"1.2 | Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    
    if msg.topic == "cnp/propose":
        if payload["task_id"] == TASK_ID:
            logging.info(f"Ricevuta offerta da {payload['node_id']}: {payload['bid']} (costo)")
            proposals[payload['node_id']] = payload['bid']
            
    elif msg.topic == "cnp/inform":
        logging.info(f"Task completato con successo dal nodo: {payload['node_id']}. Risultato: {payload['result']}")

# Inizializzazione Client MQTT
client_id = f'FogNode-{random.randint(0, 100)}' # Generate a Client ID with the subscribe prefix.
mqttc = mqtt_client.Client(
    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
    client_id=client_id
)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(BROKER_IP, 1883, 60)
mqttc.loop_start()

# 1. ATTENDI CHE IL FOG NODE SI CONNETTA
logging.info("1.1 | In attesa di connessione al broker MQTT...")
while not mqttc.is_connected():
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        logging.error("Interruzione da tastiera. Uscita...")
        sys.exit(0)

# 1.1 Esegue la funzione on_connect per confermare la connessione
# Diamo un attimo di tempo al thread MQTT per eseguire completamente la callback on_connect e stampare l'output
time.sleep(0.5)
logging.debug("1.3 | Debug: Connessione stabilita e on_connect eseguita. Termino esecuzione.")
sys.exit(0)

# 2. Resta in ascolto per vedere se nuovi Edge Node si collegano
while True:
    logging.debug("2. | In ascolto per nuovi Edge Node...")
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        logging.error("KeyboardInterrupt! Exiting...")
        sys.exit(0)




# 1. Invia la Call for Proposal (CFP)
cfp_msg = {
    "task_id": TASK_ID,
    "description": "Elaborazione dati sensori"
}

print(f"Inviando Call for Proposal per {TASK_ID}...")
mqttc.publish("cnp/cfp", json.dumps(cfp_msg))

# 2. Attendi le offerte
time.sleep(WAIT_TIME)

# 3. Valuta le offerte ed emetti Accept/Reject
if proposals:
    # Scegli l'offerta col valore più basso (es. minor carico CPU)
    winner_id = min(proposals, key=proposals.get)
    print(f"Offerte chiuse. Il vincitore è {winner_id} con l'offerta {proposals[winner_id]}")
    
    accept_msg = {"task_id": TASK_ID}
    mqttc.publish(f"cnp/accept/{winner_id}", json.dumps(accept_msg))
else:
    print("Nessuna offerta ricevuta dai nodi Edge.")

# Mantieni vivo lo script per ricevere l'Inform finale
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    mqttc.loop_stop()
