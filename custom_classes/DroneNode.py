import asyncio
import random

# --- CONFIGURAZIONE COSTANTI ---
B_MIN = 0.20        # Soglia minima batteria (20%)
ALPHA = 0.8         # Peso della batteria
BETA = 0.2          # Peso del segnale RSSI
CFP_TIMEOUT = 2.0   # Tempo massimo di attesa per le risposte (secondi)

class DroneNode:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        # Simuliamo letture dai sensori
        self.battery = round(random.uniform(0.05, 1.0), 2)
        self.rssi = round(random.uniform(0.3, 1.0), 2)

    async def receive_cfp(self):
        """Simula la ricezione della CFP e l'elaborazione dell'offerta."""
        # Simuliamo un ritardo di rete casuale
        await asyncio.sleep(random.uniform(0.1, 1.5))
        
        print(f"[{self.drone_id}] CFP Ricevuta. Batteria: {self.battery*100}%, RSSI: {self.rssi}")
        
        if self.battery < B_MIN:
            print(f"[{self.drone_id}] Batteria sotto soglia. Rifiuto di partecipare.")
            return {"drone_id": self.drone_id, "status": "Refuse"}
            
        return {
            "drone_id": self.drone_id,
            "status": "Propose",
            "battery": self.battery,
            "rssi": self.rssi
        }