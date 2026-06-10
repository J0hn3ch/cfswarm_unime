import asyncio
from DroneNode import DroneNode

# --- CONFIGURAZIONE COSTANTI ---
B_MIN = 0.20        # Soglia minima batteria (20%)
ALPHA = 0.8         # Peso della batteria
BETA = 0.2          # Peso del segnale RSSI
CFP_TIMEOUT = 2.0   # Tempo massimo di attesa per le risposte (secondi)

class FogManager:
    def __init__(self):
        self.drones = [DroneNode(f"Drone-{i+1}") for i in range(5)]

    def evaluate_bids(self, bids):
        """Applica la funzione di utilità per trovare il drone migliore."""
        valid_bids = [b for b in bids if b["status"] == "Propose"]
        
        if not valid_bids:
            return None
            
        best_drone = None
        highest_score = -1
        
        for bid in valid_bids:
            # Calcolo del fitness score
            score = (ALPHA * bid["battery"]) + (BETA * bid["rssi"])
            print(f"[FOG] Valuto {bid['drone_id']} -> Score: {score:.3f}")
            
            if score > highest_score:
                highest_score = score
                best_drone = bid["drone_id"]
                
        return best_drone

    async def start_cnp(self):
        """Gestisce il ciclo vitale del Contract Net Protocol."""
        print("\n[FOG] Avvio Call For Proposal (CFP)...")
        
        # Invio asincrono a tutti i droni e attesa delle risposte entro un timeout
        tasks = [asyncio.create_task(drone.receive_cfp()) for drone in self.drones]
        
        try:
            # Raccogliamo le bids che arrivano entro il CFP_TIMEOUT
            done, pending = await asyncio.wait(tasks, timeout=CFP_TIMEOUT)
            bids = [task.result() for task in done]
            
            # Annulliamo le task dei droni che non hanno risposto in tempo
            for p in pending:
                p.cancel()
                print("[FOG] Un drone non ha risposto in tempo (Timeout).")
                
        except Exception as e:
            print(f"Errore di rete: {e}")
            bids = []

        print("\n[FOG] Fase di valutazione delle Bids...")
        winner = self.evaluate_bids(bids)
        
        if winner:
            print(f"\n[FOG] VINCITORE ASSEGNATO: {winner} riceve Accept-Proposal e inizia la missione.")
        else:
            print("\n[FOG] NESSUN VINCITORE: Nessun drone idoneo disponibile.")