"""
==========================
CRAZYRADIO DONGLES
==========================

Test RSSI
----------------------

- Crazyradio 1, serial number: 57D5141CC2A4510C
- Crazyradio 2, serial number: 59C39A09484EB47A

https://github.com/bitcraze/crazyflie-lib-python/blob/0.1.27/examples/radio/radio-test.py 

"""
import argparse
import cflib.crtp
from cflib.crtp.radio_link_statistics import RadioLinkStatistics
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
import cflib.drivers.crazyradio as crazyradio
from cflib.utils import uri_helper

from dotenv import load_dotenv
import logging
import numpy as np
import sys
from threading import Event
import time

# ------------------------------------
# ENVIRONMENT VARIABLES
# ------------------------------------
load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
URIs = []
for d in range(1,4):
    URIs.append( uri_helper.uri_from_env(env=f'DRONE{str(d)}_URI', default='radio://0/80/2M/E7E7E7E7E7') )

#uri = uri_helper.uri_from_env(env='DRONE2_URI', default='radio://0/80/2M/E7E7E7E7E7')

# ------------------------------------
# STATISTICS
# ------------------------------------
link_quality = 0
def radio_stats_cb(data):
    global link_quality
    link_quality = data
    
def link_error_cb(data):
    raise Exception

# ------------------------------------
# LOGGING
# ------------------------------------
logging.basicConfig(level=logging.DEBUG)
print("Python Version", sys.version)
print(f"Crazyradio 2.0 Version: {crazyradio.Crazyradio().version}")

rssi_logs = []
def log_radio_conf():
    log_conf = LogConfig(name="Radio", period_in_ms=500)
    log_conf.add_variable('radio.rssi', 'uint8_t') # Radio Signal Strength Indicator [dBm]
    log_conf.add_variable('radio.isConnected', 'uint8_t') # Indicator if a packet was received from the radio within the last RADIO_ACTIVITY_TIMEOUT_MS
    log_conf.add_variable('radio.numRxBc', 'uint16_t') # Number of broadcast packets received
    log_conf.add_variable('radio.numRxUc', 'uint16_t') # Number of unicast packets received

    return log_conf

log_radio = log_radio_conf()

flag_header = False
def log_radio_cb(timestamp, data, logconf):
    global flag_header
    global link_quality
    global rssi_logs
    
    if not flag_header:
        print("|-" + " RADIO LOG " + "-" * 20)
        print("| ", end='')
        print(f"Link Quality", end=' |\t')
        for key in data.keys():
            print(f"{key}", end=' |\t')
        print("")
        flag_header = True
    print("| ", end='')
    print(f"{link_quality:2.2f}", end=f'{' '*(12-1)}|\t')
    for key, value in data.items():
        rssi_logs.append(value) if key == 'radio.rssi' else None
        print(f"{value}", end=f'{' '*(len(key)-1)}|\t')
    print("")

def log_error_cb(logconf, msg):
    print("Error when logging %s" % logconf.name)

# CrazyRadio devices currently connected to the computer
print("\n|- Crazyradio Devices ---" + "-" * 10)
dongles = []
for d in crazyradio._find_devices():
    print(f"|- \t{d.manufacturer}, {d.product} (Serial number: {d.serial_number})")
    dongles.append(d)
print("|------------------------" + "-" * 10)

def main():
    radio = crazyradio.Crazyradio()

    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_serial_driver=False)

    # Interface status
    interfaces = cflib.crtp.get_interfaces_status()
    print(f"Radio Interface: {interfaces['radio']}")

    # Crazyradio Interface scanning
    drones = dict() # Dictionary: { 'drone_uri': driverClass_uri}
    for uri in URIs:
        drone_addr = uri.split('/')[-1]
        available = cflib.crtp.scan_interfaces(address=int(drone_addr, 16))
        if available:
            #drones.add(available[0][0])
            """
            drones[available[0][0]] = cflib.crtp.get_link_driver(
                uri=available[0][0], 
                #radio_link_statistics_callback=radio_stats_cb,
                link_error_callback=link_error_cb
            )
            """

            drones[available[0][0]] = 1
    drones_uri = list(drones.keys())
    print("Crazyflie available: ", drones_uri)

    drone_uri = drones_uri[0]
    print("Link", drones[drone_uri])
    #cf_stats = Crazyflie(link=drones[drone_uri], rw_cache='./cache')
    #cf_stats = Crazyflie(link=drones[drone_uri])
    cf_stats = Crazyflie(rw_cache='./cache')

    #if not drone_uri:
    #    sys.exit()

    # -----------------------
    # Sync Crazyflie with link already open
    # -----------------------
    #scf = SyncCrazyflie(link_uri=drone_uri, cf=cf_stats)
    #scf._is_link_open = True
    # ---[ Rewrite code on SyncCrazyflie.open_link method ]---
    # scf._link_uri = drone_uri
    # scf._add_callbacks() 
    # print("Add callbacks ok")
    # scf._connect_event = Event()
    # print("connect_event ok")
    # scf._params_updated_event.clear()
    # print("Clear param ok")
    # #scf._connect_event.wait()
    # scf._connect_event.set()
    # print("wait done")
    # scf._connect_event = None

    with SyncCrazyflie(drone_uri, cf=cf_stats) as scf:

        print("Crazyflie - Wait for params..")
        scf.wait_for_params()
        #with SyncLogger(crazyflie=scf, log_config=log_radio):

        # ---[ Pre configuration ]---
        # Log configuration to logging framework
        scf.cf.log.add_config(log_radio)

        if log_radio.valid:
            scf.cf.link_statistics.link_quality_updated.add_callback(radio_stats_cb)
            log_radio.data_received_cb.add_callback(log_radio_cb)
            log_radio.error_cb.add_callback(log_error_cb)
        else:
            print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")
        
        #radio_stats = RadioLinkStatistics(radio_link_statistics_callback=log_stat_cb, alpha=0.1)
        def link_quality_cb(data):
            print(f"{data}")
        #scf.cf.link_statistics.link_quality_updated.add_callback(link_quality_cb)
        #scf.cf.link_statistics.stop()

        log_radio.start()

        while True:
            try:
                # If the RSSI > 70 or mean(RSSI) > 70 -> CLOSE LINK
                time.sleep(1)
            except KeyboardInterrupt:
                rssi_avg = np.mean(rssi_logs)
                rssi_std = np.std(rssi_logs)
                print("Average RSSI: ", rssi_avg, "\nStd RSSI: ", rssi_std)
                
                print("--- CLOSE LINK ---")
                scf.close_link()
                sys.exit()
        

    """
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(available, factory=factory) as swarm:
        print('[Swarm] - Connected to Crazyflies')
        print(f"[Radio] - Swarm: {list(swarm._cfs.keys())}")
    """

if __name__ == '__main__':
    main()