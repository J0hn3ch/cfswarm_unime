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

# ------------------------------------
# IMPORTS
# ------------------------------------
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
from cflib.utils.reset_estimator import reset_estimator

from dotenv import load_dotenv
import logging
import numpy as np
import sys
from threading import Event
import time

# --- Paths
from paths.p06_linear_motion import linear_motion

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
logging.basicConfig(level=logging.ERROR)
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

def log_pos_conf():
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    return log_conf

log_pos = log_pos_conf()

# ------------------------------------
# LOGS CALLBACKS
# ------------------------------------
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

# ------------------------------------
# EVENTS
# ------------------------------------
deck_attached_event = Event()
crazyflie_ready = Event()


# ------------------------------------
# CONFIGURATION
# ------------------------------------
def pre_checks(scf):
    print("-"*30)
    print("PRE-FLIGHT CHECKS")
    print("-"*30)
    global logconf

    # Console configuration
    def console_callback(text: str):
        print(f"|- [Console]: {text}", end='')
    
    scf.cf.console.receivedChar.add_callback(console_callback)

    # Flow deck checks callback
    def param_deck_flow(_, value_str):
        global deck_attached_event
        """The flow deck that you are using, should be correctly attached to the crazyflie. 
        If it is not, it will try to fly anyway without a good position estimate and for sure is going to crash. 
        """
        value = int(value_str)
        if value:
            deck_attached_event.set()
            print(f"|- [Deck]: {_} is attached!: {value}")
        else:
            print(f"|- [Deck]: {_} is NOT attached!: {value}")
    
    scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)

    # if not deck_attached_event.wait(timeout=20):
    #     print('No flow deck detected!')
    #     sys.exit(1)

    # Sensor checks
    # IMU - Inertial Measurement Unit
    def param_imu_sensors(_, value_str):
        group, name = _.split(".")
        sensors = {"BMP3XX":"Barometer", "AK8963":"Magnetometer", "LPS25H":"Barometer"}
        value = int(value_str)
        if value:
            print(f"[IMU Sensors]: {sensors[name]} {name} is present: {value}")
        else:
            print(f"[IMU Sensors]: {sensors[name]} {name} is NOT present: {value}")
    
    scf.cf.param.add_update_callback(group="imu_sensors", name="BMP3XX", cb=param_imu_sensors)
    scf.cf.param.add_update_callback(group="imu_sensors", name="AK8963", cb=param_imu_sensors)
    scf.cf.param.add_update_callback(group="imu_sensors", name="LPS25H", cb=param_imu_sensors)

    # Stabilizer - Estimator checks
    def param_estimator(_, value_str):
        group, name = _.split(".")
        estimator_type = { 0:"Auto select", 1:"Complementary", 2:"Extended Kalman", 3:"Unscented Kalman" }
        value = int(value_str)
        print(f"[{group}]: {name} \"{estimator_type[value]} ({value})\" is used!")
    
    scf.cf.param.add_update_callback(group="stabilizer", name="estimator", cb=param_estimator)

    # Stabilizer - Controller checks - https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/
    def param_controller(_, value_str):
        group, name = _.split(".")
        controller_type = {0:"Auto select", 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"}
        value = int(value_str)
        print(f"[{group}]: {name} \"{controller_type[value]} ({value})\" is used!")
        
    scf.cf.param.add_update_callback(group="stabilizer", name="controller", cb=param_controller)

    # State checks
    scf.cf.param.set_value('supervisor.infdmp', '1') # When nonzero, dump information about the current supervisor state to the console log

# ------------------------------------
# MISSION
# ------------------------------------
def mission(scf, trajectory_id):
    print("-"*30 + "\n MISSION CONFIGURATION \n" + "-"*30)
    duration = linear_motion(scf.cf, trajectory_id)
    print('The sequence is {:.1f} seconds long'.format(duration))
    return duration

# ------------------------------------
# COMMANDER
# ------------------------------------
def commander(scf, trajectory_id, duration):
    print("-"*30 + "\n COMMANDER CONTROL \n" + "-"*30)

    relative_yaw=False
    hl_commander = scf.cf.high_level_commander

    # Arm the Crazyflie
    #scf.cf.supervisor.send_arming_request(True)
    scf.cf.platform.send_arming_request(do_arm=True)
    print("|- [COMMANDER] - DRONE ARMED!")
    time.sleep(1.0)

    crazyflie_ready.set()
    time.sleep(3.0)

    takeoff_yaw = 3.14 / 2 if relative_yaw else 0.0

    hl_commander.takeoff(1.0, 2.0, yaw=takeoff_yaw)
    print("|- [COMMANDER] - TAKEOFF!")
    time.sleep(3.0)

    print("|- [COMMANDER] - FOLLOWING TRAJECTORY!")
    start_time = time.time()
    hl_commander.start_trajectory(trajectory_id, 1.0, relative_position=True, relative_yaw=relative_yaw)

    elapsed_time = time.time() - start_time
    while (elapsed_time < duration + 5):
        elapsed_time = time.time() - start_time
        rssi_avg = np.mean(rssi_logs)
        rssi_std = np.std(rssi_logs)
        
        # If the RSSI > 70 or mean(RSSI) > 70 -> CLOSE LINK
        if rssi_avg > 60:
            print("DRONE TOO FAR")
            break

    print("|- Real elapsed time: ", elapsed_time)
    
    # Land detection
    print("|- [COMMANDER] - LAND!")
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    scf.cf.commander.send_notify_setpoint_stop()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    hl_commander.land(0.0, 2.0)
    
    time.sleep(2)
    hl_commander.stop()

    print("|- -------- FINISH ------- ")

def main():
    radio = crazyradio.Crazyradio()

    # CrazyRadio devices currently connected to the computer
    print("\n|- Crazyradio Devices ---" + "-" * 10)
    dongles = []
    for d in crazyradio._find_devices():
        print(f"|- \t{d.manufacturer}, {d.product} (Serial number: {d.serial_number})")
        dongles.append(d)
    print("|------------------------" + "-" * 10)

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

    if not drone_uri:
        sys.exit()

    with SyncCrazyflie(drone_uri, cf=cf_stats) as scf:

        print("Crazyflie - Wait for params..")
        scf.wait_for_params()
        #with SyncLogger(crazyflie=scf, log_config=log_radio):

        # ---[ Pre configuration ]---
        # Pre checks on Crazyflie deck, sensors, 
        pre_checks(scf)
        time.sleep(1)

        # Pre configuration
        # Activate mellinger controller
        scf.cf.param.set_value('stabilizer.controller', '2')

        # Mission
        trajectory_id = 1
        duration = mission(scf, trajectory_id)

        # Reset estimator
        reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate
        time.sleep(1)

        # ---[ Logging configuration ]---
        # Log configuration to logging framework
        scf.cf.log.add_config(log_radio)

        if log_radio.valid:
            scf.cf.link_statistics.link_quality_updated.add_callback(radio_stats_cb)
            log_radio.data_received_cb.add_callback(log_radio_cb)
            log_radio.error_cb.add_callback(log_error_cb)
        else:
            print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")
        
        #scf.cf.link_statistics.link_quality_updated.add_callback(link_quality_cb)
        #scf.cf.link_statistics.stop()

        log_radio.start()

        # ---[ Commander ]---
        commander(scf, trajectory_id, duration)


        while True:
            try:
                rssi_avg = np.mean(rssi_logs)
                rssi_std = np.std(rssi_logs)
                # If the RSSI > 70 or mean(RSSI) > 70 -> CLOSE LINK
                if rssi_avg > 43:
                    #scf.cf.commander.send_notify_setpoint_stop()
                    #scf.cf.high_level_commander.land(0.0, 2.0)
                    #time.sleep(2)
                    #scf.cf.high_level_commander.stop()
                    raise KeyboardInterrupt
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