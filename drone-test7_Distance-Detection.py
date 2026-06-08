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
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.utils.multiranger import Multiranger

from dotenv import load_dotenv
import logging
import numpy as np
import sys
from threading import Event
import time

# --- Paths
from paths.p06_linear_motion import linear_motion
from paths.p02_figure_8 import upload_trajectory

# ------------------------------------
# ENVIRONMENT VARIABLES
# ------------------------------------
load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
URIs = []
for d in range(1,4):
    URIs.append( uri_helper.uri_from_env(env=f'DRONE{str(d)}_URI', default='radio://0/80/2M/E7E7E7E7E7') )

# ------------------------------------
# LOGGING
# ------------------------------------
logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework
print("Python Version", sys.version)
print(f"Crazyradio 2.0 Version: {crazyradio.Crazyradio().version}")

# ----- Drone Logs -----
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

# ----- Drone Logs for CNP -----
pm_state_lkp = { 0:"Battery", 1:"Charging", 2:"Charged", 3:"Low power", 4:"Shutdown" }
def log_bid_conf():
    log_conf = LogConfig(name="Bid", period_in_ms=500)
    log_conf.add_variable('pm.batteryLevel', 'uint8_t')
    log_conf.add_variable('pm.state', 'uint8_t')

    log_conf.add_variable('radio.rssi', 'uint8_t') # Radio Signal Strength Indicator [dBm]
    log_conf.add_variable('supervisor.info', 'uint16_t')
    return log_conf

log_pos = log_bid_conf()

# ------------------------------------
# EVENTS
# ------------------------------------
deck_attached_event = Event()
crazyflie_ready = Event()

# User Pre-Flight checks - Light checks
def activate_led_bit_mask(scf):
    """
    -https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params/#led
    """
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

# 1 - Pre-Checks
# ------------------------------------
# CONFIGURATION
# ------------------------------------
def pre_checks(scf, params_event):
    print("-"*30)
    print(f"PRE-FLIGHT CHECKS: {scf.cf.link_uri}")
    print("-"*30)
    global logconf

    # Console configuration
    def console_callback(text: str):
        print(f"|- [Console]: {text}", end='')
    scf.cf.console.receivedChar.add_callback(console_callback)

    # Flow deck checks callback
    def param_deck_flow(_, value_str):
        nonlocal params_event
        #global deck_attached_event
        """The flow deck that you are using, should be correctly attached to the crazyflie. 
        If it is not, it will try to fly anyway without a good position estimate and for sure is going to crash. 
        """
        value = int(value_str)
        if value:
            params_event.set()
            print(f"|- [Deck]: {_} is attached!: {value}")
        else:
            print(f"|- [Deck]: {_} is NOT attached!: {value}")
    
    print("Flow Deck check [", scf.cf.link_uri, "]")
    scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
    scf.cf.param.request_param_update('deck.bcFlow2')

    if not params_event.wait(timeout=5):
        print(params_event)
        print('No flow deck detected!')
        sys.exit(1)
    else:
        print(params_event)
        print("OK")

    # Multi-Ranger deck checks callback
    scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_deck_flow)
    scf.cf.param.request_param_update('deck.bcMultiranger')
    print("Multi-Ranger Deck check [", scf.cf.link_uri, "]")

    if not params_event.wait(timeout=5):
        print(params_event)
        print('No Multi-Ranger deck detected!')
        sys.exit(1)
    else:
        print(params_event)

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


# 2 - Pre-Configuration: Multi-Ranger Distance, Flow Deck Height
def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE
    
# 3 - Exploration: Boundary discovery, High Ground mapping, Blind alley detection
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
def commander(scf):

    sequence = [
        (0.0, 0.0, 0.4, 0),
        (0.2, 0.1, 0.0, 30),
        (0.2, 0.2, 0.0, 60),
        (0.1, 0.2, 0.0, 90),
        (0.1, 0.1, 0.0, 120),
        #(0.0, 0.0, 0.0, 0),
    ]
    
    with MotionCommander(scf.cf, default_height=0.3) as mc:
        with Multiranger(scf) as multi_ranger:
            keep_flying = True
            while keep_flying:
                VELOCITY = 0.5
                velocity_x = 0.0
                velocity_y = 0.0

                if is_close(multi_ranger.front):
                    velocity_x -= VELOCITY
                if is_close(multi_ranger.back):
                    velocity_x += VELOCITY

                if is_close(multi_ranger.left):
                    velocity_y -= VELOCITY
                if is_close(multi_ranger.right):
                    velocity_y += VELOCITY

                if is_close(multi_ranger.up):
                    keep_flying = False

                mc.start_linear_motion(
                    velocity_x_m=velocity_x, 
                    velocity_y_m=velocity_y, 
                    velocity_z_m=0,
                    rate_yaw=0
                )
                time.sleep(0.1)
        
        #print("|- [MOTION COMMANDER] - TAKEOFF!")
        #mc.take_off(height=None, velocity=0.2)
        #time.sleep(2)

        #print("|- [MOTION COMMANDER] - MOVE DISTANCE!")
        #mc.move_distance(distance_x_m=0.5, distance_y_m=0.0, distance_z_m=0.3, velocity=0.2)
        #mc.start_linear_motion(velocity_x_m=0.5, velocity_y_m=0.0, velocity_z_m=0.3, rate_yaw=0.0)
        """
        for position in sequence:
            mc.start_linear_motion(
                velocity_x_m=position[0], 
                velocity_y_m=position[1], 
                velocity_z_m=position[2], 
                rate_yaw=position[3]
            )
            time.sleep(0.5)
        time.sleep(2)
        """

        #print("|- [MOTION COMMANDER] - LAND!")
        #mc.land(velocity=0.2)
        #time.sleep(2)

def main():
    radio = crazyradio.Crazyradio()

    # CrazyRadio devices currently connected to the computer
    print("\n|- Crazyradio Devices ---" + "-" * 10)
    dongles = []
    for d in crazyradio._find_devices():
        print(f"|\t- {d.manufacturer}, {d.product} (Serial number: {d.serial_number})")
        dongles.append(d)
    print("|------------------------" + "-" * 10)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_serial_driver=False)

    # Interface status
    interfaces = cflib.crtp.get_interfaces_status()
    print(f"|- Radio Interface: {interfaces['radio']}")

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

    print("|------------------------" + "-" * 10)
    print("|   Crazyflies Swarm   ")
    print("|------------------------" + "-" * 10)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(drones_uri, factory=factory) as swarm:
        print(f"|- [Radio] - Swarm: {list(swarm._cfs.keys())}")

        cf_args = {}
        for uri in drones_uri:
            cf_args[uri] = [Event()]
        print(f"[Pre-checks] - Swarm: {list(swarm._cfs.keys())}")
        swarm.sequential(pre_checks, args_dict=cf_args)
        time.sleep(3)

        # Parameters
        def wait_for_param_download(scf):
            while not scf.cf.param.is_updated:
                time.sleep(1.0)
            print('|- Parameters downloaded for', scf.cf.link_uri)

        swarm.parallel(wait_for_param_download)

        # Execute the light check for each copter
        def light_check(scf):
            activate_led_bit_mask(scf)
            time.sleep(2)
            deactivate_led_bit_mask(scf)
        print(f"|- [Light checks] - Swarm: {list(swarm._cfs.keys())}")
        swarm.parallel_safe(light_check)

        # Activate mellinger controller
        for scf in swarm._cfs.values():
            scf.cf.param.set_value('stabilizer.controller', '1')
            
            # Mission
            trajectory_id = 1
            #duration = mission(scf, trajectory_id)

        # Resetting the internal position estimator until the variance of the position estimation drops below a certain threshold.
        swarm.reset_estimators()
        time.sleep(1)

        swarm_commander_args = {}
        for uri in drones_uri:
            swarm_commander_args[uri] = [trajectory_id, duration]
        # swarm.parallel_safe(commander, args_dict=swarm_commander_args)
        swarm.parallel_safe(commander)

        swarm.close_links()
    

# 4 - Handle Support requests

if __name__ == '__main__':
    main()