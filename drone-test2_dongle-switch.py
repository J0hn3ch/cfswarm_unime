"""
=======================
CRAZYFLIE DONGLE SWITCH
=======================


"""

# ------------------------------------
# IMPORTS
# ------------------------------------
import cflib.crtp
from cflib.drivers import crazyradio
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from custom_classes.LogConfigGen import LogConfigGen
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from dotenv import load_dotenv
import logging
from paths.p01_simple_takeoff import take_off_simple
import os
import sys
from threading import Event
import time
from itertools import compress

# ------------------------------------
# ENVIRONMENT VARIABLES
# ------------------------------------
load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(env='DRONE1_URI', default='radio://0/80/2M/E7E7E7E7E7')

# ------------------------------------
# LOGGING
# ------------------------------------
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
print("Python Version", sys.version)
supervisor_lookup = ["Can be armed, ", "Is armed, ", "Auto arm, ", "Can fly, ", "Is flying, ", "Is tumbled, ", "Is locked, ", "Is crashed, ", "High Level Control - Activated, ", "HL Trajectory Finish, ", "High Level Control - Disabled, "]

# Logging configuration initialization
def log_init():
    log_conf = LogConfig(name='Position', period_in_ms=200)
    """
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    """
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    return log_conf

def log_init2():
    log_conf = LogConfig(name='Supervisor', period_in_ms=500)
    log_conf.add_variable('supervisor.info', 'uint16_t')
    return log_conf

logconf = log_init()
log_supervisor = log_init2()
# ------------------------------------
# LOG GENERATOR
# ------------------------------------
def log_pos_cb2(timestamp, data, logconf):
    print("|-" + " POSITION LOG " + "-" * 20)
    for key, value in data.items():
         print(f"|\t{key}: {value:2.3f}",)
    print("-" + "--------------" + "-" * 20)
    print("\033[6A")

def log_error_cb(logconf, msg):
        print("Error when logging %s" % logconf.name)

def log_supervisor_cb(timetamp, data, logconf):
    print("-" + " SUPERVISOR LOG " + "-" * 20)
    for key, value in data.items():
        s = ""
        indexes = [int(x) for x in list('{0:0b}'.format(value))]
        indexes = list(map(bool,indexes))
        s = s.join( list(compress(supervisor_lookup, indexes)) )
        #for i in indexes:
        #    s +=  supervisor_lookup[i] + ", "
        print(f"{key}: {s}")
    print("-" + "--------------" + "-" * 20)


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
        print(f"[Console]: {text}", end='')
    
    scf.cf.console.receivedChar.add_callback(console_callback)

    # Flow deck checks callback
    def param_deck_flow(_, value_str):
        """The flow deck that you are using, should be correctly attached to the crazyflie. 
        If it is not, it will try to fly anyway without a good position estimate and for sure is going to crash. 
        """
        value = int(value_str)
        if value:
            deck_attached_event.set()
            print(f"[Deck]: {_} is attached!: {value}")
        else:
            print(f"[Deck]: {_} is NOT attached!: {value}")
    
    scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)

    #if not deck_attached_event.wait(timeout=10):
    #    print('No flow deck detected!')
    #    sys.exit(1)

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

def mission(scf, mission_id):
    print("-"*30)
    print("MISSION CONFIGURATION")
    print("-"*30)


    duration = 0
    print('The sequence is {:.1f} seconds long'.format(duration))
    return duration

def commander(scf):
    print("-" * 30)
    print("COMMANDER CONTROL")
    print("-" * 30)

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

    hl_commander.takeoff(0.3, 1.3, yaw=takeoff_yaw)
    print("|- [COMMANDER] - TAKEOFF!")
    time.sleep(3.0)
    print("|- -------- FINISH ------- ")

def main():
    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Log Crazyradio data

    # Crazyradio Interface scanning
    available = cflib.crtp.scan_interfaces(address=0xE7E7E7E701)
    for i in available:
        print("Found Crazyflie on URI [%s] with comment [%s]" % (i[0], i[1]) )
    print("")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global logconf

        # Pre checks on Crazyflie deck, sensors, 
        pre_checks(scf)
        time.sleep(1)

        # ---[ Pre configuration ]---
        # Log configuration to logging framework
        scf.cf.log.add_config(log_supervisor)

        if log_supervisor.valid:
            log_supervisor.data_received_cb.add_callback(log_supervisor_cb)
            log_supervisor.error_cb.add_callback(log_error_cb)
        else:
            print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")

        # Activate mellinger controller
        scf.cf.param.set_value('stabilizer.controller', '1')

        # Mission: Take off and hold in air crazyflie
        mission_id = 1
        duration = mission(scf, mission_id)

        # Reset estimator
        #reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate
        time.sleep(1)

        #logconf.start() # Start logging
        log_supervisor.start()
        
        # Commander
        commander(scf)
        time.sleep(3)
        print("--- END SYNC CRAZYFLIE ---")
        scf.close_link()
        print("--- CLOSE LINK ---")

    print("WAIT - 10 seconds")
    time.sleep(10)

    cf = Crazyflie()
    uri2 = "radio://1/80/2M/E7E7E7E701"
    cf.open_link(uri2)
    time.sleep(3)
    cf.high_level_commander.stop()
    print("CONNECTED WITH DONGLE 2 AND STOP")
    cf.close_link()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            scf.cf.commander.send_notify_setpoint_stop()
            scf.cf.commander.send_stop_setpoint()
            scf.cf.high_level_commander.stop()

if __name__ == '__main__':
    main()