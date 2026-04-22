# ------------------------------------
# IMPORTS
# ------------------------------------
import cflib.crtp
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
import sys
from threading import Event
import time

# ------------------------------------
# ENVIRONMENT VARIABLES
# ------------------------------------
load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(env='DRONE2_URI', default='radio://0/80/2M/E7E7E7E7E7')

# ------------------------------------
# LOGGING
# ------------------------------------
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
print("Python Version", sys.version)

# Logging callback
def log_pos_callback(timestamp, data, logconf):
    
    # replace the print function in the callback wit{h a plotter, python lib matplotlib
    #print(f"[{timestamp}][{logconf.name}]: {data}")

    global se_t, se_x, se_y, se_z
    se_t.append(len(se_t) * logconf.period_in_ms)
    se_x.append(data['stateEstimate.x'])
    se_y.append(data['stateEstimate.y'])
    se_z.append(data['stateEstimate.z'])

def logging_error(logconf, msg):
        print("Error when logging %s" % logconf.name)

# Logging configuration initialization
def log_init():

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_pos = LogConfigGen(name='Position', period_in_ms=10)
    lg_pos.add_variable('stateEstimate.x', 'float')
    lg_pos.add_variable('stateEstimate.y', 'float')
    lg_pos.add_variable('stateEstimate.z', 'float')

    return lg_pos

logconf = log_init()

# ------------------------------------
# EVENTS
# ------------------------------------
deck_attached_event = Event()
crazyflie_ready = Event()

def pre_checks(scf):
    global logconf

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

    if not deck_attached_event.wait(timeout=10):
        print('No flow deck detected!')
        sys.exit(1)

    # IMU - Inertial Measurement Unit
    def param_imu_sensors(_, value_str):
        value = int(value_str)
        if value:
            print(f"[IMU Sensors]: {_} is present: {value}")
        else:
            print(f"[IMU Sensors]: {_} is NOT present: {value}")
    
    # Sensor checks
    scf.cf.param.add_update_callback(group="imu_sensors", name="BMP3XX", cb=param_imu_sensors)
    scf.cf.param.add_update_callback(group="imu_sensors", name="AK8963", cb=param_imu_sensors)
    scf.cf.param.add_update_callback(group="imu_sensors", name="LPS25H", cb=param_imu_sensors)

    # State checks
    scf.cf.param.set_value('supervisor.infdmp', '1') # When nonzero, dump information about the current supervisor state to the console log

    # Reset estimator
    reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate
    time.sleep(1)

    # Console configuration
    def console_callback(text: str):
        print(f"[Console]: {text}", end='')
    
    scf.cf.console.receivedChar.add_callback(console_callback)

    # Log configuration to logging framework
    scf.cf.log.add_config(logconf)

    def log_stab_callback(timestamp, data, logconf):
        print('[%d][%s]: %s' % (timestamp, logconf.name, data), end='\r')

    if logconf.valid:
        logconf.data_received_cb.add_callback(log_stab_callback)
        #logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.error_cb.add_callback(logging_error)
        
        logconf.start() # Start logging
    else:
        print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")



def main():

    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Crazyradio Interface scanning
    available = cflib.crtp.scan_interfaces(address=0xE7E7E7E701)
    for i in available:
        print("Found Crazyflie on URI [%s] with comment [%s]" % (i[0], i[1]) )

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        global logconf

        # Pre checks on Crazyflie deck, sensors, 
        pre_checks(scf)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(do_arm=True)
        time.sleep(1.0)

        crazyflie_ready.set()
        time.sleep(3.0)

        take_off_simple(scf)

        # Land detection

        while True:
            time.sleep(1)

if __name__ == '__main__':
    main()