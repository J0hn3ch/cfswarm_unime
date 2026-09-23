"""
Test: Logging - stateEstimate, 
Date: 2026-09-22
Author: Gianluca Carbone

"""

# 0. IMPORTS
import cflib.drivers.crazyradio as crazyradio   # Communication medium device
import cflib.crtp                               # Communication protocol
from cflib.crazyflie import Crazyflie           # Drone class
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie # Drone class for synchronized tasks
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from dotenv import load_dotenv
import logging
import sys
import threading
from threading import Event
import time

# 1. ENVIROMENT VARIABLE
load_dotenv()  # reads variables from a .env file and sets them in os.environ
URIs = [] # URI to the Crazyflie to connect to
for d in range(1,4):
    URIs.append( uri_helper.uri_from_env(env=f'DRONE{str(d)}_URI', default='radio://0/80/2M/E7E7E7E7E7') )

logging.basicConfig(level=logging.ERROR)
deck_attached_event = Event()
crazyflie_ready = Event()

stateEstimate = [0.0, 0.0]
def log_pos_state_cb(timestamp, data, logconf):
    global stateEstimate
    print(f"{timestamp} - {data}")
    stateEstimate[0] = data['stateEstimate.x']
    stateEstimate[1] = data['stateEstimate.y']

def log_pos_state_conf():
    log_conf = LogConfig(name='State', period_in_ms=50)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    #log_conf.data_received_cb.add_callback(log_pos_state_cb)
    return log_conf

logconf = []
logconf.append( log_pos_state_conf() )

# CrazyRadio (Communication Medium) - Dongles currently connected to the USB Port of the computer
radio = crazyradio.Crazyradio()
print(f"|- Crazyradio 2.0 Version: {crazyradio.Crazyradio().version}")
print("\n|- Crazyradio Devices ---" + "-" * 10)
dongles = []
for d in crazyradio._find_devices():
    print(f"|\t- {d.manufacturer}, {d.product} (Serial number: {d.serial_number})")
    dongles.append(d)
print("|------------------------" + "-" * 10)

# Communication protocol driver - Initialize the low-level drivers
cflib.crtp.init_drivers(enable_serial_driver=False)

interfaces = cflib.crtp.get_interfaces_status()
print(f"|- Radio Interface: {interfaces['radio']}")

# CrazyRadio - Interface scanning
print(f"|- Wait for Crazyflie scanning...", end=' ')
drones = dict() # Dictionary: { 'drone_uri': driverClass_uri}

for uri in URIs:
    drone_addr = uri.split('/')[-1]
    available = cflib.crtp.scan_interfaces(address=int(drone_addr, 16))
    if available:
        drones[available[0][0]] = 1
drones_uri = list(drones.keys())
print("Crazyflie available: ", drones_uri)

drone_uri = drones_uri[0]
print("|- Link", drones[drone_uri])

cf_stats = Crazyflie(rw_cache='./cache')

if not drone_uri:
    print("No drone_uri")
    sys.exit()
    
print(f"|- Wait for Crazyflie Synchronization...", end=' ')
scf = SyncCrazyflie(drone_uri, cf=cf_stats)
scf.open_link()
print("Link opened")

time.sleep(3)  # give some time to establish connection; 3 sec seems to work
    
print("|- Crazyflie - Wait for params..", end=' ')
scf.wait_for_params()
print("Ok")

# ---[ Logs configuration ]---
print("|- 2. Log Configurations ", end=': ')
log_configs = logconf
print([log_conf.name for log_conf in log_configs])

# Log configuration to logging framework
for log_conf in log_configs:
    scf.cf.log.add_config(log_conf)

    if log_conf.valid:
        print(f"|- Loading {log_conf.name}")
        match(log_conf.name):
            case "State":
                log_conf.data_received_cb.add_callback(log_pos_state_cb)
    else:
        print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")

# ---[ Pre configuration ]---
# Pre checks on Crazyflie deck, sensors..

# Stabilizer - Estimator checks
def param_estimator(_, value_str):
    group, name = _.split(".")
    estimator_type = { 0:"Auto select", 1:"Complementary", 2:"Extended Kalman", 3:"Unscented Kalman" }
    value = int(value_str)
    print(f"|- [Parameter] {group} - {name}: \"{estimator_type[value]} ({value})\" is used!")

scf.cf.param.add_update_callback(group="stabilizer", name="estimator", cb=param_estimator)

# Stabilizer - Controller checks - https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/
def param_controller(_, value_str):
    group, name = _.split(".")
    controller_type = {0:"Auto select", 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"}
    value = int(value_str)
    print(f"|- [Parameter] {group} - {name}: \"({value}) - {controller_type[value]}\" is used!")

scf.cf.param.add_update_callback(group="stabilizer", name="controller", cb=param_controller)

time.sleep(1)

# Stabilizer Estimator selection : 0:"Auto select", 1:"Complementary", 2:"Extended Kalman", 3:"Unscented Kalman"
scf.cf.param.set_value('stabilizer.estimator', str(1))
# Stabilizer Controller selection : 0:"Auto select", 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"
scf.cf.param.set_value('stabilizer.controller', str(0))


scf.cf.param.get_value("hlCommander.vtoff", timeout=60)

# Reset estimator
reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate

for log_conf in log_configs:
    log_conf.start()

while True:
    time.sleep(0.1)