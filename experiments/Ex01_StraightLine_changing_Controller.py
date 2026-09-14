'''
Experiment 01 : Perform a straight line fly using different Controllers available in the Crazyflie Firmware.
-------------
1. Select the controller
2. Take off the drone
3. Perform a straight line fly using the Motion Commander
4. Rotate yaw of 180 degree
5. Land and turn off the motors
6. Start again after changing the controller and reset the estimator.
'''

# ------------------------------------
# 1. IMPORTS
# ------------------------------------
import cflib.drivers.crazyradio as crazyradio   # Communication medium device
import cflib.crtp                               # Communication protocol
from cflib.crazyflie import Crazyflie           # Drone class
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie # Drone class for synchronized tasks
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from dotenv import load_dotenv
import sys
from threading import Event
import time

# --- Paths
sys.path.append('../cfswarm_unime') # setting path
from paths.p06_linear_motion import linear_motion
#from paths.p02_figure_8 import upload_trajectory

# ------------------------------------
# 1.1 ENVIRONMENT VARIABLES
# ------------------------------------
load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
URIs = []
for d in range(1,4):
    URIs.append( uri_helper.uri_from_env(env=f'DRONE{str(d)}_URI', default='radio://0/80/2M/E7E7E7E7E7') )

# ------------------------------------
# EVENTS
# ------------------------------------
deck_attached_event = Event()
crazyflie_ready = Event()

# ------------------------------------
# 2. DRONE CONFIGURATION AND CHECKS
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
# 2.1 MISSION CONFIGURATION
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

    hl_commander.takeoff(0.8, 2.0, yaw=takeoff_yaw)
    print("|- [COMMANDER] - TAKEOFF!")
    time.sleep(3.0)

    print("|- [COMMANDER] - FOLLOWING TRAJECTORY!")
    hl_commander.start_trajectory(trajectory_id, 1.0, relative_position=True, relative_yaw=relative_yaw)
    time.sleep(duration)

    # Rotate 180 degree
    
    # Land detection
    print("|- [COMMANDER] - LAND!")
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    scf.cf.commander.send_notify_setpoint_stop()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    hl_commander.land(0.0, 3.0, yaw=3.14159)
    
    time.sleep(2)
    hl_commander.stop()

    print("|- -------- FINISH ------- ")

def main():

    # CrazyRadio - Dongles currently connected to the USB Port of the computer
    radio = crazyradio.Crazyradio()
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

    cf_stats = Crazyflie(rw_cache='./cache')

    if not drone_uri:
        print("No drone_uri")
        sys.exit()

    with SyncCrazyflie(drone_uri, cf=cf_stats) as scf:

        print("Crazyflie - Wait for params..")
        scf.wait_for_params()

        # ---[ Pre configuration ]---
        # Pre checks on Crazyflie deck, sensors..
        pre_checks(scf)
        time.sleep(1)

        # ---[ Mission ]---
        trajectory_id = 1
        duration = mission(scf, trajectory_id)


        # Controller selection
        controllers = set(range(1,6)) # 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"
        while controllers:
            scf.cf.param.set_value('stabilizer.controller', str(controllers.pop()))
        
            # Reset estimator
            reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate
            time.sleep(1)

            # ---[ Commander ]---
            commander(scf, trajectory_id, duration)

        print("\nEND\n")

if __name__ == '__main__':
    main()