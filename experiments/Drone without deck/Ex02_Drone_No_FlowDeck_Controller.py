'''
Experiment 02 : Try to fly offboard with a drone without FlowDeck and Multi-Ranger Deck. Using only IMU and Barometer sensors
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
import numpy as np
import sys
from threading import Event
import time

# --- Paths
sys.path.append('../../../cfswarm_unime') # setting path
#from paths.p06_linear_motion import linear_motion
#from paths.p02_figure_8 import upload_trajectory
from lib.pyusb.usb import core, util

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
# KEYBOARD CONFIGURATION
# ------------------------------------
USB_IF      = 0 # Interface
USB_TIMEOUT = 5 # Timeout in MS
USB_VENDOR  = 0x0513 # Rii
USB_PRODUCT = 0x0318 # Mini Wireless Keyboard

# Keyboard Mapping
ARROW_UP = 82
ARROW_DOWN = 81
ARROW_LEFT = 80 
ARROW_RIGHT = 79
LETTER_A = 4
LETTER_Z = 29
LETTER_P = 19
LETTER_O = 18

dev = core.find(idVendor=USB_VENDOR, idProduct=USB_PRODUCT)
if dev is None:
    raise ValueError('Device not found')

endpoint = dev[0][(0,0)][0]
if dev.is_kernel_driver_active(USB_IF) is True:
    dev.detach_kernel_driver(USB_IF)

util.claim_interface(dev, USB_IF)

"""
print("KEYBOARD PRINT")
while True:
    control = None
    try:
        control = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, USB_TIMEOUT)
        print(control)
    except KeyboardInterrupt: # Let CTRL+C actually exit
        print("EXIT")
        sys.exit()
    except:
        pass
    time.sleep(0.02)
"""

# ------------------------------------
# 2. DRONE CONFIGURATION AND CHECKS
# ------------------------------------
def pre_checks(scf):
    print("-"*30)
    print("PRE-FLIGHT CHECKS")
    print("-"*30)
    global logconf

    def information_callback(text: str):
        print(f"|- [Platform]: {text}", end='')
        
    scf.cf.platform.fetch_platform_informations(information_callback)

    print("CRTP Protocol Version: ", scf.cf.platform.get_protocol_version())

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
        print(f"|- [Parameter] {group} - {name}: \"{estimator_type[value]} ({value})\" is used!")
    
    scf.cf.param.add_update_callback(group="stabilizer", name="estimator", cb=param_estimator)

    # Stabilizer - Controller checks - https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/
    def param_controller(_, value_str):
        group, name = _.split(".")
        controller_type = {0:"Auto select", 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"}
        value = int(value_str)
        print(f"|- [Parameter] {group} - {name}: \"({value}) - {controller_type[value]}\" is used!")
        
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
def commander(scf, trajectory_id=None, duration=None):
    print("-"*30 + "\n COMMANDER CONTROL \n" + "-"*30)
    global dev
    global endpoint

    relative_yaw=False

    # High Level Commander
    hl_commander = scf.cf.high_level_commander
    # Commander
    commander = scf.cf.commander
    commander.set_client_xmode(enabled=True)

    # Supervisor
    sup = scf.cf.supervisor
    
    # Arm the Crazyflie
    #scf.cf.supervisor.send_arming_request(True)
    #scf.cf.platform.send_arming_request(do_arm=True)
    #time.sleep(1.0)

    takeoff_yaw = 3.14 / 2 if relative_yaw else 0.0

    # External Position Sender
    position = scf.cf.extpos
    recovery = False
    print("READ BITFIELD", sup.read_bitfield())
    print("READ BITFIELD", sup.read_state_list())

    """
    print("Bit 0:", sup.decode_bitfield(value=0))
    print("Bit 1:", sup.decode_bitfield(value=1))
    print("Bit 2:", sup.decode_bitfield(value=2))
    print("Bit 3:", sup.decode_bitfield(value=3))
    print("Bit 4:", sup.decode_bitfield(value=4))
    print("Bit 5:", sup.decode_bitfield(value=5))
    print("Bit 6:", sup.decode_bitfield(value=6))
    print("Bit 7:", sup.decode_bitfield(value=7))
    print("Bit 8:", sup.decode_bitfield(value=8))
    print("Bit 9:", sup.decode_bitfield(value=9))
    print("Bit 10:", sup.decode_bitfield(value=10))
    """

    # Arm the Crazyflie
    while not ('Can be armed' in sup.read_state_list()): # Can be armed ?
        if 'Is armed' in sup.read_state_list():
            break
        time.sleep(0.3)
        print("a")
        continue

    sup.send_arming_request(do_arm=True) # ARM
    print("|- [Commander] - Drone ARMED!")

    # Check Arm
    while not ('Is armed' in sup.read_state_list()): # Is armed ?
        time.sleep(0.3)
        print("b")
        continue

    crazyflie_ready.set()
    print("|- [Commander] - Drone IS ARMED!")
    
    while not ('Can fly' in sup.read_state_list()): # Can fly ?
        time.sleep(0.3)
        print("c")
        continue

    roll = 0
    pitch = 0
    yaw = 0
    thrust = 20 
    # min = 10001 , max = 60000 
    #print(f"Is Flying: {bool(sup.decode_bitfield(4))}", end="")

    print("Can fly: ", bool('Can fly' in sup.read_state_list()))
    print("|- [Commander] - Drone CAN FLY!")

    while 'Can fly' in sup.read_state_list(): # Is flying ?
        control = None
        try:
            control = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, USB_TIMEOUT)
            
            #commander.send_setpoint_manual(0, 0, 0, 20.0, True)
            #commander.send_velocity_world_setpoint(0.0, 0.0, 0.2, 0)

            match control[2]:
                case 82:
                    pitch = 25
                    commander.send_setpoint_manual(0, pitch, 0, thrust, rate=False)
                case 81:
                    pitch = -25
                    commander.send_setpoint_manual(0, pitch, 0, thrust, rate=False)
                case 80:
                    roll = -25
                    commander.send_setpoint_manual(roll, 0, 0, thrust, rate=False)
                case 79:
                    roll = 25
                    commander.send_setpoint_manual(roll, 0, 0, thrust, rate=False)
                case 4:
                    print("THRUST INCREASE") # thrust + 1
                    thrust += 2.5
                case 29:
                    print("THRUST DECREASE") # thrust - 1
                    thrust -= 2.5
                case 16:
                    print("STOP WATCHDOG")
                    sup.send_emergency_stop_watchdog()           
                case 17:
                    print("CRASH RECOVERY REQUEST")
                    sup.send_crash_recovery_request() # Use Ctrl + R to send a "Crash Recovery" request
                case 18:
                    yaw -= 20
                    #commander.send_setpoint_manual(0, 0, yaw, thrust, rate=False)
                case 19:
                    yaw += 20
                    #commander.send_setpoint_manual(0, 0, yaw, thrust, rate=False)
                case None:
                    raise Exception
                case _: # wildcard - simile ad un else, deve stare alla fine
                    print("Roll: ", roll, "Pitch: ", pitch, "Yaw: ", yaw, "Thrust: ", thrust)
                    roll = 0
                    pitch = 0

            # Position
            #commander.send_setpoint(0, 0, 0, thrust) # roll, pitch, yawrate, thrust
            #commander.send_position_setpoint(0.0, 0.0, 0.2, 0)
            #commander.send_velocity_world_setpoint(0.0, 0.0, 0.2, 0)

            # Distance
            # - Only with Flow Deck for Z distance from the ground.
            #commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.1) # vx, vy, yawrate, zdistance

            # Land
            #commander.send_notify_setpoint_stop(remain_valid_milliseconds=0)
            #print(".", end="")
        
        except KeyboardInterrupt: # Let CTRL+C actually exit
            sup.send_emergency_stop_watchdog()
            sup.send_emergency_stop() # Emergency Stop Command
            print("|- [Commander] - Emergency Stop Command sent!")
            break

        except:
            #commander.send_setpoint(0, 0, 0, int(0.5 * 1000))
            commander.send_setpoint_manual(0, 0, 0, thrust, rate=False)
            #commander.send_setpoint_manual(0, 0, yaw, thrust, rate=False) # roll, pitch, yawrate, thrust_percentage, rate
            pass

        sup.send_emergency_stop_watchdog()
        time.sleep(0.05)

    print("Is flying: ", bool('Is flying' in sup.read_state_list()))
    if bool('Is flying' in sup.read_state_list()):
        print("|- [Commander] - Drone TAKEOFF!")
        print("|- [Commander] - Drone IS FLYING!")
    
    while 'Is flying' in sup.read_state_list():
        #position = [],
        #velocity = [],
        #acceleration = []
        #commander.send_full_state_setpoint() # pos, vel, acc, orientation, rollrate, pitchrate, yawrate
        
        control = None
        try:
            control = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, USB_TIMEOUT)
            
            #commander.send_setpoint(0, 0, 0, thrust) # roll, pitch, yawrate, thrust
            #commander.send_setpoint_manual(0, 0, 0, 20.0, True)
            #commander.send_velocity_world_setpoint(0.0, 0.0, 0.2, 0)
            
            #sup.send_emergency_stop_watchdog()

            match control[2]:
                case 82:
                    print("Bit 0:", sup.decode_bitfield(value=0))
                    pitch = 10
                    commander.send_setpoint_manual(0, pitch, 0, thrust, rate=False)
                case 81:
                    pitch = -10
                    commander.send_setpoint_manual(0, pitch, 0, thrust, rate=False)
                case 80:
                    roll = -10
                    commander.send_setpoint_manual(roll, 0, 0, thrust, rate=False)
                case 79:
                    roll = 20
                    commander.send_setpoint_manual(roll, 0, 0, thrust, rate=False)
                case 4:
                    print("LETTER A") # thrust + 1
                    thrust += 2.0
                case 29:
                    print("LETTER Z") # thrust - 1
                    thrust -= 2.0
                case 17:
                    sup.send_crash_recovery_request()
                case 18:
                    yaw -= 10
                    commander.send_setpoint_manual(0, 0, yaw, thrust, rate=False)
                case 19:
                    yaw += 10
                    commander.send_setpoint_manual(0, 0, yaw, thrust, True)
                case None:
                    raise Exception
                case _: # wildcard - simile ad un else, deve stare alla fine
                    print("Sent:", thrust)
                
                    #commander.send_setpoint(0, 0, 0, thurst)

            # Position
            #commander.send_position_setpoint(0.0, 0.0, 0.2, 0)

            # Distance
            # - Only with Flow Deck for Z distance from the ground.
            #commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.1) # vx, vy, yawrate, zdistance

            # Land
            #commander.send_notify_setpoint_stop(remain_valid_milliseconds=0)
            commander.send_setpoint_manual(0, 0, 0, thrust, True) # roll, pitch, yawrate, thrust_percentage, rate
            time.sleep(0.01)

            if sup.decode_bitfield(7):
                # Use Ctrl + R to send a "Crash Recovery" request
                sup.send_crash_recovery_request()

            #print(".", end="")
        
        except KeyboardInterrupt: # Let CTRL+C actually exit
            sup.send_emergency_stop_watchdog()
            sup.send_emergency_stop() # Emergency Stop Command
            print("|- [Commander] - Emergency Stop Command sent!")
            break
        except:
            pass

    print("|- [Commander] - Drone OUT WHILE!")

    #hl_commander.takeoff(0.8, 2.0, yaw=takeoff_yaw)
    #print("|- [COMMANDER] - TAKEOFF!")
    #time.sleep(3.0)

    #print("|- [COMMANDER] - INPUT")
    #d = {}
    #hl_commander.start_trajectory(trajectory_id, 1.0, relative_position=True, relative_yaw=relative_yaw)
    #time.sleep(20)

    # Land detection
    #print("|- [COMMANDER] - LAND!")
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    #scf.cf.commander.send_notify_setpoint_stop()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    #hl_commander.land(0.0, 3.0, yaw=3.14159)
    
    #time.sleep(2)
    #hl_commander.stop()

    print("-"*30 + "\n COMMANDER - END \n" + "-"*30 + "\n")

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
        #trajectory_id = 1
        #duration = mission(scf, trajectory_id)

        # Stabilizer Estimator selection : 0:"Auto select", 1:"Complementary", 2:"Extended Kalman", 3:"Unscented Kalman"
        scf.cf.param.set_value('stabilizer.estimator', str(0))
        # Stabilizer Controller selection : 1:"PID", 2:"Mellinger", 3:"INDI", 4:"Brescianini", 5:"Lee"
        scf.cf.param.set_value('stabilizer.controller', str(1))
        

        scf.cf.param.get_value("hlCommander.vtoff", timeout=60)
        
        # Reset estimator
        reset_estimator(scf) # resets the Kalman filter and makes the Crazyflie wait until it has an accurate position estimate
        time.sleep(1)

        # ---[ Commander ]---
        commander(scf)

        print("|- -------- Main - END ------- ")

if __name__ == '__main__':
    main()