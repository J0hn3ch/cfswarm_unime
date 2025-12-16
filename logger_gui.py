import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from custom_classes.LogConfigGen import LogConfigGen
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from dotenv import load_dotenv
from functools import partial
from custom_classes.PlotApp import PlotApp
import itertools
import logging
import math
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
from threading import Event, Thread
import time

# Python
print("Python Version", sys.version)

# Matplotlib config - Backend
# https://github.com/matplotlib/mplcairo
matplotlib.use('TkCairo')
print("Matplotlib Version ", matplotlib.__version__)
print("Matplotlib Backend ", matplotlib.get_backend())

load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Events
deck_attached_event = Event()
crazyflie_ready = Event()

# Logging
logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework
                                         # logging.DEBUG 
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

position_estimate = [0, 0]
def log_gen():
    global position_estimate
    yield position_estimate

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.3

# Matplotlib animation callback
def init():
    ax.set_ylim(-1.1, 1.1)
    ax.set_xlim(0, 0.25)
    
    del xdata[:]
    del ydata[:]
    line.set_data(xdata, ydata)
    return line,

def data_picker():
    global position_estimate
    for cnt in itertools.count():
        t = cnt / 10
        yield t, position_estimate[1]

def run(data):

    # Plot update
    global xdata
    global ydata

    t, y = data
    xdata.append(t)
    ydata.append(y)

    global ax
    global line
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    if t >= xmax:
        ax.set_xlim(xmin, 2*xmax)
        ax.figure.canvas.draw()
    if y >= ymax or y <= ymin:
        ax.set_ylim(1.5*ymin, 1.5*ymax)
        ax.figure.canvas.draw()

    line.set_data(xdata, ydata)
    plt.pause(0.05)
    return line,


# Console callback
def console_callback(text: str):
    print(f"[Console]: {text}", end='')

# Logging callback
def log_pos_callback(timestamp, data, logconf):
    
    # replace the print function in the callback wit{h a plotter, python lib matplotlib
    #print(f"[{timestamp}][{logconf.name}]: {data}")
    
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

def logging_error(logconf, msg):
        print("Error when logging %s" % logconf.name)

# Parameters
# group = 'stabilizer'
# name = 'estimator'
def param_imu_sensors(_, value_str):
    value = int(value_str)
    if value:
        print(f"[IMU Sensors]: {_} is present: {value}")
    else:
        print(f"[IMU Sensors]: {_} is NOT present: {value}")

# Flow deck checks callback
def param_deck_flow(_, value_str):
    """ The flow deck that you are using, should be correctly attached to the crazyflie. 
        If it is not, it will try to fly anyway without a good position estimate and for sure is going to crash. """
    
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f"[Deck]: {_} is attached!: {value}")
    else:
        print(f"[Deck]: {_} is NOT attached!: {value}")

# Offboard control - Take off
def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.up(0.3)
        time.sleep(3)
        mc.stop()

# Offboard control - Move
def bounce_box_limit(scf):
    """Move the drone and bounce around in a virtual box of which the size is indicated by BOX_LIMIT"""
    
    body_x_cmd = 0.2
    body_y_cmd = 0.1
    max_vel = 0.2
    
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        while (1):
            if position_estimate[0] > BOX_LIMIT:
                 body_x_cmd=-max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd=max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd=-max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd=max_vel

            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)
            time.sleep(0.1)

def move_box_limit(scf):
    """Move the drone using nonblocking functions mc.start_forward and mc.start_back"""

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:

        while (1):
            if position_estimate[0] > BOX_LIMIT:
                mc.start_back()
            elif position_estimate[0] < -BOX_LIMIT:
                mc.start_forward()

def move_linear_simple(scf):
    """Move the drone using blocking functions mc.forward and mc.back"""
    # Blocking functions won't continue the code until the distance has been reached

    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.back(0.5)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(90)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)

# Offboard control - Figures
def figure_8(scf):

    cf = scf.cf

    for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)

    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        time.sleep(0.1)

    for _ in range(50):
        cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
        time.sleep(0.1)

    for _ in range(50):
        cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 0.4)
        time.sleep(0.1)

    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        time.sleep(0.1)

    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        time.sleep(0.1)
    
    cf.commander.send_stop_setpoint()
    
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

def update_signal(frame, signal):
    if frame:

        x = signal.get_xdata() # numpy.ndarray class
        y = signal.get_ydata()

        x = np.append(x, frame[1]['stateEstimate.x'])
        y = np.append(y, frame[1]['stateEstimate.y'])

        signal.set_xdata(x)
        signal.set_ydata(y)

        #ax.relim() # recompute the ax.dataLim
        #ax.autoscale()
        #ax.autoscale_view() # update ax.viewLim using the new dataLim
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()

        if x[-1] >= xmax:
            ax.set_xlim(xmin, 1.3*xmax)
            ax.figure.canvas.draw()

        if x[-1] <= xmin:
            ax.set_xlim(-1.3*xmin, xmax)
            ax.figure.canvas.draw()

        if y[-1] >= ymax:
            ax.set_ylim(ymin, 1.3*ymax)
            ax.figure.canvas.draw()
        
        if y[-1] <= ymin:
            ax.set_ylim(-1.3*ymin, ymax)
            ax.figure.canvas.draw()

        #plt.pause(0.005)
        return signal,

# Crazyflie Commander worker Thread ???

# Crazyflie Logging worker thread
def crazyflie_thread(uri):

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Crazyradio Interface scanning
    available = cflib.crtp.scan_interfaces(address=0xE7E7E7E7E7)
    for i in available:
        print("Found Crazyflie on URI [%s] with comment [%s]" % (i[0], i[1]) )

    global logconf

    # A Crazyflie instance is created and is now connected. If the connection fails, an exception is raised.
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        # The underlying crazyflie object can be accessed through the cf member
        # Security for flying
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        
        if not deck_attached_event.wait(timeout=10):
            print('No flow deck detected!')
            sys.exit(1)
        
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
        scf.cf.console.receivedChar.add_callback(console_callback)

        # Log configuration to logging framework
        scf.cf.log.add_config(logconf)

        if logconf.valid:
            logconf.data_received_cb.add_callback(log_pos_callback)
            logconf.error_cb.add_callback(logging_error)
            
            #logconf.start() # Start logging
        else:
            print("One or more of the variables in the configuration was not found in log TOC. No logging will be possible.")

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(do_arm=True)
        time.sleep(1.0)

        crazyflie_ready.set()

        global position_estimate

        #take_off_simple(scf)
        #move_linear_simple(scf)
        #move_box_limit(scf)
        #bounce_box_limit(scf)
        #figure_8(scf)

        #logconf.stop()
        while True:
            time.sleep(1)
            #print(f"[Script]: Position estimate ({position_estimate[0]}, {position_estimate[1]})")

# Main Thread
if __name__ == '__main__':
    
    position_estimate = [0, 0, 0]

    logconf = log_init()

    cf_thread = Thread(target=crazyflie_thread, args=(uri, ), daemon=True) # kwargs={"":None}
    cf_thread.start()

    if not crazyflie_ready.wait(timeout=15):
        print('[Script]: Timeout Crazyflie connection')
        sys.exit(1)

    root = PlotApp(logconf, position_estimate)

    root.mainloop()

    print("", end='\n')
    sys.exit(0)

    # Initialize Interactive mode of Matplotlib 
    plt.ion()  # Turn on interactive mode

    # Matplotlib config
    fig, ax = plt.subplots()
    ax.grid()
    ax.set(xlim=[0, 100], ylim=[-0.5, 0.5], xlabel='Time [ms]', ylabel='Y [m]')
    #ax.set(xlim=[-1.0, 1.0], ylim=[-1.0, 1.0], xlabel='X [m]', ylabel='Y [m]')
    line, = ax.plot(np.array([]), np.array([]), linewidth=1.5, marker='')
    # Only save last 100 frames, but run forever
    #ani = animation.FuncAnimation(fig, run, data_picker, interval=500, init_func=init, blit=True, save_count=100)
    #plt.show(block=False)

    #plt.autoscale(enable=True, axis='both', tight=None)

    # Visualize pre-build trajectory

    # A Crazyflie instance is created and is now connected. If the connection fails, an exception is raised.
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # OR comment logconf.start() and use SyncLogger
        with SyncLogger(scf, logconf) as logger:
            print("[Script]: Enter in SyncLogger")
            flag_exit = 0

            # Arm the Crazyflie
            scf.cf.platform.send_arming_request(do_arm=True)
            time.sleep(1.0)

            ani = animation.FuncAnimation(fig, func=partial(update_signal, signal=line), frames=logger, event_source=logconf, blit=True, save_count=100)
            
            plt.show(block=True)
            plt.pause(10)

            print("[Script]: After FuncAnimation")

        """
        # Take off when the commander is created
        with MotionCommander(scf) as mc:
            print("Taking off!")
            time.sleep(1)

            # We land when the MotionCommander goes out of scope
            print("Landing!")
        """
        
        #logconf.stop()
        print("", end='\n')