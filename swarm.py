"""
- https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_swarm_interface/

CachedCfFactory : To reduce connection time, the factory is chosen to be instance 
of the CachedCfFactory class that will cache the Crazyflie objects in the ./cache directory.
"""

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

from dotenv import load_dotenv
import logging
import sys
from threading import Event
import time

# Python
print("Python Version", sys.version)

# URIs in a swarm using the same radio must also be on the same channel.
uris = {
    'radio://0/80/2M/E7E7E7E701',
    'radio://0/80/2M/E7E7E7E702',
    #'radio://0/80/2M/E7E7E7E703'
    # Add more URIs if you want more copters in the swarm
}

uris = {'radio://0/80/2M/E7E7E7E701','radio://0/80/2M/E7E7E7E702'}

# Logging
logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework
                                         # logging.DEBUG, logging.INFO

def log_battery_cb(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data)) 

# Events
deck_attached_event = Event()#[Event()] * len(uris)

# Parameters
def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

# - Flow deck checks callback
def param_deck_flow(_, value_str):
    """ The flow deck that you are using, should be correctly attached to the crazyflie. 
        If it is not, it will try to fly anyway without a good position estimate and for sure is going to crash. """

    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f"[Deck]: {_} is attached!: {value}")
    else:
        print(f"[Deck]: {_} is NOT attached!: {value}")

# Console callback
def console_callback(text: str):
    print(f"[Console]: {text}", end='')

# User Pre-Flight checks - Light checks
def activate_led_bit_mask(scf):
    """
    -https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params/#led
    """
    scf.cf.param.set_value('led.bitmask', 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value('led.bitmask', 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)

def deck_check(scf, bcFlow2_event):
    
    def deck_flow_check(_, value_str):
        nonlocal bcFlow2_event
        value = int(value_str)
        if value:
            bcFlow2_event.set()
            print(f"[Deck]: {_} is attached!: {value}")
        else:
            print(f"[Deck]: {_} is NOT attached!: {value}")

    scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=deck_flow_check)
    print("Deck check [", scf.cf.link_uri, "]")
    bcFlow2_event.wait(timeout=5)

# Offboard control
"""
High Level Commander is usually preferred since it needs less communication 
and provides more autonomy for the Crazyflie
"""
def take_off(scf, bcFlow2_event):

    if not bcFlow2_event.wait(timeout=10):
        print('[No Deck code]')

        scf.cf.platform.send_arming_request(do_arm=True)
        time.sleep(1)
        
        commander = scf.cf.commander
        commander.set_client_xmode(enabled=True)

        commander.send_setpoint_manual(0,0,0, thrust_percentage=float(30), rate=False)
        time.sleep(4)
        #commander.send_stop_setpoint()
    else:
        print('[Deck code]')
        try:
            commander = scf.cf.high_level_commander
            commander.takeoff(1.0, 4.0)
            time.sleep(3)
        except Exception as e:
            print(e)

    time.sleep(3)

def land(scf):
    commander = scf.cf.high_level_commander
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def hover_sequence(scf):
    # 1. Identify the drone
    print(f"\n[DroneInfo] - Link: {scf.cf.link_uri}")
    print("-"*46)

    # 2. Log the drone state
    # 3. Log the drone battery
    #scf.cf.log.get_value('pm.vbat')
    #scf.cf.log.get_value('pm.batteryLevel')

    # Deck check
    bcFlow2_event = Event()
    def deck_flow_check(_, value_str):
        nonlocal bcFlow2_event
        print("IM HERE")
        value = int(value_str)
        if value:
            bcFlow2_event.set()
            print(f"[Deck]: {_} is attached!: {value}")
        else:
            print(f"[Deck]: {_} is NOT attached!: {value}")
    
    # Deck check - Security for flying
    scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=deck_flow_check)
    bcFlow2_event.wait(timeout=10)
    deck = scf.cf.param.get_value('deck.bcFlow2')
    print("DECK: ", deck)

    # State checks
    scf.cf.param.set_value('supervisor.infdmp', '1')

    # Preflight Timeout
    #scf.cf.param.set_value('supervisor.prefltTimeout', '1')
    
    # Console configuration
    scf.cf.console.receivedChar.add_callback(console_callback)

    take_off(scf, bcFlow2_event)
    #land(scf)

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Crazyradio Interface scanning
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces(address=0xE7E7E7E701)
    print('Crazyflies URI found:')
    for i in available:
        print("\t - %s with comment [%s]" % (i[0], i[1]) )

    # Logs
    #logconf = LogConfig(name='Swarm',)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        
        print('[Swarm] - Connected to Crazyflies')

        """ Parameter description:
         * Crazyflie URI : [ 'bcFlow2 Event', ]
        """
        cf_args = {
            'radio://0/80/2M/E7E7E7E701': [Event()],
            'radio://0/80/2M/E7E7E7E702': [Event()]
        }

        print(f"[Deck check] - Swarm: {list(swarm._cfs.keys())}")
        swarm.sequential(deck_check, args_dict=cf_args)
        time.sleep(3)

        swarm.parallel(wait_for_param_download)

        # Execute the light check for each copter
        print(f"[Light checks] - Swarm: {list(swarm._cfs.keys())}")
        swarm.parallel_safe(light_check)

        for key in cf_args:
            print(key, "has deck:", cf_args[key][0])


        """
        1. Don't worry about threads since they are handled internally.
        2. Each Crazyflie is treated as a SyncCrazyflie instance.
        3. One thread per Crazyflie is started to execute the function.
        4. The threads are joined at the end.
        """
        """
        # Security for flying
        for scf in swarm._cfs.values():
            scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        """

        # Resetting the internal position estimator until the variance of the position estimation drops below a certain threshold.
        #swarm.reset_estimators()

        # Only one copter at a time executing the hover_sequence
        #swarm.sequential(hover_sequence)

        #swarm.parallel_safe(hover_sequence)