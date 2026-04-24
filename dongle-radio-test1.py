"""
==========================
CRAZYRADIO DONGLES
==========================
- Crazyradio 1, serial number: 57D5141CC2A4510C
- Crazyradio 2, serial number: 59C39A09484EB47A

"""
import cflib.crtp
from cflib.drivers import crazyradio
from cflib.utils import uri_helper

from dotenv import load_dotenv
import logging
import sys
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
logging.basicConfig(level=logging.DEBUG)
print("Python Version", sys.version)

print(f"Crazyradio 2.0 Version: {crazyradio.Crazyradio().version}")

# CrazyRadio devices currently connected to the computer
print("\n|- Crazyradio Devices ---" + "-" * 10)
dongles = []
for d in crazyradio._find_devices():
    print(f"|- \t{d.manufacturer}, {d.product} (Serial number: {d.serial_number})")
    dongles.append(d)
print("|------------------------" + "-" * 10)

def main():
    cr = crazyradio.Crazyradio()

    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_serial_driver=False)

    # Interface status
    interfaces = cflib.crtp.get_interfaces_status()
    print(f"Radio Interface: {interfaces['radio']}")

    # Crazyradio Interface scanning
    #available = cflib.crtp.scan_interfaces(address=0xE7E7E7E702)
    #print(available)

    def link_radio_stat_cb(data):
        pass
    
    def link_error_cb():
        raise Exception
        
    crazyflie = cflib.crtp.get_link_driver("radio://1/80/2M/E7E7E7E702", link_radio_stat_cb, link_error_cb)
    print(type(crazyflie))
    print(crazyflie.get_name())
    print(crazyflie.uri)
    

    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
