import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from dotenv import load_dotenv
import logging
import time

load_dotenv()  # reads variables from a .env file and sets them in os.environ

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr + "." + namestr

    def param_stab_est_callback(name, value):
        print('The crazyflie has parameter ' + name + ' set at number: ' + value)
    
    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name, 2)
    time.sleep(1)
    cf.param.set_value(full_name, 1)
    time.sleep(1)

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)

    def log_stab_callback(timestamp, data, logconf):
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    
    logconf.data_received_cb.add_callback(log_stab_callback)

    logconf.start()
    time.sleep(5)
    logconf.stop()

def simple_log(scf, logconf):
    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            break

def simple_connect():

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    group = 'stabilizer'
    name = 'estimator'

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # simple_connect()
        # simple_log_async(scf, lg_stab)
        simple_param_async(scf, group, name)

        # Connection test
        # Synchronous Logger Test
        # Asynchronous Logger Test
        # Parameter Test
        
        