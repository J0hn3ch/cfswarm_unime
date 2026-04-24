import time

def figure8(scf):
    cf = scf.cf

    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
        time.sleep(0.1)
    
    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        time.sleep(0.1)
    
    for _ in range(50):
        cf.commander.send_hover_setpoint(0.23, 0, -30 * 2, 0.4)
        time.sleep(0.1)

    for _ in range(50):
        cf.commander.send_hover_setpoint(0.23, 0, 30 * 2, 0.4)
        time.sleep(0.1)

    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
        time.sleep(0.1)

    for y in range(10):
        cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
        time.sleep(0.3)
    
    time.sleep(1)
    cf.commander.send_notify_setpoint_stop()