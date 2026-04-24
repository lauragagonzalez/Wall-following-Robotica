import robotica
import time

def control(readings):
    front        = min(readings[3], readings[4])
    front_r_diag = readings[5]
    right        = readings[6]
    right_diag   = readings[7]
    left         = readings[1]
    d_obj = 0.5
    base  = 1.7
    k     = 0.5
    if front < 0.6 and right < 0.4 and left < 0.4:
        return -1.2, 1.5
    if front < 0.3:
        return 0.0, 2.5
    if front < 0.5:
        return 0.1, 2.0
    if front_r_diag < 0.6:
        return 0.4, 1.1
    if right > d_obj and right_diag > d_obj and front < 2.0:
        return 1.5, 0.5
    if right > d_obj and right_diag > d_obj and front >= 2.0:
        return base, base

    error = (0.5 * (d_obj - right) +
             0.3 * (d_obj - front_r_diag) +
             0.2 * (d_obj - right_diag))
    lspeed = max(min(base + k * error, 1.6), 0.3)
    rspeed = max(min(base - k * error, 1.6), 0.3)
    return lspeed, rspeed

def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    time.sleep(1)
    while coppelia.is_running():
        readings = robot.get_sonar()
        lspeed, rspeed = control(readings)
        robot.set_speed(lspeed, rspeed)
    coppelia.stop_simulation()

if __name__ == '__main__':
    main()
