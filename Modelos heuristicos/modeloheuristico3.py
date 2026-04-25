import robotica
import time

fase = 0
sin_pared = 0
ciclos_giro = 0
iniciado = False

def control(readings):
    global fase, sin_pared, ciclos_giro, iniciado

    front        = min(readings[3], readings[4])
    front_r_diag = readings[5]
    right        = readings[6]
    right_diag   = readings[7]
    left         = readings[1]

    d_obj = 0.8
    base  = 1.2
    k     = 0.5

    # bloque total
    if front < 0.6 and right < 0.4 and left < 0.4:
        fase = 0
        sin_pared = 0
        return -1.2, 1.5

    # peligro crítico
    if front < 0.3 and fase == 0:
        return 0.0, 2.5

    # muro frontal
    if front < 0.5 and fase == 0:
        return 0.1, 2.0

    # diagonal delantera derecha
    if front_r_diag < 0.6 and fase == 0:
        sin_pared = 0
        return 0.4, 1.1

    # búsqueda inicial
    if not iniciado:
        if right < d_obj:
            iniciado = True
        else:
            return 1.2, 0.8

    # contar ciclos sin pared derecha
    if right > d_obj and right_diag > d_obj and fase == 0:
        sin_pared += 1
    else:
        sin_pared = 0

    # esquina confirmada → fase 1
    if sin_pared > 4 and fase == 0:
        fase = 1
        ciclos_giro = 0
        sin_pared = 0

    # fase 1: girar con protección frontal
    if fase == 1:
        ciclos_giro += 1
        if front < 0.4:
            fase = 0
            return 0.0, 2.0
        if right_diag < d_obj or ciclos_giro > 80:
            fase = 2
        else:
            return 1.3, 0.6

    # fase 2: girar suave hasta encontrar pared derecha
    if fase == 2:
        if right > d_obj:
            return 1.4, 0.4
        else:
            fase = 0

    # seguimiento de pared
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