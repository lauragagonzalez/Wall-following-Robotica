import robotica
import time

estado = "INICIO"
iniciado = False

def cerca(x, umbral=0.5):
    return max(0, min(1, (umbral - x) / umbral))


def medio(x, centro=0.6, ancho=0.3):
    return max(0, 1 - abs(x - centro) / ancho)


def lejos(x, umbral=0.8):
    return max(0, min(1, (x - umbral) / (1.5 - umbral)))


def defuzz(reglas):
    num = sum(p * v for p, v in reglas)
    den = sum(p for p, v in reglas)
    return num / den if den > 0 else 0



def control(readings):
    global estado, iniciado

    front      = min(readings[3], readings[4])
    front_r    = readings[5]
    right      = readings[6]
    right_diag = readings[7]
    back_right = readings[0]
    left       = min(readings[1], readings[2])

    # estado de inicio
    if min(front, front_r) < 0.25:
        estado = "INICIO"
        return -0.7, 1.3

    if estado == "INICIO":

        if right < 1.2 or right_diag < 1.2:
            estado = "SEGUIR"
            return 0.85, 0.8

        corr = (left - right) * 0.12
        base = 0.78

        lspeed = base - corr
        rspeed = base + corr

        lspeed = max(min(lspeed, 0.9), 0.6)
        rspeed = max(min(rspeed, 0.9), 0.6)

        return lspeed, rspeed

    # velocidades
    front_avg = (front + front_r) / 2

    vel = defuzz([
        (lejos(front_avg, 1.2), 1.3),
        (medio(front_avg, 1.0, 0.3), 0.95),
        (cerca(front_avg, 0.6), 0.6),
    ])

    vel *= 1.25


    right_f = (
        0.45 * right +
        0.35 * right_diag +
        0.20 * front_r
    )

    target = 0.45
    error = right_f - target

    #esquinas
    corner_avoid = (1 - lejos(front_r, 0.8))
    error += corner_avoid * 0.25

    error = 0.7 * error + 0.3 * (right - back_right)

    if abs(error) < 0.03:
        error = 0

    giro = error * 1.15
    giro = max(min(giro, 0.42), -0.42)

    vel = vel * (1 - abs(giro) * 0.22)
    vel = max(min(vel, 1.85), 0.55)

    lspeed = vel + giro
    rspeed = vel - giro

    lspeed = max(min(lspeed, 1.35), 0.2)
    rspeed = max(min(rspeed, 1.35), 0.2)

    return lspeed, rspeed


def main():
    global estado
    estado = "INICIO"

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
