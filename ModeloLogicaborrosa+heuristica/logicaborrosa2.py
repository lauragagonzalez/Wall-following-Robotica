import robotica
import time

iniciado = False


# =========================
# HELPERS
# =========================
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


# =========================
# CONTROL
# =========================
def control(readings):
    global iniciado

    front      = min(readings[3], readings[4])
    front_r    = readings[5]
    right      = readings[6]
    right_diag = readings[7]
    back_right = readings[0]

    # =========================
    # 🚨 EMERGENCIA
    # =========================
    if min(front, front_r) < 0.25:
        return -0.7, 1.3

    # =========================
    # 🟡 INICIO
    # =========================
    if not iniciado:
        if right < 1.2 or right_diag < 1.2:
            iniciado = True
        else:
            return 0.7, 0.65

    # =========================
    # 📉 SIN PARED
    # =========================
    wall_presence = (
        (1 - lejos(right)) +
        (1 - lejos(right_diag)) +
        (1 - lejos(back_right))
    ) / 3

    if wall_presence < 0.2:
        return 1.0, 0.5  # 🔥 más rápido en vacío

    # =========================
    # 🚀 VELOCIDAD BASE (↑ MÁS RÁPIDO)
    # =========================
    front_avg = (front + front_r) / 2

    vel = defuzz([
        (lejos(front_avg, 1.2), 1.3),
        (medio(front_avg, 1.0, 0.3), 0.95),
        (cerca(front_avg, 0.6), 0.6),
    ])

    # 🔥 boost general de velocidad
    vel *= 1.25

    # =========================
    # 🧠 SENSOR FUSION
    # =========================
    right_f = (
        0.45 * right +
        0.35 * right_diag +
        0.20 * front_r
    )

    target = 0.45
    error = right_f - target

    # =========================
    # 🧭 ANTICIPACIÓN
    # =========================
    corner_avoid = (1 - lejos(front_r, 0.8))
    error += corner_avoid * 0.25

    error = 0.7 * error + 0.3 * (right - back_right)

    if abs(error) < 0.03:
        error = 0

    # =========================
    # 🧭 GIRO (más agresivo para velocidad alta)
    # =========================
    giro = error * 1.15   # 🔥 más reacción

    giro = max(min(giro, 0.42), -0.42)

    # =========================
    # 🚗 VELOCIDAD FINAL
    # =========================
    vel = vel * (1 - abs(giro) * 0.22)

    vel = max(min(vel, 1.85), 0.55)  # 🔥 MÁS VELOCIDAD FINAL

    # =========================
    # 🛞 SALIDA
    # =========================
    lspeed = vel + giro
    rspeed = vel - giro

    lspeed = max(min(lspeed, 1.35), 0.2)
    rspeed = max(min(rspeed, 1.35), 0.2)

    return lspeed, rspeed


# =========================
# MAIN
# =========================
def main():
    global iniciado
    iniciado = False

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