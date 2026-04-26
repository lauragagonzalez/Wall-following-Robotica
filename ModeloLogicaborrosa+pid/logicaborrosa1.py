import robotica
import time

iniciado = False


# =========================
# HELPERS (igual que el tuyo)
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
    # 🚨 EMERGENCIA REAL (FRONTAL COMPLETO)
    # =========================
    front_risk = min(front, front_r)

    if front_risk < 0.25:
        # ahora usa frontal completo, no solo front
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
    # 📉 SIN PARED (mejor sensor fusion)
    # =========================
    wall_presence = (
        (1 - lejos(right)) +
        (1 - lejos(right_diag)) +
        (1 - lejos(back_right))
    ) / 3

    if wall_presence < 0.2:
        return 0.9, 0.45

    # =========================
    # 🚀 VELOCIDAD (usa FRONT + FRONT_R bien)
    # =========================
    front_avg = (front + front_r) / 2

    reglas_vel = [
        (lejos(front_avg, 1.2), 1.0),
        (medio(front_avg, 1.0, 0.3), 0.75),
        (cerca(front_avg, 0.6), 0.4),
    ]

    vel = defuzz(reglas_vel)

    # =========================
    # 🧠 SENSOR FUSION REAL (clave)
    # =========================
    # ahora el robot "ve esquina", no solo pared derecha
    right_f = (
        0.45 * right +
        0.35 * right_diag +
        0.20 * front_r
    )

    # objetivo pared (más estable que 0.45 fijo)
    target = 0.55
    error = right_f - target

    # =========================
    # 🧭 ANTICIPACIÓN (MUY IMPORTANTE)
    # =========================
    # front_r evita chocar en esquinas
    corner_avoid = (1 - lejos(front_r, 0.8))

    error += corner_avoid * 0.25  # empuja hacia dentro si hay esquina

    # =========================
    # 🧊 ANTI-ZIGZAG
    # =========================
    error = 0.7 * error + 0.3 * (right - back_right)

    if abs(error) < 0.03:
        error = 0

    # =========================
    # 🧭 GIRO
    # =========================
    giro = error * 1.0
    giro = max(min(giro, 0.45), -0.45)

    # =========================
    # 🚗 VELOCIDAD FINAL
    # =========================
    vel = vel * (1 - abs(giro) * 0.25)
    vel = max(min(vel, 1.6), 0.5)

    # =========================
    # 🛞 SALIDA
    # =========================
    lspeed = vel + giro
    rspeed = vel - giro

    lspeed = max(min(lspeed, 1.2), 0.15)
    rspeed = max(min(rspeed, 1.2), 0.15)

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