"""
wall_follow_pid_variable_speed.py

PID simple para seguimiento de pared derecha
con velocidad lineal variable + búsqueda de pared
"""

import robotica
import time

# =========================
# Parámetros del controlador
# =========================

D_REF = 0.2

Kp = 3
Ki = 0.02
Kd = 0.6

DT = 0.05

# Velocidades
BASE_SPEED_MAX = 2.3
BASE_SPEED_MIN = 1.0

MAX_SPEED = 3.0
MIN_SPEED = -3.0

FRONT_THRESHOLD = 0.25
INTEGRAL_LIMIT = 1.5

# NUEVO: umbral para detectar pared
WALL_DETECTED = 0.8


def clamp(value, low, high):
    return max(low, min(value, high))


def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    integral = 0.0
    prev_error = 0.0

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # Sensores relevantes
        front_dist = min(readings[3], readings[4])
        right_dist = min(readings[5], readings[6], readings[7])

        # -------------------------
        # 1. Emergencia frontal
        # -------------------------
        if front_dist < FRONT_THRESHOLD:
            left_speed = -1.0
            right_speed = 1.5

            integral = 0.0
            prev_error = 0.0

            robot.set_speed(left_speed, right_speed)
            time.sleep(DT)
            continue

        # -------------------------
        # 2. NUEVO: Buscar pared
        # -------------------------
        if right_dist > WALL_DETECTED:
            print("no se detecta pared")
            # No hay pared → girar suavemente hacia la derecha
            left_speed = 2.0
            right_speed = 1.6

            # Reset PID
            integral = 0.0
            prev_error = 0.0

            robot.set_speed(left_speed, right_speed)
            time.sleep(DT)
            continue

        # -------------------------
        # 3. Error lateral (PID)
        # -------------------------
        error = D_REF - right_dist

        # -------------------------
        # 4. PID
        # -------------------------
        print("pared detectada")
        integral += error * DT
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        derivative = (error - prev_error) / DT
        prev_error = error

        u = Kp * error + Ki * integral + Kd * derivative

        u = clamp(u, -1.5, 1.5)

        # -------------------------
        # 5. Velocidad base variable
        # -------------------------
        front_factor = clamp((front_dist - FRONT_THRESHOLD) / 0.8, 0.0, 1.0)

        error_factor = clamp(1.0 - abs(error) / 0.5, 0.2, 1.0)

        speed_factor = min(front_factor, error_factor)

        base_speed = BASE_SPEED_MIN + (BASE_SPEED_MAX - BASE_SPEED_MIN) * speed_factor

        # -------------------------
        # 6. Velocidades finales
        # -------------------------
        left_speed = base_speed - u
        right_speed = base_speed + u

        left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
        right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

        robot.set_speed(left_speed, right_speed)

        time.sleep(DT)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()