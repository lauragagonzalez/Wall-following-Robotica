"""
wall_follow_pid_simple.py

Controlador PID simple para el Pioneer P3DX en CoppeliaSim.
Objetivo: seguir la pared derecha manteniendo una distancia deseada.
Incluye una protección frontal básica para evitar choques.
"""

import robotica
import time

# =========================
# Parámetros del controlador
# =========================

# Distancia deseada a la pared derecha (setpoint)
D_REF = 0.2     # cambiado

# Ganancias PID
Kp = 3
Ki = 0.02
Kd = 0.6

# Tiempo de muestreo
DT = 0.05

# Velocidad base de avance
BASE_SPEED = 2.0

# Límite de velocidades de rueda
MAX_SPEED = 3.0
MIN_SPEED = -3.0

# Umbral de emergencia frontal
FRONT_THRESHOLD = 0.3   # cambio

# Límite para evitar integral excesiva
INTEGRAL_LIMIT = 1.5


def clamp(value, low, high):
    """Limita un valor al intervalo [low, high]."""
    return max(low, min(value, high))


def main(args=None):
    # Crear conexión con Coppelia y robot
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    # Variables internas del PID
    integral = 0.0
    prev_error = 0.0

    coppelia.start_simulation()

    while coppelia.is_running():
        # Leer sensores ultrasónicos
        readings = robot.get_sonar()

        # -------------------------------------------------
        # 1. Obtener distancias relevantes
        # -------------------------------------------------
        # Sensores frontales: usamos los centrales para detectar choque
        front_dist = min(readings[3], readings[4])

        # Sensores del lado derecho:
        # tomamos varios para tener una medida más robusta
        right_dist = min(readings[5], readings[6], readings[7])

        # -------------------------------------------------
        # 2. Protección frontal básica
        # -------------------------------------------------
        if front_dist < FRONT_THRESHOLD:
            # Si hay obstáculo delante, giramos a la izquierda
            # para evitar la colisión
            left_speed = -1.0
            right_speed = 1.5

            # Reiniciamos PID para que no acumule error inútil
            integral = 0.0
            prev_error = 0.0

            robot.set_speed(left_speed, right_speed)
            time.sleep(DT)
            continue

        # -------------------------------------------------
        # 3. Calcular error
        # -------------------------------------------------
        # Error positivo: demasiado cerca de la pared
        # Error negativo: demasiado lejos
        error = D_REF - right_dist

        # -------------------------------------------------
        # 4. Término integral
        # -------------------------------------------------
        integral += error * DT
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        # -------------------------------------------------
        # 5. Término derivativo
        # -------------------------------------------------
        derivative = (error - prev_error) / DT
        prev_error = error

        # -------------------------------------------------
        # 6. Salida del PID
        # -------------------------------------------------
        u = Kp * error + Ki * integral + Kd * derivative

        # -------------------------------------------------
        # 7. Convertir corrección en velocidades de rueda
        # -------------------------------------------------
        # Si u > 0, queremos girar a la izquierda:
        # reducimos rueda izquierda y aumentamos derecha
        left_speed = BASE_SPEED - u
        right_speed = BASE_SPEED + u

        # Limitar velocidades
        left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
        right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

        # -------------------------------------------------
        # 8. Enviar velocidades al robot
        # -------------------------------------------------
        robot.set_speed(left_speed, right_speed)

        time.sleep(DT)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()