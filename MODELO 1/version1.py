"""
wall_follow_pid.py
Controlador híbrido mejorado para el Pioneer P3DX:
- Braitenberg (evitación distribuida)
- PID con anti-windup (seguimiento de pared derecha)
- Evitación frontal de emergencia
- Detección y escape de atasco inteligente
"""

import robotica
import time

# ─── Constantes ──────────────────────────────────────────────
MIN_SPEED          = 2.0
NO_DETECT_DIST     = 0.5
MAX_DETECT_DIST    = 0.2
MAX_SPEED          = 3.0    # clamp de seguridad

# PID
Kp, Ki, Kd        = 1.2, 0.01, 0.3
D_REF             = 0.5     # distancia deseada a la pared
INTEGRAL_MAX      = 2.0     # anti-windup
DT                = 0.05

# Umbrales de obstáculo
FRONT_EMERGENCY   = 0.30    # m – giro de emergencia frontal
SIDE_ESCAPE       = 0.12    # m – hueco muy estrecho lateral

# Detección de atasco
STUCK_CYCLES      = 15      # ciclos seguidos antes de declarar atasco
STUCK_VEL_THR     = 0.05    # si |v_cmd| < este valor, el robot no avanza
STUCK_OBS_THR     = 0.35    # distancia que consideramos "obstáculo cercano"
ESCAPE_CYCLES     = 25      # ciclos que dura la maniobra de escape

# Braitenberg (solo sonares delanteros, índices 0-7)
LBRAITENBERG = [-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6,
                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
RBRAITENBERG = [-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2,
                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def choose_escape_turn(readings):
    """Devuelve (lspeed, rspeed) de escape girando hacia el lado más libre."""
    left_space  = min(readings[0], readings[1], readings[2])
    right_space = min(readings[5], readings[6], readings[7])
    if left_space > right_space:
        # más espacio a la izquierda → girar izquierda
        return (-1.0, 1.5)
    else:
        # más espacio a la derecha → girar derecha
        return (1.5, -1.0)


def main(args=None):
    coppelia = robotica.Coppelia()
    robot    = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    # Estado PID
    prev_error = 0.0
    integral   = 0.0

    # Estado anti-atasco
    stuck_count  = 0      # ciclos consecutivos de posible atasco
    escape_count = 0      # ciclos restantes de maniobra de escape
    escape_turn  = (0, 0) # velocidades de giro elegidas

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # ── 0. FASE DE ESCAPE (si está activa) ─────────────────
        if escape_count > 0:
            escape_count -= 1
            # Primera mitad: retroceder; segunda mitad: girar
            half = ESCAPE_CYCLES // 2
            if escape_count >= half:
                robot.set_speed(-1.2, -1.2)   # retroceso
            else:
                robot.set_speed(*escape_turn)  # giro hacia el lado libre
            continue

        # ── 1. DETECCIÓN DE ATASCO ──────────────────────────────
        front_min = min(readings[3], readings[4])
        side_min  = min(readings[4], readings[5], readings[6])
        any_obs   = (front_min < STUCK_OBS_THR) or (side_min < STUCK_OBS_THR)

        # Velocidad de comando efectiva (la calcularemos al final,
        # pero aquí estimamos usando el estado anterior)
        if any_obs:
            stuck_count += 1
        else:
            stuck_count = 0

        if stuck_count >= STUCK_CYCLES:
            stuck_count  = 0
            escape_count = ESCAPE_CYCLES
            escape_turn  = choose_escape_turn(readings)
            robot.set_speed(*escape_turn)
            continue

        # ── 2. EMERGENCIA FRONTAL ────────────────────────────────
        if front_min < FRONT_EMERGENCY:
            left_space  = min(readings[0], readings[1])
            right_space = min(readings[6], readings[7])
            if left_space > right_space:
                lspeed, rspeed = -1.0, 1.5
            else:
                lspeed, rspeed = 1.5, -1.0
            robot.set_speed(lspeed, rspeed)
            # Resetear integral para no acumular error durante el giro
            integral = 0.0
            prev_error = 0.0
            continue

        # ── 3. BRAITENBERG ──────────────────────────────────────
        detect = [0.0] * robot.num_sonar
        for i in range(robot.num_sonar):
            d = readings[i]
            if d < NO_DETECT_DIST:
                d = max(d, MAX_DETECT_DIST)
                detect[i] = 1.0 - (d - MAX_DETECT_DIST) / (NO_DETECT_DIST - MAX_DETECT_DIST)

        lspeed = rspeed = MIN_SPEED
        for i in range(robot.num_sonar):
            lspeed += LBRAITENBERG[i] * detect[i]
            rspeed += RBRAITENBERG[i] * detect[i]

        # ── 4. PID PARED DERECHA (con anti-windup) ──────────────
        right_dist = min(readings[4], readings[5], readings[6])
        error      = D_REF - right_dist

        # Anti-windup: solo integra si la salida no está saturada
        new_integral = integral + error * DT
        if abs(new_integral) < INTEGRAL_MAX:
            integral = new_integral

        derivative = (error - prev_error) / DT
        prev_error = error

        u       = Kp * error + Ki * integral + Kd * derivative
        lspeed -= u
        rspeed += u

        # ── 5. CLAMP DE VELOCIDADES ─────────────────────────────
        lspeed = clamp(lspeed, -MAX_SPEED, MAX_SPEED)
        rspeed = clamp(rspeed, -MAX_SPEED, MAX_SPEED)

        # Heurística extra: si ambas velocidades son muy bajas
        # y hay obstáculo lateral, forzar mínimo de avance
        if abs(lspeed) < 0.1 and abs(rspeed) < 0.1 and side_min > SIDE_ESCAPE:
            lspeed = rspeed = MIN_SPEED * 0.5

        # ── 6. ENVIAR VELOCIDAD ──────────────────────────────────
        robot.set_speed(lspeed, rspeed)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()