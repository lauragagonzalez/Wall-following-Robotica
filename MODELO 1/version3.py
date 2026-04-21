"""
wall_follow_pid.py
Controlador híbrido para el Pioneer P3DX:
- Braitenberg (evitación distribuida)
- PID con anti-windup (seguimiento de pared derecha)
- Evitación frontal de emergencia
- Detección rápida de trampa (1 ciclo)
- Escape progresivo: arco → arco invertido → rotación completa
- Búsqueda del ángulo más libre para escapar
- Velocidad preventiva cerca de objetos pequeños (bolas)
"""

import robotica

# ─── Constantes generales ────────────────────────────────────
MIN_SPEED        = 2.0
NO_DETECT_DIST   = 0.5
MAX_DETECT_DIST  = 0.2
MAX_SPEED        = 3.0

# PID
Kp, Ki, Kd      = 1.2, 0.01, 0.3
D_REF           = 0.5
INTEGRAL_MAX    = 2.0
DT              = 0.05

# Umbrales de obstáculo
FRONT_EMERGENCY  = 0.30
STUCK_OBS_THR    = 0.35
STUCK_CYCLES     = 15

# Detección rápida de trampa
TRAP_FRONT_THR   = 0.55
TRAP_LEFT_THR    = 0.60
TRAP_RIGHT_THR   = 0.60
TRAP_CYCLES      = 1

# Velocidad preventiva cerca de objetos pequeños
SMALL_OBJ_DIST   = 0.55
SMALL_OBJ_SPEED  = 0.8

# Niveles de escape progresivo (ciclos_totales, factor_velocidad)
# Nivel 1: arco hacia el lado más libre
# Nivel 2: arco hacia el lado contrario
# Nivel 3: rotación completa sobre sí mismo
ESCAPE_LEVELS = [
    (30, 1.3),
    (45, 1.8),
    (60, 2.0),
]

LBRAITENBERG = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RBRAITENBERG = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def is_trap(readings):
    """Encaje total: frente + ambos lados bloqueados."""
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < TRAP_FRONT_THR and
            left  < TRAP_LEFT_THR  and
            right < TRAP_RIGHT_THR)


def is_corner(readings):
    """Esquina: ambos lados bloqueados + frente."""
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < 0.40 and left < 0.45 and right < 0.45)


def best_arc_dir(readings):
    """
    Busca el sonar individual con mayor distancia (más libre)
    entre los 8 frontales/laterales y decide hacia dónde girar.
    Más preciso que comparar min() de grupos.
    """
    best_i = max(range(8), key=lambda i: readings[i])
    return 1 if best_i <= 3 else -1


def arc_back_speeds(factor, arc_dir):
    """Retroceso en arco: una rueda más rápida que la otra."""
    fast = -1.4 * factor
    slow = -0.4 * factor
    return (fast, slow) if arc_dir == 1 else (slow, fast)


def rotation_speeds(factor, arc_dir):
    """
    Rotación completa sobre sí mismo (nivel 3).
    No necesita saber dónde está el hueco,
    simplemente gira hasta encontrar espacio.
    """
    v = 1.5 * factor
    return (-v, v) if arc_dir == 1 else (v, -v)


def open_turn_speeds(readings, factor):
    """Giro final hacia el sonar más libre."""
    best_i = max(range(8), key=lambda i: readings[i])
    if best_i <= 3:
        return (-0.8 * factor, 1.6 * factor)
    else:
        return (1.6 * factor, -0.8 * factor)


def main(args=None):
    coppelia = robotica.Coppelia()
    robot    = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    prev_error       = 0.0
    integral         = 0.0

    stuck_count      = 0
    trap_count       = 0
    escape_level     = 0
    escape_count     = 0
    escape_is_corner = False
    arc_dir          = 1
    last_readings    = [1.0] * 16

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # ── MANIOBRA DE ESCAPE ACTIVA ────────────────────────
        if escape_count > 0:
            level_idx       = min(escape_level - 1, len(ESCAPE_LEVELS) - 1)
            total_c, factor = ESCAPE_LEVELS[level_idx]

            if escape_is_corner:
                if level_idx < 2:
                    # Niveles 1 y 2: retroceso en arco (75%) + giro (25%)
                    arc_cycles = (total_c * 3) // 4
                    phase = 'arc' if escape_count > (total_c - arc_cycles) else 'turn'
                    if phase == 'arc':
                        lspd, rspd = arc_back_speeds(factor, arc_dir)
                    else:
                        lspd, rspd = open_turn_speeds(last_readings, factor)
                else:
                    # Nivel 3: retroceso recto (mitad) + rotación completa (mitad)
                    half = total_c // 2
                    phase = 'back' if escape_count > (total_c - half) else 'spin'
                    if phase == 'back':
                        v = -1.2 * factor
                        lspd, rspd = v, v
                    else:
                        lspd, rspd = rotation_speeds(factor, arc_dir)
            else:
                half  = total_c // 2
                phase = 'back' if escape_count > (total_c - half) else 'turn'
                if phase == 'back':
                    v = -1.1 * factor
                    lspd, rspd = v, v
                else:
                    lspd, rspd = open_turn_speeds(last_readings, factor)

            robot.set_speed(clamp(lspd, -MAX_SPEED, MAX_SPEED),
                            clamp(rspd, -MAX_SPEED, MAX_SPEED))
            escape_count -= 1

            if escape_count == 0:
                front = min(readings[3], readings[4])
                side  = min(readings[4], readings[5], readings[6])
                still_stuck = (front < STUCK_OBS_THR or side < STUCK_OBS_THR)
                if still_stuck and escape_level < len(ESCAPE_LEVELS):
                    escape_level    += 1
                    arc_dir          = -arc_dir
                    total_n, _       = ESCAPE_LEVELS[min(escape_level - 1, len(ESCAPE_LEVELS) - 1)]
                    escape_count     = total_n
                    escape_is_corner = is_corner(readings)
                    stuck_count = trap_count = 0
                else:
                    escape_level = 0
                    stuck_count  = trap_count = 0
                    integral     = 0.0
                    prev_error   = 0.0
            continue

        last_readings = readings

        # ── DETECCIÓN RÁPIDA DE TRAMPA ────────────────────────
        if is_trap(readings):
            trap_count += 1
        else:
            trap_count = max(0, trap_count - 1)

        if trap_count >= TRAP_CYCLES:
            trap_count       = 0
            stuck_count      = 0
            escape_is_corner = True
            arc_dir          = best_arc_dir(readings)
            total, _         = ESCAPE_LEVELS[escape_level]
            escape_count     = total
            escape_level     = min(escape_level + 1, len(ESCAPE_LEVELS))
            lspd, rspd       = arc_back_speeds(1.6, arc_dir)
            robot.set_speed(clamp(lspd, -MAX_SPEED, MAX_SPEED),
                            clamp(rspd, -MAX_SPEED, MAX_SPEED))
            integral   = 0.0
            prev_error = 0.0
            continue

        # ── DETECCIÓN DE ATASCO NORMAL ────────────────────────
        front_min = min(readings[3], readings[4])
        side_min  = min(readings[4], readings[5], readings[6])
        any_obs   = (front_min < STUCK_OBS_THR) or (side_min < STUCK_OBS_THR)
        stuck_count = (stuck_count + 1) if any_obs else max(0, stuck_count - 1)

        if stuck_count >= STUCK_CYCLES:
            stuck_count      = 0
            corner           = is_corner(readings)
            escape_is_corner = corner
            arc_dir          = best_arc_dir(readings)
            total, _         = ESCAPE_LEVELS[escape_level]
            escape_count     = total
            escape_level     = min(escape_level + 1, len(ESCAPE_LEVELS))
            if corner:
                lspd, rspd = arc_back_speeds(1.0, arc_dir)
            else:
                lspd, rspd = (-1.1, -1.1)
            robot.set_speed(clamp(lspd, -MAX_SPEED, MAX_SPEED),
                            clamp(rspd, -MAX_SPEED, MAX_SPEED))
            continue

        # ── EMERGENCIA FRONTAL ────────────────────────────────
        if front_min < FRONT_EMERGENCY:
            left  = min(readings[0], readings[1])
            right = min(readings[6], readings[7])
            if left > right:
                robot.set_speed(-1.0, 1.5)
            else:
                robot.set_speed(1.5, -1.0)
            integral   = 0.0
            prev_error = 0.0
            continue

        # ── BRAITENBERG ───────────────────────────────────────
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

        # ── VELOCIDAD PREVENTIVA (bolas) ──────────────────────
        front_close = min(readings[2], readings[3], readings[4], readings[5])
        if front_close < SMALL_OBJ_DIST:
            slow_factor = max(SMALL_OBJ_SPEED, front_close / SMALL_OBJ_DIST)
            lspeed *= slow_factor
            rspeed *= slow_factor

        # ── PID PARED DERECHA ─────────────────────────────────
        right_dist   = min(readings[4], readings[5], readings[6])
        error        = D_REF - right_dist
        new_integral = integral + error * DT
        if abs(new_integral) < INTEGRAL_MAX:
            integral = new_integral
        derivative   = (error - prev_error) / DT
        prev_error   = error

        u       = Kp * error + Ki * integral + Kd * derivative
        lspeed -= u
        rspeed += u

        robot.set_speed(clamp(lspeed, -MAX_SPEED, MAX_SPEED),
                        clamp(rspeed, -MAX_SPEED, MAX_SPEED))

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()