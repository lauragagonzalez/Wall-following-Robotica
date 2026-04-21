"""
wall_follow_pid.py
Mejoras:
- Detección de encaje rápida (sin esperar stuck_count)
  usando varianza de lecturas: si todas son bajas = trampa
- Velocidad reducida cerca de objetos pequeños (bolas)
- Marcha atrás potente al detectar encaje total
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

# ── Detección rápida de encaje total ─────────────────────────
# Si TODOS estos grupos están bloqueados simultáneamente → trampa
TRAP_FRONT_THR   = 0.35   # sonares 3,4
TRAP_LEFT_THR    = 0.40   # sonares 0,1,2
TRAP_RIGHT_THR   = 0.40   # sonares 5,6,7
# Cuántos ciclos consecutivos con patrón trampa para reaccionar
TRAP_CYCLES      = 3      # mucho más rápido que stuck_count

# ── Velocidad preventiva cerca de objetos pequeños ───────────
# Si hay algo muy cerca pero no en todos los lados → objeto pequeño
# Reducir velocidad para no volcarlo con tanta inercia
SMALL_OBJ_DIST   = 0.55
SMALL_OBJ_SPEED  = 0.8    # factor multiplicador

# Niveles de escape
ESCAPE_LEVELS = [
    (20, 1.0),
    (30, 1.5),
    (45, 2.0),
]

LBRAITENBERG = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RBRAITENBERG = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def is_trap(readings):
    """
    Detecta encaje total: frente + izquierda + derecha bloqueados.
    Diferente a is_corner porque los umbrales son más altos (reacciona antes)
    y no requiere que los dos lados sean iguales.
    """
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < TRAP_FRONT_THR and
            left  < TRAP_LEFT_THR  and
            right < TRAP_RIGHT_THR)


def is_corner(readings):
    """Esquina: ambos lados muy bloqueados + frente."""
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < 0.40 and left < 0.45 and right < 0.45)


def has_small_object_nearby(readings):
    """
    Detecta objeto pequeño aislado (bola): un sonar muy cerca
    pero los sonares adyacentes están libres → objeto puntual.
    Devuelve True si hay riesgo de colisión con objeto pequeño.
    """
    for i in range(1, robot_num_sonar - 1):
        if readings[i] < SMALL_OBJ_DIST:
            # Comprueba que los vecinos están más libres
            neighbor_free = (readings[i-1] > readings[i] + 0.15 or
                             readings[i+1] > readings[i] + 0.15)
            if neighbor_free:
                return True
    return False


def best_escape_arc(readings):
    """
    Calcula la dirección y velocidades del arco de escape.
    Mira qué lateral tiene más espacio total y elige el arco
    que aleja al robot hacia ese lado.
    """
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    # arc_dir +1 → arco izquierda (rueda izq rápida al retroceder)
    arc_dir = 1 if left > right else -1
    return arc_dir


def arc_back_speeds(factor, arc_dir):
    fast = -1.4 * factor
    slow = -0.4 * factor
    if arc_dir == 1:
        return (fast, slow)
    else:
        return (slow, fast)


def open_turn_speeds(readings, factor):
    left  = min(readings[0], readings[1])
    right = min(readings[6], readings[7])
    if left > right:
        return (-0.8 * factor, 1.6 * factor)
    else:
        return (1.6 * factor, -0.8 * factor)


# Variable global necesaria para has_small_object_nearby
robot_num_sonar = 16


def main(args=None):
    global robot_num_sonar

    coppelia = robotica.Coppelia()
    robot    = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    robot_num_sonar = robot.num_sonar

    prev_error    = 0.0
    integral      = 0.0

    stuck_count   = 0
    trap_count    = 0      # contador rápido de trampa
    escape_level  = 0
    escape_count  = 0
    escape_is_corner = False
    arc_dir       = 1
    last_readings = [1.0] * 16

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # ── MANIOBRA DE ESCAPE ACTIVA ────────────────────────
        if escape_count > 0:
            level_idx       = min(escape_level - 1, len(ESCAPE_LEVELS) - 1)
            total_c, factor = ESCAPE_LEVELS[level_idx]

            if escape_is_corner:
                arc_cycles = (total_c * 2) // 3
                phase = 'arc' if escape_count > (total_c - arc_cycles) else 'turn'
                if phase == 'arc':
                    lspd, rspd = arc_back_speeds(factor, arc_dir)
                else:
                    lspd, rspd = open_turn_speeds(last_readings, factor)
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
                    escape_level += 1
                    arc_dir = -arc_dir
                    total_n, _ = ESCAPE_LEVELS[min(escape_level-1, len(ESCAPE_LEVELS)-1)]
                    escape_count     = total_n
                    escape_is_corner = is_corner(readings)
                    stuck_count = trap_count = 0
                else:
                    escape_level = 0
                    stuck_count = trap_count = 0
                    integral    = 0.0
                    prev_error  = 0.0
            continue

        last_readings = readings

        # ── DETECCIÓN RÁPIDA DE TRAMPA (sofá+pared) ──────────
        if is_trap(readings):
            trap_count += 1
        else:
            trap_count = max(0, trap_count - 1)

        if trap_count >= TRAP_CYCLES:
            # Reacción inmediata: marcha atrás potente en arco
            trap_count       = 0
            stuck_count      = 0
            escape_is_corner = True
            arc_dir          = best_escape_arc(readings)
            total, _         = ESCAPE_LEVELS[escape_level]
            escape_count     = total
            escape_level     = min(escape_level + 1, len(ESCAPE_LEVELS))
            lspd, rspd       = arc_back_speeds(1.2, arc_dir)   # factor extra al inicio
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
            arc_dir          = best_escape_arc(readings)
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

        # ── VELOCIDAD PREVENTIVA (bolas y objetos pequeños) ───
        # Reduce velocidad si hay un objeto puntual cerca
        # para no golpearlo con inercia y volcarlo
        front_close = min(readings[2], readings[3], readings[4], readings[5])
        if front_close < SMALL_OBJ_DIST:
            # Factor proporcional: más cerca → más lento
            slow_factor = max(SMALL_OBJ_SPEED,
                              front_close / SMALL_OBJ_DIST)
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