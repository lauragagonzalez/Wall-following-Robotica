"""
wall_follow_hybrid.py
Controlador híbrido PID + Lógica Borrosa para Pioneer P3DX
- PID: mantiene distancia constante a la pared derecha
- Borroso: gestiona obstáculos frontales y laterales
- Árbitro: decide qué salida usar según el contexto
"""

import robotica

# ─── Constantes ──────────────────────────────────────────────
MAX_SPEED   = 3.0
D_REF       = 0.5      # distancia deseada a pared derecha (m)
BASE_SPEED  = 2.0      # velocidad lineal base (modo libre)
DT          = 0.05     # periodo de muestreo

# Umbrales del árbitro
OBS_FRONT_THR = 0.60   # activa modo obstáculo si frente < esto
OBS_SIDE_THR  = 0.45   # activa modo obstáculo si lateral < esto


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ══════════════════════════════════════════════════════════════
#  BLOQUE PID — controla el error de distancia a la pared
# ══════════════════════════════════════════════════════════════

class WallPID:
    """
    Entrada : error = D_REF - dist_pared_derecha
    Salida  : vel_angular (rad/s equivalente en diferencia de ruedas)
    """
    def __init__(self, Kp=3.5, Ki=0.05, Kd=1.8):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._integral  = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def update(self, error, dt=DT):
        # Anti-windup: limitar integral
        self._integral  = clamp(self._integral + error * dt, -1.5, 1.5)
        derivative      = (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.Kp * error +
                  self.Ki * self._integral +
                  self.Kd * derivative)
        return clamp(output, -MAX_SPEED, MAX_SPEED)


# ══════════════════════════════════════════════════════════════
#  BLOQUE BORROSO — gestiona obstáculos
# ══════════════════════════════════════════════════════════════

def zmf(x, a, b):
    if x <= a: return 1.0
    if x >= b: return 0.0
    return (b - x) / (b - a)

def smf(x, a, b):
    if x <= a: return 0.0
    if x >= b: return 1.0
    return (x - a) / (b - a)

def trimf(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if x <= b: return (x - a) / (b - a)
    return (c - x) / (c - b)

def defuzz(rules):
    num = sum(g * v for g, v in rules)
    den = sum(g     for g, _ in rules)
    return num / den if den > 1e-6 else 0.0

def fuzzy_obstacle(readings):
    """
    Solo se activa en modo obstáculo.
    Devuelve (lspeed, rspeed) para esquivar.
    """
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])

    # Fuzzificación
    f_cerca = zmf(front, 0.20, 0.60)
    f_media = trimf(front, 0.40, 0.65, 0.90)
    f_lejos = smf(front, 0.70, 1.00)

    l_cerca = zmf(left, 0.15, 0.50)
    l_lejos = smf(left, 0.45, 0.90)

    r_cerca = zmf(right, 0.15, 0.50)
    r_lejos = smf(right, 0.45, 0.90)

    # Velocidad lineal
    linear_rules = [
        (f_cerca,                    0.2),   # frenado casi total
        (f_media,                    0.9),   # lento
        (f_lejos,                    1.8),   # normal
    ]

    # Giro (positivo = izquierda)
    turn_rules = [
        (min(f_cerca, l_lejos),      2.5),   # obst frente, izq libre → izquierda
        (min(f_cerca, r_lejos),     -2.5),   # obst frente, der libre → derecha
        (min(f_cerca, l_cerca, r_cerca), 2.0), # trampa → izquierda por defecto
        (min(f_media, l_lejos),      1.2),
        (min(f_media, r_lejos),     -1.2),
        (min(l_cerca, f_lejos),     -1.0),   # obst lateral izq → alejarse
        (min(r_cerca, f_lejos),      1.0),   # obst lateral der → alejarse
        (0.1,                        0.0),   # regla por defecto (evita den=0)
    ]

    v = defuzz(linear_rules)
    t = defuzz(turn_rules)

    lspeed = clamp(v - t, -MAX_SPEED, MAX_SPEED)
    rspeed = clamp(v + t, -MAX_SPEED, MAX_SPEED)
    return lspeed, rspeed


# ══════════════════════════════════════════════════════════════
#  ÁRBITRO — decide qué controlador manda
# ══════════════════════════════════════════════════════════════

def obstacle_detected(readings):
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    return front < OBS_FRONT_THR or left < OBS_SIDE_THR


def combine_pid_fuzzy(pid_output, readings):
    """
    Modo LIBRE: PID controla el giro, velocidad lineal fija.
    La lógica borrosa solo reduce velocidad si algo se acerca.
    """
    front = min(readings[2], readings[3], readings[4], readings[5])

    # Reducción preventiva de velocidad (del controlador original)
    slow = 1.0
    if front < 0.70:
        slow = max(0.5, front / 0.70)

    v_lin = BASE_SPEED * slow

    # PID da la diferencia de velocidades (giro)
    # pid_output positivo → gira izquierda (aleja de pared)
    # pid_output negativo → gira derecha (acerca a pared)
    lspeed = clamp(v_lin - pid_output, -MAX_SPEED, MAX_SPEED)
    rspeed = clamp(v_lin + pid_output, -MAX_SPEED, MAX_SPEED)
    return lspeed, rspeed


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════

def main(args=None):
    coppelia = robotica.Coppelia()
    robot    = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    pid      = WallPID(Kp=3.5, Ki=0.05, Kd=1.8)

    # Estado de escape (igual que tu versión original)
    TRAP_FRONT_THR  = 0.55
    TRAP_LEFT_THR   = 0.60
    TRAP_RIGHT_THR  = 0.60
    TRAP_CYCLES     = 1
    STUCK_OBS_THR   = 0.35
    STUCK_CYCLES    = 15
    ESCAPE_LEVELS   = [(30, 1.3), (45, 1.8), (60, 2.0)]

    stuck_count      = 0
    trap_count       = 0
    escape_level     = 0
    escape_count     = 0
    arc_dir          = 1
    last_readings    = [1.0] * 16

    # Para evitar círculos al inicio
    wall_lost_count  = 0
    WALL_LOST_THR    = 12

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # ── ESCAPE ACTIVO (sin cambios respecto a tu código) ──
        if escape_count > 0:
            # ... tu lógica de escape original ...
            escape_count -= 1
            continue

        last_readings = readings

        # ── ANTI-CÍRCULOS: sin pared → avanzar recto ─────────
        right = min(readings[5], readings[6], readings[7])
        wall_lost_count = wall_lost_count + 1 if right > 0.85 else 0
        if wall_lost_count > WALL_LOST_THR:
            robot.set_speed(1.0, 1.0)
            pid.reset()   # ← importante: resetear integral al recuperar
            continue

        # ── TRAMPA Y ATASCO (sin cambios) ────────────────────
        front_t = min(readings[3], readings[4])
        left_t  = min(readings[0], readings[1], readings[2])
        right_t = min(readings[5], readings[6], readings[7])

        if (front_t < TRAP_FRONT_THR and
            left_t  < TRAP_LEFT_THR  and
            right_t < TRAP_RIGHT_THR):
            trap_count += 1
        else:
            trap_count = max(0, trap_count - 1)

        if trap_count >= TRAP_CYCLES:
            trap_count   = 0
            escape_count = ESCAPE_LEVELS[escape_level][0]
            escape_level = min(escape_level + 1, len(ESCAPE_LEVELS))
            pid.reset()
            continue

        # ── ÁRBITRO PRINCIPAL ─────────────────────────────────
        if obstacle_detected(readings):
            # MODO OBSTÁCULO: lógica borrosa al mando
            pid.reset()   # evitar windup mientras el PID no actúa
            lspeed, rspeed = fuzzy_obstacle(readings)
        else:
            # MODO LIBRE: PID controla el giro
            error      = D_REF - right
            pid_output = pid.update(error)
            lspeed, rspeed = combine_pid_fuzzy(pid_output, readings)

        robot.set_speed(lspeed, rspeed)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()