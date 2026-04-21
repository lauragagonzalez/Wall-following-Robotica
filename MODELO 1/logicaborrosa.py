"""
wall_follow_fuzzy.py
Controlador híbrido para el Pioneer P3DX con lógica borrosa:
- Lógica borrosa para seguimiento de pared y evitación
- Anti-círculos: avance recto cuando no hay pared detectada
- Histéresis de pared perdida para evitar corrillos
- Clamp del error de pared para evitar giros extremos
- Detección rápida de trampa
- Escape progresivo con arco diagonal y rotación
- Velocidad preventiva cerca de objetos pequeños
"""

import robotica

# ─── Constantes generales ────────────────────────────────────
MAX_SPEED        = 3.0
DT               = 0.05

# Distancia deseada a la pared derecha
D_REF            = 0.5

# Detección de trampa
TRAP_FRONT_THR   = 0.55
TRAP_LEFT_THR    = 0.60
TRAP_RIGHT_THR   = 0.60
TRAP_CYCLES      = 1
STUCK_OBS_THR    = 0.35
STUCK_CYCLES     = 15

# Velocidad preventiva
SMALL_OBJ_DIST   = 0.55
SMALL_OBJ_SPEED  = 0.8

# Anti-círculos: umbral y ciclos sin pared antes de avanzar recto
NO_WALL_DIST     = 0.85   # si right > esto → pared "perdida"
WALL_LOST_THR    = 12     # ciclos consecutivos sin pared → avanzar recto

# Niveles de escape
ESCAPE_LEVELS = [
    (30, 1.3),
    (45, 1.8),
    (60, 2.0),
]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ══════════════════════════════════════════════════════════════
#  FUNCIONES DE MEMBRESÍA
# ══════════════════════════════════════════════════════════════

def trimf(x, a, b, c):
    """Triangular: sube de a a b, baja de b a c."""
    if x <= a or x >= c:
        return 0.0
    if x <= b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


def trapmf(x, a, b, c, d):
    """Trapezoidal: sube de a a b, plano de b a c, baja de c a d."""
    if x <= a or x >= d:
        return 0.0
    if x <= b:
        return (x - a) / (b - a)
    if x <= c:
        return 1.0
    return (d - x) / (d - c)


def zmf(x, a, b):
    """Z-shaped: 1 hasta a, baja de a a b, 0 desde b."""
    if x <= a:
        return 1.0
    if x >= b:
        return 0.0
    return (b - x) / (b - a)


def smf(x, a, b):
    """S-shaped: 0 hasta a, sube de a a b, 1 desde b."""
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    return (x - a) / (b - a)


# ══════════════════════════════════════════════════════════════
#  VARIABLES LINGÜÍSTICAS DE ENTRADA
# ══════════════════════════════════════════════════════════════

def fuzzify_dist(d):
    """
    Distancia genérica [0, 1.0]:
      cerca   → zmf(0.0, 0.35)
      media   → trimf(0.25, 0.50, 0.75)
      lejos   → smf(0.60, 1.0)
    """
    return {
        'cerca': zmf(d, 0.0, 0.35),
        'media': trimf(d, 0.25, 0.50, 0.75),
        'lejos': smf(d, 0.60, 1.0),
    }


def fuzzify_wall_error(e):
    """
    Error de pared derecha = D_REF - dist_derecha  ∈ [-0.5, 0.5]
    (saturado en fuzzy_control antes de llamar aquí):
      muy_lejos   → trapmf(-0.5, -0.5, -0.4, -0.15)
      lejos       → trimf(-0.35, -0.15, 0.0)
      ok          → trimf(-0.10,  0.0,  0.10)
      cerca       → trimf(0.0,    0.15,  0.35)
      muy_cerca   → trapmf(0.15,  0.4,   0.5,  0.5)
    """
    return {
        'muy_lejos':  trapmf(e, -0.5, -0.5, -0.4, -0.15),
        'lejos':      trimf(e, -0.35, -0.15,  0.0),
        'ok':         trimf(e, -0.10,  0.0,   0.10),
        'cerca':      trimf(e,  0.0,   0.15,  0.35),
        'muy_cerca':  trapmf(e,  0.15,  0.4,  0.5,  0.5),
    }


# ══════════════════════════════════════════════════════════════
#  DEFUZZIFICACIÓN (centro de gravedad simplificado)
# ══════════════════════════════════════════════════════════════

def defuzz_speed(rules):
    """
    rules: lista de (grado, valor_crisp)
    Devuelve el centro de gravedad.
    """
    num = sum(g * v for g, v in rules)
    den = sum(g for g, _ in rules)
    return num / den if den > 1e-6 else 0.0


# ══════════════════════════════════════════════════════════════
#  CONTROLADOR BORROSO PRINCIPAL
# ══════════════════════════════════════════════════════════════

def fuzzy_control(readings):
    """
    Entradas:
      - dist frontal (min sonares 3,4)
      - dist lateral izquierda (min sonares 0,1,2)
      - dist lateral derecha (min sonares 5,6,7)
      - error de pared derecha (saturado a ±0.5)

    Salidas:
      - lspeed, rspeed

    Correcciones aplicadas:
      1. Sin pared en ningún lado → avanzar recto (evita círculos al inicio)
      2. Error saturado a ±0.5 (evita que muy_lejos domine en espacio abierto)
      3. Pesos de giro de búsqueda reducidos (-0.8/-0.5 en vez de -2.0/-1.0)
    """
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])

    # FIX 1: sin pared visible en ningún lado → avanzar recto
    # Esto evita que el robot haga círculos buscando una pared inexistente
    if right > NO_WALL_DIST and left > NO_WALL_DIST and front > 0.80:
        spd = MAX_SPEED * 0.5
        return spd, spd

    # FIX 2: saturar el error para que las reglas no se disparen al máximo
    # en espacio abierto (right muy grande → error muy negativo → círculos)
    error = clamp(D_REF - right, -0.5, 0.5)

    f = fuzzify_dist(front)
    l = fuzzify_dist(left)
    r = fuzzify_dist(right)
    w = fuzzify_wall_error(error)

    # ── Reglas para velocidad lineal (media de ruedas) ────────
    # Singletons de velocidad: parado=0, lento=1, normal=2, rapido=2.5
    linear_rules = [
        # Si frente cerca → parar
        (f['cerca'],                          0.0),
        # Si frente media → lento
        (f['media'],                          1.0),
        # Si frente lejos → normal/rapido según estado de pared
        (min(f['lejos'], w['ok']),            2.5),
        (min(f['lejos'], w['lejos']),         2.0),
        (min(f['lejos'], w['muy_lejos']),     1.5),
        (min(f['lejos'], w['cerca']),         2.0),
        (min(f['lejos'], w['muy_cerca']),     1.5),
    ]

    # ── Reglas para giro (lspeed - rspeed) ───────────────────
    # Singletons: negativo=gira derecha, positivo=gira izquierda
    # FIX 3: pesos de búsqueda de pared reducidos para evitar círculos
    #   muy_lejos: -2.0 → -0.8  |  lejos: -1.0 → -0.5
    turn_rules = [
        # Seguimiento de pared derecha (giro suave al buscar)
        (w['muy_lejos'],   -0.8),   # antes -2.0 → causaba círculos
        (w['lejos'],       -0.5),   # antes -1.0
        (w['ok'],           0.0),   # recto
        (w['cerca'],        1.0),   # cerca → gira izquierda suave
        (w['muy_cerca'],    2.0),   # muy cerca → gira izquierda fuerte

        # Evitación frontal (sin cambios: prioridad alta)
        (min(f['cerca'], l['lejos']),   2.0),   # frente + izq libre → izq
        (min(f['cerca'], r['lejos']),  -2.0),   # frente + der libre → der
        (min(f['media'], l['lejos']),   1.0),
        (min(f['media'], r['lejos']),  -1.0),

        # Evitación lateral (sin cambios)
        (min(l['cerca'], f['lejos']),  -1.5),   # izq cerca → aleja izquierda
        (min(r['cerca'], f['lejos']),   1.5),   # der cerca → aleja derecha
    ]

    v_linear = defuzz_speed(linear_rules)
    v_turn   = defuzz_speed(turn_rules)

    lspeed = clamp(v_linear - v_turn, -MAX_SPEED, MAX_SPEED)
    rspeed = clamp(v_linear + v_turn, -MAX_SPEED, MAX_SPEED)

    # Velocidad preventiva cerca de objetos pequeños (sin cambios)
    front_close = min(readings[2], readings[3], readings[4], readings[5])
    if front_close < SMALL_OBJ_DIST:
        slow_factor = max(SMALL_OBJ_SPEED, front_close / SMALL_OBJ_DIST)
        lspeed *= slow_factor
        rspeed *= slow_factor

    return lspeed, rspeed


# ══════════════════════════════════════════════════════════════
#  FUNCIONES DE ESCAPE
# ══════════════════════════════════════════════════════════════

def is_trap(readings):
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < TRAP_FRONT_THR and
            left  < TRAP_LEFT_THR  and
            right < TRAP_RIGHT_THR)


def is_corner(readings):
    front = min(readings[3], readings[4])
    left  = min(readings[0], readings[1], readings[2])
    right = min(readings[5], readings[6], readings[7])
    return (front < 0.40 and left < 0.45 and right < 0.45)


def best_arc_dir(readings):
    best_i = max(range(8), key=lambda i: readings[i])
    return 1 if best_i <= 3 else -1


def arc_back_speeds(factor, arc_dir):
    fast = -1.4 * factor
    slow = -0.4 * factor
    return (fast, slow) if arc_dir == 1 else (slow, fast)


def rotation_speeds(factor, arc_dir):
    v = 1.5 * factor
    return (-v, v) if arc_dir == 1 else (v, -v)


def open_turn_speeds(readings, factor):
    best_i = max(range(8), key=lambda i: readings[i])
    if best_i <= 3:
        return (-0.8 * factor, 1.6 * factor)
    else:
        return (1.6 * factor, -0.8 * factor)


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════

def main(args=None):
    coppelia = robotica.Coppelia()
    robot    = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    stuck_count      = 0
    trap_count       = 0
    escape_level     = 0
    escape_count     = 0
    escape_is_corner = False
    arc_dir          = 1
    last_readings    = [1.0] * 16

    # FIX 4: contador de histéresis para pérdida de pared
    # Evita corrillos alrededor de obstáculos pequeños:
    # solo actúa el modo "sin pared" si se mantiene N ciclos seguidos
    wall_lost_count  = 0

    coppelia.start_simulation()

    while coppelia.is_running():
        readings = robot.get_sonar()

        # ── MANIOBRA DE ESCAPE ACTIVA ────────────────────────
        if escape_count > 0:
            level_idx       = min(escape_level - 1, len(ESCAPE_LEVELS) - 1)
            total_c, factor = ESCAPE_LEVELS[level_idx]

            if escape_is_corner:
                if level_idx < 2:
                    arc_cycles = (total_c * 3) // 4
                    phase = 'arc' if escape_count > (total_c - arc_cycles) else 'turn'
                    if phase == 'arc':
                        lspd, rspd = arc_back_speeds(factor, arc_dir)
                    else:
                        lspd, rspd = open_turn_speeds(last_readings, factor)
                else:
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
                    total_n, _       = ESCAPE_LEVELS[min(escape_level - 1,
                                                         len(ESCAPE_LEVELS) - 1)]
                    escape_count     = total_n
                    escape_is_corner = is_corner(readings)
                    stuck_count = trap_count = 0
                else:
                    escape_level = 0
                    stuck_count  = trap_count = 0
            continue

        last_readings = readings

        # ── FIX 4: HISTÉRESIS DE PARED PERDIDA ───────────────
        # Si la pared derecha desaparece (obstáculo pequeño, esquina abierta,
        # inicio en espacio libre), esperar WALL_LOST_THR ciclos antes de
        # activar el modo "avanzar recto". Esto evita corrillos alrededor
        # de objetos pequeños donde la pared aparece y desaparece rápido.
        right_now = min(readings[5], readings[6], readings[7])
        if right_now > NO_WALL_DIST:
            wall_lost_count += 1
        else:
            wall_lost_count = 0   # resetear en cuanto la pared vuelve

        if wall_lost_count > WALL_LOST_THR:
            # Pared perdida durante demasiado tiempo → avanzar recto suave
            # hasta encontrar alguna superficie
            robot.set_speed(1.0, 1.0)
            continue

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

        # ── CONTROL BORROSO ───────────────────────────────────
        lspeed, rspeed = fuzzy_control(readings)
        robot.set_speed(lspeed, rspeed)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()