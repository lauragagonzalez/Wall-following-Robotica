"""
wall_follow_neuroevolution.py

Neuroevolución básica para seguimiento de pared derecha
con robot Pioneer P3DX en Coppelia.

Cada individuo es una red neuronal pequeña:
5 sensores -> 6 neuronas ocultas -> 2 velocidades de rueda
"""

import robotica
import time
import numpy as np


# =========================
# Parámetros generales
# =========================

DT = 0.05
EPISODE_TIME = 20.0

D_REF = 0.2

MAX_SENSOR_DIST = 1.0
COLLISION_DIST = 0.12

BASE_SPEED = 1.2
TURN_GAIN = 1.2

MAX_SPEED = 3.0
MIN_SPEED = -3.0


# =========================
# Parámetros neuroevolución
# =========================

N_INPUTS = 5
N_HIDDEN = 6
N_OUTPUTS = 2

GENOME_SIZE = (
    N_INPUTS * N_HIDDEN
    + N_HIDDEN
    + N_HIDDEN * N_OUTPUTS
    + N_OUTPUTS
)

POP_SIZE = 20
ELITE_SIZE = 4
N_GENERATIONS = 15

MUTATION_STD = 0.25


def clamp(value, low, high):
    return max(low, min(value, high))


# =========================
# Red neuronal
# =========================

def create_individual():
    return np.random.randn(GENOME_SIZE) * 0.5


def decode_genome(genome):
    idx = 0

    W1 = genome[idx:idx + N_INPUTS * N_HIDDEN].reshape(N_INPUTS, N_HIDDEN)
    idx += N_INPUTS * N_HIDDEN

    b1 = genome[idx:idx + N_HIDDEN]
    idx += N_HIDDEN

    W2 = genome[idx:idx + N_HIDDEN * N_OUTPUTS].reshape(N_HIDDEN, N_OUTPUTS)
    idx += N_HIDDEN * N_OUTPUTS

    b2 = genome[idx:idx + N_OUTPUTS]

    return W1, b1, W2, b2


def neural_policy(genome, sensors):
    W1, b1, W2, b2 = decode_genome(genome)

    x = np.array(sensors, dtype=float)

    h = np.tanh(x @ W1 + b1)
    y = np.tanh(h @ W2 + b2)

    # Salidas entre -1 y 1
    turn_left = y[0]
    turn_right = y[1]

    left_speed = BASE_SPEED + TURN_GAIN * turn_left
    right_speed = BASE_SPEED + TURN_GAIN * turn_right

    left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

    return left_speed, right_speed


# =========================
# Sensores
# =========================

def normalize_distance(d):
    """
    Convierte distancia en valor normalizado.
    Si el sensor no detecta nada o da un valor raro, se considera distancia máxima.
    """
    if d is None:
        d = MAX_SENSOR_DIST

    d = clamp(d, 0.0, MAX_SENSOR_DIST)

    # 0 = muy cerca, 1 = lejos
    return d / MAX_SENSOR_DIST


def get_inputs(readings):
    front_left_corner = readings[0]   # sensor 1
    front_left = readings[1]          # sensor 2
    front = min(readings[2], readings[3])  # sensores 3 y 4
    front_right = readings[4]         # sensor 5
    front_right_corner = readings[5]  # sensor 6
    right_front = readings[6]         # sensor 7
    right = readings[7]               # sensor 8

    return [
        normalize_distance(front_left_corner),
        normalize_distance(front),
        normalize_distance(front_right_corner),
        normalize_distance(right_front),
        normalize_distance(right),
    ]


def get_relevant_distances(readings):
    front_dist = min(
        readings[0],  # sensor 1
        readings[1],  # sensor 2
        readings[2],  # sensor 3
        readings[3],  # sensor 4
        readings[4],  # sensor 5
        readings[5],  # sensor 6
    )

    right_dist = min(
        readings[6],  # sensor 7
        readings[7],  # sensor 8
    )

    front_dist = clamp(front_dist, 0.0, MAX_SENSOR_DIST)
    right_dist = clamp(right_dist, 0.0, MAX_SENSOR_DIST)

    return front_dist, right_dist


# =========================
# Fitness
# =========================

def evaluate_individual(robot, genome):
    """
    Ejecuta un episodio con un individuo y devuelve su fitness.
    """

    fitness = 0.0
    elapsed = 0.0

    previous_left_speed = 0.0
    previous_right_speed = 0.0

    while elapsed < EPISODE_TIME:
        readings = robot.get_sonar()

        inputs = get_inputs(readings)
        front_dist, right_dist = get_relevant_distances(readings)

        left_speed, right_speed = neural_policy(genome, inputs)

        robot.set_speed(left_speed, right_speed)

        # -------------------------
        # Fitness por supervivencia
        # -------------------------
        fitness += 1.0

        # -------------------------
        # Penalización por colisión
        # -------------------------
        if front_dist < COLLISION_DIST or right_dist < COLLISION_DIST:
            fitness -= 150.0
            break

        # -------------------------
        # Recompensa por avanzar
        # -------------------------
        forward_speed = (left_speed + right_speed) / 2.0

        if forward_speed > 0:
            fitness += 2.0 * forward_speed
        else:
            fitness -= 5.0

        # -------------------------
        # Penalización por alejarse de la pared derecha
        # -------------------------
        wall_error = abs(D_REF - right_dist)
        fitness -= 8.0 * wall_error

        # -------------------------
        # Penalización si no hay pared a la derecha
        # -------------------------
        if right_dist > 0.7:
            fitness -= 2.0

        # -------------------------
        # Penalización por giros excesivos
        # -------------------------
        turning = abs(left_speed - right_speed)
        fitness -= 0.3 * turning

        # -------------------------
        # Penalización por cambios bruscos
        # -------------------------
        speed_change = abs(left_speed - previous_left_speed) + abs(right_speed - previous_right_speed)
        fitness -= 0.1 * speed_change

        previous_left_speed = left_speed
        previous_right_speed = right_speed

        time.sleep(DT)
        elapsed += DT

    robot.set_speed(0.0, 0.0)

    return fitness


# =========================
# Evolución
# =========================

def mutate(genome):
    child = genome.copy()
    noise = np.random.randn(GENOME_SIZE) * MUTATION_STD
    child += noise
    return child


def create_next_generation(population, fitnesses):
    sorted_indices = np.argsort(fitnesses)[::-1]

    elites = [population[i].copy() for i in sorted_indices[:ELITE_SIZE]]

    new_population = []

    # Mantener élite
    for elite in elites:
        new_population.append(elite)

    # Rellenar población con mutaciones de los mejores
    while len(new_population) < POP_SIZE:
        parent = elites[np.random.randint(0, ELITE_SIZE)]
        child = mutate(parent)
        new_population.append(child)

    return new_population


# =========================
# Main
# =========================

def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    population = [create_individual() for _ in range(POP_SIZE)]

    best_genome = None
    best_fitness = -float("inf")

    coppelia.start_simulation()
    time.sleep(1.0)

    for generation in range(N_GENERATIONS):
        print(f"\n===== GENERACIÓN {generation + 1}/{N_GENERATIONS} =====")

        fitnesses = []

        for i, genome in enumerate(population):
            print(f"Evaluando individuo {i + 1}/{POP_SIZE}...")

            # IMPORTANTE:
            # Si podéis resetear la escena desde vuestra clase robotica,
            # hacedlo aquí antes de evaluar cada individuo.
            #
            # Ejemplo aproximado, dependiendo de vuestra API:
            # coppelia.stop_simulation()
            # time.sleep(0.5)
            # coppelia.start_simulation()
            # time.sleep(1.0)

            coppelia.stop_simulation()
            time.sleep(0.5)

            coppelia.start_simulation()
            time.sleep(1.0)

            fitness = evaluate_individual(robot, genome)
            fitnesses.append(fitness)

            print(f"Fitness individuo {i + 1}: {fitness:.2f}")

            if fitness > best_fitness:
                best_fitness = fitness
                best_genome = genome.copy()
                np.save("best_wall_follow_genome.npy", best_genome)
                print(f"Nuevo mejor individuo guardado. Fitness = {best_fitness:.2f}")

            time.sleep(0.5)

        fitnesses = np.array(fitnesses)

        print(f"Mejor fitness generación: {fitnesses.max():.2f}")
        print(f"Fitness medio generación: {fitnesses.mean():.2f}")
        print(f"Mejor fitness global: {best_fitness:.2f}")

        population = create_next_generation(population, fitnesses)

    robot.set_speed(0.0, 0.0)
    coppelia.stop_simulation()

    print("\nEntrenamiento terminado.")
    print(f"Mejor fitness final: {best_fitness:.2f}")
    print("Mejor genoma guardado en: best_wall_follow_genome.npy")


if __name__ == '__main__':
    main()