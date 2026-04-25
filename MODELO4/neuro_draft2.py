"""
wall_follow_neuroevolution_multi_pose.py

Neuroevolución para wall following:
cada individuo se evalúa desde varias posiciones iniciales.
"""

import robotica
import time
import numpy as np


# =========================
# Parámetros generales
# =========================

DT = 0.05
EPISODE_TIME = 6.0

D_REF = 0.2

MAX_SENSOR_DIST = 1.0
COLLISION_DIST = 0.12

BASE_SPEED = 1.2
TURN_GAIN = 1.2

MAX_SPEED = 3.0
MIN_SPEED = -3.0


# =========================
# Neuroevolución
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


# =========================
# Posiciones iniciales
# =========================
# Formato: x, y, z, theta
# Ajusta estas coordenadas a tu escena.

INITIAL_POSES = [
    (1.46903, 1.50907, 0.13875, -1.57),
    (-1.55597, 0.30907, 0.13875, -1.57),
    (1.53053, -0.850, 0.13879, 0.0)
]


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

    left_speed = BASE_SPEED + TURN_GAIN * y[0]
    right_speed = BASE_SPEED + TURN_GAIN * y[1]

    left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

    return left_speed, right_speed


# =========================
# Sensores
# =========================

def normalize_distance(d):
    if d is None:
        d = MAX_SENSOR_DIST

    d = clamp(d, 0.0, MAX_SENSOR_DIST)
    return d / MAX_SENSOR_DIST


def get_inputs(readings):
    """
    Sensores según lo que me dijiste:
    sensores frontales: 1 al 6
    sensores derecha: 7 y 8

    En Python:
    sensor 1 -> readings[0]
    sensor 8 -> readings[7]
    """

    front_left_corner = readings[0]          # sensor 1
    front = min(readings[2], readings[3])    # sensores 3 y 4
    front_right_corner = readings[5]         # sensor 6
    right_front = readings[6]                # sensor 7
    right = readings[7]                      # sensor 8

    return [
        normalize_distance(front_left_corner),
        normalize_distance(front),
        normalize_distance(front_right_corner),
        normalize_distance(right_front),
        normalize_distance(right),
    ]


def get_relevant_distances(readings):
    front_dist = min(
        readings[0],
        readings[1],
        readings[2],
        readings[3],
        readings[4],
        readings[5],
    )

    right_dist = min(
        readings[6],
        readings[7],
    )

    front_dist = clamp(front_dist, 0.0, MAX_SENSOR_DIST)
    right_dist = clamp(right_dist, 0.0, MAX_SENSOR_DIST)

    return front_dist, right_dist


# =========================
# Reset de simulación
# =========================

def reset_robot(coppelia, pose):
    """
    Reinicia la simulación y recoloca el robot.

    Si tu API no permite setObjectPosition / setObjectOrientation,
    comenta esa parte y deja solo stop/start.
    """

    x, y, z, theta = pose

    coppelia.stop_simulation()
    time.sleep(0.5)

    coppelia.start_simulation()
    time.sleep(0.8)

    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    try:
        robot_handle = coppelia.sim.getObject('/PioneerP3DX')

        coppelia.sim.setObjectPosition(
            robot_handle,
            -1,
            [x, y, z]
        )

        coppelia.sim.setObjectOrientation(
            robot_handle,
            -1,
            [0.0, 0.0, theta]
        )

        time.sleep(0.2)

    except Exception as e:
        print("Aviso: no se pudo recolocar el robot automáticamente.")
        print("Se usará solo reinicio de simulación.")
        print(e)

    return robot


# =========================
# Evaluación
# =========================

def evaluate_trial(robot, genome):
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

        # Sobrevivir suma
        fitness += 1.0

        # Choque penaliza mucho
        if front_dist < COLLISION_DIST or right_dist < COLLISION_DIST:
            fitness -= 150.0
            break

        # Avanzar suma
        forward_speed = (left_speed + right_speed) / 2.0

        if forward_speed > 0:
            fitness += 2.0 * forward_speed
        else:
            fitness -= 5.0

        # Mantener distancia a pared derecha
        wall_error = abs(D_REF - right_dist)
        fitness -= 8.0 * wall_error

        # Si no hay pared derecha, penaliza un poco
        if right_dist > 0.7:
            fitness -= 2.0

        # Penaliza giros demasiado bruscos
        turning = abs(left_speed - right_speed)
        fitness -= 0.3 * turning

        # Penaliza cambios bruscos entre pasos
        speed_change = (
            abs(left_speed - previous_left_speed)
            + abs(right_speed - previous_right_speed)
        )
        fitness -= 0.1 * speed_change

        previous_left_speed = left_speed
        previous_right_speed = right_speed

        time.sleep(DT)
        elapsed += DT

    robot.set_speed(0.0, 0.0)
    return fitness


def evaluate_individual(coppelia, genome):
    trial_fitnesses = []

    for pose in INITIAL_POSES:
        robot = reset_robot(coppelia, pose)
        fitness = evaluate_trial(robot, genome)
        trial_fitnesses.append(fitness)

    return np.mean(trial_fitnesses)


# =========================
# Evolución
# =========================

def mutate(genome):
    child = genome.copy()
    child += np.random.randn(GENOME_SIZE) * MUTATION_STD
    return child


def create_next_generation(population, fitnesses):
    sorted_indices = np.argsort(fitnesses)[::-1]

    elites = [
        population[i].copy()
        for i in sorted_indices[:ELITE_SIZE]
    ]

    new_population = []

    for elite in elites:
        new_population.append(elite)

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

    coppelia.start_simulation()
    time.sleep(1.0)

    population = [create_individual() for _ in range(POP_SIZE)]

    best_genome = None
    best_fitness = -float("inf")

    for generation in range(N_GENERATIONS):
        print(f"\n===== GENERACIÓN {generation + 1}/{N_GENERATIONS} =====")

        fitnesses = []

        for i, genome in enumerate(population):
            print(f"\nEvaluando individuo {i + 1}/{POP_SIZE}...")

            fitness = evaluate_individual(coppelia, genome)
            fitnesses.append(fitness)

            print(f"Fitness medio individuo {i + 1}: {fitness:.2f}")

            if fitness > best_fitness:
                best_fitness = fitness
                best_genome = genome.copy()
                np.save("best_wall_follow_genome.npy", best_genome)
                print(f"Nuevo mejor individuo guardado. Fitness = {best_fitness:.2f}")

        fitnesses = np.array(fitnesses)

        print("\nResumen generación:")
        print(f"Mejor fitness generación: {fitnesses.max():.2f}")
        print(f"Fitness medio generación: {fitnesses.mean():.2f}")
        print(f"Mejor fitness global: {best_fitness:.2f}")

        population = create_next_generation(population, fitnesses)

    coppelia.stop_simulation()

    print("\nEntrenamiento terminado.")
    print(f"Mejor fitness final: {best_fitness:.2f}")
    print("Genoma guardado en: best_wall_follow_genome.npy")


if __name__ == '__main__':
    main()