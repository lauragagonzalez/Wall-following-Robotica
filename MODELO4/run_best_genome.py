"""
run_best_neuro_wall_follow.py

Ejecuta el mejor genoma obtenido por neuroevolución.
"""

import robotica
import time
import numpy as np


DT = 0.05

D_REF = 0.2

MAX_SENSOR_DIST = 1.0

BASE_SPEED = 1.2
MAX_SPEED = 3.0
MIN_SPEED = -3.0

N_INPUTS = 5
N_HIDDEN = 6
N_OUTPUTS = 2

GENOME_SIZE = (
    N_INPUTS * N_HIDDEN
    + N_HIDDEN
    + N_HIDDEN * N_OUTPUTS
    + N_OUTPUTS
)


def clamp(value, low, high):
    return max(low, min(value, high))


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


def normalize_distance(d):
    if d is None:
        d = MAX_SENSOR_DIST

    d = clamp(d, 0.0, MAX_SENSOR_DIST)
    return d / MAX_SENSOR_DIST


def get_inputs(readings):
    front_left_corner = readings[0]
    front = min(readings[2], readings[3])
    front_right_corner = readings[5]
    right_front = readings[6]
    right = readings[7]

    return [
        normalize_distance(front_left_corner),
        normalize_distance(front),
        normalize_distance(front_right_corner),
        normalize_distance(right_front),
        normalize_distance(right),
    ]


def neural_policy(genome, sensors):
    W1, b1, W2, b2 = decode_genome(genome)

    x = np.array(sensors, dtype=float)

    h = np.tanh(x @ W1 + b1)
    y = np.tanh(h @ W2 + b2)

    forward = 1.0 + 0.8 * y[0]
    turn = 1.6 * y[1]

    left_speed = forward - turn
    right_speed = forward + turn

    left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

    return left_speed, right_speed


def main(args=None):
    genome = np.load("best_wall_follow_genome.npy")

    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')

    coppelia.start_simulation()
    time.sleep(1.0)

    while coppelia.is_running():
        readings = robot.get_sonar()

        inputs = get_inputs(readings)

        left_speed, right_speed = neural_policy(genome, inputs)

        robot.set_speed(left_speed, right_speed)

        time.sleep(DT)

    robot.set_speed(0.0, 0.0)
    coppelia.stop_simulation()


if __name__ == "__main__":
    main()