
import numpy as np
import pygame
import sys
import math
import random

import beaconwrapper as bw

from Vec2D import Vec2D

PX_PER_METER = 400
WIDTH = int(3.0 * PX_PER_METER)
HEIGHT = int(2.0 * PX_PER_METER)
pygame.init()
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
PURPLE = (255, 0, 255)

POINT_A = Vec2D(3.0, 1.0)
POINT_B = Vec2D(0.0, 2.0)
POINT_C = Vec2D(0.0, 0.0)

MAX_V = 1.6

FRAME_RATE = 50

MEAS_STD_X = 0.03
MEAS_STD_Y = 0.03
MEAS_COV_XY = 0.0

MEAS_FREQ = 15   # Hz

def draw_state(state, color):
    "draw a position and the covariance around it"
    (pos_x, pos_y, var_x, var_y, cov_xy) = state

    evals, evecs = np.linalg.eig(np.array([[var_x, cov_xy], [cov_xy, var_y]]))

    if evals[0] > evals[1]:
        major = 2.0*math.sqrt(5.991 * evals[0])
        minor = 2.0*math.sqrt(5.991 * evals[1])
        angle = math.atan2(evecs[:, 0][1], evecs[:, 0][0])
    else:
        major = 2.0*math.sqrt(5.991 * evals[1])
        minor = 2.0*math.sqrt(5.991 * evals[0])
        angle = math.atan2(evecs[:, 1][1], evecs[:, 1][0])

    px_pos_x = int(pos_x * PX_PER_METER)
    px_pos_y = HEIGHT - int(pos_y * PX_PER_METER)
    px_major = int(major * PX_PER_METER)
    px_minor = int(minor * PX_PER_METER)

    if px_major > 5:
        size = (0, 0, px_major, px_minor)

        e_surface = pygame.Surface((px_major, px_minor))

        pygame.draw.ellipse(e_surface, color, size, 1)

        e_surface = pygame.transform.rotate(e_surface, angle * (180.0 / math.pi))

        center_x = px_pos_x - (e_surface.get_width() / 2)
        center_y = px_pos_y - (e_surface.get_height() / 2)

        SCREEN.blit(e_surface, (center_x, center_y))

    pygame.draw.circle(SCREEN, color, (px_pos_x, px_pos_y), 0, 0)
    pygame.draw.circle(SCREEN, color, (px_pos_x, px_pos_y), 5, 1)

def angles_to_triangle(pos):
    v_pa = POINT_A - pos
    v_pb = POINT_B - pos
    v_pc = POINT_C - pos

    alpha = v_pb.directed_angle(v_pc)
    beta = v_pc.directed_angle(v_pa)
    gamma = v_pa.directed_angle(v_pb)

    return (alpha, beta, gamma)

def update_speed(speed, acc, delta_t):
    if acc.length() < Vec2D.EPSILON:
        n_speed = speed
    else:
        n_speed = speed + (acc.normalized() * delta_t)

    if n_speed.length() > MAX_V:
        return n_speed.normalized() * MAX_V
    else:
        return n_speed

def update_pos(pos, speed, delta_t):
    n_pos = pos + speed * delta_t

    if n_pos.x < 0.0:
        n_pos.x = 0.0
    if n_pos.x > 3.0:
        n_pos.x = 3.0
    if n_pos.y < 0.0:
        n_pos.y = 0.0
    if n_pos.y > 2.0:
        n_pos.y = 2.0

    return n_pos

def main():
    "main..."

    paused = False
    measure = True

    pos = Vec2D(1.0, 1.0)
    speed = Vec2D(0.0, 0.0)

    time_between_measurements = 1 / MEAS_FREQ
    accumulated_time = 0.0

    bw.setup(pos.x, pos.y)

    bw.update_meas_cov(MEAS_STD_X**2, MEAS_STD_Y**2, MEAS_COV_XY)

    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    paused = not paused
                if event.key == pygame.K_m:
                    pass
                    #measure = not measure

        pressed_key = pygame.key.get_pressed()
        acc = Vec2D(0, 0)
        if pressed_key[pygame.K_w]:
            acc = acc + Vec2D(0.0, 1.0)
        if pressed_key[pygame.K_a]:
            acc = acc + Vec2D(-1.0, 0.0)
        if pressed_key[pygame.K_s]:
            acc = acc + Vec2D(0.0, -1.0)
        if pressed_key[pygame.K_d]:
            acc = acc + Vec2D(1.0, 0.0)

        delta_t = float(clock.tick(FRAME_RATE)) / 1000

        accumulated_time = accumulated_time + delta_t

        if accumulated_time > time_between_measurements:
            accumulated_time = accumulated_time - time_between_measurements
            measure = True
        else:
            measure = False

        if not paused:
            SCREEN.fill(BLACK)

            speed = update_speed(speed, acc, delta_t)
            pos = update_pos(pos, speed, delta_t)
            err_pos = Vec2D(random.gauss(0, MEAS_STD_X), random.gauss(0, MEAS_STD_Y))
            err_pos += pos

            angles = angles_to_triangle(err_pos)
            kalman_state = bw.next_state(
                angles[0],
                angles[1],
                angles[2],
                delta_t,
                measure
            )

            draw_state(kalman_state, RED)
            draw_state((pos.x, pos.y, 0, 0, 0), BLUE)


            pygame.display.update()

if __name__ == "__main__":
    main()

