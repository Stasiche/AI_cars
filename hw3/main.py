import time
import gym

import pyglet
from pyglet.window import key

import numpy as np
from gym_duckietown.envs import DuckietownEnv

env = DuckietownEnv(
    seed=1,
    map_name='straight_road',
    draw_curve=False,
    draw_bbox=False,
    domain_rand=False,
    frame_skip=1,
    distortion=False,
    camera_rand=False,
    dynamics_rand=False,
)


def straight(steps: int, k: int):
    for _ in range(steps):
        env.step(np.array([k * 0.1, 0]))
        env.render()


def turn(steps: int, k: int):
    for _ in range(steps):
        img, _, _, _ = env.step(np.array([0, k * 1]))
        env.render()


env.reset()
img, _, _, _ = env.step(np.array([0, 0]))

straight(20, 1)
turn(100, 1)
straight(125, 1)
turn(100, -1)
straight(500, 1)

env.close()
