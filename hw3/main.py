import time
import gym

import pyglet
from pyglet.window import key
from recoder import VideoRecorder

import numpy as np
from gym_duckietown.envs import DuckietownEnv

def straight(steps: int, k: int):
    for _ in range(steps):
        env.step(np.array([k * 0.1, 0]))
        env.render()


def turn(steps: int, k: int):
    for _ in range(steps):
        img, _, _, _ = env.step(np.array([0, k * 1]))
        env.render()


def change_side(k):
    turn(100, k * 1)
    straight(125, 1)
    turn(100, k * -1)


def go_around():
    change_side(1)
    straight(50, 1)
    change_side(-1)


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

env = VideoRecorder(env, './results')


env.reset()
img, _, _, _ = env.step(np.array([0, 0]))
straight(20, 1)
go_around()
straight(20, 1)
env.close()
