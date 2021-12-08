from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2
import logging


def calc_metrics(intens):
    return {
        'mean_100': np.mean(intens[-100:])
    }


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.intens = []
        self.env = self.generated_task['env']
        self.side = 1
        self.logger = logging.getLogger("gym-duckietown")

    def solve(self):
        img, _, _, _ = self.env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = self.env.step([1, 0])
            self.calc_yellow_int(img)
            if (self.side == 1) and (np.mean(self.intens[-50:]) > 135e4):
                self.change_side(self.side / 3)
                self.side = -1
            elif (self.side == -1) and (np.mean(self.intens[-1]) > 22e5):
                self.straight(30, 10)
                self.change_side(self.side / 3)
                self.side = -2
                self.logger.warning(self.intens)
                condition = False
            self.env.render()
        self.straight(30, 10)
        self.env.step([0, 0])

    def change_side(self, k):
        self.turn(100, k * 1)
        self.straight(60, 1)
        self.turn(100, k * -1)

    def calc_yellow_int(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower = np.array([0, 10, 85])
        upper = np.array([60, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        yellow_int = mask.sum()
        self.intens.append(yellow_int)
        self.logger.warning(calc_metrics(self.intens))

        return yellow_int

    def straight(self, steps, k):
        for _ in range(steps):
            self.env.render()
            img, _, _, _ = self.env.step(np.array([k * 0.1, 0]))
            self.calc_yellow_int(img)

    def turn(self, steps, k):
        for _ in range(steps):
            self.env.render()
            img, _, _, _ = self.env.step(np.array([0, k * 1]))
            self.calc_yellow_int(img)
