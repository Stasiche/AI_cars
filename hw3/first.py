from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.intens = []

    def solve(self):
        env = self.generated_task['env']
        img, _, _, _ = env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            self.calc_yellow_int(img)
            condition = np.mean(self.intens) < 16e5
            env.render()
        env.step([0, 0])

    def calc_yellow_int(self, img):
        global intens
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        lower = np.array([0, 10, 85])
        upper = np.array([60, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        yellow_int = mask.sum()
        self.intens.append(yellow_int)

        return yellow_int
