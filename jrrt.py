
import numpy as np
import matplotlib.pyplot as plt


class J_RRT:
    def __init__(self, robot, goal, step_size=0.02, max_iter=1000, delta=0.01):
        self.robot = robot
        self.goal = goal
        self.step_size = step_size
        self.max_iter = max_iter
        self.delta = delta
        self.nodes = [robot.ee_position()]

    def dist(self, p1, p2):
        return np.linalg.norm(p2 - p1)

    def generate_random_point(self):
        return self.goal._pos   # Goal-biased sampling

    def nearest_neighbor(self, point):
        distances = [self.dist(point, node) for node in self.nodes]
        min_index = np.argmin(distances)
        return self.nodes[min_index]

    def new_point(self, q_nearest, q_rand):
        direction = q_rand - q_nearest
        distance = np.linalg.norm(direction)
        if distance <= self.step_size:
            return q_rand
        return q_nearest + (direction / distance) * self.step_size

    def plan_path(self):
        for _ in range(self.max_iter):
            q_rand = self.generate_random_point()
            q_nearest = self.nearest_neighbor(q_rand)
            q_new = self.new_point(q_nearest, q_rand)

            if not self.robot.in_collision():
                self.nodes.append(q_new)
                if self.dist(q_new, self.goal._pos) < self.delta:
                    return True, self.nodes

                # Move robot towards q_new using Jacobian Pseudoinverse
                J_inv = np.linalg.pinv(self.robot.get_jacobian())
                delta_position = q_new - self.robot.ee_position()
                delta_joint_angles = np.dot(J_inv, delta_position)
                new_position = self.robot.get_joint_pos() + delta_joint_angles
                self.robot.reset_joint_pos(new_position)

        return False, self.nodes
