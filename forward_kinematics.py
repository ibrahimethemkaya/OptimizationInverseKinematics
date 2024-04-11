import numpy as np

class DH:
    def __init__(self, r, alpha, d, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)

        self.matrix = np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, r * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, r * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])

    def get_matrix(self):
        return self.matrix
