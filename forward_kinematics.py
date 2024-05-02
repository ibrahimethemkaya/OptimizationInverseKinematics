import numpy as np

class DH:
    def __init__(self, r, alpha, d, theta):
        cos_theta = np.cos(np.deg2rad(theta))
        sin_theta = np.sin(np.deg2rad(theta))
        cos_alpha = np.cos(np.deg2rad(alpha))
        sin_alpha = np.sin(np.deg2rad(alpha))

        self.matrix = np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, r * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, r * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])

    def get_matrix(self):
        #this for loops convert each number in the matrix to two decimal format
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[i])):
                self.matrix[i][j] = round(self.matrix[i][j], 4)
        return self.matrix
