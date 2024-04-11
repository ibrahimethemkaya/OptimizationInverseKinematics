from Inverse_Kinematics import forward_kinematics, find_position
import numpy as np

class Objective:
    def __init__(self, x_desired, y_desired, z_desired):
       self.x_desired = x_desired
       self.y_desired = y_desired
       self.z_desired = z_desired

    def objective_func(self):
        self.actual_position = find_position.EndPosition(0,0,0,0)
        self.actual_position_matrix = self.actual_position.get_final_matrix()
        self.x_actual = self.actual_position_matrix[0, 3]
        self.y_actual = self.actual_position_matrix[2, 3]
        self.z_actual = self.actual_position_matrix[1, 3]
        self.x_error = self.x_desired - self.x_actual
        self.y_error = self.y_desired - self.y_actual
        self.z_error = self.z_desired - self.z_actual

        errors = np.array([self.x_error, self.y_error, self.z_error])

        # Euclidean norm = sqrt(x_error^2 + y_error^2 + z_error^2)
        sum_error = np.linalg.norm(errors)
        return sum_error

obj = Objective(28.935,13.1179,0)
print(obj.objective_func())