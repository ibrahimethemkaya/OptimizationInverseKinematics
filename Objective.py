from Inverse_Kinematics import forward_kinematics, find_position
import numpy as np
import math

class Objective:
    def __init__(self, x_desired, y_desired, z_desired):
       self.x_desired = x_desired
       self.y_desired = y_desired
       self.z_desired = z_desired
       self.min_error = math.sqrt(self.x_desired**2 + self.y_desired**2 + self.z_desired**2)

    def objective_func(self):
        #brut force algorithm
        for angle1 in range(0, 6, 1):
            for angle2 in range(0, 6, 1):
                for angle3 in range(0, 6, 1):
                    for angle4 in range(0, 6, 1):
                        self.actual_position = find_position.EndPosition(angle1, angle2, angle3, angle4)
                        print(angle1, angle2, angle3, angle4)
                        print(self.actual_position.get_final_matrix())
                        self.actual_position_matrix = self.actual_position.get_final_matrix()
                        self.x_actual = self.actual_position_matrix[0, 3]
                        self.y_actual = self.actual_position_matrix[2, 3]
                        self.z_actual = self.actual_position_matrix[1, 3]
                        self.x_error = self.x_desired - self.x_actual
                        self.y_error = self.y_desired - self.y_actual
                        self.z_error = self.z_desired - self.z_actual
                        self.errors = np.array([self.x_error, self.y_error, self.z_error])

                        # Euclidean norm = sqrt(x_error^2 + y_error^2 + z_error^2)
                        self.total_error = np.linalg.norm(self.errors)
                        print(self.total_error)


                        print(self.min_error)
                        # Compare new error with min error
                        if self.total_error < self.min_error:
                            self.min_error = self.total_error

#when angles are equals to '5' degree, robot in this position
obj = Objective(2.42667966e+01, 1.34389000e+01,5.57209716e+00)
obj.objective_func()

