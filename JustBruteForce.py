import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Inverse_Kinematics import forward_kinematics, find_position

class Objective:
    def __init__(self, x_desired, y_desired, z_desired):
        self.x_desired = x_desired
        self.y_desired = y_desired
        self.z_desired = z_desired
        self.min_error = math.sqrt(x_desired**2 + y_desired**2 + z_desired**2)
        self.data = []

    def brute_force_algorithm(self, find_position):
        best_angles = None

        for angle1 in range(0, 6, 1):
            for angle2 in range(0, 6, 1):
                for angle3 in range(0, 6, 1):
                    for angle4 in range(0, 6, 1):
                        actual_position = find_position.EndPosition(angle1, angle2, angle3, angle4)
                        final_matrix = actual_position.get_final_matrix()

                        x_actual = final_matrix[0, 3]
                        y_actual = final_matrix[2, 3]
                        z_actual = final_matrix[1, 3]

                        x_error = self.x_desired - x_actual
                        y_error = self.y_desired - y_actual
                        z_error = self.z_desired - z_actual
                        errors = np.array([x_error, y_error, z_error])

                        # Euclidean norm = sqrt(x_error^2 + y_error^2 + z_error^2)
                        total_error = np.linalg.norm(errors)

                        # Store the data
                        self.data.append((angle1, angle2, angle3, angle4, total_error))

                        # Compare new error with min error
                        if total_error < self.min_error:
                            self.min_error = total_error
                            best_angles = (angle1, angle2, angle3, angle4)

        return best_angles, self.min_error

    def objective_func(self, find_position):
        best_angles, min_error = self.brute_force_algorithm(find_position)
        print("Best angles:", best_angles)
        print("Minimum error:", min_error)
        return best_angles, min_error

    def visualize_data(self):
        angles = np.array([d[:4] for d in self.data])
        errors = np.array([d[4] for d in self.data])

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Scatter plot of the errors
        sc = ax.scatter(angles[:, 0], angles[:, 1], angles[:, 2], c=errors, cmap='viridis')
        plt.colorbar(sc)

        ax.set_xlabel('Angle 1')
        ax.set_ylabel('Angle 2')
        ax.set_zlabel('Angle 3')
        ax.set_title('Error Visualization for Brute Force')

        plt.show()

# Example of how to use the Objective class:
# find_position = ...  # This should be the object that has the EndPosition method
obj = Objective(24.6, 13.2, 3.9)
best_angles, min_error = obj.objective_func(find_position)
obj.visualize_data()
