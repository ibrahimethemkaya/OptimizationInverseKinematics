from Inverse_Kinematics import forward_kinematics, find_position
import numpy as np
import math
import matplotlib.pyplot as plt

class Objective:
    def __init__(self, x_desired, y_desired, z_desired):
        self.x_desired = x_desired
        self.y_desired = y_desired
        self.z_desired = z_desired
        self.min_error = 100

    '''
    def brute_force_algorithm(self):

        self.best_angles = None

        for angle1 in range(0, 30, 1):
            for angle2 in range(0, 30, 1):
                for angle3 in range(0, 30, 1):
                    for angle4 in range(0, 30, 1):
                        actual_position = find_position.EndPosition(angle1, angle2, angle3, angle4)
                        print("current angles: ", angle1, angle2, angle3, angle4)
                        final_matrix = actual_position.get_final_matrix()
                        print("current matrix: \n", final_matrix)

                        x_actual = final_matrix[0, 3]
                        y_actual = final_matrix[2, 3]
                        z_actual = final_matrix[1, 3]

                        x_error = self.x_desired - x_actual
                        y_error = self.y_desired - y_actual
                        z_error = self.z_desired - z_actual
                        errors = np.array([x_error, y_error, z_error])

                        # Euclidean norm = sqrt(x_error^2 + y_error^2 + z_error^2)
                        total_error = np.linalg.norm(errors)
                        print("total error : ", total_error)
                        print("min error : ", self.min_error)

                        # Compare new error with min error
                        if total_error < self.min_error:
                            self.min_error = total_error
                            self.best_angles = (angle1, angle2, angle3, angle4)
        print("best angles : ", self.best_angles)
        return self.min_error, self.best_angles
        '''

    def pso(self, num_particles=30, max_iterations=500, w=0.7, c1=1.5, c2=1.5):
        # Initialize particles
        lower_bounds = np.array([0, 0, 0, 0])
        upper_bounds = np.array([90, 90, 90, 90])
        particles_position = np.random.uniform(low=lower_bounds, high=upper_bounds, size=(num_particles, 4))
        particles_velocity = np.random.uniform(low=-1, high=1, size=(num_particles, 4))

        pBest_position = particles_position.copy()
        pBest_value = np.array([self.objective_func(p) for p in particles_position])

        gBest_position = pBest_position[np.argmin(pBest_value)]
        gBest_value = np.min(pBest_value)

        for iteration in range(max_iterations):
            for i in range(num_particles):
                r1 = np.random.rand(4)
                r2 = np.random.rand(4)
                particles_velocity[i] = (
                    w * particles_velocity[i]
                    + c1 * r1 * (pBest_position[i] - particles_position[i])
                    + c2 * r2 * (gBest_position - particles_position[i])
                )
                particles_position[i] = particles_position[i] + particles_velocity[i]
                particles_position[i] = np.clip(particles_position[i], lower_bounds, upper_bounds)
                particles_position[i] = np.round(particles_position[i])

                current_value = self.objective_func(particles_position[i])

                if current_value < pBest_value[i]:
                    pBest_position[i] = particles_position[i]
                    pBest_value[i] = current_value

                if current_value < gBest_value:
                    gBest_position = particles_position[i]
                    gBest_value = current_value

            print(f"Iteration {iteration+1}/{max_iterations}, Best Error: {gBest_value}")

        self.gBest_position = gBest_position
        self.min_error = gBest_value
        return gBest_position, gBest_value

    def objective_func(self, p):
        self.actual_position = find_position.EndPosition(p[0],p[1],p[2],p[3])
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
        return self.total_error

    def simulate(self):
        time_steps = np.arange(0, 1.1, 0.1)
        x_positions = []
        y_positions = []
        z_positions = []

        for t in time_steps:
            # Adjust angles over time (example: linearly scaling with time)
            adjusted_angles = self.gBest_position * t
            actual_position = find_position.EndPosition(*adjusted_angles)
            actual_position_matrix = actual_position.get_final_matrix()
            x_actual = actual_position_matrix[0, 3]
            y_actual = actual_position_matrix[2, 3]
            z_actual = actual_position_matrix[1, 3]

            x_positions.append(x_actual)
            y_positions.append(y_actual)
            z_positions.append(z_actual)

        return time_steps, x_positions, y_positions, z_positions

# Desired position
x_desired = 14
y_desired = 16.8
z_desired = 15.6

# Create Objective instance
objective = Objective(x_desired, y_desired, z_desired)

# Run PSO
optimal_joint_angles, min_error = objective.pso()

print("Optimal joint angles:", optimal_joint_angles)
print("Minimum error:", min_error)

# Simulate and collect positions over time
time_steps, x_positions, y_positions, z_positions = objective.simulate()

# Plotting
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_steps, x_positions, 'r-', label='x(t)')
plt.xlabel('Time (s)')
plt.ylabel('x position')
plt.title('Time vs x Position')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_steps, y_positions, 'g-', label='y(t)')
plt.xlabel('Time (s)')
plt.ylabel('y position')
plt.title('Time vs y Position')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_steps, z_positions, 'b-', label='z(t)')
plt.xlabel('Time (s)')
plt.ylabel('z position')
plt.title('Time vs z Position')
plt.legend()

plt.tight_layout()
plt.show()