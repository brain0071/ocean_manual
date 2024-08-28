import numpy as np

motor_max = 3.5
max_force = [158.4276, 158.4276, 158.4276]
max_moment = [54.6344, 59.6087, 57.6819]

motor_max = motor_max * 9.8
max_forceMoment = np.concatenate((max_force, max_moment))
motor_matrix = np.array([[0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736, 0.57736, -0.57736],  # x
                                          [-0.57736, -0.57736, 0.57736, 0.57736, 0.57736, 0.57736, -0.57736, -0.57736],  # y
                                          [0.57736, -0.57736, -0.57736, 0.57736, -0.57736, 0.57736, 0.57736, -0.57736],  # z
                                          [0.199105, 0.199105, 0.199105, 0.199105, -0.199105, -0.199105, -0.199105, -0.199105], # roll
                                          [-0.217233, 0.217233, 0.217233, -0.217233, -0.217233, 0.217233, 0.217233, -0.217233],
                                          [-0.210211, -0.210211, 0.210211, 0.210211, -0.210211, -0.210211, 0.210211, 0.210211]]).reshape((6, 8)) # yaw

max_motor_ = np.diag(np.array([motor_max, motor_max, motor_max, motor_max, motor_max,
                                            motor_max, motor_max, motor_max]))

next_control = [0.6, 0, 0, 0, 0, 0]
            # motor_percent = f_max(8*8)^-1 * A^-1 * tau_max(6*6) * u 
motor_ = np.dot(np.linalg.inv(max_motor_),
                            np.dot(np.linalg.pinv(motor_matrix), 
                                   np.dot(np.diag(max_forceMoment), next_control)))

print(np.linalg.inv(max_motor_))
print(np.linalg.pinv(motor_matrix))
print(np.diag(max_forceMoment))
print(next_control)
print(motor_)