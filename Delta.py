from math import sqrt, atan2, pi

import numpy as np
from scipy.optimize import fsolve

from Robot import Robot, WorkspaceViolation


class Delta(Robot):
    def __init__(self, stepper_mode=1 / 32, steps_per_rev=200, step_delay=0.0208):
        super().__init__(dof=3, stepper_mode=stepper_mode, steps_per_rev=steps_per_rev, step_delay=step_delay,
                         rot_comp=np.array([-1] * 3))
        # [R, r, l1, l2]
        # R - distance from base center to driven joints
        # r - distance from end-effector center to joints
        # l1 - length of bar between driven joint and first undriven joint
        # l2 - length of bar between first and second undriven joint
        self.geometricParams = [41.7, 27.6, 48.6, 166.8]

    def inv_kinematic(self, pose: list):
        R, r, l1, l2 = self.geometricParams
        a = R - r

        # Cartesian target
        x = float(pose[0])
        y = float(pose[1])
        z = -float(pose[2])

        # Auxiliary variables
        G = [
            l1 * (2 * a + y + sqrt(3) * x),
            2 * l1 * (y + a),
            l1 * (2 * a - y + sqrt(3) * x)
        ]

        q = x ** 2 + y ** 2 + z ** 2 + l1 ** 2 + a ** 2 - l2 ** 2
        H = [
            q + a * (sqrt(3) * x - y),
            q + 2 * a * y,
            q - a * (sqrt(3) * x + y)
        ]

        F = - 2 * z * l1

        # Calculate driven joint angles
        thetas = []
        for i in range(3):
            denom = H[i] - G[i]
            try:
                p = sqrt(F ** 2 - H[i] ** 2 + G[i] ** 2)
            except ValueError as e:
                raise WorkspaceViolation from e
            theta1 = 2 * atan2((-F + p), denom)

            # Pick solution with arms pointing outwards
            if abs(theta1) <= pi / 2 :
                if theta1 < 0:
                    raise WorkspaceViolation
                elif theta1 > pi:
                    theta1 = theta1-2*pi

                thetas.append(theta1)
            else:
                theta2 = 2 * atan2((-F - p), denom)
                if theta2 < 0:
                    raise WorkspaceViolation
                elif theta2 > pi:
                    theta2 = theta2-2*pi

                thetas.append(theta2)

        return thetas

    def forward_kinematic(self, angles):
        angles_as_np_array = np.array(angles)  # convert to numpy array to subtract from another array

        # create function to minimize
        def func(x, h1=angles_as_np_array):
            """This function returns the difference between the current position (`H1`) and a guess (`X`).
            It is used for the numeric fsolve."""

            motor_angles = self.inv_kinematic(x)
            h2 = np.array(motor_angles)

            difference = h1 - h2  # calculate the difference between calulated and real angles

            return difference

        # initial guess/startingvalue
        # (first arm is pointing downward, pythagoras for second arm and effector/base radii)
        R, r, l1, l2 = self.geometricParams
        z = l1 + sqrt(l2 ** 2 - (R - r) ** 2)
        x_0 = np.array([0.0, 0.0, -z])

        curr_pose = fsolve(func, x_0)  # solve numerically with initial guess

        return list(curr_pose)

    def change_robot_dimensions(self, R, r, l1, l2, *_):
        self.geometricParams = [R, r, l1, l2]


if __name__ == '__main__':
    R, r, l1, l2 = [41.7, 27.6, 48.6, 166.8]
    a = R - r

    # Cartesian target
    x = 0
    y = 0
    z = l1 + sqrt(l2 ** 2 - a ** 2)

    # Auxiliary variables
    G = [
        l1 * (2 * a + y + sqrt(3) * x),
        2 * l1 * (y + a),
        l1 * (2 * a + sqrt(3) * x - y)
    ]

    q = x ** 2 + y ** 2 + z ** 2 + l1 ** 2 + a ** 2 - l2 ** 2
    H = [
        q + a * (sqrt(3) * x - y),
        q + 2 * a * y,
        q - a * (sqrt(3) * x + y)
    ]

    F = - 2 * z * l1

    # Calculate driven joint angles
    thetas = []
    for i in range(3):
        denom = H[i] - G[i]
        try:
            p = sqrt(F ** 2 - H[i] ** 2 + G[i] ** 2)
        except ValueError:
            continue
        theta1 = 2 * atan2((-F + p), denom)

        # Pick solution with arms pointing outwards
        if abs(theta1) <= pi / 2:
            thetas.append(theta1)
        else:
            theta2 = 2 * atan2((-F - p), denom)
            thetas.append(theta2)

    print(thetas)
