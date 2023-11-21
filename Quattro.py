from math import atan2, sqrt, cos, sin, pi
from operator import inv

import numpy as np
from kinematics import for_kinematic, inv_kinematic
from scipy.optimize import fsolve
from math import radians as rad
from math import degrees as deg

from Robot import Robot, WorkspaceViolation


class Quattro(Robot):
    def __init__(self, stepper_mode=1 / 32, steps_per_rev=200, step_delay=0.0208):
        super().__init__(dof=4, stepper_mode=stepper_mode, steps_per_rev=steps_per_rev, step_delay=step_delay,
                         rot_comp=np.array([-1] * 4))
        # [R, a, l1, l2]
        # R - distance from base center to driven joints
        # a - length of an end-effector bar
        # l1 - length of bar between driven joint and first undriven joint
        # l2 - length of bar between first and second undriven joint
        self.geometricParams = [55.6, 39.3, 75.8, 166.8]

    def inv_kinematic(self, pose: list):
        R, a, l1, l2 = self.geometricParams

        # Cartesian target
        x = float(pose[0])
        y = float(pose[1])
        z = -float(pose[2])
        phi = float(pose[3])

        # Calculate trigonometrics only once
        c_phi = cos(phi)
        s_phi = sin(phi)

        # Auxiliary variables
        G = [
            2 * l1 * (x - a * c_phi + R),
            2 * l1 * (y - a * s_phi + R),
            2 * l1 * (-x - a * c_phi + R),
            2 * l1 * (-y - a * s_phi + R)
        ]

        q = x ** 2 + y ** 2 + z ** 2 + l1 ** 2 + R ** 2 - l2 ** 2
        H = [
            q + (a * c_phi) ** 2 - 2 * a * c_phi * (x + R) + 2 * x * R,
            q + (a * s_phi) ** 2 - 2 * a * s_phi * (y + R) + 2 * y * R,
            q + (a * c_phi) ** 2 + 2 * a * c_phi * (x - R) - 2 * x * R,
            q + (a * s_phi) ** 2 + 2 * a * s_phi * (y - R) - 2 * y * R,
        ]

        F = - 2 * z * l1

        # Calculate driven joint angles
        thetas = []
        for i in range(4):
            denom = H[i] - G[i]
            try:
                p = sqrt(F ** 2 - H[i] ** 2 + G[i] ** 2)
            except ValueError as e:
                raise WorkspaceViolation from e
            theta1 = 2 * atan2((-F + p), denom)

            # Pick solution with arms pointing outwards
            if abs(theta1) <= pi / 2 :
                if theta1 < rad(3):
                    raise WorkspaceViolation
                elif theta1 > pi:
                    theta1 = theta1-2*pi

                thetas.append(theta1)
            else:
                theta2 = 2 * atan2((-F - p), denom)
                if theta2 < -pi:
                    theta2 = theta2 + 2*pi
                if theta2 < rad(3):
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
        R, a, l1, l2 = self.geometricParams
        z = l1 + sqrt(l2 ** 2 - (R - a / sqrt(2)) ** 2)
        x_0 = np.array([0.0, 0.0, -z, pi / 4])

        curr_pose = fsolve(func, x_0)  # solve numerically with initial guess

        return list(curr_pose)

    def change_robot_dimensions(self, R, a, l1, l2, *_):
        self.geometricParams = [R, a, l1, l2]


if __name__ == '__main__':
    R, a, l1, l2 = [55.6, 39.3, 75.8, 166.8]

    # Cartesian target
    x = 0
    y = 0
    z = l1 + sqrt(l2 ** 2 - (R - a / sqrt(2)) ** 2) - 20
    phi = pi / 4

    # Calculate trigonometrics only once
    c_phi = cos(phi)
    s_phi = sin(phi)

    # Auxiliary variables
    G = [
        2 * l1 * (x - a * c_phi + R),
        2 * l1 * (y - a * s_phi + R),
        2 * l1 * (-x - a * c_phi + R),
        2 * l1 * (-y - a * s_phi + R)
    ]

    q = x ** 2 + y ** 2 + z ** 2 + l1 ** 2 + R ** 2 - l2 ** 2
    H = [
        q + (a * c_phi) ** 2 - 2 * a * c_phi * (x + R) + 2 * x * R,
        q + (a * s_phi) ** 2 - 2 * a * s_phi * (y + R) + 2 * y * R,
        q + (a * c_phi) ** 2 + 2 * a * c_phi * (x - R) - 2 * x * R,
        q + (a * s_phi) ** 2 + 2 * a * s_phi * (y - R) - 2 * y * R,
    ]

    F = - 2 * z * l1

    # Calculate driven joint angles
    thetas = []
    for i in range(4):
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
