from math import sin, cos, sqrt

import numpy as np
from scipy.optimize import fsolve

from Robot import Robot


class SixRUS(Robot):
    """Class for the 6-RUS-robot"""

    def __init__(self, stepper_mode=1 / 32, steps_per_rev=200, step_delay=0.0208):
        """Initialise the Robot
        `stepperMode`: float  Microstepmode e.g. 1/32, 1/16, 1/8; 1/4, 1/2 or 1
        `stepsPerRev`: int  How many Full-steps the motors have
        `stepDelay`: in [s]   SleepDelay between steps lower time allows for faster rotation but is
        more susceptible of missing steps
        """
        super().__init__(dof=6, stepper_mode=stepper_mode, steps_per_rev=steps_per_rev, step_delay=step_delay, rot_comp=np.array([1, -1] * 3))

        # Robot-Dimensions [mm]  (Hardcoded but can be changed via a function)
        # [l1, l2, dx, dy, Dx, Dy]  (more Infos in documentation)
        # self.geometricParams = [57.0, 92.0, 11.0, 9.5, 63.0, 12.0]  # small endeffector
        self.geometricParams = [58.0, 200.0, 23.6, 12.5, 50.0, 12.5]  # big endeffector

    # KINEMATICS
    def inv_kinematic(self, pose: list):
        """Inverse kinematics of 6-RUS robot:

        `pose`: list with numeric content

        `return`: list with all six motor-angles"""
        pose = pose[:self.dof]

        # convert all inputs to floats to be able to work with complex numbers
        x = float(pose[0])
        y = float(pose[1])
        z = float(pose[2])
        alpha = float(pose[3])
        beta = float(pose[4])
        gamma = float(pose[5])

        # Use given Robot dimensions
        l1, l2, dx, dy, Dx, Dy = self.geometricParams

        j = complex(0, 1)  # define complex numer (0 + i)

        # # TODO: manually implement the functions for theta 1-6 for possible time-optimisations
        # # The Caclulations can be found in the MATLAB-example
        # # Testimplementation for theta_1:
        # sig_4 = z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg)
        # sig_3 = Dx - x - dx*c_bg + dy*c_b*s_g
        # sig_2 = sig_4**2 + sig_3**2 + (y - Dy + dx*(c_a*s_g + c_g*s_ab) + dy*(c_ag - s_abg))**2 + l1**2 - l2**2
        # sig_1_squared = (sig_4**2 - (sig_2**2)/(4*l1**2) + sig_3**2)*sig_4**2

        # sig_1 = float(sig_1_squared)**0.5

        # yTeil = -(sig_1*sig_3 + (sig_4**2*sig_2)/(2*l1))/((sig_4**2 + sig_3**2)*sig_4)
        # xTeil = (sig_1 - (sig_3*sig_2)/(2*l1))/(sig_4**2 + sig_3**2)

        # theta_1_ = atan2(yTeil.real, xTeil.real)

        # calculate motor-angles
        s_a = sin(alpha)
        c_a = cos(alpha)
        s_b = sin(beta)
        c_b = cos(beta)
        s_g = sin(gamma)
        c_g = cos(gamma)
        s_ag = s_a * s_g
        s_bg = s_b * s_g
        s_ab = s_a * s_b
        s_abg = s_a * s_bg
        c_ag = c_a * c_g
        c_bg = c_b * c_g

        theta_1 = np.angle(((((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 - ((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2 + (y - Dy + dx*(c_a*s_g + c_g*s_ab) + dy*(c_ag - s_abg))**2 + l1**2 - l2**2)**2/(4*l1**2) + (Dx - x - dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2)**(1/2) - ((Dx - x - dx*c_bg + dy*c_b*s_g)*((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2 + (y - Dy + dx*(c_a*s_g + c_g*s_ab) + dy*(c_ag - s_abg))**2 + l1**2 - l2**2))/(2*l1))/((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2) - (((((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 - ((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2 + (y - Dy + dx*(c_a*s_g + c_g*s_ab) + dy*(c_ag - s_abg))**2 + l1**2 - l2**2)**2/(4*l1**2) + (Dx - x - dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2)**(1/2)*(Dx - x - dx*c_bg + dy*c_b*s_g) + ((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2*((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2 + (y - Dy + dx*(c_a*s_g + c_g*s_ab) + dy*(c_ag - s_abg))**2 + l1**2 - l2**2))/(2*l1))*j)/(((z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))**2 + (Dx - x - dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) + dy*(c_g*s_a + c_a*s_bg))))
        theta_2 = np.angle(((((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 - ((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (Dy + y + dx*(c_a*s_g + c_g*s_ab) - dy*(c_ag - s_abg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2 + l1**2 - l2**2)**2/(4*l1**2) + (x - Dx + dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2)**(1/2) + ((x - Dx + dx*c_bg + dy*c_b*s_g)*((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (Dy + y + dx*(c_a*s_g + c_g*s_ab) - dy*(c_ag - s_abg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2 + l1**2 - l2**2))/(2*l1))/((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2) + (((((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 - ((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (Dy + y + dx*(c_a*s_g + c_g*s_ab) - dy*(c_ag - s_abg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2 + l1**2 - l2**2)**2/(4*l1**2) + (x - Dx + dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2)**(1/2)*(x - Dx + dx*c_bg + dy*c_b*s_g) - ((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2*((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (Dy + y + dx*(c_a*s_g + c_g*s_ab) - dy*(c_ag - s_abg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2 + l1**2 - l2**2))/(2*l1))*j)/(((z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))**2 + (x - Dx + dx*c_bg + dy*c_b*s_g)**2)*(z + dx*(s_ag - c_ag*s_b) - dy*(c_g*s_a + c_a*s_bg))))
        theta_3 = np.angle((2*((((2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 - (((c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 - c_bg*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + (Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2)**2/l1**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2)/4)**(1/2) - ((((c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + (Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2)*(Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2)))/l1)/((2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2) + ((2*((((2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 - (((c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + (Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2)**2/l1**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2)) + ((2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2*(((c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 - c_bg*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + (Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2))/l1)*j)/(((2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 + y + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_bg*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))))
        theta_4 = np.angle((2*((((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_bg*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 - ((Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + (y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)**2/l1**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)/4)**(1/2) - (((Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + (y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)*(Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2)))/l1)/((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2) - ((2*((((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 - ((Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + (y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)**2/l1**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2)) + ((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2*((Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + (y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2))/l1)*j)/(((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*(y - Dy/2 + (sqrt(3)*Dx)/2 - (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_bg*(dx/2 + (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))))
        theta_5 = np.angle((2*((((2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 - (((c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 - c_bg*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + ((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2)**2/l1**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)/4)**(1/2) - ((((c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + ((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2)*(Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2)))/l1)/((2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2) + ((2*((((2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 - (((c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + ((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2)**2/l1**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2)) + ((2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2*(((c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - z + (s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 - c_bg*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + ((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2))**2))/l1)*j)/(((2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))**2 + (Dx/2 + x + (sqrt(3)*Dy)/2 + sqrt(3)*((sqrt(3)*Dx)/2 - y - Dy/2 + (c_a*s_g + c_g*s_ab)*(dx/2 + (sqrt(3)*dy)/2) + (c_ag - s_abg)*(dy/2 - (sqrt(3)*dx)/2)) - c_bg*(dx/2 + (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 - (sqrt(3)*dx)/2))**2)*(2*(c_g*s_a + c_a*s_bg)*(dy/2 - (sqrt(3)*dx)/2) - 2*z + 2*(s_ag - c_ag*s_b)*(dx/2 + (sqrt(3)*dy)/2))))
        theta_6 = np.angle((2*((((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 - (((sqrt(3)*Dy)/2 - x - Dx/2 + c_bg*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2)**2/l1**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2)/4)**(1/2) - ((((sqrt(3)*Dy)/2 - x - Dx/2 + c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2)*(Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2)))/l1)/((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2) - ((2*((((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 - (((sqrt(3)*Dy)/2 - x - Dx/2 + c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2)**2/l1**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2)/4)**(1/2)*(Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2)) + ((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2*(((sqrt(3)*Dy)/2 - x - Dx/2 + c_b*c_g*(dx/2 - (sqrt(3)*dy)/2) + c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2 + l1**2 - l2**2 + (z + (c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - (s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2))**2))/l1)*j)/(((2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))**2 + (Dx/2 + x - (sqrt(3)*Dy)/2 + sqrt(3)*(Dy/2 - y + (sqrt(3)*Dx)/2 + (c_a*s_g + c_g*s_ab)*(dx/2 - (sqrt(3)*dy)/2) - (c_ag - s_abg)*(dy/2 + (sqrt(3)*dx)/2)) - c_bg*(dx/2 - (sqrt(3)*dy)/2) - c_b*s_g*(dy/2 + (sqrt(3)*dx)/2))**2)*(2*z + 2*(c_g*s_a + c_a*s_bg)*(dy/2 + (sqrt(3)*dx)/2) - 2*(s_ag - c_ag*s_b)*(dx/2 - (sqrt(3)*dy)/2))))

        # TODO: Arbeitsraumbeschränkung, sobald der 6 RUS aufgebaut ist.
        
        return [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

    def forward_kinematic(self, angles):
        """Forward kinematics of 6-RUS robot. This is done with a numeric solve (fsolve)

        `angles`: list of angles in the form of [θ1, θ2, θ3, θ4, θ5, θ6]

        `return`: list with pose in the form of [x, y, z, α, β, γ]"""

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
        l1, l2, dx, dy, Dx, Dy = self.geometricParams
        z = -l1 - sqrt(l2 ** 2 - (Dx - dx) ** 2)
        x_0 = np.array([0.0, 0.0, z, 0.0, 0.0, 0.0])

        curr_pose = fsolve(func, x_0)  # solve numerically with initial guess

        return list(curr_pose)

    def change_robot_dimensions(self, l1, l2, dx, dy, Dx, Dy, *_):
        """This changes the dimensions of the robot which are important for the kinematics.
        This has to be done before homing or moving of the robot.
        See the documentation for the kinematics to see which values belong to which robot dimension"""
        self.geometricParams = [l1, l2, dx, dy, Dx, Dy]
