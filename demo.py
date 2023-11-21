import math as m

import numpy as np

class Quattro:

    def square(half_side_length=35, n=2, minHeight = -240.2, level = 60, endLevel = 30):
        """Calculates coordinates for a square
            `halfSideLength`: half length of the edge
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight und robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        #  _______
        # |       |
        # |       |
        # |_______|
        #
        # | a |
        # a = halfSideLength

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        pos_square = [[half_side_length, half_side_length, robotHeight, 0.78, 0, 0, 'mov']]

        for _ in range(n):
            pos_square.append([half_side_length, half_side_length, robotHeight, 0.78, 0, 0, 'lin'])
            pos_square.append([-half_side_length, half_side_length, robotHeight, 0.78, 0, 0, 'lin'])
            pos_square.append([-half_side_length, -half_side_length, robotHeight, 0.78, 0, 0, 'lin'])
            pos_square.append([half_side_length, -half_side_length, robotHeight, 0.78, 0, 0, 'lin'])

        pos_square.append([half_side_length, half_side_length, robotHeight, 0.78, 0, 0, 'lin'])
        pos_square.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])

        return pos_square

    
    def circle(radius=40, resolution=50, n=2, dirCirc=1, minHeight = -240.2, level = 60, endLevel = 30):
        """Calculates coordinates for a 2D-circle
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `dirCirc`: Direction of the circle
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        circle_pos = []
        circle_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])
        
        circle_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])
        circle_pos.append([radius, 0, robotHeight, 0.78, 0, 0, 'lin'])

        for num in t:
            if dirCirc == 0:
                x = m.cos(num) * radius
                y = m.sin(num) * radius
            else:
                x = m.cos(num) * radius
                y = m.sin(num - m.pi) * radius

            circle_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        circle_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'lin'])
        circle_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])

        return circle_pos

    def eight(radius=20, resolution=30, n=1, minHeight = -240.2, level = 60, endLevel = 30):
        """Calculates coordinates for a 2D-eight
            `radius`: Radius of one of the two circles
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        n = max(1, n)
        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        eight_pos = []
        eight_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])
        for num in t:
            x = -m.sin(num) * radius
            y = m.cos(num) * radius - radius
            eight_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        eight_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        for num in t:
            x = -m.sin(num) * radius
            y = -m.cos(num) * radius + radius
            eight_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        eight_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])
        return eight_pos

    def pick_place(distx=15, disty=10, mid_dist=20, lin_height=20, minHeight = -240.2, level= 50, endLevel = 30, defaultLevel= 80):
        """Calculates coordinates for a 3x2 palette
            `distx`: Distance between the palette places in x direction
            `disty`: Distance between the palette places in y direction
            `midDist`: Distance between mid and palette places 
            `linHeight`: Linear distance to pick up/ place a piece 
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `defaultLevel`: z-Coordinate for upper position of pick and place 
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel
        defaultHeight = minHeight + defaultLevel

        pick_place_pos = []
        pick_place_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])
        y_count = [0, 1]
        x_count = [-1, 0, 1]

        for numx in x_count:
            for numy in y_count:
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0.78, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight,0.78, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0.78, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0.78, 0, 0, 'mov'])

                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0.78, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight, 0.78, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0.78, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0.78, 0, 0, 'mov'])

        pick_place_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])
        return pick_place_pos


    def cylinder(radius=32, resolution=40, minHeight = -240.2, lowerLevel= 40, upperLevel = 65,  endLevel = 30):
        """Calculates coordinates for a cylinder
        `radius`: Radius of the cylinder
        `resolution`: Number of circlepoints
         `minHeight`: lowest posible z-Coordinate
        `lowerLevel`: height of lower area of the cylinder
        `upperLevel`: height of upper area of the cylinder
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        down_circ = minHeight + lowerLevel
        up_circ = minHeight + upperLevel
        endHeight = minHeight + endLevel

        t = np.linspace(0, 2 * m.pi, resolution)
        cylinder_pos = []
        cylinder_pos.append([0, 0, down_circ, 0.78, 0, 0, 'mov'])
        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, down_circ, 0.78, 0, 0, 'mov'])

        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, up_circ, 0.78, 0, 0, 'mov'])

        cylinder_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])
        return cylinder_pos

    def cone(max_radius=25, resolution=30, n=5, minHeight = -240.2, level= 40, endLevel = 30):
        """Calculates coordinates for a spiral
        `maxRadius`: Max radius of the spiral
        `resolution`: Number of circlepoints of one circle
        `n`:Numer of circles
        `minHeight`: lowest posible z-Coordinate 
        `level`: Distance between minHeight and robotHeight
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, n * resolution)
        r = np.linspace(2, max_radius, n * resolution)
        spiral_pos = []
        spiral_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])

        for i, num in enumerate(t):
            x = -m.cos(num) * r[i]
            y = m.sin(num) * r[i]
            z = robotHeight + 1.5 * r[i]
            spiral_pos.append([x, y, z, 0.78, 0, 0, 'mov'])

        spiral_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])

        return spiral_pos

    def elaborated_curve(radius=20, resolution=28, distx=20, disty=20, lines=30, minHeight = -240.2, level = 60, endLevel = 30):
        """Calculates coordinates for a 2D-Model
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints, must be a multiple of 4
            `distx`: x-distance between centerpoint of the circle and zero point of the coordinate system
            `disty`: y-distance between centerpoint of the circle and zero point of the coordinate system
            `lines`: Length of parallel lines
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        if resolution % 4 != 0:
            while resolution % 4 != 0:
                resolution += 1

        t = np.linspace(0, m.pi / 2, int(resolution / 4))
        elaborated_curve_pos = []
        elaborated_curve_pos.append([0, 0, robotHeight, 0.78, 0, 0, 'mov'])
        for num in t:
            x = m.cos(num) * radius - distx
            y = -m.sin(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        t = np.linspace(0, 2 * m.pi, resolution)
        for num in t:
            x = -m.sin(num) * radius - distx
            y = m.cos(num) * radius - disty
            elaborated_curve_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        t = np.linspace(0, 3 * m.pi / 2, int(3 * resolution / 4))
        for num in t:
            x = -m.sin(num) * radius - distx
            y = -m.cos(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        elaborated_curve_pos.append([lines, disty, robotHeight, 0.78, 0, 0, 'lin'])

        z = np.linspace(0, m.pi, int(resolution / 2))
        for num in z:
            x = m.sin(num) * radius + lines
            y = m.cos(num) * radius
            elaborated_curve_pos.append([x, y, robotHeight, 0.78, 0, 0, 'mov'])

        elaborated_curve_pos.append([0, -disty, robotHeight, 0.78, 0, 0, 'lin'])

        elaborated_curve_pos.append([0, 0, endHeight, 0.78, 0, 0, 'mov'])
        return elaborated_curve_pos

class Delta:
    
    def square(half_side_length=30, n=2, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a square
            `halfSideLength`: half length of the edge
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight und robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        #  _______
        # |       |
        # |       |
        # |_______|
        #
        # | a |
        # a = halfSideLength

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        pos_square = [[half_side_length, half_side_length, robotHeight, 0, 0, 0, 'mov']]

        for _ in range(n):
            pos_square.append([half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([-half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([-half_side_length, -half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([half_side_length, -half_side_length, robotHeight, 0, 0, 0, 'lin'])

        pos_square.append([half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
        pos_square.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return pos_square

    def triangle(half_side_length=30, n=2, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a samesided triangle
            `halfSideLength`: half sidelength of the triangle
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight und robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        #     ^
        #    / \
        #   /   \
        #  /     \
        # /_______\
        #
        # | a |
        # a = halfSideLength

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        h_half = (half_side_length * m.sqrt(3) / 2) / 2
        pos_triangle = []

        for _ in range(n):
            pos_triangle.append([-h_half, half_side_length, robotHeight, 0, 0, 0, 'mov'])
            pos_triangle.append([-h_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_triangle.append([h_half, 0, robotHeight, 0, 0, 0, 'lin'])

        pos_triangle.append([-h_half, half_side_length, robotHeight, 0, 0, 0, 'lin'])
        pos_triangle.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return pos_triangle

    def circle(radius=40, resolution=50, n=2, dirCirc=1, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-circle
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `dirCirc`: Direction of the circle
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        circle_pos = []
        
        circle_pos.append([0, 0, robotHeight, 0, 0, 0, 'mov'])
        circle_pos.append([radius, 0, robotHeight, 0, 0, 0, 'lin'])

        for num in t:
            if dirCirc == 0:
                x = m.cos(num) * radius
                y = m.sin(num) * radius
            else:
                x = m.cos(num) * radius
                y = m.sin(num - m.pi) * radius

            circle_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        circle_pos.append([0, 0, robotHeight, 0, 0, 0, 'lin'])
        circle_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return circle_pos

    def eight(radius=15, resolution=30, n=1, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-eight
            `radius`: Radius of one of the two circles
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        n = max(1, n)
        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        eight_pos = []
        for num in t:
            x = -m.sin(num) * radius
            y = m.cos(num) * radius - radius
            eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        for num in t:
            x = -m.sin(num) * radius
            y = -m.cos(num) * radius + radius
            eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        eight_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return eight_pos

    def pyramide(half_side_length=30, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a tetrahedron
            `halfSideLength`: half sidelength of the tetrahedron
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        robotHeight = minHeight + level
        endHeight = minHeight + endLevel
        h_base_half = (half_side_length * m.sqrt(3) / 2) / 2
        h_tetra = m.sqrt(6) * half_side_length / 3
        pyramide_pos = [

            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],
            [-h_base_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [h_base_half, 0, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [h_base_half, 0, robotHeight, 0, 0, 0, 'lin'],
            [-h_base_half, half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [-h_base_half, half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [-h_base_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [0, 0, endHeight, 0, 0, 0, 'mov']
        ]

        return pyramide_pos

    def pick_place(distx=15, disty=15, mid_dist=20, lin_height=20, minHeight = -214.8, level= 50, endLevel = 30, defaultLevel= 70):
        """Calculates coordinates for a 3x2 palette
            `distx`: Distance between the palette places in x direction
            `disty`: Distance between the palette places in y direction
            `midDist`: Distance between mid and palette places 
            `linHeight`: Linear distance to pick up/ place a piece 
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `defaultLevel`: z-Coordinate for upper position of pick and place 
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel
        defaultHeight = minHeight + defaultLevel

        pick_place_pos = []
        y_count = [0, 1]
        x_count = [-1, 0, 1]

        for numx in x_count:
            for numy in y_count:
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight, 0, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0, 0, 0, 'mov'])

                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight, 0, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0, 0, 0, 'mov'])

        pick_place_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return pick_place_pos

    def rectangle_signal(flank_height=50, flank_width=15, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for rectangle Signal
        `flankHeight`: Flank height
        `flankWidth`: Flank width
        `minHeight`: lowest posible z-Coordinate 
        `level`: Distance between minHeight and robotHeight
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        rectangle_pos = [[flank_height / 2, -2.5 * flank_width, robotHeight, 0, 0, 0, 'mov'],
                        [-flank_height / 2, -2.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, -1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, -1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, -0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, -0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 2.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 2.5 * flank_width, robotHeight, 0, 0, 0, 'mov'],
                        [0, 0, endHeight, 0, 0, 0, 'mov']]

        return rectangle_pos

    def cylinder(radius=25, resolution=30, minHeight = -214.8, lowerLevel= 40, upperLevel = 60,  endLevel = 30):
        """Calculates coordinates for a cylinder
        `radius`: Radius of the cylinder
        `resolution`: Number of circlepoints
         `minHeight`: lowest posible z-Coordinate
        `lowerLevel`: height of lower area of the cylinder
        `upperLevel`: height of upper area of the cylinder
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        down_circ = minHeight + lowerLevel
        up_circ = minHeight + upperLevel
        endHeight = minHeight + endLevel

        t = np.linspace(0, 2 * m.pi, resolution)
        cylinder_pos = []
        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, down_circ, 0, 0, 0, 'mov'])

        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, up_circ, 0, 0, 0, 'mov'])

        cylinder_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return cylinder_pos

    def cone(max_radius=25, resolution=30, n=5, minHeight = -214.8, level= 40, endLevel = 30):
        """Calculates coordinates for a spiral
        `maxRadius`: Max radius of the spiral
        `resolution`: Number of circlepoints of one circle
        `n`:Numer of circles
        `minHeight`: lowest posible z-Coordinate 
        `level`: Distance between minHeight and robotHeight
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, n * resolution)
        r = np.linspace(2, max_radius, n * resolution)
        spiral_pos = []

        for i, num in enumerate(t):
            x = -m.cos(num) * r[i]
            y = m.sin(num) * r[i]
            z = robotHeight + 1.5 * r[i]
            spiral_pos.append([x, y, z, 0, 0, 0, 'mov'])

        spiral_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return spiral_pos

    def elaborated_curve(radius=20, resolution=28, distx=20, disty=20, lines=30, minHeight = -214.8, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-Model
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints, must be a multiple of 4
            `distx`: x-distance between centerpoint of the circle and zero point of the coordinate system
            `disty`: y-distance between centerpoint of the circle and zero point of the coordinate system
            `lines`: Length of parallel lines
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        if resolution % 4 != 0:
            while resolution % 4 != 0:
                resolution += 1

        t = np.linspace(0, m.pi / 2, int(resolution / 4))
        elaborated_curve_pos = []
        for num in t:
            x = m.cos(num) * radius - distx
            y = -m.sin(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        t = np.linspace(0, 2 * m.pi, resolution)
        for num in t:
            x = -m.sin(num) * radius - distx
            y = m.cos(num) * radius - disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        t = np.linspace(0, 3 * m.pi / 2, int(3 * resolution / 4))
        for num in t:
            x = -m.sin(num) * radius - distx
            y = -m.cos(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        elaborated_curve_pos.append([lines, disty, robotHeight, 0, 0, 0, 'lin'])

        z = np.linspace(0, m.pi, int(resolution / 2))
        for num in z:
            x = m.sin(num) * radius + lines
            y = m.cos(num) * radius
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        elaborated_curve_pos.append([0, -disty, robotHeight, 0, 0, 0, 'lin'])

        elaborated_curve_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return elaborated_curve_pos



class sixRUS:

    def square(half_side_length=30, n=2, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a square
            `halfSideLength`: half length of the edge
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight und robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        #  _______
        # |       |
        # |       |
        # |_______|
        #
        # | a |
        # a = halfSideLength

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        pos_square = [[half_side_length, half_side_length, robotHeight, 0, 0, 0, 'mov']]

        for _ in range(n):
            pos_square.append([half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([-half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([-half_side_length, -half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_square.append([half_side_length, -half_side_length, robotHeight, 0, 0, 0, 'lin'])

        pos_square.append([half_side_length, half_side_length, robotHeight, 0, 0, 0, 'lin'])
        pos_square.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return pos_square

    def triangle(half_side_length=30, n=2, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a samesided triangle
            `halfSideLength`: half sidelength of the triangle
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight und robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        #     ^
        #    / \
        #   /   \
        #  /     \
        # /_______\
        #
        # | a |
        # a = halfSideLength

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        h_half = (half_side_length * m.sqrt(3) / 2) / 2
        pos_triangle = []

        for _ in range(n):
            pos_triangle.append([-h_half, half_side_length, robotHeight, 0, 0, 0, 'mov'])
            pos_triangle.append([-h_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'])
            pos_triangle.append([h_half, 0, robotHeight, 0, 0, 0, 'lin'])

        pos_triangle.append([-h_half, half_side_length, robotHeight, 0, 0, 0, 'lin'])
        pos_triangle.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return pos_triangle

    def circle(radius=40, resolution=50, n=2, dirCirc=1, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-circle
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `dirCirc`: Direction of the circle
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        circle_pos = []
        
        circle_pos.append([0, 0, robotHeight, 0, 0, 0, 'mov'])
        circle_pos.append([radius, 0, robotHeight, 0, 0, 0, 'lin'])

        for num in t:
            if dirCirc == 0:
                x = m.cos(num) * radius
                y = m.sin(num) * radius
            else:
                x = m.cos(num) * radius
                y = m.sin(num - m.pi) * radius

            circle_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        circle_pos.append([0, 0, robotHeight, 0, 0, 0, 'lin'])
        circle_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return circle_pos

    def eight(radius=15, resolution=30, n=1, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-eight
            `radius`: Radius of one of the two circles
            `resolution`: Number of circlepoints
            `n`: Number of rotations
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        n = max(1, n)
        t = np.linspace(0, n * 2 * m.pi, resolution * n)
        eight_pos = []
        for num in t:
            x = -m.sin(num) * radius
            y = m.cos(num) * radius - radius
            eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        for num in t:
            x = -m.sin(num) * radius
            y = -m.cos(num) * radius + radius
            eight_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        eight_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return eight_pos

    def pyramide(half_side_length=30, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a tetrahedron
            `halfSideLength`: half sidelength of the tetrahedron
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """
        robotHeight = minHeight + level
        endHeight = minHeight + endLevel
        h_base_half = (half_side_length * m.sqrt(3) / 2) / 2
        h_tetra = m.sqrt(6) * half_side_length / 3
        pyramide_pos = [

            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],
            [-h_base_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [h_base_half, 0, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [h_base_half, 0, robotHeight, 0, 0, 0, 'lin'],
            [-h_base_half, half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [-h_base_half, half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [-h_base_half, -half_side_length, robotHeight, 0, 0, 0, 'lin'],
            [0, 0, robotHeight + h_tetra, 0, 0, 0, 'mov'],

            [0, 0, endHeight, 0, 0, 0, 'mov']
        ]

        return pyramide_pos

    def pick_place(distx=15, disty=15, mid_dist=20, lin_height=20, minHeight = -256.2, level= 50, endLevel = 30, defaultLevel= 70):
        """Calculates coordinates for a 3x2 palette
            `distx`: Distance between the palette places in x direction
            `disty`: Distance between the palette places in y direction
            `midDist`: Distance between mid and palette places 
            `linHeight`: Linear distance to pick up/ place a piece 
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `defaultLevel`: z-Coordinate for upper position of pick and place 
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel
        defaultHeight = minHeight + defaultLevel

        pick_place_pos = []
        y_count = [0, 1]
        x_count = [-1, 0, 1]

        for numx in x_count:
            for numy in y_count:
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight, 0, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty - mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0, 0, 0, 'mov'])

                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight, 0, 0, 0, 'mov'])
                pick_place_pos.append(
                    [numx * distx, numy * disty + mid_dist, robotHeight + lin_height, 0, 0, 0, 'lin'])

                pick_place_pos.append([numx * distx, 0, defaultHeight, 0, 0, 0, 'mov'])

        pick_place_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return pick_place_pos

    def rectangle_signal(flank_height=50, flank_width=15, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for rectangle Signal
        `flankHeight`: Flank height
        `flankWidth`: Flank width
        `minHeight`: lowest posible z-Coordinate 
        `level`: Distance between minHeight and robotHeight
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        rectangle_pos = [[flank_height / 2, -2.5 * flank_width, robotHeight, 0, 0, 0, 'mov'],
                        [-flank_height / 2, -2.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, -1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, -1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, -0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, -0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 0.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 1.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [-flank_height / 2, 2.5 * flank_width, robotHeight, 0, 0, 0, 'lin'],
                        [flank_height / 2, 2.5 * flank_width, robotHeight, 0, 0, 0, 'mov'],
                        [0, 0, endHeight, 0, 0, 0, 'mov']]

        return rectangle_pos

    def cylinder(radius=25, resolution=30, minHeight = -256.2, lowerLevel= 40, upperLevel = 60,  endLevel = 30):
        """Calculates coordinates for a cylinder
        `radius`: Radius of the cylinder
        `resolution`: Number of circlepoints
         `minHeight`: lowest posible z-Coordinate
        `lowerLevel`: height of lower area of the cylinder
        `upperLevel`: height of upper area of the cylinder
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        down_circ = minHeight + lowerLevel
        up_circ = minHeight + upperLevel
        endHeight = minHeight + endLevel

        t = np.linspace(0, 2 * m.pi, resolution)
        cylinder_pos = []
        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, down_circ, 0, 0, 0, 'mov'])

        for num in t:
            x = -m.cos(num) * radius
            y = m.sin(num) * radius

            cylinder_pos.append([x, y, up_circ, 0, 0, 0, 'mov'])

        cylinder_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return cylinder_pos

    def cone(max_radius=25, resolution=30, n=5, minHeight = -256.2, level= 40, endLevel = 30):
        """Calculates coordinates for a spiral
        `maxRadius`: Max radius of the spiral
        `resolution`: Number of circlepoints of one circle
        `n`:Numer of circles
        `minHeight`: lowest posible z-Coordinate 
        `level`: Distance between minHeight and robotHeight
        `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
        `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
        linear moving
        """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        t = np.linspace(0, n * 2 * m.pi, n * resolution)
        r = np.linspace(2, max_radius, n * resolution)
        spiral_pos = []

        for i, num in enumerate(t):
            x = -m.cos(num) * r[i]
            y = m.sin(num) * r[i]
            z = robotHeight + 1.5 * r[i]
            spiral_pos.append([x, y, z, 0, 0, 0, 'mov'])

        spiral_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])

        return spiral_pos

    def elaborated_curve(radius=20, resolution=28, distx=20, disty=20, lines=30, minHeight = -256.2, level= 50, endLevel = 30):
        """Calculates coordinates for a 2D-Model
            `radius`: Radius of the circle
            `resolution`: Number of circlepoints, must be a multiple of 4
            `distx`: x-distance between centerpoint of the circle and zero point of the coordinate system
            `disty`: y-distance between centerpoint of the circle and zero point of the coordinate system
            `lines`: Length of parallel lines
            `minHeight`: lowest posible z-Coordinate 
            `level`: Distance between minHeight and robotHeight
            `endLevel`: Distance betwenn minHeight and endHeight at the end of programm
            `return`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for
            linear moving
            """

        robotHeight = minHeight + level
        endHeight = minHeight + endLevel

        if resolution % 4 != 0:
            while resolution % 4 != 0:
                resolution += 1

        t = np.linspace(0, m.pi / 2, int(resolution / 4))
        elaborated_curve_pos = []
        for num in t:
            x = m.cos(num) * radius - distx
            y = -m.sin(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        t = np.linspace(0, 2 * m.pi, resolution)
        for num in t:
            x = -m.sin(num) * radius - distx
            y = m.cos(num) * radius - disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        t = np.linspace(0, 3 * m.pi / 2, int(3 * resolution / 4))
        for num in t:
            x = -m.sin(num) * radius - distx
            y = -m.cos(num) * radius + disty
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        elaborated_curve_pos.append([lines, disty, robotHeight, 0, 0, 0, 'lin'])

        z = np.linspace(0, m.pi, int(resolution / 2))
        for num in z:
            x = m.sin(num) * radius + lines
            y = m.cos(num) * radius
            elaborated_curve_pos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

        elaborated_curve_pos.append([0, -disty, robotHeight, 0, 0, 0, 'lin'])

        elaborated_curve_pos.append([0, 0, endHeight, 0, 0, 0, 'mov'])
        return elaborated_curve_pos



if __name__ == '__main__':
    # Define return list values for demo sequences as this examples:
    # [x,y,z,a,b,c,'mov'] -> PTP
    # [x,y,z,a,b,c,'lin'] -> linear moving
     ans = sixRUS.square()
     print(ans)
