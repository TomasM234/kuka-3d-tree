import math
import numpy as np


def kuka_base_to_matrix(x, y, z, a, b, c):
    """Convert KUKA X,Y,Z,A,B,C (ZYX Euler) to 4x4 homogeneous transform."""
    an = np.radians(a)
    bn = np.radians(b)
    cn = np.radians(c)
    cz, sz = np.cos(an), np.sin(an)
    cy, sy = np.cos(bn), np.sin(bn)
    cx, sx = np.cos(cn), np.sin(cn)

    rz = np.array([[cz, -sz, 0, 0], [sz, cz, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    ry = np.array([[cy, 0, sy, 0], [0, 1, 0, 0], [-sy, 0, cy, 0], [0, 0, 0, 1]])
    rx = np.array([[1, 0, 0, 0], [0, cx, -sx, 0], [0, sx, cx, 0], [0, 0, 0, 1]])

    transform = rz @ ry @ rx
    transform[0, 3] = x
    transform[1, 3] = y
    transform[2, 3] = z
    return transform


def matrix_to_kuka_abc(matrix):
    """Convert 4x4 transform matrix to KUKA X,Y,Z,A,B,C (ZYX Euler)."""
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]

    sy = math.sqrt(matrix[0, 0] ** 2 + matrix[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        a = math.atan2(matrix[1, 0], matrix[0, 0])
        b = math.atan2(-matrix[2, 0], sy)
        c = math.atan2(matrix[2, 1], matrix[2, 2])
    else:
        a = math.atan2(-matrix[1, 2], matrix[1, 1])
        b = math.atan2(-matrix[2, 0], sy)
        c = 0

    return x, y, z, math.degrees(a), math.degrees(b), math.degrees(c)
