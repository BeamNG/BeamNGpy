from __future__ import annotations

import math

import numpy as np
from numpy import ndarray

from beamngpy.types import Float3, Quat


def angle_to_quat(angle: Float3) -> Quat:
    """
    Converts an euler angle to a quaternion.

    Args:
        angle: Euler angle in degrees.

    Returns:
        Quaternion with the order ``(x, y, z, w)`` with ``w`` representing the real component.
    """
    angle_rad = [math.radians(x) for x in angle]

    cy = math.cos(angle_rad[2] * 0.5)
    sy = math.sin(angle_rad[2] * 0.5)
    cp = math.cos(angle_rad[1] * 0.5)
    sp = math.sin(angle_rad[1] * 0.5)
    cr = math.cos(angle_rad[0] * 0.5)
    sr = math.sin(angle_rad[0] * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


def compute_rotation_matrix(quat: Quat) -> ndarray:
    """
    Calculates the rotation matrix for the given quaternion
    to be used in a scenario prefab.

    Args:
        quat: Quaternion with the order ``(x, y, z, w)`` with ``w`` representing the real component.

    Returns:
        The rotation matrix as a ``NumPy`` array.
    """
    norm = np.linalg.norm(quat)
    eps = np.finfo(float).eps
    if np.abs(norm - 1) > eps:
        quat = tuple(q / float(norm) for q in quat)
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    rot_mat = np.array([
        [1-2*(y**2+z**2), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x**2+z**2), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x**2+y**2)]
    ], dtype=float)
    return rot_mat


def quat_as_rotation_mat_str(quat: Quat) -> str:
    """
    For a given quaternion, the function computes the corresponding rotation
    matrix and converts it into a string.

    Args:
        quat: Quaternion with the order ``(x, y, z, w)`` with ``w`` representing the real component.

    Returns:
        Rotation matrix as a string.
    """
    mat = compute_rotation_matrix(quat)
    mat = mat.reshape(9).astype(str)
    return ' '.join(mat)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    """
    Multiplies two quaternions.

    Args:
        a: Quaternion with the order ``(x, y, z, w)`` with ``w`` representing the real component.
        b: Quaternion with the order ``(x, y, z, w)`` with ``w`` representing the real component.

    Returns:
        The product of ``a`` and ``b`` as a quaternion.
    """
    return (a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2],
            a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2])
