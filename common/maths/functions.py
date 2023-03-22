import numpy as np
from typing import Union


def barrier_function(q1, q2, c, c_dot):
    b = q1 * np.exp(q2 * c)
    b_dot = q1 * q2 * np.exp(q2 * c) * c_dot
    b_ddot = q1 * (q2 ** 2) * np.exp(q2 * c) * np.matmul(c_dot, c_dot.T)
    return b, b_dot, b_ddot


def norm(v: np.ndarray):
    """
    Get norm 2 of given vector
    Args:
        v (np.ndarray): a vector

    Returns:
        (np.float) norm2 of vector
    """
    return np.linalg.norm(v)


def two_points_equation(
        pts1: Union[np.ndarray, list, tuple],
        pts2: Union[np.ndarray, list, tuple]
) -> tuple:
    """
    Find a equation go through 2 points: pts1(x, y), pts2(x, y)
    The equation should be: ax + by + c = 0
    Args:
        pts1 (np.ndarray, list, tuple): (x1, y1)
        pts2 (np.ndarray, list, tuple): (x2, y2)

    Returns:
        (tuple): a, b, c
    """
    # must be in increasing order of x
    # or bug...?
    if pts1[0] > pts2[0]:
        pts1, pts2 = pts2, pts1

    x1, y1 = pts1
    x2, y2 = pts2

    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    return a, b, c


def angular_between_two_vector(
        u: np.ndarray,
        v: np.ndarray
) -> float:
    """
    Return angular between 2 vectors in radian
    Args:
        u (np.ndarray): vector u
        v (np.ndarray): vector v

    Returns:
        (float) angular in radian
    """
    dot_product = np.dot(u, v)
    norm_u = norm(u)
    norm_v = norm(v)
    return np.arccos(dot_product / (norm_u * norm_v))


def norm_to_range(value, start=-np.pi, end=np.pi) -> float:
    """
    Normalize value
    to [start, end]
    """
    width = end - start
    offset = value - start
    return (offset - (np.floor(offset / width) * width)) + start
