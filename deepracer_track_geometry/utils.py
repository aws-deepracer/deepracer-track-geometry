#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""utils for DeepRacer track geometry """
import math
import numpy as np
from typing import List, Union
from shapely.geometry import Point


def euler_to_quaternion(roll: float = 0.0,
                        pitch: float = 0.0,
                        yaw: float = 0.0) -> List:
    """
    Convert orientation in euler angles (in terms of roll, pitch, yaw) here
    to quarternion.
    The order of rotation applied: roll -> pitch -> yaw

    Args:
        roll (float, optional): gives the bank angle. Defaults to 0.
        pitch (float, optional): gives the elevation. Defaults to 0.
        yaw (float, optional): gives the bearing. Defaults to 0.

    Returns:
        list: a list representing x y z w of quaternion.
    """
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Quaternion
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    w = cy * cp * cr + sy * sp * sr

    return [x, y, z, w]


def calculate_yaw(p1: Union[List, np.ndarray], p2: Union[List, np.ndarray]) -> float:
    """
    Calculate yaw from two points.

    Args:
        p1 (Union[List, np.ndarray]): the first point.
        p2 (Union[List, np.ndarray]): the second point.

    Returns:
        float: yaw calculated from the two points.
    """
    return math.atan2(p1[1] - p2[1], p1[0] - p2[0])


def to_shapely_point(coordinates: Union[Point, List, np.ndarray]) -> Point:
    """
    Convert coordiantes to Shapely Point object.

    Args:
        coordinates (Union[Point, List, np.ndarray]): A list of coordinats to convert to shapely Point.

    Raises:
        ValueError: If the coordinates dimension are incorrect, raise ValueError.

    Returns:
        Point: A shapely point object.
    """
    if isinstance(coordinates, Point):
        return coordinates

    if len(coordinates) < 2:
        raise ValueError("need at least 2 dimension coordinates.")
    if len(coordinates) > 3:
        raise ValueError("max dimension of coordinates is 3.")
    return Point(coordinates)
