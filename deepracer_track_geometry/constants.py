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
"""Module to contain DeepRacer track geometry related constants"""
from enum import Enum, unique


@unique
class TrackDirection(Enum):
    """
    The directions on the track assuming the track is a closed loop.
    """
    CLOCKWISE = "cw"
    COUNTER_CLOCKWISE = "ccw"


@unique
class TrackLine(Enum):
    """
    The lines defines a track.
    """
    TRACK_CENTER_LINE = "track_center_line"
    INNER_LANE_CENTER_LINE = "inner_lane_center_line"
    OUTER_LANE_CENTER_LINE = "outer_lane_center_line"
    INNER_BORDER_LINE = "inner_border_line"
    OUTER_BORDER_LINE = "outer_border_line"


@unique
class TrackLineNdist(Enum):
    """
    The normalized distances of each track line.
    """
    TRACK_CENTER_LINE = "track_center_line_ndist"
    INNER_LANE_CENTER_LINE = "inner_lane_center_line_ndist"
    OUTER_LANE_CENTER_LINE = "outer_lane_center_line_ndist"
    INNER_BORDER_LINE = "inner_border_line_ndist"
    OUTER_BORDER_LINE = "outer_border_line_ndist"


@unique
class TrackRegion(Enum):
    """
    The region on track.
    """
    INNER_LANE = "inner_lane"
    OUTER_LANE = "outer_lane"
    INNER_OFFTRACK = "inner_offtrack"
    OUTER_OFFTRACK = "outer_offtrack"
    ROAD = "road"


@unique
class NdistMode(Enum):
    """
    Normalized Distance Mode.
    FROM_FINISH_LINE: the normalized distance from the intersection of the finish line with the center line
                        to the current point projected on center line.
    TO_FINISH_LINE: the normalized distance from the current point projected on center line to the intersection
                        of the finish line with the center line.
    """
    FROM_FINISH_LINE = "from_finish_line"
    TO_FINISH_LINE = "to_finish_line"


@unique
class FiniteDifference(Enum):
    """
    The different ways of calculating orientation.
    """
    CENTRAL_DIFFERENCE = 1
    FORWARD_DIFFERENCE = 2
