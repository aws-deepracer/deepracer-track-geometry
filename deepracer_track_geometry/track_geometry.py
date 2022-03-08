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
"""A class for DeepRacer track geometry"""
import os
import bisect
import pkg_resources
from typing import List, Tuple, Union, Optional
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing
import numpy as np
from deepracer_track_geometry.constants import (
    TrackLine, TrackLineNdist,
    TrackDirection, TrackRegion,
    NdistMode, FiniteDifference)
from deepracer_track_geometry.utils import calculate_yaw, euler_to_quaternion, to_shapely_point


class TrackGeometry(object):
    """
    Track Geometry class.
    NOTE: it only supports looped track instances.
    """
    def __init__(self, track_name: str,
                 finish_line: float = 0.0,
                 direction: Union[TrackDirection, str] = TrackDirection.COUNTER_CLOCKWISE,
                 route_path: Optional[str] = None) -> None:
        """
        Initialize a Track Geometry Class.

        Args:
            track_name (str): the name of the track.
            finish_line (float, optional): the normalized distance w.r.t waypoint[0, :].
                                           Defaults to 0.0.
            direction (Union[TrackDirection, str], optional): the direction of the track.
                                                              Defaults to TrackDirection.COUNTER_CLOCKWISE.
            route_path (str, optional): the path to the numpy file for the track.
                                        Defaults to "./routes".

        Raises:
            ValueError: raises ValueError if the track is not supported.
        """
        route_path = route_path or pkg_resources.resource_filename('deepracer_track_geometry', 'routes/')
        self._track_waypoints = TrackGeometry.get_track_waypoints(track_name, route_path)
        self._track_name = track_name
        # direction - default to counter clockwise
        # NOTE: the assumption here is that all the track numpy files
        # are arranged in counter clockwise direction by default.
        self._direction = TrackDirection(direction)
        # finish line: normalized distance w.r.t waypoint[0, :]
        self._finish_line = finish_line
        self._load_track()

    def _load_track(self) -> None:
        """
        Load the track.
        """
        inner_border_points = self._track_waypoints[:, 2:4]
        outer_border_points = self._track_waypoints[:, 4:6]
        track_center_line_points = self._track_waypoints[:, 0:2]

        self._track_data = {}
        if self._direction == TrackDirection.COUNTER_CLOCKWISE:
            # inner border
            self._track_data[TrackLine.INNER_BORDER_LINE] = LinearRing(inner_border_points)
            # outer border
            self._track_data[TrackLine.OUTER_BORDER_LINE] = LinearRing(outer_border_points)
            # track center line
            self._track_data[TrackLine.TRACK_CENTER_LINE] = LinearRing(track_center_line_points)
            # inner lane center line
            self._track_data[TrackLine.INNER_LANE_CENTER_LINE] = \
                LinearRing((inner_border_points + track_center_line_points) / 2)
            # outer lane center line
            self._track_data[TrackLine.OUTER_LANE_CENTER_LINE] = \
                LinearRing((outer_border_points + track_center_line_points) / 2)
        elif self._direction == TrackDirection.CLOCKWISE:
            self._track_data[TrackLine.INNER_BORDER_LINE] = LinearRing(inner_border_points[::-1])
            self._track_data[TrackLine.OUTER_BORDER_LINE] = LinearRing(outer_border_points[::-1])
            self._track_data[TrackLine.TRACK_CENTER_LINE] = LinearRing(track_center_line_points[::-1])
            self._track_data[TrackLine.INNER_LANE_CENTER_LINE] = \
                LinearRing((inner_border_points[::-1] + track_center_line_points[::-1]) / 2)
            self._track_data[TrackLine.OUTER_LANE_CENTER_LINE] = \
                LinearRing((outer_border_points[::-1] + track_center_line_points[::-1]) / 2)
        # Normalized distances
        self._track_data[TrackLineNdist.TRACK_CENTER_LINE] = \
            self._calculate_ndist(TrackLine.TRACK_CENTER_LINE)
        self._track_data[TrackLineNdist.INNER_LANE_CENTER_LINE] = \
            self._calculate_ndist(TrackLine.INNER_LANE_CENTER_LINE)
        self._track_data[TrackLineNdist.OUTER_LANE_CENTER_LINE] = \
            self._calculate_ndist(TrackLine.OUTER_LANE_CENTER_LINE)
        self._track_data[TrackLineNdist.INNER_BORDER_LINE] = self._calculate_ndist(TrackLine.INNER_BORDER_LINE)
        self._track_data[TrackLineNdist.OUTER_BORDER_LINE] = self._calculate_ndist(TrackLine.OUTER_BORDER_LINE)

        # track regions
        self._track_data[TrackRegion.INNER_LANE] = Polygon(self.track_center_line, [self.inner_border_line])
        self._track_data[TrackRegion.OUTER_LANE] = Polygon(self.outer_border_line, [self.track_center_line])
        self._track_data[TrackRegion.INNER_OFFTRACK] = Polygon(self.inner_border_line)
        self._track_data[TrackRegion.ROAD] = Polygon(self.outer_border_line, [self.inner_border_line])

    def _calculate_ndist(self, track_line: Union[TrackLine, str]):
        """Calculate the normalized distance along a given track line.

        Args:
            track_line (Union[TrackLine, str]): which trackline to calculate normalized distance with.

        Returns:
            List: a list of normalized distances along the track line.
        """
        track_line = TrackLine(track_line)
        return [getattr(self, track_line.value).project(Point(p), normalized=True)
                for p in getattr(self, track_line.value).coords[:-1]] + [1.0]

    @staticmethod
    def get_track_waypoints(track_name: str, track_numpy_path: str = "routes") -> np.ndarray:
        """
        Helper function for reading in the waypoints for track geometry.

        Args:
            track_name (str): the track's name.
            track_numpy_path (str): the path to the track numpy file.

        Returns:
            np.ndarray: the waypoints numpy array.
        """
        waypoints_path = os.path.join(track_numpy_path, "{}.npy".format(track_name))
        waypoints = np.load(waypoints_path)
        return waypoints

    @property
    def track_name(self) -> str:
        """
        The name of the track

        Returns:
            str: the name of the track
        """
        return self._track_name

    @property
    def track_center_line(self) -> LinearRing:
        """
        Track center line property depending on direction.

        Returns:
            LinearRing: the track center line of the track geometry.
        """
        return self._track_data[TrackLine.TRACK_CENTER_LINE]

    @property
    def track_center_line_ndist(self) -> List:
        """
        Track center line normalized distance property depending on direction.

        Returns:
            List: a list of each point's normalized distance along track center line.
        """
        return self._track_data[TrackLineNdist.TRACK_CENTER_LINE].copy()

    @property
    def inner_lane_center_line(self) -> LinearRing:
        """
        Inner lane center line depending on direction.

        Returns:
            LinearRing: the inner lane center line of the track geometry.
        """
        return self._track_data[TrackLine.INNER_LANE_CENTER_LINE]

    @property
    def inner_lane_center_line_ndist(self) -> List:
        """
        Inner lane center line normalized distance depending on direction.

        Returns:
            List: a list of each point's normalized distance along inner lane center line.
        """
        return self._track_data[TrackLineNdist.INNER_LANE_CENTER_LINE].copy()

    @property
    def outer_lane_center_line(self) -> LinearRing:
        """
        Outer lane center line depending on direction.

        Returns:
            LinearRing: the outer lane center line of the track geometry.
        """
        return self._track_data[TrackLine.OUTER_LANE_CENTER_LINE]

    @property
    def outer_lane_center_line_ndist(self) -> List:
        """
        Inner lane center line normalized distance depending on direction.

        Returns:
            List: a list of each point's normalized distance along outer lane center line.
        """
        return self._track_data[TrackLineNdist.OUTER_LANE_CENTER_LINE].copy()

    @property
    def inner_border_line(self) -> LinearRing:
        """
        Inner border property depending on direction.

        Returns:
            LinearRing: the inner border of the track geometry.
        """
        return self._track_data[TrackLine.INNER_BORDER_LINE]

    @property
    def inner_border_line_ndist(self) -> List:
        """
        Inner border normalized distance depending on direction.

        Returns:
            List: a list of each point's normalized distance along inner border.
        """
        return self._track_data[TrackLineNdist.INNER_BORDER_LINE].copy()

    @property
    def outer_border_line(self) -> LinearRing:
        """
        Outer border property depending on direction.

        Returns:
            LinearRing: the outer border of the track geometry.
        """
        return self._track_data[TrackLine.OUTER_BORDER_LINE]

    @property
    def outer_border_line_ndist(self) -> List:
        """
        Outer border normalized distance depending on direction.

        Returns:
            List: a list of each point's normalized distance along outer border.
        """
        return self._track_data[TrackLineNdist.OUTER_BORDER_LINE].copy()

    @property
    def inner_lane(self) -> Polygon:
        """
        The inner lane region of the track (excluding inner border and track center line).

        Returns:
            Polygon: Polygon of the inner lane region
        """
        return self._track_data[TrackRegion.INNER_LANE]

    @property
    def outer_lane(self) -> Polygon:
        """
        The outer lane region of the track (excluding outer border and track center line).

        Returns:
            Polygon: Polygon of the outer lane region
        """
        return self._track_data[TrackRegion.OUTER_LANE]

    @property
    def inner_offtrack(self) -> Polygon:
        """
        The inner offtrack of the track (excluding inner border).

        Returns:
            Polygon: Polygon of the inner offtrack region
        """
        return self._track_data[TrackRegion.INNER_OFFTRACK]

    @property
    def road(self) -> Polygon:
        """
        The road of the track.

        Returns:
            Polygon: Polygon of the road region.
        """
        return self._track_data[TrackRegion.ROAD]

    @property
    def direction(self) -> TrackDirection:
        """
        The currently direction of the track geometry.

        Returns:
            TrackDirection: either clockwise or counter clockwise.
        """
        return self._direction

    @direction.setter
    def direction(self, val: Union[str, TrackDirection]) -> None:
        """
        Direction setter

        Args:
            val (Union[str, TrackDirection]): set the current direction
        """
        direction = TrackDirection(val)
        if self._direction != direction:
            self._direction = direction
            self._load_track()

    @property
    def finish_line(self) -> float:
        """
        Normalized distance w.r.t waypoint[0, :].

        Returns:
            float: distance with range [0.0, 1.0)
        """
        return self._finish_line

    @finish_line.setter
    def finish_line(self, val: float) -> None:
        """
        Normalized distance w.r.t waypoint[0, :] (default finish line).

        Args:
            val (float): distance that will be converted to [0.0, 1.0).
        """
        self._finish_line = val % 1.0

    @property
    def length(self) -> float:
        """
        Return the track length in terms of the track center line.

        Returns:
            float: the length of the track.
        """
        return self.track_center_line.length

    def _convert_ndist_wrt_origin(self,
                                  ndist: float,
                                  ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE) -> float:
        """
        Convert normalized distance with respect to finish line to with respect to origin
        (waypoints[0, :]).

        Args:
            ndist (float): normalized distance w.r.t finish line.
            ndist_mode (Union[NdistMode, str], optional): normalized distance mode. Defaults to "from_finish_line".

        Returns:
            float: the normalized distance w.r.t origin.
        """
        ndist = ndist % 1.0
        ndist_mode = NdistMode(ndist_mode)
        if ndist == 0.0:
            return self.finish_line
        elif ndist_mode == NdistMode.FROM_FINISH_LINE:
            total_ndist = self.finish_line + ndist
        elif ndist_mode == NdistMode.TO_FINISH_LINE:
            total_ndist = self.finish_line - ndist
        else:
            raise ValueError("ndist_mode={} is not supported.".format(ndist_mode))
        return total_ndist % 1.0

    def get_ndist_from_point(self,
                             coordinates: Union[List, np.ndarray, Point],
                             ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE,
                             track_line: Union[TrackLine, str] = TrackLine.TRACK_CENTER_LINE) -> float:
        """
        TO_FINISH_LINE: (remaining normalize distance to finish line)
            the normalize distance w.r.t track length one needs to travel from the current point to reach finish line
        FROM_FINISH_LINE: (if we started at finish line, the distance we have traveled from finish line)
            the normalize distance w.r.t track length one had traveled if one started at finish line

        Args:
            coordinates (Union[List, np.ndarray, Point]): the coordinates we want to calcuate normalized distance with.
            ndist_mode (Union[NdistMode, str], optional): the normalized distance mode to calcuate.
                                                            Defaults to "from_finish_line".
            track_line (Union[TrackLine, str], optional): which trackline to calculate normalized distance with.
                                                            Defaults to "track_center_line".

        Returns:
            float: the normalized distance w.r.t to finish line.
        """
        point = to_shapely_point(coordinates)
        ndist_mode = NdistMode(ndist_mode)
        track_line = TrackLine(track_line)
        try:
            ndist_from_origin = getattr(self, track_line.value).project(point, normalized=True)
            diff = (self.finish_line - ndist_from_origin) % 1.0
            if ndist_mode == NdistMode.FROM_FINISH_LINE:
                return (1.0 - diff) % 1.0
            elif ndist_mode == NdistMode.TO_FINISH_LINE:
                return diff
            else:
                raise ValueError("ndist_mode={} is not supported.".format(ndist_mode))
        except Exception as ex:
            raise ValueError("Error getting ndist from point={}, ndist_mode={}, track_line={}, {}".format(
                             point, ndist_mode, track_line, ex))

    def get_closest_waypoint_indices(
            self,
            ndist: float,
            ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE,
            track_line: Union[TrackLine, str] = TrackLine.TRACK_CENTER_LINE) -> Tuple[int, int]:
        """
        Get closest waypoint indices from normalized distance w.r.t. finish line.

        Args:
            ndist (float): the normalized distance w.r.t to finish line.
            ndist_mode (Union[NdistMode, str], optional): normalized distance mode.
                                                            Defaults to "from_finish_line".
            track_line (Union[TrackLine, str], optional): which trackline to get point from.
                                                            Defaults to "track_center_line".

        Returns:
            Tuple[int, int]: previous and next waypoint index closests to the given ndist.
        """
        ndist_mode = NdistMode(ndist_mode)
        track_line = TrackLine(track_line)
        ndist_converted = self._convert_ndist_wrt_origin(ndist, ndist_mode)
        next_idx = bisect.bisect_right(getattr(self, track_line.value + "_ndist"), ndist_converted)
        next_idx = 0 if next_idx == self.length else next_idx
        # NOTE: self.length - 2 because the coords of last index equals coords of first index
        prev_idx = (self.length - 2) if next_idx == 0 else next_idx - 1
        return prev_idx, next_idx

    def get_closest_waypoints(self,
                              ndist: float,
                              ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE,
                              track_line: Union[TrackLine, str] = TrackLine.TRACK_CENTER_LINE) -> Tuple[np.ndarray,
                                                                                                        np.ndarray]:
        """
        Get closest waypoints from normalized distance w.r.t. finish line.

        Args:
            ndist (float): the normalized distance w.r.t to finish line.
            ndist_mode (Union[NdistMode, str], optional): normalized distance mode.
                                                            Defaults to "from_finish_line".
            track_line (Union[TrackLine, str], optional): which trackline to get point from.
                                                            Defaults to "track_center_line".

        Returns:
            Tuple[np.ndarray, np.ndarray]: previous and next waypoints closests to the given ndist.
        """
        prev_idx, next_idx = self.get_closest_waypoint_indices(
            ndist, ndist_mode, track_line)
        line_coords = getattr(self, track_line.value).coords
        return np.array(line_coords[prev_idx]), np.array(line_coords[next_idx])

    def get_point_from_ndist(self,
                             ndist: float,
                             ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE,
                             track_line: Union[TrackLine, str] = TrackLine.TRACK_CENTER_LINE) -> np.ndarray:
        """
        Get the closest point along the specified track line to the input normalized distance.

        Args:
            ndist (float): normalized distance w.r.t finish line.
            ndist_mode (Union[NdistMode, str], optional): normalized distance mode.
                                                            Defaults to "from_finish_line".
            track_line (Union[TrackLine, str], optional): which trackline to get point from.
                                                            Defaults to "track_center_line".

        Raises:
            ValueError: when the track line or other input has errors.

        Returns:
            np.ndarray: point coordinates.
        """
        ndist_mode = NdistMode(ndist_mode)
        track_line = TrackLine(track_line)
        try:
            ndist_converted = self._convert_ndist_wrt_origin(ndist, ndist_mode)
            return np.array(getattr(self, track_line.value).interpolate(ndist_converted, normalized=True))
        except Exception as ex:
            raise ValueError("Error getting point from ndist={}, ndist_mode={}, track_line={}, {}".format(
                             ndist, ndist_mode, track_line, ex))

    def is_on_track(self,
                    coordinates: Union[List, np.ndarray, Point]) -> bool:
        """
        Check if the given point is on track.
        On track means that the point is in inner lane region or outer lane region
        including boundary
        NOTE: if the point is on inner or outer border, we consider it as offtrack.

        Args:
            coordinates (Union[List, np.ndarray, Point]): the point to check if it's on track

        Returns:
            bool: if the given point is on track.
        """
        point = to_shapely_point(coordinates)
        return self.inner_lane.intersects(point) or \
            self.outer_lane.intersects(point)

    def get_region_on_track(self,
                            coordinates: Union[List, np.ndarray, Point]) -> TrackRegion:
        """
        Get the region the coordinates are in.

        If the point is on inner lane including boundary,  we consider it as inner lane.
        Else if the point is on outer lane including boundary,  we consider it as outer lane.
        Else if the point is on inner offtrack including boundary, we consider it as inner offtrack.
        Else if the point is on outer offtrack including boundary, we consider it as outer offtrack

        Args:
            coordinates (Union[List, np.ndarray, Point]): the coordinates to check which region it is on track.

        Returns:
            TrackRegion: enum for which track region is the point in.
        """
        point = to_shapely_point(coordinates)
        if self.inner_lane.intersects(point):
            return TrackRegion.INNER_LANE
        elif self.outer_lane.intersects(point):
            return TrackRegion.OUTER_LANE
        elif self.inner_offtrack.intersects(point):
            return TrackRegion.INNER_OFFTRACK
        else:
            return TrackRegion.OUTER_OFFTRACK

    def get_orientation(self,
                        ndist: float,
                        ndist_mode: Union[NdistMode, str] = NdistMode.FROM_FINISH_LINE,
                        track_line: Union[TrackLine, str] = TrackLine.TRACK_CENTER_LINE,
                        finite_difference: Union[FiniteDifference, int] = FiniteDifference.CENTRAL_DIFFERENCE) -> np.ndarray:
        """
        Get the orientation the current point is heading to in quaternion along the track direction.

        Args:
            ndist (float): normalized distance w.r.t finish line.
            ndist_mode (Union[NdistMode, str], optional): normalized distance mode.
                                                            Defaults to "from_finish_line".
            track_line (Union[TrackLine, str], optional): which trackline to get point from.
                                                            Defaults to "track_center_line".
            finite_difference (Union[FiniteDifference, int], optional): The different ways of calculating orientation.
                Defaults to FiniteDifference.CENTRAL_DIFFERENCE.

        Returns:
            np.ndarray: the orientation in quaternion.
        """
        ndist_mode = NdistMode(ndist_mode)
        track_line = TrackLine(track_line)
        finite_difference = FiniteDifference(finite_difference)
        prev_coords, next_coords = self.get_closest_waypoints(ndist, ndist_mode)
        ndist_from_origin = self._convert_ndist_wrt_origin(ndist, ndist_mode)
        curr_coords = np.array(getattr(self, track_line.value).interpolate(ndist_from_origin, normalized=True))
        if finite_difference == FiniteDifference.CENTRAL_DIFFERENCE:
            yaw = calculate_yaw(next_coords, prev_coords)
        elif finite_difference == FiniteDifference.FORWARD_DIFFERENCE:
            yaw = calculate_yaw(next_coords, curr_coords)
        else:
            raise ValueError("Unrecognized FiniteDifference enum value")
        return np.array(euler_to_quaternion(yaw=yaw))
