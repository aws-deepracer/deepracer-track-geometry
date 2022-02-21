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
import pkg_resources
from unittest import TestCase
from shapely.geometry import Point

from deepracer_track_geometry.track_geometry import TrackGeometry
from deepracer_track_geometry.constants import TrackDirection, TrackRegion, NdistMode, FiniteDifference


class TrackGeometryTest(TestCase):
    def setUp(self) -> None:
        self.track_name = "monaco"
        self.track = TrackGeometry(self.track_name)

    def test_get_track_name(self) -> None:
        self.assertEqual(self.track.track_name, self.track_name)

    def test_get_track_length(self) -> None:
        self.assertEqual(self.track.length, self.track.track_center_line.length)

    def test_set_finish_line_wrap(self) -> None:
        self.track.finish_line = -0.3
        self.assertEqual(self.track.finish_line, 0.7)

    def test_set_finish_line_positive(self) -> None:
        # positive
        self.track.finish_line = 0.3
        self.assertEqual(self.track.finish_line, 0.3)

    def test_set_direction_invalid(self) -> None:
        with self.assertRaises(ValueError):
            # Invalid finish line value
            self.track.direction = "reverse"

    def test_set_direction_cw(self) -> None:
        self.track.direction = TrackDirection.CLOCKWISE.value
        self.assertEqual(self.track.direction, TrackDirection.CLOCKWISE)

    def test_set_direction_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.assertEqual(self.track.direction, TrackDirection.COUNTER_CLOCKWISE)

    def test_is_on_track_invalid_dimension(self) -> None:
        with self.assertRaises(ValueError) as ex:
            # Invalid coordinate value
            self.assertFalse(self.track.is_on_track(coordinates=[-0.60]))
            self.assertEqual("need at least 2 dimension coordinates.", str(ex.exception))

        with self.assertRaises(ValueError) as ex:
            # Invalid coordinate value
            self.assertFalse(self.track.is_on_track(coordinates=[-0.60, 0.93, 0.1, 0.1]))
            self.assertEqual("max dimension of coordinates is 3.", str(ex.exception))

        with self.assertRaises(ValueError) as ex:
            # Invalid coordinate value
            self.assertFalse(self.track.get_region_on_track(coordinates=[-0.60]))
            self.assertEqual("need at least 2 dimension coordinates.", str(ex.exception))

        with self.assertRaises(ValueError) as ex:
            # Invalid coordinate value
            self.assertFalse(self.track.get_region_on_track(coordinates=[-0.60, 0.93, 0.1, 0.1]))
            self.assertEqual("max dimension of coordinates is 3.", str(ex.exception))

    def test_is_on_track_inner_offtrack(self) -> None:
        coords = [-0.60, 0.93]  # Inner Offtrack
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_OFFTRACK)

        coords = [-0.60, 0.93, 3]  # Inner Offtrack, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_OFFTRACK)

    def test_is_on_track_inner_border(self) -> None:
        coords = [-6.38, 0.93]  # Inner border
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_OFFTRACK)

        coords = [-6.38, 0.93, 1]  # Inner border, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_OFFTRACK)

    def test_is_on_track_center_line(self) -> None:
        coords = [-7.014, 1.28]  # Center Line
        self.assertTrue(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_LANE)

        coords = [-7.014, 1.28, 1]  # Center Line, z coord doesn't matter
        self.assertTrue(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.INNER_LANE)

    def test_is_on_track_outer_lane(self) -> None:
        coords = [-7.2, 1.28]  # Outer Lane
        self.assertTrue(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.OUTER_LANE)

        coords = [-7.2, 1.28, 3]  # Outer lane, z coord doesn't matter
        self.assertTrue(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.OUTER_LANE)

    def test_is_on_track_outer_offtrack_shapely_point(self) -> None:
        coords = [-8.2, 1.28]  # Outer Offtrack
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.OUTER_OFFTRACK)

        coords = [-8.2, 1.28, 3]  # Outer Offtrack, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.OUTER_OFFTRACK)

    def test_is_on_track_inner_offtrack_shapely_point(self) -> None:
        coords = [-0.60, 0.93]  # Inner Offtrack
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_OFFTRACK)

        coords = [-0.60, 0.93, 3]  # Inner Offtrack, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_OFFTRACK)

    def test_is_on_track_inner_border_shapely_point(self) -> None:
        coords = [-6.38, 0.93]  # Inner border
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_OFFTRACK)

        coords = [-6.38, 0.93, 1]  # Inner border, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_OFFTRACK)

    def test_is_on_track_center_line_shapely_point(self) -> None:
        coords = [-7.014, 1.28]  # Center Line
        self.assertTrue(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_LANE)

        coords = [-7.014, 1.28, 1]  # Center Line, z coord doesn't matter
        self.assertTrue(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.INNER_LANE)

    def test_is_on_track_outer_lane_shapely_point(self) -> None:
        coords = [-7.2, 1.28]  # Outer Lane
        self.assertTrue(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.OUTER_LANE)

        coords = [-7.2, 1.28, 3]  # Outer lane, z coord doesn't matter
        self.assertTrue(self.track.is_on_track(coordinates=Point(coords)))
        self.assertEqual(self.track.get_region_on_track(coordinates=Point(coords)),
                         TrackRegion.OUTER_LANE)

    def test_is_on_track_outer_offtrack(self) -> None:
        coords = [-8.2, 1.28]  # Outer Offtrack
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.OUTER_OFFTRACK)

        coords = [-8.2, 1.28, 3]  # Outer Offtrack, z coord doesn't matter
        self.assertFalse(self.track.is_on_track(coordinates=coords))
        self.assertEqual(self.track.get_region_on_track(coordinates=coords),
                         TrackRegion.OUTER_OFFTRACK)

    def _test_ndist(self, ndist_mode) -> None:
        test_ndist = 0.1
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.3
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.5
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.8
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.0
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 1.0
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(coords, ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, 0.0)

    def _test_ndist_shapely_point(self, ndist_mode) -> None:
        test_ndist = 0.1
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.3
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.5
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.8
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 0.0
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, test_ndist)

        test_ndist = 1.0
        coords = self.track.get_point_from_ndist(test_ndist, ndist_mode=ndist_mode)
        ndist = self.track.get_ndist_from_point(Point(coords), ndist_mode=ndist_mode)
        self.assertAlmostEqual(ndist, 0.0)

    def test_ndist_finish_line_0_0_to_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = 0.0  # Change finish line
        self.assertEqual(self.track.finish_line, 0.0)
        self._test_ndist(ndist_mode=NdistMode.TO_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_ndist_finish_line_0_0_from_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = 0.0  # Change finish line
        self.assertEqual(self.track.finish_line, 0.0)
        self._test_ndist(ndist_mode=NdistMode.FROM_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_ndist_finish_line_0_3_to_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = 0.3  # Change finish line
        self.assertEqual(self.track.finish_line, 0.3)
        self._test_ndist(ndist_mode=NdistMode.TO_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_ndist_finish_line_0_3_from_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = 0.3  # Change finish line
        self.assertEqual(self.track.finish_line, 0.3)
        self._test_ndist(ndist_mode=NdistMode.FROM_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_ndist_finish_line_neg_0_3_to_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = -0.3  # Change finish line
        self.assertEqual(self.track.finish_line, 0.7)
        self._test_ndist(ndist_mode=NdistMode.TO_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_ndist_finish_line_neg_0_3_from_finish_line_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        self.track.finish_line = -0.3  # Change finish line
        self.assertEqual(self.track.finish_line, 0.7)
        self._test_ndist(ndist_mode=NdistMode.FROM_FINISH_LINE)
        self._test_ndist_shapely_point(ndist_mode=NdistMode.TO_FINISH_LINE)

    def test_get_closest_waypoint_indices_0_1_cw(self) -> None:
        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.1
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertEqual(prev_idx, 214)
        self.assertEqual(next_idx, 215)

        test_ndist = 0.9
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertEqual(prev_idx, 214)
        self.assertEqual(next_idx, 215)

    def test_get_closest_waypoint_indices_0_1_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.1
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertEqual(prev_idx, 211)
        self.assertEqual(next_idx, 212)

        test_ndist = 0.9
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertEqual(prev_idx, 211)
        self.assertEqual(next_idx, 212)

    def test_get_closest_waypoint_indices_0_5_cw(self) -> None:
        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.5
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertEqual(prev_idx, 116)
        self.assertEqual(next_idx, 117)

        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertEqual(prev_idx, 116)
        self.assertEqual(next_idx, 117)

    def test_get_closest_waypoint_indices_0_5_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.5
        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertEqual(prev_idx, 117)
        self.assertEqual(next_idx, 118)

        prev_idx, next_idx = self.track.get_closest_waypoint_indices(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertEqual(prev_idx, 117)
        self.assertEqual(next_idx, 118)

    def test_get_closest_waypoints_0_1_cw(self) -> None:
        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.1
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], -8.01600242)
        self.assertAlmostEqual(prev_coords[1], -5.12338257)
        self.assertAlmostEqual(next_coords[0], -8.15831709)
        self.assertAlmostEqual(next_coords[1], -4.93767357)

        test_ndist = 0.9
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], -8.01600242)
        self.assertAlmostEqual(prev_coords[1], -5.12338257)
        self.assertAlmostEqual(next_coords[0], -8.15831709)
        self.assertAlmostEqual(next_coords[1], -4.93767357)

    def test_get_closest_waypoints_0_1_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.1
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], -2.43641901)
        self.assertAlmostEqual(prev_coords[1], 2.26828957)
        self.assertAlmostEqual(next_coords[0], -2.75742698)
        self.assertAlmostEqual(next_coords[1], 2.34699249)

        test_ndist = 0.9
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], -2.43641901)
        self.assertAlmostEqual(prev_coords[1], 2.26828957)
        self.assertAlmostEqual(next_coords[0], -2.75742698)
        self.assertAlmostEqual(next_coords[1], 2.34699249)

    def test_get_closest_waypoints_0_5_cw(self) -> None:
        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.5
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], 8.9961977)
        self.assertAlmostEqual(prev_coords[1], 0.3554957)
        self.assertAlmostEqual(next_coords[0], 8.7407155)
        self.assertAlmostEqual(next_coords[1], 0.1251201)

        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], 8.9961977)
        self.assertAlmostEqual(prev_coords[1], 0.3554957)
        self.assertAlmostEqual(next_coords[0], 8.7407155)
        self.assertAlmostEqual(next_coords[1], 0.1251201)

    def test_get_closest_waypoints_0_5_ccw(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.5
        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.TO_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], 8.7407155)
        self.assertAlmostEqual(prev_coords[1], 0.1251201)
        self.assertAlmostEqual(next_coords[0], 8.9961977)
        self.assertAlmostEqual(next_coords[1], 0.3554957)

        prev_coords, next_coords = self.track.get_closest_waypoints(test_ndist, ndist_mode=NdistMode.FROM_FINISH_LINE)
        self.assertAlmostEqual(prev_coords[0], 8.7407155)
        self.assertAlmostEqual(prev_coords[1], 0.1251201)
        self.assertAlmostEqual(next_coords[0], 8.9961977)
        self.assertAlmostEqual(next_coords[1], 0.3554957)

    def test_get_orientation_central_difference(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.1
        orientation = self.track.get_orientation(test_ndist,
                                                 ndist_mode=NdistMode.TO_FINISH_LINE,
                                                 finite_difference=FiniteDifference.CENTRAL_DIFFERENCE)
        self.assertEqual(len(orientation), 4)
        self.assertEqual(orientation[0], 0.0)
        self.assertEqual(orientation[1], 0.0)
        self.assertAlmostEqual(orientation[2], 0.9927828)
        self.assertAlmostEqual(orientation[3], 0.1199265)

        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.1
        orientation = self.track.get_orientation(test_ndist,
                                                 ndist_mode=NdistMode.TO_FINISH_LINE,
                                                 finite_difference=FiniteDifference.CENTRAL_DIFFERENCE)
        self.assertEqual(len(orientation), 4)
        self.assertEqual(orientation[0], 0.0)
        self.assertEqual(orientation[1], 0.0)
        self.assertAlmostEqual(orientation[2], 0.8967341)
        self.assertAlmostEqual(orientation[3], 0.4425698)

    def test_get_orientation_forward_difference(self) -> None:
        self.track.direction = TrackDirection.COUNTER_CLOCKWISE.value
        test_ndist = 0.1
        orientation = self.track.get_orientation(test_ndist,
                                                 ndist_mode=NdistMode.TO_FINISH_LINE,
                                                 finite_difference=FiniteDifference.FORWARD_DIFFERENCE)
        self.assertEqual(len(orientation), 4)
        self.assertEqual(orientation[0], 0.0)
        self.assertEqual(orientation[1], 0.0)
        self.assertAlmostEqual(orientation[2], 0.9927828)
        self.assertAlmostEqual(orientation[3], 0.1199265)

        self.track.direction = TrackDirection.CLOCKWISE.value
        test_ndist = 0.1
        orientation = self.track.get_orientation(test_ndist,
                                                 ndist_mode=NdistMode.TO_FINISH_LINE,
                                                 finite_difference=FiniteDifference.FORWARD_DIFFERENCE)
        self.assertEqual(len(orientation), 4)
        self.assertEqual(orientation[0], 0.0)
        self.assertEqual(orientation[1], 0.0)
        self.assertAlmostEqual(orientation[2], 0.8967341)
        self.assertAlmostEqual(orientation[3], 0.4425698)
