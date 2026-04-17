import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from app.calibration.transforms import apply_homography, compute_homography


def test_homography_maps_rectangle_corners():
    image_points = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)]
    world_points = [(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)]
    homography = compute_homography(image_points, world_points)

    x_mm, y_mm = apply_homography((50.0, 25.0), homography)
    assert round(x_mm, 2) == 100.0
    assert round(y_mm, 2) == 50.0
