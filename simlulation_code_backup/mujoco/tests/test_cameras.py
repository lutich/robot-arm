"""The fitted 'oak' camera reproduces the real camera's calibrated view; rendering works."""
import mujoco
import numpy as np
import pytest

from armsim.cameras import (MujocoCameraSource, Renderer, camera_pixel, fit_oak_camera, load_calibration,
                            marker_points, scene_cameras)
from armsim.model_builder import build_xml
from armsim.physics import Physics


@pytest.fixture(scope='module')
def calibration():
    return load_calibration()


@pytest.fixture(scope='module')
def fit(calibration):
    return fit_oak_camera(calibration)


def test_fit_puts_the_markers_on_their_calibrated_pixels(calibration, fit):
    table, pixels = marker_points(calibration)
    assert fit['error_px'] < 3.5
    for (x, y), pixel in zip(table, pixels):
        assert np.linalg.norm(camera_pixel(fit, (x / 1000, y / 1000, 0)) - pixel) < 6


def test_fit_places_the_camera_at_the_far_edge_looking_back(fit):
    x, y, z = fit['pos']
    assert 0.4 < x < 0.7 and abs(y) < 0.1 and 0.2 < z < 0.5
    forward = -np.cross(fit['xyaxes'][:3], fit['xyaxes'][3:])
    assert forward[0] < -0.5                                  # looking toward the arm, -X


def test_rendered_marker_lands_on_the_fitted_pixel(robot, scene, calibration, fit):
    cameras = scene_cameras(scene, calibration)
    table, pixels = marker_points(calibration)
    x, y = table[3] / 1000                                     # a mid-table marker with no block on it
    xml = build_xml(robot, scene, {k: v for k, v in cameras.items() if k != 'oak_fit'})
    xml = xml.replace('</worldbody>', f'<geom name="beacon" type="sphere" pos="{x} {y} 0.002" size="0.006" rgba="1 0 1 1" contype="0" conaffinity="0"/></worldbody>')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    renderer.update_scene(data, camera='oak')
    image = renderer.render()
    renderer.close()
    r, g, b = (image[:, :, i].astype(int) for i in range(3))
    magenta = (r > g + 60) & (b > g + 60)
    assert magenta.sum() > 10
    v, u = np.argwhere(magenta).mean(axis=0)
    assert np.linalg.norm((u, v) - pixels[3]) < 12


def test_every_named_camera_renders_the_scene(robot, scene):
    physics = Physics(robot, scene, {k: v for k, v in scene_cameras(scene).items() if k != 'oak_fit'})
    renderer = Renderer(physics)
    assert set(renderer.names()) >= {'oak', 'top', 'side', 'front'}
    for name in renderer.names():
        image = renderer.render(name)
        assert image.shape == (720, 1280, 3)
        assert image.std() > 10, name
    png = renderer.png('top')
    assert png.startswith(b'\x89PNG')
    renderer.close()


def test_top_camera_sees_the_red_cube_in_red(robot, scene):
    physics = Physics(robot, scene, {k: v for k, v in scene_cameras(scene).items() if k != 'oak_fit'})
    renderer = Renderer(physics)
    image = renderer.render('top')
    red = (image[:, :, 0] > 120) & (image[:, :, 1] < 90) & (image[:, :, 2] < 90)
    assert red.sum() > 200
    renderer.close()


def test_camera_source_follows_the_robot_arm_port(robot, scene):
    physics = Physics(robot, scene, {k: v for k, v in scene_cameras(scene).items() if k != 'oak_fit'})
    renderer = Renderer(physics)
    source = MujocoCameraSource(renderer, fps=50)
    caps = source.open()
    assert caps['has_autofocus'] is False and caps['has_depth'] is False
    source.apply(dict(exposure=dict(mode='manual', time_us=8000, iso=200)))
    with pytest.raises(ValueError):
        source.apply(dict(exposure=dict(mode='manual', time_us=8000)))
    sample = source.read()
    assert sample.jpeg.startswith(b'\xff\xd8') and sample.jpeg.endswith(b'\xff\xd9')
    assert sample.observed['width'] == 1280 and sample.observed['time_us'] == 8000 and sample.depth is None
    assert source.read().observed['source_sequence'] == 2
    source.close()
    renderer.close()
