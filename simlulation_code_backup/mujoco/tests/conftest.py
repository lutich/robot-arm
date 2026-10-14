import pytest

from armsim.config import load_robot, load_scene
from armsim.physics import Physics


@pytest.fixture(scope='session')
def robot():
    return load_robot()


@pytest.fixture(scope='session')
def scene():
    return load_scene()


@pytest.fixture
def physics(robot, scene):
    return Physics(robot, scene)
