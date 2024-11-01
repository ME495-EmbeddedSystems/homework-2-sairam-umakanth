import pytest
from turtle_brick.physics import World
from geometry_msgs.msg import Point

@pytest.fixture
def world_instance():
    brick_position = Point(x=0, y=0, z=10)
    gravity = 9.81
    radius = 1.0
    dt = 0.1
    return World(brick_position, gravity, radius, dt)

def test_initial_velocity(world_instance):
    assert world_instance.velocity == 0, "Initial velocity should be zero."

def test_brick_setter(world_instance):
    new_position = Point(x=1, y=1, z=5)
    world_instance.brick = new_position
    assert world_instance.brick == new_position, "Brick setter did not update position correctly."

def test_drop(world_instance):
    initial_z = world_instance.brick.z
    world_instance.drop()
    # Check that brick z-position decreased
    assert world_instance.brick.z < initial_z, "Brick position did not fall as expected."
    # Check that brick z doesn't go below zero
    for _ in range(100):  # Simulate multiple drops
        world_instance.drop()
    assert world_instance.brick.z > 0.0, "Brick z-position should not be below ground level."

def test_gravity_effect(world_instance):
    world_instance.drop()
    assert world_instance.velocity < 0, "Velocity should be negative after first drop under gravity."
