from geometry_msgs.msg import Point

class World:
    """Keep track of the physics of the world."""

    def __init__(self, brick, gravity, radius, dt):
        """
        Initialize the world.

        Args:
        brick - The (x,y,z) location of the brick
        gravity - the acceleration due to gravity in m/s^2
        radius - the radius of the platform
        dt - timestep in seconds of the physics simulation
        """
        self.velocity = 0
        self._brick = brick
        self.gravity = gravity
        self.radius = radius
        self.dt = dt
        pass

    @property
    def brick(self):
        """
        Get the brick's location.

        Return:
            (x,y,z) location of the brick
        """
        return self._brick
        pass

    @brick.setter
    def brick(self, location):
        """
        Set the brick's location.

        Args:
           location - the (x,y,z) location of the brick
        """
        self._brick = location
        

    def drop(self):
        """
        Update the brick's location by having it fall in gravity for one timestep
        """
        self.velocity += -self.gravity*self.dt
        self._brick.z += self.velocity*self.dt

        if float(self.brick.z) < 0.0:
             self._brick.z = 0.0

        pass
