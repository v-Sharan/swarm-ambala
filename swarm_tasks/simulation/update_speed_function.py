# This is the suggested method to be added to the Simulation class in simulation.py


def update_swarm_speed(self, speed):
    """
    Update the Simulation speed and set all bots' speed to the new value.
    Args:
        speed (float): The new speed value.
    """
    self.speed = speed
    for bot in self.swarm:
        bot.set_speed(speed)
