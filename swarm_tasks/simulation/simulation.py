import swarm_tasks.envs as envs
import swarm_tasks.utils as utils
import os
import numpy as np
import pandas as pd
import random
from shapely.geometry import Point, Polygon

DEFAULT_SIZE = (20, 20)


class Simulation:

    def __init__(
        self,
        uav_home_pos,
        env_name=None,
        num_bots=5,
        speed=utils.robot.MAX_SPEED,
        initialization="random",
        num_initial_states=1,
        contents_file=None,
        init_neighbourhood_radius=utils.robot.DEFAULT_NEIGHBOURHOOD_VAL,
    ):
        # Load world into self.env
        if env_name == None:
            self.env = envs.world.World()
            self.env_name = "rectangles"
            print("Using rectangles world")

        else:
            # Load from the full file path passed in env_name
            if not os.path.exists(env_name):
                raise FileNotFoundError(f"Environment YAML not found: {env_name}")
            self.env = envs.world.World(filename=env_name)
            self.env_name = env_name
            print(f"Loaded world from: {env_name}")

        # Set simulation parameters based on world and robots
        self.size = self.env.size

        # Load external items on top of env/world
        if contents_file == None:
            self.contents = envs.items.Contents()
            self.contents_name = "None"
            print("No external items loaded")
        else:
            self.contents = envs.items.Contents(
                filename=contents_file + ".yaml"
            )  # Add filename
            self.contents_name = contents_file

        self.has_item_moved = True  # (Used to decide whether to update polygons in viz)

        # Grid
        self.grid = np.zeros(self.env.size, dtype=bool)  # Default grid

        # Populate world with robots
        self.swarm = []
        self.num_initial_states = num_initial_states
        self.nr = init_neighbourhood_radius
        self.initial_drone_pos = uav_home_pos
        # print("initial_drone_pos",self.initial_drone_pos)
        self.speed = speed
        self.num_bots = self.populate(
            num_bots, initialization, self.num_initial_states, self.nr
        )

        # Other instance variables:
        self.time_elapsed = 0  # Number of iterations
        self.state_list = None

        print("Initialized simulation with " + str(self.num_bots) + " robots")

    def update_swarm_speed(self, speed):
        """
        Update the Simulation speed and set all bots' speed to the new value.
        Args:
            speed (float): The new speed value.
        """
        self.speed = speed
        for bot in self.swarm:
            bot.set_speed(speed)

    def populate(self, n, initialization, num_states, nr):
        """
        Populates the simulation by spawning Bot objects
        Args:
                n:	number of robots
                initialization
                num_states
                nr: neighbourhood radius of the bots
        Returns:
                size of final swarm (self.swarm)
        """
        for i in range(n):
            x, y, theta = None, None, 0
            """
			while(1):
				if initialization=='random':
					x = np.random.rand()*self.size[0]
					y = np.random.rand()*self.size[1]
					theta = np.random.rand()*2*np.pi

					state = random.randint(0,num_states-1)
				else:
					print("Failed to initialize")

				if self.check_free(x,y,utils.robot.DEFAULT_SIZE):
					break
			"""
            size = utils.robot.DEFAULT_SIZE
            initial_pos = self.initial_drone_pos
            state = random.randint(0, num_states - 1)
            self.swarm.append(
                utils.robot.Bot(
                    initial_pos[i][0],
                    initial_pos[i][1],
                    theta,
                    state=state,
                    size=size,
                    neighbourhood_radius=nr,
                    speed=self.speed,
                )
            )
            print("self.swarm", self.swarm[i])

        for bot in self.swarm:
            bot.set_sim(self)
        print(len(self.swarm))
        return len(self.swarm)

    def add_bot(self, bot_ind, bot_pos):
        fixed_state = 0
        print("ADDBOt function called!!!!!!!!!!")
        self.add_bot_pos = bot_pos
        print("add_bot_pos", self.add_bot_pos)
        # state = random.randint(0, num_states - 1)
        print("bot_ind", bot_ind)
        self.swarm.insert(
            bot_ind,
            (
                utils.robot.Bot(
                    self.add_bot_pos[0],
                    self.add_bot_pos[1],
                    theta=0,
                    state=fixed_state,
                    neighbourhood_radius=self.nr,
                    speed=self.speed,
                )
            ),
        )
        print("self.swarm", len(self.swarm))

        for bot in self.swarm:
            bot.set_sim(self)
        # Update the speed for all bots after adding a new one
        # self.update_swarm_speed(self.speed)
        print("len(self.swarm)", len(self.swarm))
        return len(self.swarm)

    def remove_bot(self, remove_index):
        print("len(self.swarm)", len(self.swarm))
        print("self.swarm", self.swarm)
        if 0 <= remove_index < len(self.swarm):
            print("remove_index", remove_index)
            removed_bot = self.swarm.pop(remove_index)
            print("remove_index", remove_index)
            print("Removed bot:", removed_bot)
            for i, bot in enumerate(self.swarm):
                print("self.swarm", self.swarm[i])
                # print("bot.set_sim(self)", bot.set_sim(self))
                bot.set_sim(self)
                print(f"Updated bot at index {i}: {bot}")
            # Update the speed for all bots after removing one
            # self.update_swarm_speed(self.speed)
            return len(self.swarm)
        else:
            print("Invalid remove_index. Index out of range.")
            return len(self.swarm)

    def check_free(self, x, y, r, ignore=None):
        """
        Checks if point (x,y) is free for
        a robot to occupy
        Args:
                x,y: Robot position
                r: Robot radius
                ignore: Robot to ignore
                (use ignore=self in Bot class to ignore collision with self)
        Returns: bool

        """
        # Check for borders of simlation
        if x < r or x > (self.size[0] - r):
            return False
        if y < r or y > (self.size[1] - r):
            return False

        # Check for obstacles in env
        for obs in self.env.obstacles:
            if obs.distance(Point(x, y)) <= r:
                return False

        # Check for external items (TODO)
        for item in self.contents.items:
            if ignore == item:
                continue
            if item.polygon.distance(Point(x, y)) <= r:
                return False

        # Check for other bots
        for bot in self.swarm:
            if ignore == bot:
                continue
            if bot.dist(x, y) < 2 * bot.size:
                return False

        return True

    def create_custom_grid(self, size, _type="bool"):
        self.grid = np.zeros(size, dtype=_type)
        return True

    def update_grid(self, r=1, new_val=True):
        """
        Updates grid values in areas of radius r around all bots
        (For area coverage)
        The value cahnges to new_val

        Returns the fraction of True values for info
        """
        for bot in self.swarm:
            x, y = bot.get_position()
            self.grid[
                max(0, int(y - r)) : min(int(y + r + 1), self.grid.shape[0]),
                max(0, int(x - r)) : min(int(x + r + 1), self.grid.shape[1]),
            ] = True

        return (np.sum(self.grid) / (self.grid.size)) * 100

    # LOG THE SIMULATION PARAMETERS (Not the same as save sim)
    def get_sim_param_log(self):

        params = str()
        params += "# SIMULATION PARAMETERS\n"
        params += "\nsize: " + str(self.size)
        params += "\nnum_bots: " + str(self.num_bots)
        params += "\nenv_name: " + str(self.env_name)
        params += "\ncontents_name: " + str(self.contents_name)

        return params + "\n\n"

    # FOR LOADED SIMULATIONS:
    # The visualizer should work for loaded sims without modifications

    def save_state(filename):
        """
        Appends the state (x,y,theta) of each robot in the swarm
        to a csv file (filename)
        """
        return True

    def save_sim(filename):
        """
        Creates a yaml file (filename.yaml) with the simulation parameters
        i.e. num_bots, size, obstacles file, items, states_file, etc.
        Also creates an empty csv file (states_file) to save states

        """
        return True

    def load_sim(filename):
        """
        Loads a simulation (env, items, etc.) from its yaml file
        Loads the first state (if exists) from the corresponding csv file
        Loads the state_list using sim.create)state_list
        """
        return True

    def load_state(state_array):
        """
        Changes robot states to the states from the paramter array
        """
        return True

    def create_state_list(filename):
        """
        Reads the csv file and creates a list of states
        for the simulation to load one by one

        Returns: Pandas dataframe
        TODO: Can this be done better without a DF?
        """

        return state_list
