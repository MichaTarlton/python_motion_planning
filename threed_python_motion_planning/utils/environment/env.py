"""
@file: env.py
@breif: 2-dimension environment
@author: Winter
@update: 2023.1.13
"""
from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np

from .node import Node

class Env(ABC):
    """
    Class for building 2-d workspace of robots.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        eps (float): tolerance for float comparison

    Examples:
        >>> from utils import Env
        >>> env = Env(30, 40)
    """
    def __init__(self, x_range: int, y_range: int, z_range: int, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range
        self.z_range = z_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        return {(i, j, k) for i in range(self.x_range) for j in range(self.y_range) for k in range(self.z_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid(Env):
    """
    Class for discrete 2-d grid map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        z_range (int): z-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int, z_range: int ) -> None:
        super().__init__(x_range, y_range, z_range)
        # allowed motions
        self.motions = [

            # Node((-1, 0, 0), None, 1, None), Node((-1, 1, 0),  None, sqrt(2), None),
            # Node((0, 1, 0),  None, 1, None), Node((1, 1, 0),   None, sqrt(2), None),
            # Node((1, 0, 0),  None, 1, None), Node((1, -1, 0),  None, sqrt(2), None),
            # Node((0, -1, 0), None, 1, None), Node((-1, -1, 0), None, sqrt(2), None)
            # ]

            #! For those lookng to copy this repo see the notes on what you are setting the path cost as
            # Node((dx, dy, dz), None, sqrt(dx**2 + dy**2 + dz**2), None) #! This one assumes that the cost is a (normalized) sum of movements, as if it can only move from one cube to the next
            
            Node((dx, dy, dz), None, sqrt(2), None) # this assumes that all directions (including diagonals) cost the same amount. This is more like assuming a spherical field of movement
            for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1]
            if not (dx == 0 and dy == 0 and dz == 0) #! Don't include this if you want to a "do not move step", which you may actually want to give "no cost" or perhaps play around with different levels of costs 
            ]
        # obstacles
        self.obstacles = None
        self.obstacles_tree = None
        self.init()
    
    def init(self) -> None:
        """
        Initialize grid map.
        """
        x, y, z = self.x_range, self.y_range, self.z_range
        obstacles = set()

        # boundary of environment
        
        # for i in range(x): lol nope
        #     obstacles.add((i, 0, 0))
        #     obstacles.add((i, y - 1, z -1))
        # for i in range(y):
        #     obstacles.add((0, i, 0))
        #     obstacles.add((x - 1, i, z - 1))
        # for i in range(z):
        #     obstacles.add((0, 0, i))
        #     obstacles.add((x - 1, i)

        # for i in range(z):
        #     for j in range(x):
        #         obstacles.add((j, 0, i))
        #         obstacles.add((j, y-1, i)) # why do we set these as -1 the range we want
        #     for j in range(y):
        #         obstacles.add((0, j, i))
        #         obstacles.add((x-1, j, i))

        # for i in range(z):
        #     for j in range(x):
        #             obstacles.add((i,j,0))
        #             obstacles.add((i,j,z-1))
#! Alright I tried it my way and it doesn't work
#! Nope this does not seem to init boundaries either
        for i in range(x):
            for j in range(y):
                obstacles.add((i, j, 0))  # Floor
                obstacles.add((i, j, z - 1))  # Ceiling

        # XZ faces
        for i in range(x):
            for k in range(z):
                obstacles.add((i, 0, k))  # Front wall
                obstacles.add((i, y - 1, k))  # Back wall

        # YZ faces
        for j in range(y):
            for k in range(z):
                obstacles.add((0, j, k))  # Left wall
                obstacles.add((x - 1, j, k))  # Right wall


        self.update(obstacles)

    def update(self, obstacles):
        self.obstacles = obstacles 
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))


class Map(Env):
    """
    Class for continuous 3-d map.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        z_range (int): z-axis range of environmet
    """
    def __init__(self, x_range: int, y_range: int, z_range: int) -> None:
        super().__init__(x_range, y_range, z_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        """
        Initialize map.
        """
        x, y, z = self.x_range, self.y_range, self.z_range

        # boundary of environment
        #? HOLY FUCK ME DOCUMENT WHAT THE FUCK THIS MEANS
        #? For those here apparently this is describing line to be drawn
        #? Each vector draws a rectangle
        #? Where origin coordinates (x,y) are the first two places, and the last two are the widht and height of the rectangle
        #? From line 61 of plot.py: "for (ox, oy, w, h) in self.env.boundary:"
        #? they also make it more confusing by not overlapping boundaries, instead moving one w/h to the side, thus the 1 values
        #? this is crazy hard to keep track of unless you are visualizing it
        #? just add an extra +1 to the end of each range and let god sort it out
        self.boundary = [ 
            [0, 0, 0, x+1, y+1, 1],  # Floor
            [0, 0, z, x+1, y+1, 1],   # ceiling

            [0, 0, 0, 1, y+1, z+1],  # Left wall 
            [0, 0, 0, x+1, 1, z+1],  # Front wall

            [0, y, 0, x+1, 1, z+1],  # Back wall
            [x, 0, 0, 1, y+1, z+1],  # Right wall
        ]
        self.obs_rect = []
        self.obs_circ = []
        #? JFYI (ox, oy, r) in self.env.obs_circ

    def update(self, boundary=None, obs_circ=None, obs_rect=None):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
