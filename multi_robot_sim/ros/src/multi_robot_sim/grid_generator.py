import numpy as np
import os
import sys
import yaml

class GridGenerator:

    """Generate a grid of robots considering num of robots and walls

    :world_filepath: str

    """

    def __init__(self, world_filepath):
        self._world_filepath = world_filepath

        self.bottom_left = None
        self.top_right = None
        self.orientation = None
        self.grid_spacing = None

    def load_config_file(self):
        yaml_data = None
        with open(self._world_filepath, 'r') as stream:
            try:
                yaml_data =  yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        if yaml_data is not None:
            self.orientation = yaml_data["Orientation"]
            self.grid_spacing = yaml_data["Spacing"]
            self.bottom_left = yaml_data["BBox"]["BL"]
            self.top_right = yaml_data["BBox"]["TR"]

    def generate_poses(self):
        self.load_config_file()

        x_lims = sorted([self.bottom_left[0], self.top_right[0]])
        y_lims = sorted([self.bottom_left[1], self.top_right[1]])

        # Since np.arange does not include the stop value, add the grid spacing to include it.
        x_pos = np.arange(x_lims[0], x_lims[1] + self.grid_spacing, self.grid_spacing)
        y_pos = np.arange(y_lims[0], y_lims[1] + self.grid_spacing, self.grid_spacing)

        mesh = np.meshgrid(x_pos, y_pos)

        pose_list = []
        for i in range(mesh[0].shape[0]):
            for j in range(mesh[0].shape[1]):
                x = mesh[0][i, j].astype(float)
                y = mesh[1][i, j].astype(float)
                pose_list.append([x, y, self.orientation])

        return pose_list
