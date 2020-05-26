import os
import numpy as np
from matplotlib import cm
import glob
import shutil
import yaml

class Utils(object):

    """Utility functions for multi robot simulation"""

    @staticmethod
    def generate_rviz_config(generated_files_dir, config_dir, num_of_robots,
                             filename="multi_robot_sim"):
        """Generate rviz config file containing all robots along with their
           necessary topics

        :generated_files_dir: str
        :config_dir: str
        :num_of_robots: int
        :filename:str
        :returns: None

        """
        print("Generating RViz config file")
        rviz_config_file = os.path.join(generated_files_dir, filename + '.rviz')
        with open(rviz_config_file, 'w') as rviz_cfg:
            rviz_config_dir = os.path.join(config_dir, "rviz_config")
            pre_group_cfg = os.path.join(rviz_config_dir, "pre_group_config.yaml")
            post_group_cfg = os.path.join(rviz_config_dir, "post_group_config.yaml")
            robot_group_cfg = os.path.join(rviz_config_dir, "robot_group_config.yaml")

            # Write the rviz config sections that appear before the robot configurations 
            with open(pre_group_cfg, 'r') as pre_group:
                rviz_cfg.write(pre_group.read())

            # Write configurations for each individual robots
            with open(robot_group_cfg, 'r') as robot_group:
                group_description = robot_group.read()
                cmap =  cm.get_cmap('gist_rainbow')
                color_id = np.linspace(0.0, 1.0, num_of_robots)
                for i in range(num_of_robots):
                    color = (np.array(cmap(color_id[i]))[0:3] * 255).astype(int)
                    data = {'id':"{}".format(i+1), 'r':color[0], 'g':color[1], 'b':color[2]}
                    rviz_cfg.write(group_description.format(**data))

            # Write th rviz config that should follow the robots configurations
            with open(post_group_cfg, 'r') as post_group:
                rviz_cfg.write(post_group.read())

    @staticmethod
    def generate_move_base_configs(generated_files_dir, config_dir, num_of_robots):
        """Generate costmap config files for each robot

        :generated_files_dir: str
        :config_dir: str
        :num_of_robots: int
        :returns: None

        """
        print("Generating move_base config files")
        move_base_config_dir = os.path.join(config_dir, "move_base_config")

        # Read costmap param template files
        move_base_params = {}
        param_names = ["costmap_common_params", "global_costmap_params", "local_costmap_params"]
        for param_name in param_names:
            param_filepath = os.path.join(move_base_config_dir, param_name + '_template.yaml')
            with open(param_filepath, 'r') as f:
                move_base_params[param_name] = f.read()

        # Create new costmap param files for all the robots
        for i in range(num_of_robots):
            # Create a directory for the current robot config
            id_num = "{}".format(i+1)
            dir_name = os.path.join(generated_files_dir, "robot_" + id_num)
            os.mkdir(dir_name)
            # Generate the costmap param files
            data = {'id':id_num}
            for param_name in param_names:
                param_filepath = os.path.join(dir_name, param_name + '.yaml')
                with open(param_filepath, 'w') as f:
                    f.write(move_base_params[param_name].format(**data))

    @staticmethod
    def reinitialise_generated_files_dir(generated_files_dir):
        """Deletes all files inside `generated_files_dir` directory.
           Assumption: generated_files_dir is an absolute path

        :generated_files_dir: str
        :returns: None

        """
        print("Clearing previously generated config files")
        shutil.rmtree(generated_files_dir, ignore_errors=True)
        os.mkdir(generated_files_dir)

    @staticmethod
    def load_pose_list(pose_filename):
        """Load the YAML file containing the custom spawn poses for the robots

        :pose_filename: str

        """
        pose_list = None
        if os.path.isfile(pose_filename):
            with open(pose_filename) as f:
                pose_list = yaml.load(f)
        else:
            raise Exception("ERROR! YAML pose file not found at", pose_filename)
        return pose_list

    @staticmethod
    def get_rviz_robot_group_config(id, x, y, theta, model_name, config_dir):
        """Generate the robot group config string to be used in the RViz launch file

        :id: str
        :x: float
        :y: float
        :theta: float
        :model_name: str
        :config_dir: str

        """
        data = {'id': id, 'x': x, 'y': y, 'theta': theta, 'model': model_name}
        launch_config_dir = os.path.join(config_dir, 'launch_config')
        robot_group_filepath = os.path.join(launch_config_dir, 'robot_group')
        with open(robot_group_filepath, 'r') as file_obj:
            robot_group_str = file_obj.read()

        return robot_group_str.format(**data)

    @staticmethod
    def generate_launch_file(pose_list, num_robots, config_dir, model_name, filename="spawn_multiple_robots"):
        """Generate a RViz launch file that can load multiple robots with unique namespaces

        :pose_list: list[list[float]]
        :num_robots: int
        :config_dir: str
        :model_name: str
        :filename: str

        """
        print("Generating launch files")
        if pose_list is not None:
            max_robots = len(pose_list)
            if num_robots > max_robots:
                print("WARNING: Cannot generate launch file for more than", max_robots, "robots")
                print("Limiting number of robots to", max_robots, "robots")
            main_dir = os.path.dirname(config_dir)
            generated_files_dir = os.path.join(main_dir, 'generated_files')
            launch_filepath = os.path.join(generated_files_dir, filename + '.launch')

            # read the header for launch file
            launch_config_dir = os.path.join(config_dir, 'launch_config')
            pre_group_filepath = os.path.join(launch_config_dir, 'pre_group')
            with open(pre_group_filepath, 'r') as file_obj:
                pre_group_str = file_obj.read()

            # Open file in write mode to overwrite existing contents
            with open(launch_filepath, 'w') as f:
                f.write(pre_group_str)

            # Open file in append mode to append robot configurations
            with open(launch_filepath, 'a') as f:
                for i, pose in enumerate(pose_list):
                    robot_id = i+1
                    f.write(Utils.get_rviz_robot_group_config("{}".format(robot_id), 
                                                              pose[0], pose[1], pose[2],
                                                              model_name, config_dir))
                    if robot_id >= num_robots:
                        break
                f.write("</launch>")
        else:
            raise Exception("Invalid robot spawn pose list!")
