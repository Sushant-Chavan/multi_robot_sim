#!/usr/bin/env python
# coding: utf-8

import os
import argparse
from multi_robot_sim.grid_generator import GridGenerator
from multi_robot_sim.utils import Utils

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("world", type=str, help="Name of the world which will be used for launching the robots")
    parser.add_argument("--nRobots", type=int, help="Number of robots to be placed", default=2)
    parser.add_argument("--model", type=str, help="Name of the URDF model to be used for the robots", default="ropod")
    parser.add_argument("--custom_poses", help="Set to True to use custom robot spawn poses", action='store_true')
    args = parser.parse_args()

    scripts_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(scripts_dir)
    config_dir = os.path.join(main_dir, 'config')
    generated_files_dir = os.path.join(main_dir, 'generated_files')

    # Load/ generate robot spawn poses
    spawn_poses = None
    if args.custom_poses:
        print("Loading custom robot spawn poses")
        init_poses_filepath = os.path.join(config_dir, 'spawn_pose_config', args.world + '_init_poses.yaml')
        if os.path.isfile(init_poses_filepath):
            spawn_poses = Utils.load_pose_list(init_poses_filepath)
        else:
            raise Exception("Config file for robot spawn poses not found at", init_poses_filepath)
        print("Loading complete")
    else:
        print("Generating robot spawn poses using the grid generator")
        world_bbox_filepath = os.path.join(config_dir, 'spawn_pose_config', args.world + '_bbox.yaml')
        if os.path.isfile(world_bbox_filepath):
            grid_gen = GridGenerator(world_bbox_filepath)
            spawn_poses = grid_gen.generate_poses()
        else:
            raise Exception("BBox config file for world", args.world, "not found at", world_bbox_filepath)
        print("Generation complete")

    Utils.reinitialise_generated_files_dir(generated_files_dir)
    Utils.generate_launch_file(spawn_poses, args.nRobots, config_dir, args.model)
    Utils.generate_rviz_config(generated_files_dir, config_dir, args.nRobots)
    Utils.generate_move_base_configs(generated_files_dir, config_dir, args.nRobots)

if __name__ == "__main__":
    main()
