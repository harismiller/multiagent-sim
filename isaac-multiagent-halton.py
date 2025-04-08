###################################
##### Simple Drone Simulation #####
#####     using Isaac Sim     #####
###################################
#####      Haris Miller       #####
#####     GA Tech - LIDAR     #####
###################################

## Start Simulation
import argparse
import sys
import time
import numpy as np
import csv
import json
import json
import os

import rclpy
from rclpy.node import Node
from interfaces_hmm_sim.msg import AgentPoses, Status
from ltl_automaton_msgs.msg import TransitionSystemState, TransitionSystemStateStamped, LTLPlan, RelayResponse


def get_parser():
    parser = argparse.ArgumentParser(description="Isaac Sim Multi-Agent Halton Simulation")
    parser.add_argument("--world", type=str, default="Halton_env.usd", help="World to load")
    parser.add_argument("--num_quads", type=int, default=1, help="Number of quadcopter agents")
    parser.add_argument("--num_turtle", type=int, default=1, help="Number of TurtleBot agents")
    return parser

def keyboard_event(event, *args, **kwargs):
    global start

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input == carb.input.KeyboardInput.P:
            if start:
                print("P key pressed: Pausing simulation.")
            else:
                print("P key pressed: Starting simulation.")
            start = not start
        if event.input == carb.input.KeyboardInput.X:
            print("X key pressed: Exiting simulation.")
            simulation_app.close()
            rclpy.shutdown()

def main(args, **kwargs):
    global start
    global simulation_app
    global current_directory
                
    if not args.world:
        raise ValueError("The 'world' argument must be provided and cannot be empty.")
    usd_path = os.path.join(current_directory,"environments",args.world)

    open_stage(usd_path)
    world = World()

    world.reset()

    while rclpy.ok():
        appwindow = omni.appwindow.get_default_app_window()
        input = carb.input.acquire_input_interface()
        input.subscribe_to_keyboard_events(appwindow.get_keyboard(), keyboard_event)

        # print(args.world)
        # print(args.num_quads)
        # print(args.num_turtle)

        try:
            # Simulate the world
            world.step(render=True)
        except (KeyboardInterrupt, SystemExit):
            print("Simulation interrupted. Closing simulation...")
            break

                
    

if __name__ == "__main__":
    parser = get_parser()
    args = parser.parse_args()

    import isaacsim
    from omni.isaac.kit import SimulationApp

    CONFIG = {"width": 2560, "height": 1440, "window_width": 3840, "window_height": 2160, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}
    simulation_app = SimulationApp(CONFIG)

    import carb
    import omni

    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import open_stage

    from omni.isaac.dynamic_control import _dynamic_control
    from omni.isaac.core.articulations import Articulation

    from pxr import Gf, UsdPhysics, Usd, UsdGeom, Sdf
    import omni.usd

    import drone
    import helper as hp
    from helper import elapsed_time, parse_action_sequence, print_state, read_selected_columns, load_replans
    
    current_directory = os.getcwd()
    start = False
    rclpy.init(args=sys.argv)
    main(args=args)

    simulation_app.close()
    rclpy.shutdown()
    