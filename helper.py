import numpy
import json
import csv
import yaml
import time

def elapsed_time(start_time):
    return time.time() - start_time

def parse_action_sequence(action_sequence, bumps):
    x_coords = []
    y_coords = []
    z_coords = []
    flags = []

    last_x, last_y = None, None

    for action in action_sequence:
        z = 2  # Default z value

        if action.startswith("goto_c"):
            parts = action.split("_r")
            x = int(parts[0].split("c")[-1])
            y = int(parts[1])
            flag = 'g'
            if (x, y) in bumps:
                z = 3  # Bump condition
        elif action == "load":
            x, y = last_x, last_y
            flag = 'l'
            z = 1  # Load condition
        elif action == "unload":
            x, y = last_x, last_y
            flag = 'u'
            z = 1.5  # Unload condition
        elif action == "stay":
            x, y = last_x, last_y
            flag = 's'
            z = 0.5  # Stay condition
        else:
            continue

        x_coords.append(x)
        y_coords.append(y)
        z_coords.append(z)
        flags.append(flag)

        last_x, last_y = x, y

    return x_coords, y_coords, z_coords, flags