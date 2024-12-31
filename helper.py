import numpy

def parse_action_sequence(action_sequence):
    x_coords = []
    y_coords = []
    flags = []

    last_x, last_y = None, None

    for action in action_sequence:
        if action.startswith("goto_c"):
            parts = action.split("_r")
            x = int(parts[0].split("c")[-1])
            y = int(parts[1])
            flag = 'g'
        elif action == "load":
            x, y = last_x, last_y
            flag = 'l'
        elif action == "unload":
            x, y = last_x, last_y
            flag = 'u'
        elif action == "stay":
            x, y = last_x, last_y
            flag = 's'
        else:
            print("Unknown action: {action}")
            continue

        x_coords.append(x)
        y_coords.append(y)
        flags.append(flag)

        last_x, last_y = x, y

    return x_coords, y_coords, flags

def print_state(name,gp,d,v,goal):
    if not goal:
        # print('Going to pose...')
        print(f'Quad: {name}')
        print(f'Global Pose: {gp}')
        print(f'Goal: ({d[0]}, {d[1]}, {d[2]})')
        # print(f'Velocity: {v[0]},{v[1]},{v[2]}')
        print('-------------------------------------')