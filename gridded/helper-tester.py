import helper as hp
import os

if __name__ == "__main__":
    plans_direc = "./plans"
    robot_name = "robot1"
    file_path = os.path.join(plans_direc,f"{robot_name}_as.csv")  # Update with the actual file path
    columns_to_read = ["x", "y", "z", "flag"]
    x_list, y_list, z_list, flag_list = hp.read_selected_columns(file_path, columns_to_read)

    print("x:", x_list)
    print("y:", y_list)
    print("z:", z_list)
    print("flags:", flag_list)

    # file_path = f"./plans/{robot_name}_as.yaml"  # Path to the YAML file
    # bumps = [(3,1),(4,1),(5,2),(5,3),(2,4),(3,4)]
    # x, y, z, flags = hp.parse_action_sequence_from_yaml(file_path, bumps)

    # print("x:", x)
    # print("y:", y)
    # print("z:", z)
    # print("flags:", flags)

    # output_csv = os.path.join(plans_direc,f"{robot_name}_as.csv")
    
    # hp.save_action_sequence_to_csv(file_path, bumps, output_csv)

    # print("Action sequence saved to CSV.")