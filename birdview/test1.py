import json

import ast

def get_average_coordinates(file_path):
    x_sum = 0
    y_sum = 0
    count = 0

    try:
        with open(file_path, 'r') as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue
                
                # Safely parse the string "[x, y]" into a Python list
                try:
                    coords = ast.literal_eval(line)
                    if isinstance(coords, list) and len(coords) == 2:
                        x_sum += coords[0]
                        y_sum += coords[1]
                        count += 1
                except (ValueError, SyntaxError) as e:
                    print(f"Skipping malformed line: {line}")
                print(line)	

        if count > 0:
            avg_x = x_sum / count
            avg_y = y_sum / count
            return avg_x, avg_y, count
        else:
            return None, None, 0

    except FileNotFoundError:
        print("The file was not found.")
        return None, None, 0


file_name = './hummer_path/gps_history_before_start.txt'
avg_x, avg_y, total_lines = get_average_coordinates(file_name)

if total_lines > 0:
    print(f"Processed {total_lines} lines.")
    print(f"Average X: {avg_x}")
    print(f"Average Y: {avg_y}")

    with open("./hummer_path/gps_ref_p2.txt", 'w') as f:
        f.write("[%f,%f]"%(float(avg_x), float(avg_y)))

