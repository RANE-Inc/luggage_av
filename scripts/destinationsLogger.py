import subprocess
import time
import re
import threading

DEST_FILE = "destinations.log"

def enter_destination():
    coords_input = input("Enter destination coordinates as x,y (e.g., 2.0,0.0): ")
    try:
        x_str, y_str = coords_input.split(',')
        x = float(x_str.strip())
        y = float(y_str.strip())
    except Exception as e:
        print("Invalid input. Please enter two numbers separated by a comma.")
        return
    goal_str = ("{pose: {header: {frame_id: 'luggage_av/map'}, "
                "pose: {position: {x: %s, y: %s, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}") % (x, y)
    cmd = ["ros2", "action", "send_goal", "/luggage_av/navigate_to_pose",
           "nav2_msgs/action/NavigateToPose", goal_str]
    print("Sending goal:", goal_str)
    # Run in a separate thread if needed; here we assume sending a goal is fast enough.
    subprocess.Popen(cmd)

def log_destination():
    dest_name = input("Enter a name to log the current destination: ")
    print("Logging destination in background; it will record the pose once available...")
    thread = threading.Thread(target=log_destination_worker, args=(dest_name,))
    thread.daemon = True
    thread.start()

def log_destination_worker(dest_name):
    try:
        result = subprocess.run(["ros2", "topic", "echo", "/luggage_av/pose", "--once"],
                                capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        print(f"Timed out waiting for pose data for destination '{dest_name}'.")
        return
    output = result.stdout
    # Try to extract the x and y values from the output
    # Look for lines starting with 'x:' and 'y:' under 'position:'
    x_match = re.search(r"position:\s*\n\s*x:\s*([-\d\.e]+)", output)
    y_match = re.search(r"position:\s*\n\s*x:.*\n\s*y:\s*([-\d\.e]+)", output, re.DOTALL)
    if not x_match or not y_match:
        # Fallback: search for any lines starting with 'x:' and 'y:' (multiline)
        x_match = re.search(r"^\s*x:\s*([-\d\.e]+)", output, re.MULTILINE)
        y_match = re.search(r"^\s*y:\s*([-\d\.e]+)", output, re.MULTILINE)
    if x_match and y_match:
        try:
            x = float(x_match.group(1))
            y = float(y_match.group(1))
        except ValueError:
            print(f"Error parsing pose data for destination '{dest_name}'.")
            return
        print(f"Logged destination '{dest_name}' with coordinates: x={x}, y={y}")
        with open(DEST_FILE, "a") as f:
            f.write(f"{dest_name},{x},{y}\n")
    else:
        print(f"Could not parse pose data for destination '{dest_name}'.\nOutput:\n{output}")

def set_destination():
    try:
        with open(DEST_FILE, "r") as f:
            lines = f.readlines()
    except FileNotFoundError:
        print("No destinations have been logged yet.")
        return
    destinations = {}
    for line in lines:
        parts = line.strip().split(",")
        if len(parts) == 3:
            name, x_str, y_str = parts
            try:
                destinations[name] = (float(x_str), float(y_str))
            except ValueError:
                continue
    if not destinations:
        print("No valid destinations found.")
        return
    print("Logged destinations:")
    for name, (x, y) in destinations.items():
        print(f" - {name}: x={x}, y={y}")
    dest_choice = input("Enter the name of the destination to set: ").strip()
    if dest_choice in destinations:
        x, y = destinations[dest_choice]
        print(f"Setting destination '{dest_choice}' with coordinates: x={x}, y={y}")
        goal_str = ("{pose: {header: {frame_id: 'luggage_av/map'}, "
                    "pose: {position: {x: %s, y: %s, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}") % (x, y)
        cmd = ["ros2", "action", "send_goal", "/luggage_av/navigate_to_pose",
               "nav2_msgs/action/NavigateToPose", goal_str]
        subprocess.Popen(cmd)
    else:
        print("Destination not found.")

def action():
    print("\nChoose an action:")
    print("1. Enter a destination (manual coordinates)")
    print("2. Log the current destination (store pose with a name)")
    print("3. Set a destination (go to a stored location)")
    choice = input("Enter 1, 2, or 3: ").strip()
    if choice == "1":
        enter_destination()
    elif choice == "2":
        log_destination()
    elif choice == "3":
        set_destination()
    else:
        print("Invalid choice.")

def main():
    print("Destination Command Script")
    try:
        while True:
            action()
            print("\nWaiting before next action...")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()
