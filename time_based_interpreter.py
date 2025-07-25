import re
import time
# We import the manager classes directly from your provided file.
from DeviceInterface import XYStageManager, ZPStageManager

# --- Command Execution Functions ---
# These functions directly call the methods from the DeviceInterface classes.

def handle_goto_xy(xy_stage, params):
    """
    Interprets the GOTO_XY command and uses the xy_stage object to execute it.
    """
    if len(params) != 3:
        print(f"  [Error] GOTO_XY requires 3 arguments (X, Y, time), but got {len(params)}.")
        return

    target_x, target_y, duration_s = params
    print(f"Executing GOTO_XY: Target=({target_x}, {target_y}) over {duration_s}s")

    if duration_s <= 0:
        print("  [Error] Duration must be positive.")
        return

    # 1. Use the get_current_position() method from XYStageManager
    start_x, start_y, _ = xy_stage.get_current_position()
    print(f"  Start position: ({start_x:.2f}, {start_y:.2f})")

    # 2. Calculate velocities required for the time-based move
    delta_x = target_x - start_x
    delta_y = target_y - start_y
    vx = delta_x / duration_s  # velocity in microns/sec
    vy = delta_y / duration_s  # velocity in microns/sec
    print(f"  Calculated velocity: (vx={vx:.2f}, vy={vy:.2f}) µm/s")

    # 3. Use the move_stage_at_velocity() method from XYStageManager
    xy_stage.move_stage_at_velocity(vx, vy)

    # 4. Wait for the move to complete
    time.sleep(duration_s)

    # 5. Stop the stage by calling move_stage_at_velocity() again with zero velocity
    xy_stage.move_stage_at_velocity(0, 0)
    print("  GOTO_XY complete.")


def handle_goto_z(zp_stage, params):
    """
    Interprets the GOTO_Z command and uses the zp_stage object to execute it.
    """
    if len(params) != 2:
        print(f"  [Error] GOTO_Z requires 2 arguments (Z, time), but got {len(params)}.")
        return

    target_z, duration_s = params
    print(f"Executing GOTO_Z: Target=({target_z}) over {duration_s}s")

    if duration_s <= 0:
        print("  [Error] Duration must be positive.")
        return

    # 1. Use the get_current_position() method from ZPStageManager
    _, _, start_z, _ = zp_stage.get_current_position()
    print(f"  Start position: Z={start_z:.2f}")

    # 2. Calculate relative distance and required feedrate for the G-code command
    delta_z = target_z - start_z
    if delta_z == 0:
        print("  Target Z is the same as current Z. No move needed.")
        return

    velocity_mms = delta_z / duration_s      # velocity in mm/sec
    feedrate_mmpm = abs(velocity_mms * 60) # feedrate in mm/min (must be positive)
    print(f"  Calculated feedrate: {feedrate_mmpm:.2f} mm/min for a distance of {delta_z:.2f} mm")

    # 3. Use the movecommand() method from ZPStageManager.
    #    This method sends a relative G-code move.
    zp_stage.movecommand({'Z': delta_z}, feedrate=feedrate_mmpm)

    # 4. Wait for the move to complete
    time.sleep(duration_s)
    print("  GOTO_Z complete.")


def handle_wait(params):
    """Interprets and executes the WAIT command."""
    if not params:
        print("  [Error] WAIT requires a duration in seconds.")
        return
    duration = params[0]
    print(f"Executing WAIT: Pausing for {duration} seconds...")
    time.sleep(duration)
    print("  WAIT complete.")


# --- Parsing and Main Execution ---

def parse_line(line):
    """Parses a line into a command and its float parameters."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None, None

    match = re.search(r'^(\w+):\s*(.*?);', line)
    if not match:
        return None, None

    command = match.group(1).upper()
    values_str = match.group(2)
    params = [float(num) for num in re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', values_str)]
    return command, params


def main(filepath='procedure2.txt'):
    """
    Main function to initialize stages and run the interpreter.
    """
    # Initialize the stage managers from DeviceInterface.py in simulation mode.
    # This automatically starts their internal threads.
    xy_stage = XYStageManager(simulate=True)
    zp_stage = ZPStageManager(simulate=True)

    print("--- Initializing Bioprinter Control System (Simulated) ---")
    time.sleep(1) # Allow simulator threads to start up properly

    try:
        print(f"\n--- Reading and Executing Procedure from '{filepath}' ---")
        with open(filepath, 'r') as f:
            for i, line in enumerate(f):
                command, params = parse_line(line)
                if not command:
                    continue

                print(f"\n[Line {i+1}] Processing: {line.strip()}")
                if command == "GOTO_XY":
                    handle_goto_xy(xy_stage, params)
                elif command == "GOTO_Z":
                    handle_goto_z(zp_stage, params)
                elif command == "WAIT":
                    handle_wait(params)
                else:
                    print(f"  [Warning] Unknown command: '{command}'")

        print("\n--- Procedure file finished. ---")

    except FileNotFoundError:
        print(f"[FATAL ERROR] The procedure file was not found at '{filepath}'.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("\n--- Shutting down system. ---")
        # Deleting the objects will trigger their __del__ methods,
        # which correctly stops the simulator threads.
        del xy_stage
        del zp_stage
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
