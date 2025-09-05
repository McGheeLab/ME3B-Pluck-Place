#!/usr/bin/env python3
"""
Simple command-line interface for device control using DeviceInterface.py, added Calibration functions

Commands:
  GOTO X,Y,Z      -- Move XY stage by X,Y and printer Z axis by Z relative to current positions
  PICK P1,P2,P3   -- Move printer axes X += P1, Y += P2, E += P3 relative to current positions
  PLACE P1,P2,P3  -- Move printer axes X -= P1, Y -= P2, E -= P3 relative to current positions
  HOME            -- Set current stage X,Y and printer Z,X,Y,E as reference zero
"""
import sys
import re
import json
import os
from datetime import datetime
from DeviceInterfaceAMSB import XYStageManager, ZPStageManager


def parse_floats(arg_str, count):
    """Parse a comma-separated list of floats of expected length."""
    parts = [p.strip() for p in arg_str.split(',')]
    if len(parts) != count:
        raise ValueError
    return [float(p) for p in parts]


def parse_line(line):
    """Parse a line from a text file into a command and arguments."""
    line = line.strip()
    if not line or line.startswith('#'):
        return None, None  # Skip empty lines and comments

    parts = re.split(r"\s+", line, maxsplit=1)
    cmd = parts[0].upper()
    args = parts[1] if len(parts) > 1 else ''

    return cmd, args


def map_printer_moves(z=0.0, p1=0.0, p2=0.0, p3=0.0):
    """
    Return a move dictionary for ZPStageManager mapping:
      - Z axis => 'Z'
      - P1       => 'X'
      - P2       => 'Y'
      - P3       => 'E'G
    """
    moves = {}
    if z:
        moves['X'] = z
    if p1:
        moves['E'] = p1
    if p2:
        moves['Z'] = p2
    if p3:
        moves['Y'] = p3
    return moves


################################# Calibration Functions ##################################

def load_calibration_config(config_file="CalibrationConfigAMSB.json"):
    """Load calibration configuration from JSON file."""
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
        print(f"Calibration config loaded from: {config_file}")
        return config
    except FileNotFoundError:
        print(f"Error: Calibration config file '{config_file}' not found.")
        return None
    except json.JSONDecodeError as e:
        print(f"Error parsing calibration config: {e}")
        return None


def save_calibration_config(config, config_file="CalibrationConfigAMSB.json"):
    """Save calibration configuration to JSON file."""
    try:
        # Update timestamp
        config["calibration_info"]["date"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Calibration config saved to: {config_file}")
        return True
    except Exception as e:
        print(f"Error saving calibration config: {e}")
        return False


def handle_calibrate(arg_str, xy: XYStageManager, zp: ZPStageManager, state: dict, config: dict, ref: dict):
    """
    Save the current HOME reference to the calibration config file.
    This allows GOTO 0,0,0 to return to this exact position in future runs.
    Usage: CALIBRATE (saves current HOME reference to config)
    """
    if not config:
        print("Error: No calibration config loaded")
        return
    
    if 'hw_x' not in ref or 'hw_y' not in ref or 'hw_z' not in ref:
        print("Error: No HOME reference set. Run HOME command first.")
        return
    
    # Save the hardware reference positions to config
    config["needle_reference"]["x"] = ref['hw_x']
    config["needle_reference"]["y"] = ref['hw_y'] 
    config["needle_reference"]["z"] = ref['hw_z']
    
    print(f"CALIBRATE executed: Saved HOME reference to config")
    print(f"Reference position: X={ref['hw_x']}, Y={ref['hw_y']}, Z={ref['hw_z']}")
    
    # Save updated calibration to file
    save_calibration_config(config)


#def handle_goto_well(arg_str, xy: XYStageManager, zp: ZPStageManager, state: dict, config: dict):
    """
    Move to a specific well position with calibration applied.
    Usage: GOTO_WELL A1 (moves to well A1 with offsets applied)
    """
    if not config:
        print("Error: No calibration config loaded")
        return
    
    well_name = arg_str.strip().upper()
    if not well_name:
        print("Syntax: GOTO_WELL <well_name> (e.g., GOTO_WELL A1)")
        return
    
    # Check if well exists in config
    wells = config.get("well_plate_positions", {})
    if well_name not in wells:
        print(f"Error: Well '{well_name}' not found in calibration config")
        available_wells = list(wells.keys())
        print(f"Available wells: {available_wells}")
        return
    
    # Get well position and needle reference
    well_pos = wells[well_name]
    needle_ref = config["needle_reference"]
    
    # Calculate absolute positions (well position + needle reference)
    target_x = needle_ref["x"] + well_pos["x"]
    target_y = needle_ref["y"] + well_pos["y"]
    target_z = well_pos["z"]  # Z is relative to needle reference Z
    
    # Move to position
    xy.move_stage_to_position(target_x, target_y)
    zp.move_relative(map_printer_moves(z=target_z - state['z']))
    
    # Update state
    state.update(x=well_pos["x"], y=well_pos["y"], z=target_z)
    
    print(f"GOTO_WELL executed: Moved to {well_name} at relative position X={well_pos['x']}, Y={well_pos['y']}, Z={target_z}")


def handle_load_config(arg_str, config_dict):
    """
    Load a specific calibration config file.
    Usage: LOAD_CONFIG <filename> (e.g., LOAD_CONFIG my_calibration.json)
    """
    config_file = arg_str.strip()
    if not config_file:
        config_file = "CalibrationConfigAMSB.json"
    
    new_config = load_calibration_config(config_file)
    if new_config:
        config_dict.clear()
        config_dict.update(new_config)
        print(f"Configuration loaded from {config_file}")
        return True
    return False


def handle_goto(arg_str, xy: XYStageManager, zp: ZPStageManager, state: dict, config: dict, ref: dict):
    try:
        dx, dy, dz = parse_floats(arg_str, 3)
    except ValueError:
        print("Syntax: GOTO X,Y,Z")
        return
    
    # Check if this is a goto 0,0,0 and we have calibration loaded
    if dx == 0.0 and dy == 0.0 and dz == 0.0 and config and "needle_reference" in config:
        # Go to the calibrated reference position
        needle_ref = config["needle_reference"]
        target_x = needle_ref["x"]
        target_y = needle_ref["y"] 
        target_z = needle_ref["z"]
        
        xy.move_stage_to_position(target_x, target_y)
        zp.move_relative(map_printer_moves(z=target_z - state['z']))
        
        # Update state to show we're at software (0,0,0)
        state.update(x=0.0, y=0.0, z=0.0)
        print(f"GOTO executed: Moved to calibrated reference (0,0,0)")
        print(f"Hardware position: X={target_x}, Y={target_y}, Z={target_z}")
    else:
        # Normal relative movement
        tx = state['x'] + dx
        ty = state['y'] + dy
        tz = state['z'] + dz
        
        # If we have calibration and a hardware reference, calculate absolute positions
        if config and "needle_reference" in config and 'hw_x' in ref:
            needle_ref = config["needle_reference"]
            abs_x = needle_ref["x"] + tx
            abs_y = needle_ref["y"] + ty
            xy.move_stage_to_position(abs_x, abs_y)
        else:
            # Fallback to relative movement from current position
            xy.move_stage_to_position(tx, ty)
        
        zp.move_relative(map_printer_moves(z=dz))
        # Update state
        state.update(x=tx, y=ty, z=tz)
        print(f"GOTO executed: software position X={tx}, Y={ty}, Z={tz}")


def handle_pick(arg_str, zp: ZPStageManager, state: dict):
    try:
        p1, p2, p3 = parse_floats(arg_str, 3)
    except ValueError:
        print("Syntax: PICK P1,P2,P3")
        return
    # Move printer axes negatively via mapping function
    zp.move_relative(map_printer_moves(p1=-p1, p2=-p2, p3=-p3))
    # Update state
    state['p1'] -= p1
    state['p2'] -= p2
    state['p3'] -= p3
    print(f"PICK executed: printer X-={p1}, Y-={p2}, E-={p3}")


def handle_place(arg_str, zp: ZPStageManager, state: dict):
    try:
        p1, p2, p3 = parse_floats(arg_str, 3)
    except ValueError:
        print("Syntax: PLACE P1,P2,P3")
        return
    # Move printer axes using mapping function
    zp.move_relative(map_printer_moves(p1=p1, p2=p2, p3=p3))
    # Update state
    state['p1'] += p1
    state['p2'] += p2
    state['p3'] += p3
    print(f"PLACE executed: printer X+={p1}, Y+={p2}, E+={p3}")


def handle_home(xy: XYStageManager, zp: ZPStageManager, state: dict, ref: dict):
    # Read current stage and printer positions
    x, y, _ = xy.get_current_position()
    px, py, pz, pe = zp.get_current_position()
    # Set current position as software reference (0,0,0)
    state.update(x=0.0, y=0.0, z=0.0, p1=px, p2=py, p3=pe)
    # Store the actual hardware positions as reference
    ref.update(hw_x=x, hw_y=y, hw_z=pz, x=0.0, y=0.0, z=0.0, p1=px, p2=py, p3=pe)
    print(f"HOME set: Current position is now software (0,0,0)")
    print(f"Hardware reference: stage X={x}, Y={y}; printer Z={pz}")


def print_help():
    print("Available commands:")
    print("  CALIBRATE           -- set current position as needle reference (0,0,0)")
    print("  GOTO X,Y,Z          -- move relative stage and printer Z")
    print("  GOTO_WELL <name>    -- move to well position (e.g., GOTO_WELL A1)")
    print("  PICK P1,P2,P3       -- printer X/Y/E -")
    print("  PLACE P1,P2,P3      -- printer X/Y/E +")
    print("  HOME                -- set reference zero")
    print("  LOAD_CONFIG <file>  -- load calibration config file")
    print("  EXIT, QUIT          -- exit program")


def main():
    # Run CameraFeedAMSB.py first and wait for it to close
    import subprocess
    import sys
    print("Launching CameraFeedAMSB.py for calibration...")
    subprocess.run([sys.executable, "CameraFeedAMSB.py"])

    # Check for command line argument (filename)
    if len(sys.argv) != 2:
        print("Usage: python CalibrationCodeAMSB.py <command_file>")
        print("Example: python CalibrationCodeAMSB.py DemoInstructionsAMSB.txt")
        return

    filename = sys.argv[1]

    # Load calibration configuration
    calibration_config = load_calibration_config()
    if not calibration_config:
        print("Warning: Running without calibration config")
        calibration_config = {}

    # Instantiate managers
    xy = XYStageManager(simulate=False)
    zp = ZPStageManager(simulate=False)

    # State: stage X,Y; printer Z; P1->printer X, P2->printer Y, P3->printer E
    state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'p1': 0.0, 'p2': 0.0, 'p3': 0.0}
    reference = state.copy()

    print(f"Processing commands from: {filename}")

    try:
        with open(filename, 'r') as file:
            for line_num, line in enumerate(file, 1):
                cmd, args = parse_line(line)

                # Skip empty lines and comments
                if cmd is None:
                    continue

                print(f"Line {line_num}: {line.strip()}")

                if cmd == 'CALIBRATE':
                    handle_calibrate(args, xy, zp, state, calibration_config, reference)
                elif cmd == 'GOTO':
                    handle_goto(args, xy, zp, state, calibration_config, reference)
                elif cmd == 'GOTO_WELL':
                    handle_goto_well(args, xy, zp, state, calibration_config)
                elif cmd == 'PICK':
                    handle_pick(args, zp, state)
                elif cmd == 'PLACE':
                    handle_place(args, zp, state)
                elif cmd == 'HOME':
                    handle_home(xy, zp, state, reference)
                elif cmd == 'LOAD_CONFIG':
                    handle_load_config(args, calibration_config)
                else:
                    print(f"  WARNING: Unknown command: {cmd}")

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"Error processing file: {e}")

    print("Command processing complete.")

if __name__ == '__main__':
    main()
