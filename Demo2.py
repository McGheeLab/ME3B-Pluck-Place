#!/usr/bin/env python3
"""
Simple command-line interface for device control using DeviceInterface.py

Commands:
  GOTO X,Y,Z      -- Move XY stage by X,Y and printer Z axis by Z relative to current positions
  PICK P1,P2,P3   -- Move printer axes X += P1, Y += P2, E += P3 relative to current positions
  PLACE P1,P2,P3  -- Move printer axes X -= P1, Y -= P2, E -= P3 relative to current positions
  HOME            -- Set current stage X,Y and printer Z,X,Y,E as reference zero
"""
import sys
import re
from DeviceInterface import XYStageManager, ZPStageManager


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


def handle_goto(arg_str, xy: XYStageManager, zp: ZPStageManager, state: dict):
    try:
        dx, dy, dz = parse_floats(arg_str, 3)
    except ValueError:
        print("Syntax: GOTO X,Y,Z")
        return
    # Compute new stage positions
    tx = state['x'] + dx
    ty = state['y'] + dy
    # Move XY stage and printer Z axis
    xy.move_stage_to_position(tx, ty)
    zp.move_relative(map_printer_moves(z=dz))
    # Update state
    state.update(x=tx, y=ty, z=state['z'] + dz)
    print(f"GOTO executed: stage X={tx}, Y={ty}; printer Z={state['z']}")


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
    # Update state and set as reference
    state.update(x=x, y=y, z=pz, p1=px, p2=py, p3=pe)
    ref.update(state)
    print(f"HOME set: stage X={x}, Y={y}; printer Z={pz}, X={px}, Y={py}, E={pe}")


def print_help():
    print("Available commands:")
    print("  GOTO X,Y,Z      -- move relative stage and printer Z")
    print("  PICK P1,P2,P3   -- printer X/Y/E +")
    print("  PLACE P1,P2,P3  -- printer X/Y/E -")
    print("  HOME            -- set reference zero")
    print("  EXIT, QUIT      -- exit program")


def main():
    # Check for command line argument (filename)
    if len(sys.argv) != 2:
        print("Usage: python Demo2.py <command_file>")
        print("Example: python Demo2.py procedure2.txt")
        return
    
    filename = sys.argv[1]
    
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
                
                if cmd == 'GOTO':
                    handle_goto(args, xy, zp, state)
                elif cmd == 'PICK':
                    handle_pick(args, zp, state)
                elif cmd == 'PLACE':
                    handle_place(args, zp, state)
                elif cmd == 'HOME':
                    handle_home(xy, zp, state, reference)
                else:
                    print(f"  WARNING: Unknown command: {cmd}")
    
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"Error processing file: {e}")
    
    print("Command processing complete.")

if __name__ == '__main__':
    main()
