import sys
from DeviceInterface import XYStageManager, ZPStageManager

def main(simulate=False):
    # Initialize stage managers
    xy = XYStageManager(simulate=simulate)
    zp = ZPStageManager(simulate=simulate)

    # Track pick/place axes (P1, P2, P3) and home reference
    p_axes = {'p1': 0.0, 'p2': 0.0, 'p3': 0.0}
    home_ref = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                'p1': 0.0, 'p2': 0.0, 'p3': 0.0}

    print("Available commands: GOTO X,Y,Z | PICK P1,P2,P3 | PLACE P1,P2,P3 | HOME | EXIT")

    try:
        while True:
            line = input("Enter command: ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].upper()

            # GOTO: move X,Y,Z relative
            if cmd == 'GOTO':
                if len(parts) != 2:
                    print("Syntax: GOTO X,Y,Z")
                    continue
                try:
                    dx, dy, dz = map(float, parts[1].split(','))
                except ValueError:
                    print("Syntax: GOTO X,Y,Z (numbers)")
                    continue

                # Query current
                x0, y0, z0 = xy.get_current_position()
                if x0 is None:
                    print("Error reading current XY position.")
                    continue
                # Move XY
                xy.move_stage_to_position(x0 + dx, y0 + dy)
                # Move Z
                zp.move_relative({'Z': dz})
                print(f"GOTO: moved by ΔX={dx}, ΔY={dy}, ΔZ={dz}")

            # PICK: positive relative move and track P axes
            elif cmd == 'PICK':
                if len(parts) != 2:
                    print("Syntax: PICK P1,P2,P3")
                    continue
                try:
                    p1, p2, p3 = map(float, parts[1].split(','))
                    if p1 < 0 or p2 < 0 or p3 < 0:
                        raise ValueError
                except ValueError:
                    print("Syntax: PICK P1,P2,P3 (positive numbers)")
                    continue

                # Query and move
                x0, y0, z0 = xy.get_current_position()
                xy.move_stage_to_position(x0 + p1, y0 + p2)
                zp.move_relative({'Z': p3})

                # Update P axes
                p_axes['p1'] += p1
                p_axes['p2'] += p2
                p_axes['p3'] += p3
                print(f"PICK: P axes moved +({p1},{p2},{p3}); current P positions: {p_axes}")

            # PLACE: negative relative move and track P axes
            elif cmd == 'PLACE':
                if len(parts) != 2:
                    print("Syntax: PLACE P1,P2,P3")
                    continue
                try:
                    p1, p2, p3 = map(float, parts[1].split(','))
                    if p1 < 0 or p2 < 0 or p3 < 0:
                        raise ValueError
                except ValueError:
                    print("Syntax: PLACE P1,P2,P3 (positive numbers)")
                    continue

                # Query and move
                x0, y0, z0 = xy.get_current_position()
                xy.move_stage_to_position(x0 - p1, y0 - p2)
                zp.move_relative({'Z': -p3})

                # Update P axes
                p_axes['p1'] -= p1
                p_axes['p2'] -= p2
                p_axes['p3'] -= p3
                print(f"PLACE: P axes moved -({p1},{p2},{p3}); current P positions: {p_axes}")

            # HOME: set current as reference
            elif cmd == 'HOME':
                x0, y0, z0 = xy.get_current_position()
                if x0 is None:
                    print("Error reading current XY position.")
                    continue
                home_ref = {
                    'x': x0, 'y': y0, 'z': z0,
                    'p1': p_axes['p1'], 'p2': p_axes['p2'], 'p3': p_axes['p3']
                }
                print(f"HOME reference set to: {home_ref}")

            # EXIT/Quit
            elif cmd in ('EXIT', 'QUIT'):
                print("Exiting.")
                break

            # Unknown
            else:
                print("Command not found. Available: GOTO, PICK, PLACE, HOME, EXIT")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting.")

    finally:
        xy.stop()
        zp.stop()


if __name__ == '__main__':
    # Pass simulate=True to test without hardware
    simulate_flag = False 
    main(simulate=simulate_flag)
