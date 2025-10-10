#!/usr/bin/env python3
import threading
import time

import numpy as np
import pandas as pd

from DeviceInterfaceAMSB import XYStageManager, ZPStageManager


class StageController:
    def __init__(self, csv_path, simulate=False,
                 dt_xy=0.5, dt_z=0.33, dt_ctrl=0.1,
                 Kp=0.01, Ki=0, Kd=0):
        """
        csv_path: path to raw trajectory CSV with columns [time,x,y,z]
        simulate: pass-through to your DeviceInterface managers
        dt_xy / dt_z: how often to update the open-loop setpoints
        dt_ctrl: PID control loop polling interval (seconds)
        Kp, Ki, Kd: PID gains for XY velocity control
        """
        
        self.makespiral()  
        
        # load the raw trajectory
        self.df = pd.read_csv(csv_path)
        self.simulate = simulate

        # managers
        self.xy_mgr = XYStageManager(simulate=simulate)
        self.z_mgr  = ZPStageManager(simulate=simulate)
        
        # Verify hardware initialization if not simulating
        if not simulate:
            print(f"XY Stage Manager initialized - Hardware connected: {self.xy_mgr.spo is not None}")
            print(f"Z Stage Manager initialized - Hardware connected: {self.z_mgr.COM is not None}")
            
            if self.xy_mgr.spo is None:
                print("WARNING: XY Stage hardware not found - check ProScan connection")
            if self.z_mgr.COM is None:
                print("WARNING: Z Stage hardware not found - check printer connection")

        # interpolation time steps
        self.dt_xy   = dt_xy
        self.dt_z    = dt_z
        self.dt_ctrl = dt_ctrl

        # PID state & gains for XY and P axes
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.err_sum_p = 0.0  # P1 axis error sum
        self.last_err_x = 0.0
        self.last_err_y = 0.0
        self.last_err_p = 0.0  # P1 axis last error

        # placeholders for interpolated trajectories
        self.xy_times = None
        self.xy_x     = None
        self.xy_y     = None
        self.xy_p1    = None  # P1 axis trajectory
        self.z_times  = None
        self.z_z      = None

        # logs of actual vs. ideal
        self.ideal_xy  = []   # list of (t_rel, x_target, y_target)
        self.actual_xy = []   # list of (t_rel, x_act, y_act)
        self.ideal_p   = []   # list of (t_rel, p1_target)
        self.actual_p  = []   # list of (t_rel, p1_act)
        self.ideal_p   = []   # list of (t_rel, p1_target)
        self.actual_p  = []   # list of (t_rel, p1_act)

    def makespiral(self):
                # Trajectory parameters
        duration    = 40.0       # total time [s] Should be 120 for spriral
        dt          = 0.1         # time step [s]
        turns       = 3           # number of spiral revolutions
        max_radius  = 100000.0        # final spiral radius [um]
        z_diameter  = 10.0         # circle diameter in Z [mm]
        z_amp       = z_diameter / 2.0
        p_amplitude = 0.02         # P1 axis amplitude [mm]

        # build time vector
        times = np.arange(0.0, duration + dt/2, dt)

        # spiral in XY: r grows linearly, theta runs through 'turns' revolutions
        r     = max_radius * (times / duration)
        theta = 2 * np.pi * turns * (times / duration)
        x     = r * np.cos(theta)
        y     = r * np.sin(theta)

        # circle in Z: simple sinusoid for a full period over 'duration'
        z     = z_amp * np.sin(2 * np.pi * times / duration)
        
        # P1 axis: sinusoid with different phase for pluck/place motion
        p1    = p_amplitude * np.sin(2 * np.pi * times / duration + np.pi/4)

        # assemble and save
        df = pd.DataFrame({
            'time': times,
            'x':    x,
            'y':    y,
            'z':    z,
            'p1':   p1
        })
        df.to_csv('spiral_circle_trajectory.csv', index=False)
        print("Wrote spiral_circle_trajectory.csv with", len(df), "points.")
        print(f"P1 range: {p1.min():.6f} to {p1.max():.6f} mm (amplitude: {p_amplitude})")
        print(f"First few P1 values: {p1[:5]}")
    
    
    def interpolate_trajectories(self):
        """Build uniform time axes & simple linear interp for x,y,z,p1."""
        t0, t1 = self.df['time'].iloc[0], self.df['time'].iloc[-1]
        self.xy_times = np.arange(t0, t1 + self.dt_xy, self.dt_xy)
        self.z_times  = np.arange(t0, t1 + self.dt_z,  self.dt_z)

        self.xy_x = np.interp(self.xy_times, self.df['time'], self.df['x'])
        self.xy_y = np.interp(self.xy_times, self.df['time'], self.df['y'])
        self.xy_p1 = np.interp(self.xy_times, self.df['time'], self.df['p1'])  # P1 follows XY timing
        self.z_z  = np.interp(self.z_times,  self.df['time'], self.df['z'])

    def pid_velocity(self, err_x, err_y, err_p, dt):
        """Compute PID output velocities given current errors and dt for X, Y, and P1 axes."""
        # Proportional
        P_x, P_y, P_p = self.Kp * err_x, self.Kp * err_y, self.Kp * err_p

        # Integral
        self.err_sum_x += err_x * dt
        self.err_sum_y += err_y * dt
        self.err_sum_p += err_p * dt
        I_x, I_y, I_p = self.Ki * self.err_sum_x, self.Ki * self.err_sum_y, self.Ki * self.err_sum_p

        # Derivative
        D_x = self.Kd * ((err_x - self.last_err_x) / dt if dt > 0 else 0)
        D_y = self.Kd * ((err_y - self.last_err_y) / dt if dt > 0 else 0)
        D_p = self.Kd * ((err_p - self.last_err_p) / dt if dt > 0 else 0)

        # cache for next time
        self.last_err_x, self.last_err_y, self.last_err_p = err_x, err_y, err_p

        return P_x + I_x + D_x, P_y + I_y + D_y, P_p + I_p + D_p

    def run(self):
        """Fire off XY and Z loops in parallel and wait for both to finish."""
        # record wall-clock origin
        start_wall = time.time()
        t0 = self.xy_times[0]

        # get the initial actual stage origin for XY and P1
        x0_init, y0_init, z = self.xy_mgr.get_current_position()
        zp_x, zp_y, zp_z, p1_init = self.z_mgr.get_current_position()  # P1 is the E axis in printer

        # XY and P1 control loop
        def xyp_loop():
            for i in range(len(self.xy_times) - 1):
                # next setpoint relative to CSV origin
                t_set = self.xy_times[i+1]
                x_rel, y_rel = self.xy_x[i+1], self.xy_y[i+1]
                p1_rel = self.xy_p1[i+1]
                
                # absolute targets = initial + relative
                x_target = x0_init + x_rel
                y_target = y0_init + y_rel
                p1_target = p1_init + p1_rel
                target_wall = start_wall + (t_set - t0)

                # run PID until it's time for next outer setpoint
                while True:
                    now = time.time()
                    if now >= target_wall:
                        break

                    # fetch actual positions
                    x_act, y_act, z = self.xy_mgr.get_current_position()
                    zp_x, zp_y, zp_z, p1_act = self.z_mgr.get_current_position()

                    # compute PID commanded velocities for X, Y, and P1
                    err_x = x_target - x_act
                    err_y = y_target - y_act
                    err_p1 = p1_target - p1_act
                    vx, vy, vp1 = self.pid_velocity(err_x, err_y, err_p1, self.dt_ctrl)
                    
                    # send velocity to XY stage
                    try:
                        self.xy_mgr.move_stage_at_velocity(vx, vy)
                        # Print debug info every 20 iterations to avoid spam
                        if i % 20 == 0:
                            print(f"[DEBUG] XY velocity command sent: VS,{vx:.3f},{vy:.3f}")
                    except Exception as e:
                        print(f"[ERROR] Failed to send XY velocity: {e}")
                    
                    # send P1 velocity using move_relative for continuous control
                    try:
                        # Convert velocity to small incremental movement
                        p1_increment = vp1 * self.dt_ctrl  # velocity * time = distance
                        if abs(p1_increment) > 0.0001:  # Lower threshold for small movements
                            self.z_mgr.move_relative({'E': p1_increment}, feedrate=abs(vp1 * 60))  # Convert to mm/min
                            if i % 20 == 0:
                                print(f"[DEBUG] P1 increment: {p1_increment:.4f}mm, velocity: {vp1:.4f}mm/s, error: {err_p1:.4f}")
                        else:
                            if i % 50 == 0:  # Less frequent debug for small movements
                                print(f"[DEBUG] P1 increment too small: {p1_increment:.6f}mm (threshold: 0.0001)")
                    except Exception as e:
                        print(f"[ERROR] Failed to send P1 movement: {e}")
                        if i % 20 == 0:
                            print(f"[DEBUG] P1 increment was: {p1_increment:.4f}mm, velocity: {vp1:.4f}mm/s")
                    
                    # print current position and target and velocities
                    if i % 20 == 0:  # Print every 20th iteration to reduce spam
                        print(f"Time: {now - start_wall:.2f}s | "
                              f"XY Target: ({x_target:.2f}, {y_target:.2f}) | "
                              f"XY Actual: ({x_act:.2f}, {y_act:.2f}) | "
                              f"P1 Target: {p1_target:.4f} | P1 Actual: {p1_act:.4f} | "
                              f"Velocity: XY({vx:.2f}, {vy:.2f}) P1({vp1:.4f})")
                    elif i % 100 == 0:  # Less frequent summary
                        print(f"Time: {now - start_wall:.2f}s | P1 Error: {err_p1:.4f} | P1 Velocity: {vp1:.4f}")
                    
                    # log timestamps and positions
                    t_rel = now - start_wall
                    self.ideal_xy.append((t_rel, x_target, y_target))
                    self.actual_xy.append((t_rel, x_act, y_act))
                    self.ideal_p.append((t_rel, p1_target))
                    self.actual_p.append((t_rel, p1_act))

                    time.sleep(self.dt_ctrl)

            # stop XY motion at end
            self.xy_mgr.move_stage_at_velocity(0.0, 0.0)
            print("[XYP-CONTROL] XY and P1 motion stopped")

        # Z open-loop update with debugging
        def z_loop():
            for i in range(len(self.z_times) - 1):
                t_curr, t_next = self.z_times[i], self.z_times[i+1]
                dx = self.z_z[i+1] - self.z_z[i]
                dt = t_next - t_curr

                feed = abs(dx) / dt * 60.0
                
                try:
                    self.z_mgr.move_relative({'X': dx}, feedrate=feed)
                    print(f"[Z-DEBUG] Move: dx={dx:.3f}mm, feedrate={feed:.1f}mm/min")
                except Exception as e:
                    print(f"[Z-ERROR] Failed to send Z movement: {e}")

                target = start_wall + (t_curr + dt - t0)
                wait = target - time.time()
                if wait > 0:
                    time.sleep(wait)

        # launch both loops
        t_xyp = threading.Thread(target=xyp_loop, name="XYP-PID-Loop")
        t_z  = threading.Thread(target=z_loop,  name="Z-Open-Loop")
        t_xyp.start()
        t_z.start()
        t_xyp.join()
        t_z.join()

        print("Run complete.")
        print(f"Logged {len(self.actual_xy)} PID control points.")

if __name__ == "__main__":
    print("=== PID Controller for DeviceInterfaceAMSB ===")
    
    ctrl = StageController(
        csv_path="spiral_circle_trajectory.csv",
        simulate=False,  # Set to True for simulation mode
        dt_xy=1.0,
        dt_z=0.33,
        dt_ctrl=0.1,
        Kp=0.3, Ki=0.01, Kd=0.005
    )
    
    # Hardware status check
    print("\n=== Hardware Status ===")
    try:
        x, y, z = ctrl.xy_mgr.get_current_position()
        print(f"✓ XY Stage connected - Position: X={x}, Y={y}, Z={z}")
    except Exception as e:
        print(f"✗ XY Stage error: {e}")
        
    try:
        x, y, z, e = ctrl.z_mgr.get_current_position()
        print(f"✓ Z Stage connected - Position: X={x}, Y={y}, Z={z}, E={e}")
    except Exception as e:
        print(f"✗ Z Stage error: {e}")
    
    print("\n=== Starting PID Control ===")
    ctrl.interpolate_trajectories()
    ctrl.run()

    # After run, plot ideal vs. actual accounting for initial offset
    import matplotlib.pyplot as plt
    ideal = np.array(ctrl.ideal_xy)
    actual = np.array(ctrl.actual_xy)

    plt.figure(figsize=(8,6))
    plt.plot(ideal[:,1], ideal[:,2], label="Ideal Path")
    plt.plot(actual[:,1], actual[:,2], '--', label="Actual Path")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Ideal vs Actual XY Path (offset accounted)")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()
