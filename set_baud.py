#!/usr/bin/env python3

### Use this code to change the baud rate of the ProScan III controller via an interactive prompt. This is at 38400 which is the max that the ProScan III supports. Do not change this value. 
"""
Interactive terminal prompt to set the ProScan III controller baud rate.

Usage:
    python set_baud.py
    
The script will prompt you to enter the desired baud rate.
"""

import sys
from DeviceInterfaceAMSB import XYStageManager

def get_user_input():
    """Get baud rate from user via interactive prompt"""
    print("ProScan III Baud Rate Setter")
    print("=" * 40)
    print("Available baud rates: 9600, 19200, 38400")
    print()
    
    while True:
        try:
            user_input = input("Enter desired baud rate (or 'q' to quit): ").strip()
            
            if user_input.lower() in ['q', 'quit', 'exit']:
                print("Goodbye!")
                return None
            
            baudrate = int(user_input)
            
            if baudrate in [9600, 19200, 38400]:
                return baudrate
            else:
                print(f"❌ Invalid baud rate: {baudrate}")
                print("   Please enter 9600, 19200, or 38400")
                print()
                
        except ValueError:
            print(f"❌ Invalid input: '{user_input}'")
            print("   Please enter a number (9600, 19200, or 38400)")
            print()
        except KeyboardInterrupt:
            print("\n\nGoodbye!")
            return None

def ask_simulation_mode():
    """Ask user if they want to use simulation mode"""
    while True:
        try:
            response = input("Use simulation mode? (y/n): ").strip().lower()
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
            else:
                print("Please enter 'y' for yes or 'n' for no")
        except KeyboardInterrupt:
            print("\n\nGoodbye!")
            return None

def set_baud_rate(baudrate, simulate=False):
    """Set the ProScan controller baud rate"""
    print(f"\nSetting ProScan controller baud rate to {baudrate}...")
    
    try:
        # Create XY stage manager
        xy_stage = XYStageManager(simulate=simulate)
        
        if not simulate and not xy_stage.spo:
            print("❌ Error: No ProScan controller found. Please check connections.")
            return False
        
        if not simulate:
            print(f"📡 Connected to ProScan controller at {xy_stage.spo.baudrate} baud")
        
        # Set the baud rate
        success = xy_stage.set_baudrate(baudrate)
        
        if success:
            print(f"✅ Successfully set baud rate to {baudrate}")
            
            # Test communication
            pos = xy_stage.get_current_position()
            if pos[0] is not None:
                print(f"🔄 Communication test successful")
                print(f"   Current position: X={pos[0]}, Y={pos[1]}")
            else:
                print("⚠️  Warning: Position query failed after baud rate change")
                
        else:
            print(f"❌ Failed to set baud rate to {baudrate}")
        
        # Clean up
        xy_stage.stop()
        return success
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    """Main interactive loop"""
    try:
        while True:
            # Get baud rate from user
            baudrate = get_user_input()
            if baudrate is None:
                break
            
            # Ask about simulation mode
            simulate = ask_simulation_mode()
            if simulate is None:
                break
            
            print()
            if simulate:
                print("🔧 Running in SIMULATION mode")
            else:
                print("🔧 Running with HARDWARE")
            
            # Set the baud rate
            success = set_baud_rate(baudrate, simulate)
            
            if success:
                print(f"\n🎉 Baud rate successfully set to {baudrate}!")
            else:
                print(f"\n💥 Failed to set baud rate to {baudrate}")
            
            print("\n" + "=" * 40)
            
            # Ask if user wants to set another baud rate
            while True:
                try:
                    again = input("\nSet another baud rate? (y/n): ").strip().lower()
                    if again in ['y', 'yes']:
                        print()
                        break
                    elif again in ['n', 'no']:
                        print("Goodbye!")
                        return
                    else:
                        print("Please enter 'y' for yes or 'n' for no")
                except KeyboardInterrupt:
                    print("\n\nGoodbye!")
                    return
    
    except KeyboardInterrupt:
        print("\n\nGoodbye!")

if __name__ == "__main__":
    main()