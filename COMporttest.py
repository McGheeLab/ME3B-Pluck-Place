import serial

try:
    print("Trying to open COM4...")
    ser = serial.Serial('COM4', 9600, timeout=1)
    print("COM4 opened successfully!")
except serial.SerialException as e:
    print(f"SerialException: {e}")
except PermissionError as e:
    print(f"PermissionError: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("COM4 closed.")
