import sys
import cv2
import numpy as np
import time

# ------------------- Global Calibration Parameters -------------------
gamma = 1.0       # Gamma correction
brightness = 0    # Brightness offset
contrast = 1.0    # Contrast multiplier

# Desired width/height for camera frames.
frame_width = 320
frame_height = 240

def get_teslong_camera_indices():
    """
    Discover camera indices that match "Teslong Camera".
    """
    indices = []
    print("Scanning for cameras...")
    
    try:
        from pygrabber.dshow_graph import FilterGraph
        graph = FilterGraph()
        devices = graph.get_input_devices()
        print(f"Found {len(devices)} video devices:")
        for i, dev in enumerate(devices):
            print(f"  {i}: {dev}")
            
            if "Teslong Camera" in dev:
                indices.append(i)
                print(f"    -> Added Teslong camera at index {i}")
    except Exception as e:
        print(f"pygrabber not available, using fallback method: {e}")
        # Fallback: try indices 0 to 9 and look for ALL working cameras
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"  Camera {i}: Working")
                    indices.append(i)
                else:
                    print(f"  Camera {i}: Found but can't read frames")
                cap.release()
            else:
                print(f"  Camera {i}: Not available")
    
    if indices:
        print(f"Selected camera indices: {indices}")
    else:
        print("No working cameras found!")
    
    return indices

def adjust_image(img, gamma_val, brightness_val, contrast_val):
    """
    Adjust brightness, contrast and gamma of the image.
    """
    adjusted = cv2.convertScaleAbs(img, alpha=contrast_val, beta=brightness_val)
    if gamma_val <= 0:
        gamma_val = 0.1
    invGamma = 1.0 / gamma_val
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in range(256)]).astype("uint8")
    adjusted = cv2.LUT(adjusted, table)
    return adjusted

def overlay_grid(img):
    """
    Overlays a 3x3 grid on the frame.
    """
    h, w, _ = img.shape
    cell_w = w / 3
    cell_h = h / 3
    for i in range(3):
        for j in range(3):
            pt1 = (int(j * cell_w), int(i * cell_h))
            pt2 = (int((j + 1) * cell_w), int((i + 1) * cell_h))
            color = (0, 0, 255) if (i == 1 and j == 1) else (255, 255, 255)
            cv2.rectangle(img, pt1, pt2, color, 2)
    return img

def initialize_cameras(indices):
    """
    Initialize cameras with proper error handling and validation.
    """
    caps = []
    working_indices = []
    
    print(f"Attempting to initialize {len(indices)} camera(s): {indices}")
    
    for idx in indices:
        print(f"Attempting to initialize camera {idx}...")
        try:
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                print(f"  Camera {idx}: Failed to open")
                continue
            
            # Set camera properties for better stability
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to prevent lag
            
            # Test if we can actually read frames
            ret, test_frame = cap.read()
            if ret and test_frame is not None:
                print(f"  Camera {idx}: Successfully initialized (frame size: {test_frame.shape})")
                caps.append(cap)
                working_indices.append(idx)
            else:
                print(f"  Camera {idx}: Can't read frames (ret={ret}, frame={test_frame is not None})")
                cap.release()
                
        except Exception as e:
            print(f"  Camera {idx}: Error during initialization - {e}")
            if 'cap' in locals() and cap:
                cap.release()
    
    print(f"Successfully initialized {len(caps)} camera(s): {working_indices}")
    
    if len(caps) == 1:
        print("WARNING: Only 1 camera initialized - you should see 2 Teslong cameras")
    elif len(caps) == 2:
        print("SUCCESS: Both cameras initialized - you should see side-by-side feeds")
    elif len(caps) == 0:
        print("ERROR: No cameras could be initialized")
    
    return caps

def display_separate_cameras(caps):
    """
    Display each camera in its own separate window with adjustments and grid.
    """
    global gamma, brightness, contrast
    
    for i, cap in enumerate(caps):
        try:
            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"Warning: Camera {i} failed to read frame")
                frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
                cv2.putText(frame, f"Camera {i} Error", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                frame = cv2.resize(frame, (frame_width, frame_height))
                frame = adjust_image(frame, gamma, brightness, contrast)
                frame = overlay_grid(frame)
            
            # Add text overlay with current settings and instructions
            text_y = 30
            cv2.putText(frame, f"Camera {i+1}", (10, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            text_y += 25
            cv2.putText(frame, f"Gamma: {gamma:.1f}", (10, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            text_y += 15
            cv2.putText(frame, f"Brightness: {brightness}", (10, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            text_y += 15
            cv2.putText(frame, f"Contrast: {contrast:.1f}", (10, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Add calibration instruction on each frame
            instruction_y = frame.shape[0] - 60
            cv2.putText(frame, "Line needle up to red corner", (10, instruction_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            instruction_y += 20
            cv2.putText(frame, "Press Y to accept calibration", (10, instruction_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            instruction_y += 20
            cv2.putText(frame, "Y=Accept, N=Reject, Q=Quit", (10, instruction_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            # Display in separate window
            window_name = f"Camera {i+1} - Calibration"
            cv2.imshow(window_name, frame)
            
        except Exception as e:
            print(f"Error processing camera {i}: {e}")
            error_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
            cv2.putText(error_frame, f"Cam {i+1} Error", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            window_name = f"Camera {i+1} - Error"
            cv2.imshow(window_name, error_frame)

def start_camera_feed_with_calibration():
    """
    Start camera feed in terminal mode with calibration acceptance option.
    Returns True if calibration is accepted, False otherwise.
    """
    print("Starting camera feed...")
    print("Controls:")
    print("  Press 'y' or 'Y' to accept calibration and exit")
    print("  Press 'n' or 'N' to reject calibration and exit")
    print("  Press 'q' to quit without calibration")
    print("  Press '+' to increase brightness")
    print("  Press '-' to decrease brightness")
    print("  Press 'g' to increase gamma")
    print("  Press 'G' to decrease gamma")
    print("  Press 'c' to increase contrast")
    print("  Press 'C' to decrease contrast")
    print("\nInitializing cameras...")
    
    # Get camera indices
    indices = get_teslong_camera_indices()
    if not indices:
        print("No cameras found.")
        return False
    
    # Initialize cameras with proper error handling
    caps = initialize_cameras(indices)
    if not caps:
        print("No cameras could be initialized properly.")
        return False
    
    # Create separate windows for each camera
    for i in range(len(caps)):
        window_name = f"Camera {i+1} - Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    global gamma, brightness, contrast
    
    # Show calibration prompt AFTER cameras are initialized
    print("\n" + "="*60)
    print("The two cameras must be connected on separate ports due to bandwidth limitations.")
    print("Line the needle up on both feeds to the red corner.")
    print("Press Y to accept calibration.")
    print("="*60)
    
    frame_count = 0
    try:
        while True:
            # Display each camera in separate windows
            display_separate_cameras(caps)
            frame_count += 1
            
            # Handle keyboard input (works with any active window)
            key = cv2.waitKey(30) & 0xFF  # Increased wait time to reduce CPU usage
            
            if key == ord('y') or key == ord('Y'):
                print("\nCalibration ACCEPTED")
                break
            elif key == ord('n') or key == ord('N'):
                print("\nCalibration REJECTED")
                return False
            elif key == ord('q') or key == 27:  # 'q' or ESC
                print("\nExiting without calibration")
                return False
            elif key == ord('+') or key == ord('='):
                brightness = min(brightness + 5, 100)
                # Removed print statement to keep terminal clean
            elif key == ord('-'):
                brightness = max(brightness - 5, -100)
                # Removed print statement to keep terminal clean
            elif key == ord('g'):
                gamma = min(gamma + 0.1, 3.0)
                # Removed print statement to keep terminal clean
            elif key == ord('G'):
                gamma = max(gamma - 0.1, 0.1)
                # Removed print statement to keep terminal clean
            elif key == ord('c'):
                contrast = min(contrast + 0.1, 3.0)
                # Removed print statement to keep terminal clean
            elif key == ord('C'):
                contrast = max(contrast - 0.1, 0.1)
                # Removed print statement to keep terminal clean
    
    finally:
        # Clean up
        print(f"Processed {frame_count} frames")
        cv2.destroyAllWindows()
        for cap in caps:
            if cap:
                cap.release()
    
    return True

def get_calibration_acceptance():
    """
    Simple terminal-based yes/no prompt for calibration acceptance.
    """
    while True:
        response = input("Accept this calibration? (y/n): ").strip().lower()
        if response in ['y', 'yes']:
            return True
        elif response in ['n', 'no']:
            return False
        else:
            print("Please enter 'y' for yes or 'n' for no.")

# Main execution function
def main():
    """
    Main function to run the camera calibration interface.
    """
    print("=== Camera Calibration Interface ===")
    accepted = start_camera_feed_with_calibration()
    
    if accepted:
        print("Calibration process completed successfully!")
        return True
    else:
        print("Calibration was not accepted.")
        return False

if __name__ == "__main__":
    main()