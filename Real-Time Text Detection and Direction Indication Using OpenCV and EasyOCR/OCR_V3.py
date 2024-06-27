import time
import threading
import cv2
import pyrealsense2 as rs
import numpy as np
import serial
import keyboard
from adafruit_servokit import ServoKit
import subprocess
import easyocr

# Create a lock for thread synchronization
lock = threading.Lock()

# Instantiate the text detector
reader = easyocr.Reader(['en'], gpu=False)

# Function to move the camera to a specific pan-tilt position
kit = ServoKit(channels=16)

# Define the serial port and baud rate
serial_port = '/dev/ttyUSB0'  # Update with your actual serial port
baud_rate = 115200

# Create a serial object
ser = serial.Serial(serial_port, baud_rate, timeout=1)

POSITION_DURATION = 1  # seconds taken for each position

# Function to run the nvgstcapture-1.0 command and automatically press keys
def run_ls_command():
    command = ['nvgstcapture-1.0']

    try:
        output = subprocess.check_output(command, universal_newlines=True)

        # Wait for 5 seconds
        time.sleep(1)
        keyboard.press_and_release('j')
        keyboard.press_and_release('enter')
        keyboard.press_and_release('q')
        keyboard.press_and_release('enter')

    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

# Rest of your code (including pan-tilt automation and obstacle detection) goes here

def move_camera(pan_angle, tilt_angle):
    with lock:
        kit.servo[0].angle = pan_angle
        kit.servo[1].angle = tilt_angle
        time.sleep(0.5)

# Thread function for capturing images at different positions
def capture_thread(pan_angle, tilt_angle):
    for _ in range(POSITION_DURATION):
        move_camera(pan_angle, tilt_angle)
        run_ls_command()  # Capture an image
        time.sleep(1)  # Simulating image capture time

# Define the pan and tilt positions to capture images
positions = [
    (15, 45),
    (30, 45),
    (60, 45),
    (75, 45),
    (90, 45)
]

# Function to start the pan-tilt automation
def pan_tilt_image():
    while True:
        for pan_angle, tilt_angle in positions:
            capture_thread(pan_angle, tilt_angle)
            time.sleep(1)  # Wait for 1 seconds before moving to the next position

# Start the pan-tilt automation in a separate thread
pan_tilt_thread = threading.Thread(target=pan_tilt_image)
pan_tilt_thread.start()

# This is the code for obstacle detection

depth_thresh = 1000.0000
depth_thresh_left_right = 1000.0000

# Create a pipeline for RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming from the camera
pipeline.start(config)

# Define function to send commands over serial communication
def send_command(command):
    ser.write(command.encode())

# Define function to set motor speeds for rover movement
def set_motor_speeds(left_speed, right_speed):
    command = 'L{} R{}\n'.format(left_speed, right_speed)
    send_command(command)


    # Thread function for obstacle detection
def obstacle_det():
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            rows, cols = depth_image.shape
            slice_size = cols // 3

            left_part = depth_image[:, :slice_size]
            middle_part = depth_image[:, slice_size:2 * slice_size]
            right_part = depth_image[:, 2 * slice_size:]

            middle_distance = np.mean(middle_part)
            left_distance = np.mean(left_part)
            right_distance = np.mean(right_part)

            # Initialize a variable to store detected text
            detected_text = ""

            # Perform text detection on the color_image
            text_ = reader.readtext(color_image)

            for t_, t in enumerate(text_):
                bbox, text, score = t

                if score > 0.1:
                    bbox = [(int(coord[0]), int(coord[1])) for coord in bbox]

                    cv2.rectangle(color_image, bbox[0], bbox[2], (0, 255, 0), 5)
                    cv2.putText(color_image, text, bbox[0], cv2.FONT_HERSHEY_COMPLEX, 0.65, (255, 0, 0), 2)

                    print(f"Detected Text: {text}, Score: {score}")

                    # Concatenate detected text to the existing text
                    detected_text += text

            # Now, you can use the 'detected_text' variable for further processing
            print(f"Combined Detected Text: {detected_text}")

            # Continue with your obstacle detection logic based on 'detected_text'

            if middle_distance < depth_thresh_left_right:
                set_motor_speeds(0, 0)
                print("STOP THE ROVER --> THERE IS AN OBJECT")
            elif 'R' in detected_text:
                set_motor_speeds(-2, 255)
                print("MOVE LEFT --> BECAUSE THE RIGHT SIDE HAS AN OBJECT")
                time.sleep(1)
            elif 'L' in detected_text:
                set_motor_speeds(2, -255)
                print("MOVE RIGHT --> BECAUSE THE LEFT SIDE HAS AN OBJECT")
                time.sleep(1)
            elif keyboard.is_pressed('s'):
                set_motor_speeds(-1, -255)
                print("Moving backward")
            else:
                set_motor_speeds(1, 250)
                print("MOVE FORWARD --> THE PATH IS CLEAR")


        pipeline.stop()


'''
        cv2.imshow("Color Frame", color_image)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow("Depth Frame", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    '''





# Start the text detection in a separate thread
# Start the obstacle detection in a separate thread
obstacle_thread = threading.Thread(target=obstacle_det)
obstacle_thread.start()

# Wait for the threads to complete
pan_tilt_thread.join()
obstacle_thread.join()
