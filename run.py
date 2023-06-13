from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import pyvesc
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition, SetPosition, SetServoPosition
import serial
#import Jetson.GPIO as GPIO

# Set your serial port here (either /dev/ttyX or COMX)
serialport = '/dev/tty.usbmodem3041'
RELAY_PIN = 8


def turn_on_fan():
    GPIO.output(RELAY_PIN, GPIO.LOW)
    print("Fan turned on")

def turn_off_fan():
    GPIO.output(RELAY_PIN, GPIO.HIGH)
    print("Fan turned off")

# Initialize the GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.HIGH)


lower_color = np.array([0, 0, 0])  # Adjust these values
upper_color = np.array([100, 100, 100])

# Define the initial minimum area threshold
min_area_threshold = 500  # Adjust this value

# Open the camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, or specify the camera index

# Scale the OpenCV HSV values to standard HSV values (0-100)
def scale_hsv_opencv_to_std(hsv):
    hsv_scaled = np.copy(hsv)
    hsv_scaled[0] = int(hsv[0] * (100 / 179))
    hsv_scaled[1] = int(hsv[1] * (100 / 255))
    hsv_scaled[2] = int(hsv[2] * (100 / 255))
    return hsv_scaled

# Scale the standard HSV values (0-100) to OpenCV HSV values
def scale_hsv_std_to_opencv(hsv):
    hsv_scaled = np.copy(hsv)
    hsv_scaled[0] = int(hsv[0] * (179 / 100))
    hsv_scaled[1] = int(hsv[1] * (255 / 100))
    hsv_scaled[2] = int(hsv[2] * (255 / 100))
    return hsv_scaled

# Callback function for trackbar changes
def on_trackbar_change(value):
    global lower_color, upper_color, min_area_threshold

    # Get current trackbar values for lower color (in the standard HSV range)
    lower_h = cv2.getTrackbarPos('Lower H', 'Color Adjustment')
    lower_s = cv2.getTrackbarPos('Lower S', 'Color Adjustment')
    lower_v = cv2.getTrackbarPos('Lower V', 'Color Adjustment')

    # Get current trackbar values for upper color (in the standard HSV range)
    upper_h = cv2.getTrackbarPos('Upper H', 'Color Adjustment')
    upper_s = cv2.getTrackbarPos('Upper S', 'Color Adjustment')
    upper_v = cv2.getTrackbarPos('Upper V', 'Color Adjustment')

    # Get current trackbar value for minimum area threshold
    min_area_threshold = cv2.getTrackbarPos('Min Area Threshold', 'Color Adjustment')

    # Scale the standard HSV values to OpenCV HSV values
    lower_hsv_opencv = scale_hsv_std_to_opencv([lower_h, lower_s, lower_v])
    upper_hsv_opencv = scale_hsv_std_to_opencv([upper_h, upper_s, upper_v])

    # Update the lower and upper color boundaries
    lower_color = np.array(lower_hsv_opencv)
    upper_color = np.array(upper_hsv_opencv)

cv2.namedWindow('Color Adjustment')

# Resize the color adjustment window
cv2.resizeWindow('Color Adjustment', 400, 300)

# Create trackbars for lower color (in the standard HSV range)
cv2.createTrackbar('Lower H', 'Color Adjustment', lower_color[0], 100, on_trackbar_change)
cv2.createTrackbar('Lower S', 'Color Adjustment', lower_color[1], 100, on_trackbar_change)
cv2.createTrackbar('Lower V', 'Color Adjustment', lower_color[2], 100, on_trackbar_change)

# Create trackbars for upper color (in the standard HSV range)
cv2.createTrackbar('Upper H', 'Color Adjustment', upper_color[0], 100, on_trackbar_change)
cv2.createTrackbar('Upper S', 'Color Adjustment', upper_color[1], 100, on_trackbar_change)
cv2.createTrackbar('Upper V', 'Color Adjustment', upper_color[2], 100, on_trackbar_change)

# Create a trackbar for minimum area threshold
cv2.createTrackbar('Min Area Threshold', 'Color Adjustment', min_area_threshold, 1000, on_trackbar_change)

if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    rf = RoboflowOak(model="trash-detection-1fjjc", confidence=0.3, overlap=0.5, version="1", api_key="3OhAiptoY0ftMlIYK0ZJ", rgb=True, depth=False, device=None, device_name="roboflowak", blocking=True)
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        while True:
            t0 = time.time()
            result, oakd, raw_frame, depth = rf.detect()
            predictions = result["predictions"]
            ret, frame = cap.read()

            # Convert the frame to the HSV color space
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a binary mask based on the specified color range
            mask = cv2.inRange(hsv_frame, lower_color, upper_color)

            # Apply the mask to the original frame
            result = cv2.bitwise_and(frame, frame, mask=mask)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
            # Iterate through the contours and draw centroid on color regions with a significant area
            for contour in contours:
            # Calculate the contour area
                area = cv2.contourArea(contour)

                if area > min_area_threshold:
                    # Fan control: Turn on the fan by setting the GPIO pin to HIGH
                    turn_on_fan()
                    #print("fan on")

                    # Calculate the centroid of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Draw a circle at the centroid
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            # Turn off the fan if no contours meet the area threshold
            if len(contours) == 0 or all(cv2.contourArea(contour) <= min_area_threshold for contour in contours):
                turn_off_fan()
                #print("fan off")

            # Display the camera image with color adjustment
            hsv_lower_color_std = scale_hsv_opencv_to_std(lower_color)
            hsv_upper_color_std = scale_hsv_opencv_to_std(upper_color)
            cv2.putText(frame, 'Lower H: {} S: {} V: {}'.format(hsv_lower_color_std[0], hsv_lower_color_std[1], hsv_lower_color_std[2]),
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, 'Upper H: {} S: {} V: {}'.format(hsv_upper_color_std[0], hsv_upper_color_std[1], hsv_upper_color_std[2]),
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, 'Min Area Threshold: {}'.format(min_area_threshold),
                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('Camera Image', frame)

            # Display the color detection result
            #cv2.imshow('Color Detection', result)

            # Break the loop if the 'q' key is pressed
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break
        #{
        #    predictions:
        #    [ { 
        #        x: (middle),
        #        y:(middle),
        #        width: ,
        #        height: ,
        #        depth: ###->,
        #        confidence: ,
        #        class: ,
        #        mask: { }
        #       }
        #    ]
        #}
        # frame - frame after preprocs, with predictions
        # raw_frame - original frame from your OAK
        # depth - depth map for raw_frame, center-rectified to the center camera
        # To access specific values within "predictions" use:
        # p.json()[a] for p in predictions
        # set "a" to the index you are attempting to access
        # Example: accessing the "y"-value:
        # p.json()['y'] for p in predictions
        #this changes speed of car
            ser.write(pyvesc.encode(SetRPM(2000)))

            ser.write(pyvesc.encode_request(GetValues))
    
            t = time.time()-t0
            print("INFERENCE TIME IN MS ", 1/t)
            print("PREDICTIONS ", [p.json() for p in predictions])
            if(predictions):
                counter = 0
                confidence = 0
                item_tracking = 0
                for p in predictions:
                    first_prediction = predictions[counter].json()
                    first_item_key = list(first_prediction.keys())[5]
                    first_item_value = first_prediction[first_item_key]
                    if confidence < first_item_value:
                        confidence = first_item_value
                        item_tracking = counter
                    #print("First Item Key:", first_item_key)
                    #print("First Item Value:", first_item_value)
                    counter = counter + 1
                if(predictions):
                    first_prediction = predictions[item_tracking].json()
                    first_item_key = list(first_prediction.keys())[0]
                    first_item_value = first_prediction[first_item_key]
                    if first_item_value <= 700 and first_item_value >= 500:
                        ser.write(pyvesc.encode(SetServoPosition(0.8)))
                    if first_item_value <= 499 and first_item_value >= 400:
                        ser.write(pyvesc.encode(SetServoPosition(0.6)))
                    if first_item_value <= 399 and first_item_value >= 300:
                        ser.write(pyvesc.encode(SetServoPosition(0.5)))
                    if first_item_value <= 299 and first_item_value >= 200:
                        ser.write(pyvesc.encode(SetServoPosition(0.4)))
                    if first_item_value <= 199 and first_item_value >= 100:
                        ser.write(pyvesc.encode(SetServoPosition(0.3)))
                    if first_item_value < 100:
                        ser.write(pyvesc.encode(SetServoPosition(0.2)))         
                #first_prediction = predictions[0].json()
                #first_item_key = list(first_prediction.keys())[0]
                #first_item_value = first_prediction[first_item_key]
                #print("First Item Key:", first_item_key)
                #print("First Item Value:", first_item_value)
    
        # setting parameters for depth calculation
        # comment out the following 2 lines out if you're using an OAK without Depth
            #max_depth = np.amax(depth)
            #cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
            cv2.imshow("frame", oakd)

            
        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
            if cv2.waitKey(1) == ord('q'):
                ser.write(pyvesc.encode(SetCurrent(0)))
                break