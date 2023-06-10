from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import pyvesc
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition, SetPosition, SetServoPosition
import serial

# Set your serial port here (either /dev/ttyX or COMX)
serialport = '/dev/tty.usbmodem3041'


if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    rf = RoboflowOak(model="trash-detection-1fjjc", confidence=0.3, overlap=0.5, version="1", api_key="3OhAiptoY0ftMlIYK0ZJ", rgb=True, depth=True, device=None, device_name="roboflowak", blocking=True)
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        while True:
            t0 = time.time()
            result, frame, raw_frame, depth = rf.detect()
            predictions = result["predictions"]
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
            max_depth = np.amax(depth)
            cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
            cv2.imshow("frame", frame)

            
        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
            if cv2.waitKey(1) == ord('q'):
                ser.write(pyvesc.encode(SetCurrent(0)))
                break