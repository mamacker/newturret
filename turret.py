import os
import numpy as np
from pose_engine import PoseEngine
import cv2, os
import threading
import time
from PIL import Image
import subprocess
import pixelmapping.convert as convert
import servo
import random
import yaml
import requests
import pigpio
pi = pigpio.pi()

curframe = None;
mutex = threading.Lock()
stop = True
nose_pose = (0,0);

def ticcmd(*args):
    print("Setting slide height.");
    return subprocess.check_output(['ticcmd'] + list(args))

status = yaml.load(ticcmd('-s', '--full'), Loader=yaml.FullLoader)
position = status['Current position']

pan = servo.Servo(12)
tilt = servo.Servo(13)

pan.stop()
tilt.stop()
    
quad_coords = {
    "lonlat": np.array([
        [.9, 0], # Upper Left
        [-.7, 0], #  Upper Right
        [.9, -1], # Bottom Left
        [-.7, -1] # Bottom Right
    ]),
    "pixel": np.array([
        [600, 0], # Upper Left
        [0, 0], # Upper Right
        [600, 450], # Bottom Left
        [0, 450] # Bottom Right
    ])
}

def position_on_nose(nose):
    global quad_coords
    global pan
    global tilt
    global pan_history
    global tilt_history
    pm = convert.PixelMapper(quad_coords["pixel"], quad_coords["lonlat"])

    lonlat = pm.pixel_to_lonlat(nose)
    pan.value(lonlat[0][0]);
    tilt.value(lonlat[0][1]);
    pi.write(23,1)

def milli_time():
    return round(time.time() * 1000)

def move_to_nose():
    global keep_running
    global nose_pose
    global stop

    pan_direction = -1
    tilt_direction = -1
    last_nose_pose = None
    last_nose_time = milli_time()
    while not stop:
        if (last_nose_pose != nose_pose):
            position_on_nose(nose_pose)
            last_nose_time = milli_time()
            last_nose_pose = nose_pose
        elif(milli_time() - last_nose_time > 3000):
            pi.write(23,1)
            cur_pan = pan.value();
            cur_tilt = tilt.value();

            if (cur_pan <= -.8):
                pan_direction = 1
                cur_pan = -.8
            elif (cur_pan >= .9):
                pan_direction = -1
                cur_pan = .9 

            if (cur_tilt <= -1):
                tilt_direction = 1
                cur_tilt = -1
            elif (cur_tilt >= .6):
                tilt_direction = -1
                cur_tilt =.6 
                
            pan.value(cur_pan + (pan_direction * .01 * random.randint(1,5))) 
            tilt.value(cur_tilt + (tilt_direction * .01 * random.randint(1,5))) 
        time.sleep(.1);

def runCap():
    global stop;
    global curframe;
    
    vcap = None
    try:
        vcap = cv2.VideoCapture(0)
    except:
        print("Trying again...")
        time.sleep(2)
        vcap = cv2.VideoCapture(0)
    print("... opened.");

    vcap.set(3, 640)
    vcap.set(4, 480)

    while not stop:
        ret, img = vcap.read()
        if (img is None):
            if (count % 100 == 0):
                print("frame is None")
            count += 1
            if count > 10:
                break
            continue
        else:
            img.flags.writeable = False
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img)
            curframe = pil_image;
        count = 0;

    vcap.release()

def runPose():
    global curframe
    global stop
    global nose_pose

    engine = PoseEngine(
    'models/mobilenet/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite')
    while not stop:
        if curframe is None:
            print(".", end='')
            time.sleep(.1)
            continue;


        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image = curframe
        curframe = None

        poses, inference_time = engine.DetectPosesInImage(image)

        for pose in poses:
            if pose.score < 0.4: continue
            print('\nPose Score: ', pose.score)
            for label, keypoint in pose.keypoints.items():
                print('  %-20s x=%-4d y=%-4d score=%.1f' %
                        (label.name, keypoint.point[0], keypoint.point[1], keypoint.score))

                if label.name == "NOSE":
                    nose_pose = (keypoint.point[0], keypoint.point[1]);
                    break;

def start_sentry():
    print("Starting sentry laser.")
    ticcmd('--exit-safe-start', '--position', str(6392))
    threading.Thread(target=runCap, args=[]).start()
    threading.Thread(target=runPose, args=[]).start()
    threading.Thread(target=move_to_nose).start()

def stop_sentry():
    ticcmd('--exit-safe-start', '--position', str(0))
    print("Stoping sentry laser.")

def check_status():
    global stop
    url = "http://nodecore3.local/sentry"
    while True:
        r = requests.get(url)
        if r.status_code == requests.codes.ok:
            print("Sentry status is: ", r.text)
            isOn = r.text == "on"
            if isOn and stop is True:
                stop = False
                start_sentry()
            elif not isOn and stop is False:
                stop = True
                stop_sentry()
        else:
            print("Device server on nodecore3 failing.")

        time.sleep(1)

check_status()