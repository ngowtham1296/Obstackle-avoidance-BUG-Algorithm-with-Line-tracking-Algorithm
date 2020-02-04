import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import time
import RPi.GPIO as GPIO
from AlphaBot import AlphaBot
import datetime

# INITIALISING THE VALUS
CS = 5
Clock = 25
Address = 24
DataOut = 23
cntl = 8;
cntr = 7;
EncR = 0.0;
EncL = 0.0;
id_to_find  = 72
marker_size  = 2.5 #- [cm]
ids_global=0;
X=0;
Y=0;
Z=0;
TIME_IN_SEC = 4
SERVO_START = 2
incre = 0.500    
# FUNCTION DEFINITIONS

class TRSensor(object):
    def __init__(self,numSensors = 5):
        self.numSensors = numSensors
        self.calibratedMin = [0] * self.numSensors
        self.calibratedMax = [1023] * self.numSensors
        self.last_value = 0
  
    def AnalogRead(self):
        value = [0,0,0,0,0,0]
        #Read Channel0~channel4 AD value
        for j in range(0,6):
            GPIO.output(CS, GPIO.LOW)
            for i in range(0,4):
                #sent 4-bit Address
                if(((j) >> (3 - i)) & 0x01):
                    GPIO.output(Address,GPIO.HIGH)
                else:
                    GPIO.output(Address,GPIO.LOW)
                #read MSB 4-bit data
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
            for i in range(0,6):
                #read LSB 8-bit data
                value[j] <<= 1
                if(GPIO.input(DataOut)):
                    value[j] |= 0x01
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
            #no mean ,just delay
            for i in range(0,6):
                GPIO.output(Clock,GPIO.HIGH)
                GPIO.output(Clock,GPIO.LOW)
#           time.sleep(0.0001)
            GPIO.output(CS,GPIO.HIGH)
        return value[1:]
   
    def calibrate(self):
        max_sensor_values = [0]*self.numSensors
        min_sensor_values = [0]*self.numSensors
        for j in range(0,10):
        
            sensor_values = self.AnalogRead();
            
            for i in range(0,self.numSensors):
            
                # set the max we found THIS time
                if((j == 0) or max_sensor_values[i] < sensor_values[i]):
                    max_sensor_values[i] = sensor_values[i]

                # set the min we found THIS time
                if((j == 0) or min_sensor_values[i] > sensor_values[i]):
                    min_sensor_values[i] = sensor_values[i]

        # record the min and max calibration values
        for i in range(0,self.numSensors):
            if(min_sensor_values[i] > self.calibratedMin[i]):
                self.calibratedMin[i] = min_sensor_values[i]
            if(max_sensor_values[i] < self.calibratedMax[i]):
                self.calibratedMax[i] = max_sensor_values[i]

   
    def readCalibrated(self):
        value = 0
        #read the needed values
        sensor_values = self.AnalogRead();

        for i in range (0,self.numSensors):

            denominator = self.calibratedMax[i] - self.calibratedMin[i]

            if(denominator != 0):
                value = (sensor_values[i] - self.calibratedMin[i])* 1000 / denominator
                
            if(value < 0):
                value = 0
            elif(value > 1000):
                value = 1000
                
            sensor_values[i] = value
        
        print("readCalibrated",sensor_values)
        return sensor_values
            
    
    def readLine(self, white_line = 0):

        sensor_values = self.readCalibrated()
        avg = 0
        sum = 0
        on_line = 0
        for i in range(0,self.numSensors):
            value = sensor_values[i]
            if(white_line):
                value = 1000-value
            # keep track of whether we see the line at all
            if(value > 200):
                on_line = 1
                
            # only average in values that are above a noise threshold
            if(value > 50):
                avg += value * (i * 1000);  # this is for the weighted total,
                sum += value;                  #this is for the denominator 

        if(on_line != 1):
            # If it last read to the left of center, return 0.
            if(self.last_value < (self.numSensors - 1)*1000/2):
                #print("left")
                return 0;
    
            # If it last read to the right of center, return the max.
            else:
                #print("right")
                return (self.numSensors - 1)*1000

        self.last_value = avg/sum
        
        return self.last_value

def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print 'valEncL = %d' %EncL

    
def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print 'valEncR = %d' %EncR

#ARUCO DEFINITION
    

def Camera():
    ids_global=0;
    X=0;
    Y=0;
    Z=0;
    startTime=datetime.datetime.now()
    incre = 0.500
    
        
    print ( " starting at " + str(startTime.second))

    def isRotationMatrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6



    def rotationMatrixToEulerAngles(R):
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def servo():
        
        while True:
            i=2
            while i < 12.5:
                i=i+0.15;
                #print (i)
                p.ChangeDutyCycle(i)  
                time.sleep(0.12) 
                
            while i > 2.5:
                i=i-0.15;
                #print (i)
                p.ChangeDutyCycle(i)  
                time.sleep(0.12)
        
    def servomotor():
        p.start(7.5)
        if i > 12.5:
            incre = -0.1
        if i < 2:
            incre = 0.1
        i=i+incre;
                #print (i)
        p.ChangeDutyCycle(i)  
        time.sleep(0.25) 
                
        
        print( ids_global)
        
        if ids_global == id_to_find:
           # servo()
            p.stop()
    #--- Get the camera calibration path
    calib_path  = ""
    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

    #--- 180 deg rotation matrix around the x axis
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters_create()


    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv2.VideoCapture(0)
    #-- Set the camera size as the one it was calibrated with
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    timer = 0
    servoValue = 2
    while timer<99:
        timer = timer + 1

        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                  cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        
        ids_global=ids;
        
        print (ids, "ids")     
        if ids is not None and ids[0] == id_to_find:
            
            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            


            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print("MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1],tvec[2]))
            Z=tvec[2];
            X=tvec[0];
            Y=tvec[1];

            #-- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            

        #--- Display the frame
       
        cv2.imshow('frame', frame)
        key = cv2.waitKey(1) & 0xFF
        
        
        
        
        print ("X=%4.0f Y=%4.0f Z=%4.0f"%(X,Y,Z))
        if servoValue > 12.5:
            incre = -0.500
        if servoValue < 2:
            incre = 0.500
        servoValue=servoValue+incre
        SERVO_START = servoValue
                #print (i)
        p.ChangeDutyCycle(servoValue)  
        time.sleep(0.20) 
                
        endTime = datetime.datetime.now()
        if endTime.second - startTime.second > TIME_IN_SEC:
            print ("Camera stops after 10 sec")
            break;

        #print( ids_global)
        
        
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)
#GPIO.setmode(GPIO.BOARD)
GPIO.setup(27, GPIO.OUT)
GPIO.setwarnings(False) 
p = GPIO.PWM(27, 50)
p.start(7.5)

# MAIN FUNCTION





if __name__ == '__main__':

    from AlphaBot import AlphaBot
    
    maximum = 27;
    integral = 0;
    last_proportional = 0
    
    TR = TRSensor()
    Ab = AlphaBot()
    Ab.stop()
    print("Line follow Example")
    time.sleep(3)
    for i in range(0,400):
        TR.calibrate()
        print (i)
    print(TR.calibratedMin)
    print(TR.calibratedMax)
    time.sleep(0.5) 
    Ab.backward()
    SERVO_START = 2
    motorStart=datetime.datetime.now()


    
    
    while True:
        
        motorEnd=datetime.datetime.now()
        if motorEnd.second - motorStart.second > 3:
            print (str(motorEnd.second) + "stop now")
            Ab.stop()
            
            #time.sleep(TIME_IN_SEC)
            Camera()
            #Function Call
            #code here.....
            
            Ab.backward()
            print("wheel  starts now. ")
            motorStart=datetime.datetime.now()

            
        position = TR.readLine()
        #print(position)
        
        # The "proportional" term should be 0 when we are on the line.
        proportional = position - 2000
        
        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional
        integral += proportional
        
        # Remember the last position.
        last_proportional = proportional
  
        power_difference = proportional/25 + derivative/100 #+ integral/1000;  

        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        #print(position,power_difference)
        if (power_difference < 0):
            Ab.setPWMB(maximum + power_difference)
            Ab.setPWMA(maximum);
        else:
            Ab.setPWMB(maximum);
            Ab.setPWMA(maximum - power_difference)
        print ('EncR=%d' %EncR)

    

