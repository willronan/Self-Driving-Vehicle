''' Lab0_drive_test.py

This walkthrough is designed to give you a basic understanding of
how to write commands to the QCar's lights, steering servo, and
motor. Please take the time read the code's imports, sampling
general setup, etc. to gain an understanding of coding for QCar in
python

'''
import numpy as np
import time
from pal.products.qcar import QCar, IS_PHYSICAL_QCAR
import os

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()


#Sampling setup
sampleRate = 200 # Hz
runTime = 6.0 # seconds

#OtherVariables
MAX_THROTTLE = 0.1
MAX_STEERING = 0.5


 #------- 1. BUILD QCAR OBJECT -------#
myCar = QCar(readMode=1, frequency=sampleRate)

#Main loop
try:
    #Set starttime
    t0 = time.time()
    while time.time() - t0  < runTime:
        t = time.time()

        # 2. READ & OUTPUT SENSOR DATA
        
        myCar.read()

        os.system('clear')
        print(
            f'time: {(t-t0):.2f}'
            + f', Battery Voltage: {myCar.batteryVoltage:.2f}'
            + f', Motor Current: {myCar.motorCurrent:.2f}'
            + f', Motor Encoder: {myCar.motorEncoder}'
            + f', Motor Tach: {myCar.motorTach:.2f}'
            + f', Accelerometer: {myCar.accelerometer}'
            + f', Gyroscope: {myCar.gyroscope}'
        )

        # 3. WRITE DRIVE COMMANDS TO THE MOTORS
        #throttle = MAX_THROTTLE
        throttle = MAX_THROTTLE * np.sin(t*2*np.pi/5)
        
        #steering = 0 
        #steering = MAX_STEERING
        steering = MAX_STEERING * np.sin(t*2*np.pi/2.5)
        

        # 4. TURN ON THE LIGHTS
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        if steering > 0.15:
            LEDs[0] = 1
            LEDs[2] = 1
        elif steering < -0.15:
            LEDs[1] = 1
            LEDs[3] = 1
        if throttle < 0:
            LEDs[5] = 1

        myCar.write(throttle, steering, LEDs)

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    myCar.terminate()