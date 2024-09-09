''' lab1_drive_test.py

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



 #------------------------------------#

#Main loop
try:
    #Set starttime
    t0 = time.time()
    while time.time() - t0  < runTime:
        t = time.time()

        #----- 2. READ & OUTPUT SENSOR DATA -----#
        





        #----------------------------------------#
        #- 3. WRITE DRIVE COMMANDS TO THE MOTORS -#








        #----------------------------------------#
        #--------- 4. TURN ON THE LIGHTS ---------#






        #----------------------------------------#
        #-- 3b, 4b, ADD QCAR WRITE COMMAND HERE --#



        #----------------------------------------#




except KeyboardInterrupt:
    print("User interrupted!")

finally:
    myCar.terminate()