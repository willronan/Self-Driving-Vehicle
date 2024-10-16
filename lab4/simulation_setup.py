# region: package imports
import os
import numpy as np


from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar import QLabsQCar
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime

# environment objects
from qvl.crosswalk import QLabsCrosswalk
from qvl.roundabout_sign import QLabsRoundaboutSign
from qvl.yield_sign import QLabsYieldSign
from qvl.traffic_light import QLabsTrafficLight
from qvl.basic_shape import QLabsBasicShape
from qvl.stop_sign import QLabsStopSign
import pal.resources.rtmodels as rtmodels

#endregion

# Change to move stop sign
SIGN_DISTANCE = 4.5

def setup(
        initialPosition=[-0.031, 1.311, 0.000],
        initialOrientation=[0, 0, -np.pi/2],
        rtModel=rtmodels.QCAR
    ):

    # Try to connect to Qlabs
    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Delete any previous QCar instances and stop any running spawn models
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    # region: QCar spawn description


    # Spawn a QCar at the given initial pose
    hqcar = QLabsQCar(qlabs)
    hqcar.spawn_id(
        actorNumber=0,
        location=[x for x in initialPosition],
        rotation=initialOrientation,
        waitForConfirmation=True
    )

    # Create a new camera view and attach it to the QCar
    hcamera = QLabsFreeCamera(qlabs)
    hcamera.spawn([8.484, 1.973, 12.209], [-0, 0.748, 0.792])
    hqcar.possess()
    # endregion


    # region: Spawning Basic Shapes
    hBasicShape = QLabsBasicShape(qlabs)
    xOff = 0 #0.586

    # region: Spawn stopsign
    stop = QLabsStopSign(qlabs)
    stop.spawn(location=[0, -SIGN_DISTANCE, 0.2], rotation=[0,0, np.pi/2],
            scale=[1,1,1], configuration=0, waitForConfirmation=True)
    # endregion

    # Start spawn model
    QLabsRealTime().start_real_time_model(rtModel)


    return hqcar


if __name__ == '__main__':
    # XXX Add processing of command line arguments
    setup()