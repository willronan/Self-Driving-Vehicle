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

    # Front Wall
    hBasicShape.spawn_id_box_walls_from_end_points(
        actorNumber=1,
        startLocation=[2.508, -4.327, 0],
        endLocation=[-2.508, -4.327, 0],
        height=2,
        thickness=1,
        color=[(154/255),(101/255),(14/255)],
        waitForConfirmation=False
    )

    # Rear Wall
    hBasicShape.spawn_id_box_walls_from_end_points(
        actorNumber=2,
        startLocation=[2.508, 4.327, 0],
        endLocation=[-2.508, 4.327, 0],
        height=2,
        thickness=1,
        color=[(154/255),(101/255),(14/255)],
        waitForConfirmation=False
    )


    # Start spawn model
    QLabsRealTime().start_real_time_model(rtModel)


    return hqcar


if __name__ == '__main__':
    # XXX Add processing of command line arguments
    setup()