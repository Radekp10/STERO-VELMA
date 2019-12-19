#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import math
import PyKDL
from threading import Thread

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import MarkerPublisher, exitError

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm


if __name__ == "__main__":

    # pozycja poczatkowa robota
    q_map_zero = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25,
        'right_arm_3_joint':0.85,
        'right_arm_4_joint':0,
        'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.8,
        'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85,
        'left_arm_4_joint':0,
        'left_arm_5_joint':0.5,
        'left_arm_6_joint':0
        }

    
    """ --------------INICJALIZACJA SYSTEMU-------------- """
    rospy.init_node('test_jimp_planning_attached', anonymous=False)

    rospy.sleep(0.5)

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    print "Waiting for Planner initialization..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit(timeout_s=10.0):
        print "Could not initialize Planner"
        exitError(2)
    print "Planner initialization ok!"

    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(3)

    print "Moving to the current position..."
    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 1.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)



    """ --------------FUNKCJE PROGRAMU-------------- """
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.07, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)


    def resetPoseTol():
        print "Resetowanie pozycji koncowki..."
        tolerance = makeTwist(1,1,1,1,1,1)    
        dest_reset = velma.getTf("B", "Wr")
        velma.moveCartImpRight([dest_reset], [5.0], None ,None , [makeWrench(30,30,1000,100,100,100)], [1], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol = tolerance)
        if velma.waitForEffectorRight() != 0:      
            exitError(17)
        rospy.sleep(0.5)


    def makeWrench(lx,ly,lz,rx,ry,rz):
        return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))


    def makeTwist(lx,ly,lz,rx,ry,rz):  
        return PyKDL.Twist( PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))  
 

    def openFingers():
 	print "Palce..."
        dest_q=[0,0,0,0]
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5) 

    
    def makeGrip():
        print "Ukladanie dloni do chwytu klamki..."
        dest_q=[0.6*math.pi,0.6*math.pi,0.6*math.pi,math.pi]
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)
       

    def setImpedance():
        print "Set impedance to (10,10,1000,150,150,150) in tool frame."
        imp_list = [makeWrench(1000,1000,1000,150,150,150),
                    makeWrench(500,500,1000,150,150,150),
                    makeWrench(100,100,1000,150,150,150),
                    makeWrench(10,10,1000,150,150,150)]
        if not velma.moveCartImpRight(None, None, None, None, imp_list, [0.5,1.0,1.5,2.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(16)
        if velma.waitForEffectorRight() != 0:
            exitError(17)


    def moveToDoor():
	print "Ruch do dzwiczek..."
        DoorR = velma.getTf("B", "right_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorR_x, doorR_y, doorR_z = DoorR.p

        DoorL = velma.getTf("B", "left_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorL_x, doorL_y, doorL_z = DoorL.p

        position = velma.getLastJointState()

        Goal = PyKDL.Vector(1/3*doorL_x+2/3*doorR_x+0.5*math.cos(alfa),1/3*doorL_y+2/3*doorR_y+0.5*math.sin(alfa),doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: Dotykam drzwi :p"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)


    def moveToHandle():
        print "Ruch do klamki..."
        DoorR = velma.getTf("B", "right_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorR_x, doorR_y, doorR_z = DoorR.p

        DoorL = velma.getTf("B", "left_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorL_x, doorL_y, doorL_z = DoorL.p

        position = velma.getLastJointState()

        Goal = PyKDL.Vector(doorL_x,doorL_y,doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: Dotykam klamki :p"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)


    def turnTorsoToCabinet():
        print "Obrot tulowia..."
        Cabinet = velma.getTf("B", "cabinet")
        cabX, cabY, cabZ = Cabinet.p
        position = velma.getLastJointState()
        kat_szafki=math.atan2(cabY,cabX)

        print "Kat przed korekta: ", kat_szafki

        if kat_szafki>1.5:
            kat_szafki=1.5

        if kat_szafki<-1.5:
            kat_szafki=-1.5

        if kat_szafki<0:
	    znak=-1
        else:
	    znak=1

        print "Kat po korekcie: ", kat_szafki

        q_pom = {'torso_0_joint':kat_szafki,        
	'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':1.25,
        'right_arm_3_joint':0.85,
        'right_arm_4_joint':0,
        'right_arm_5_joint':-0.5,
        'right_arm_6_joint':0,
        'left_arm_0_joint':0.3,
        'left_arm_1_joint':1.8,
        'left_arm_2_joint':-1.25,
        'left_arm_3_joint':-0.85,
        'left_arm_4_joint':0,
        'left_arm_5_joint':0.5,
        'left_arm_6_joint':0
        }

      
        print "Switch to jnt_imp mode (no trajectory)..."
        velma.moveJointImpToCurrentPos(start_time=0.2)
        error = velma.waitForJoint()
        if error != 0:
            print "The action should have ended without error, but the error code is", error
            exitError(7)
        planAndExecute(q_pom)

        return kat_szafki


    def openDoor():

        DoorR = velma.getTf("B", "right_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorR_x, doorR_y, doorR_z = DoorR.p
  
        DoorL = velma.getTf("B", "left_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorL_x, doorL_y, doorL_z = DoorL.p

        Cabinet = velma.getTf("B", "cabinet")
        cabX, cabY, cabZ = Cabinet.p
        position = velma.getLastJointState()
        kat_szafki=math.atan2(cabY,cabX)


	print "Otwieranie drzwi: ruch (1)..."

        Goal = PyKDL.Vector(cabX-math.cos(alfa),cabY-math.sin(alfa),doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi*0.4+alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: otwieram drzwi"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)

	print "Otwieranie drzwi: ruch (2)..."

        Goal = PyKDL.Vector(cabX-2.0*math.cos(alfa)+1.5*math.sin(alfa),cabY-2.0*math.sin(alfa)-1.5*math.cos(alfa),doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi*0.8+alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: otwieram drzwi"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)
	
	print "Otwieranie drzwi: ruch (3)..."

        Goal = PyKDL.Vector(cabX-1.5*math.cos(alfa)+3.0*math.sin(alfa),cabY-1.5*math.sin(alfa)-3.0*math.cos(alfa),doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi+alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: otwieram drzwi"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)

	print "Otwieranie drzwi: ruch (4)..."

        Goal = PyKDL.Vector(cabX-1.0*math.cos(alfa)+2.0*math.sin(alfa),cabY-1.0*math.sin(alfa)-2.0*math.cos(alfa),doorR_z+0.1)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(0.6*math.pi+alfa), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
	    print "Velma: otwieram drzwi"
        rospy.sleep(0.5)
        resetPoseTol()
        rospy.sleep(0.5)


    def leaveHandle():
	print "Wyjazd z klamki..."

        DoorR = velma.getTf("B", "right_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
    	doorR_x, doorR_y, doorR_z = DoorR.p

    	DoorL = velma.getTf("B", "left_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
    	doorL_x, doorL_y, doorL_z = DoorL.p

    	position = velma.getLastJointState()

    	Goal = PyKDL.Vector(doorR_x+0.1*math.cos(alfa)+0.25*math.sin(alfa),doorR_y+0.1*math.sin(alfa)-0.25*math.cos(alfa),doorR_z+0.1)
    	T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(0.25*math.pi+alfa), Goal)
    
    	if not velma.moveCartImpRight([T_B_Door], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(0.2,0.2,1),PyKDL.Vector(1,1,1))):
            exitError(8)
    	if velma.waitForEffectorRight() != 0:
	    print "Velma: :p"
    	rospy.sleep(0.5)
    	resetPoseTol()
    	rospy.sleep(0.5)


    def leaveCabinet():
        print "Oddalenie od szafki..."

        DoorR = velma.getTf("B", "right_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorR_x, doorR_y, doorR_z = DoorR.p

   	DoorL = velma.getTf("B", "left_door")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
        doorL_x, doorL_y, doorL_z = DoorL.p
    
        Goal = PyKDL.Vector(1/3*doorL_x+2/3*doorR_x-0.4,1/3*doorL_y+2/3*doorR_y-0.2,doorR_z+0.1-0.25)
        T_B_Door = PyKDL.Frame(PyKDL.Rotation.RotZ(0), Goal)
    
        if not velma.moveCartImpRight([T_B_Door], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(100,100,0.7), PyKDL.Vector(0.7,0.7,0.7)), start_time=0.5, path_tol=PyKDL.Twist(PyKDL.Vector(1,1,1),PyKDL.Vector(1,1,1))):
            exitError(8)
        if velma.waitForEffectorRight() != 0:
            exitError(9)
        rospy.sleep(0.5)



    """ --------------START PROGRAMU-------------- """

    # Pobranie pozycji szafki i drzwi

    Cabinet = velma.getTf("B", "cabinet")
    cabX, cabY, cabZ = Cabinet.p
    position = velma.getLastJointState()

    alfa=turnTorsoToCabinet()
    makeGrip()

    print "Ruch do pozycji poczatkowej..."
    # define some configurations
    q_pocz = {'torso_0_joint':alfa,
        'right_arm_0_joint':-0.1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0.25,      'left_arm_4_joint':0,
        'right_arm_5_joint':-1.2,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':-0.3,      'left_arm_6_joint':0 }
    planAndExecute(q_pocz)
    rospy.sleep(0.5)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)
 
    rospy.sleep(0.5)
 
    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(12)

    setImpedance()
 
    rospy.sleep(1.0)
   
    moveToDoor()
    
    moveToHandle()

    openDoor()

    leaveHandle()

    leaveCabinet()
    
    rospy.sleep(2)
    print "Pozycja zero..."
    planAndExecute(q_map_zero)
    rospy.sleep(0.5)
   
    openFingers()
