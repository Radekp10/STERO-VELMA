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

class MarkerPublisherThread:
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default', frame_id=obj.link_name, m_type=Marker.CYLINDER, scale=Vector3(0.02, 0.02, 1.0), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break

        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass

    def __init__(self, obj):
        self.thread = Thread(target = self.threaded_function, args = (obj, ))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()

if __name__ == "__main__":

    # define some configurations
    q_pocz = {'torso_0_joint':0,
        'right_arm_0_joint':-0.1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0.25,      'left_arm_4_joint':0,
        'right_arm_5_joint':-1.2,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_pom1 = {'torso_0_joint':0.8,
        'right_arm_0_joint':-0.1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0.25,      'left_arm_4_joint':0,
        'right_arm_5_joint':-1.2,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_pom2 = {'torso_0_joint':1.4,
        'right_arm_0_joint':1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0.25,      'left_arm_4_joint':0,
        'right_arm_5_joint':-1.2,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_zero = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,
        'right_arm_1_joint':-1.8,
        'right_arm_2_joint':-1.25,
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



    #zaladowanie mapy do planera
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)



    # define a function for frequently used routine in this test
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
        '''
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)
        '''

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
    

    print "Ruch do pozycji poczatkowej..."
    planAndExecute(q_pocz)

    '''
    print "Zamkniecie palcow lewej reki..."
 
 
    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(2)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(3)
    velma.moveHandLeft([2.5,2.5,2.5,0], [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandLeft() != 0:
        exitError(6)
    rospy.sleep(0.5)   
    '''


    #uzyskanie pozycji puszki
    print "Ruch do puszki..."
    Jar = velma.getTf("B", "jar")	#getTf(frame_from, frame_to); "B"-robot base frame (torso_base); returns: PyKDL.Frame transformation from base frame to target frame
    jarX, jarY, jarZ = Jar.p
    position = velma.getLastJointState()

    Goal = PyKDL.Vector(jarX-0.25,jarY,jarZ+0.1)
    T_B_Jar = PyKDL.Frame(PyKDL.Rotation.RotZ(0), Goal)
    
    if not velma.moveCartImpRight([T_B_Jar], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
    

    print "Chwytanie puszki..."
    dest_q=[0.5*math.pi,0.5*math.pi,0.5*math.pi,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)

    print "Podnoszenie puszki..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.5 , -0.3 , 1.5 ))
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)


    Bookshelf = velma.getTf("B", "bookshelf")
    bookX, bookY, bookZ = Bookshelf.p
    position = velma.getLastJointState()
    kat_szafki=math.atan2(bookY,bookX)

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
        'right_arm_0_joint':1,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':2,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0.25,      'left_arm_4_joint':0,
        'right_arm_5_joint':-1.2,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    
    print "Ruch do kolejnej pozycji..."
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)
    planAndExecute(q_pom)

    print "Ruch do szafki..."

    Goal = PyKDL.Vector(bookX,bookY-0.5*znak,bookZ+1.4)
    T_B_Book = PyKDL.Frame(PyKDL.Rotation.RotZ(znak*0.5*math.pi), Goal)
    
    if not velma.moveCartImpRight([T_B_Book], [6.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Puszczanie puszki..."
    dest_q=[0,0,0,0]
    velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(8)
    rospy.sleep(0.5)
 
    print "Podnoszenie reki..."
    Goal = PyKDL.Vector(bookX,bookY-0.7*znak,bookZ+1.55)
    T_B_Book = PyKDL.Frame(PyKDL.Rotation.RotZ(znak*0.5*math.pi), Goal)
    
    if not velma.moveCartImpRight([T_B_Book], [6.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    rospy.sleep(0.5)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)


    print "Ruch do pozycji zero..."
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)
    planAndExecute(q_map_zero)
    '''planAndExecute(js_start[1])'''


