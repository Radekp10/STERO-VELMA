#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
 
from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError
 
if __name__ == "__main__":

    # define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.57,
        'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.57,
        'right_arm_3_joint':1.57,   'left_arm_3_joint':-1.7,
        'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,  'left_arm_5_joint':1.57,
        'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }

    rospy.init_node('test_cimp_pose')

    rospy.sleep(0.5)

    print "This test/tutorial executes simple motions"\
        " in Cartesian impedance mode.\n"

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

    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
    print "Planner init ok"

    ####### KONIEC INICJALIZACJI ######

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
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
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            exitError(6)
    
    if velma.enableMotors() != 0:
        exitError(14)
    
    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()

    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)
      
    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)

    # get initial configuration
    js_init = velma.getLastJointState()

    #planAndExecute(q_map_1)
    
    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
     
    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)
    
    print "Reset tools for both arms..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_B_Wl = velma.getTf("B", "Wl")

    #######################

    if not velma.moveCartImpRight([T_B_Wr], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)

    if not velma.moveCartImpLeft([T_B_Wl], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)

    if velma.waitForEffectorRight() != 0:
        exitError(9)

    if velma.waitForEffectorLeft() != 0:
        exitError(9)
   
    print "Moving right wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.3 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Tr"), 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)
    ### lapa w przod i dalej bedzie rysowanie ###


    print "Proba ruchu (1) ..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.3 , -0.6 , 1.3 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Proba ruchu (2) ..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , -0.6 , 1.5 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Proba ruchu (3) ..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.7 , 0.0 , 1.5 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Proba ruchu (4) ..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.3 , 0.0 , 1.3 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Proba ruchu (5) ..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.3 , -0.3 , 1.3 ))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

    print "Powrot do pozycji poczatkowej..."
    velma.moveJoint(q_map_starting , 9.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()
