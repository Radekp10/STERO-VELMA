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

    q_map_1 = {'torso_0_joint':1.56,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_2 = {'torso_0_joint':-1.56,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

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

    ####### KONIEC INICJALIZACJI ######
    
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

    print "moving head to position: down..."
    q_dest = (0, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso left..."
    velma.moveJoint(q_map_1, 9.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()

    print "moving head to position: left..."
    q_dest = (1.56, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: up.."
    q_dest = (1.56, -0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: zero, up..."
    q_dest = (0, -0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso right..."
    velma.moveJoint(q_map_2 , 18.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()

    print "moving head to position: right..."
    q_dest = (-1.56, -0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: down..."
    q_dest = (-1.56, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: zero, down..."
    q_dest = (0, 0.7)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso position 0 ..."
    velma.moveJoint(q_map_starting , 9.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()

    print "moving head to position: zero.."
    q_dest = (0, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso left..."
    velma.moveJoint(q_map_1, 9.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()

    print "moving head to position: left..."
    q_dest = (1.56, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: zero..."
    q_dest = (0, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso right..."
    velma.moveJoint(q_map_2 , 18.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()

    print "moving head to position: right..."
    q_dest = (-1.56, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "moving head to position: zero..."
    q_dest = (0, 0)
    velma.moveHead(q_dest, 5.0, start_time=0.5)
    if velma.waitForHead() != 0:
        exitError(6)
    rospy.sleep(0.5)
    if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
        exitError(7)

    print "Moving torso position 0 ..."
    velma.moveJoint(q_map_starting , 9.0, start_time=0.5, position_tol=0.1)
    velma.waitForJoint()
