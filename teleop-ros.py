#!/usr/bin/python3
import rospy
import select, termios
import sys, tty
import time
import numpy as np

from std_msgs.msg import Float32MultiArray

old_settings = termios.tcgetattr(sys.stdin)

MAX_MOTOR_SPD = 50.0
MIN_MOTOR_SPD = 10.0

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def set_motor_value(motor_vel):
    msg = Float32MultiArray()
    msg.data.append(motor_vel)
    return msg

def limit_vel(vel):
    vel = vel if abs(vel) < MAX_MOTOR_SPD else np.sign(vel) * MAX_MOTOR_SPD
    return vel

def publish_motor_spd(pub, spd):
    pub.publish( set_motor_value(spd) )

def main():
    rospy.init_node("bringup_controller")
    motor_vel_pub = rospy.Publisher("/set_setpoint_motor", Float32MultiArray, queue_size=10)

    tty.setcbreak(sys.stdin.fileno())

    spd = 4
    spd_step = 2

    while not rospy.is_shutdown():
        input = select.select([sys.stdin], [], [], 0.1)[0]
        if input:
            key = sys.stdin.read(1)
            if key is not None:
                spd = limit_vel(spd)
                if key == 'w':
                    publish_motor_spd(motor_vel_pub, spd)
                elif key == 's':
                    publish_motor_spd(motor_vel_pub, -1* spd)
                elif key == '+':
                    spd += spd_step
                elif key == '-':
                    if spd > MIN_MOTOR_SPD:
                        spd -= spd_step
        else:
            publish_motor_spd(motor_vel_pub, 0.0)

        rospy.loginfo(f"motor_spd : {spd}")
        time.sleep(0.01)
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()