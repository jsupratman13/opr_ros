#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import serial
from std_msgs.msg import Bool


def valid_packet(packet: bytes) -> bool:
    header = packet[0]
    data = packet[1:3]
    checksum = packet[3]

    if header != 0x7E:
        return False

    calculated_checksum = header ^ data[0] ^ data[1]
    return calculated_checksum == checksum


def main() -> None:
    rospy.init_node('start_stop_button_node')
    port = rospy.get_param('~port')
    baudrate = rospy.get_param('~baudrate', 9600)

    start_pub = rospy.Publisher('button/start', Bool, queue_size=10)
    stop_pub = rospy.Publisher('button/stop', Bool, queue_size=10)

    ser = serial.Serial(port, baudrate, timeout=1)
    rospy.loginfo(f'Reading from serial port {port}')

    while not rospy.is_shutdown():
        try:
            # read package 4 bytes: header, stop, start, checksum
            if ser.in_waiting >= 4:
                packet = ser.read(4)
                if not valid_packet(packet):
                    continue
                stop_pub.publish(bool(packet[1]))
                start_pub.publish(bool(packet[2]))
        except serial.SerialException as e:
            rospy.logerr(f'Serial error: {e}')
            break


if __name__ == '__main__':
    main()
