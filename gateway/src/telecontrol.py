#!/usr/bin/env python
# -*-encoding:utf-8-*-

import rospy
from can_msgs.msg import TeleControl, TeleReport


class TeleControlService(object):
    def __init__(self):
        self._car_info = TeleReport()
        self._car_info.gear = 0
        self._car_info.direction = 0
        self._car_info.wheelAngle = 0
        self._car_info.speed = 0
        self._car_info.driveMode = 0
        self._car_info.accGear = 0
        self._car_info.decGear = 0
        self._car_info.accumulateDistance = 0
        self._car_info.leftSignal = 0
        self._car_info.rightSignal = 0
        self._car_info.flashLights = 0
        self._car_info.parkingLights = 0
        self._car_info.dippedHeadLight = 0
        self._car_info.highBeam = 0
        self._car_info.rearFogLight = 0
        self._car_info.horn = 0
        self._car_info.enduranceMileage = 0
        self._car_info.health = 0
        self._car_info.charge = 0
        self._car_info.voltage = 0
        self._init_pub_sub()

    def _init_pub_sub(self):
        tele_control_topic = rospy.get_param("~tele_control_topic",
                                             "tele_control")
        car_info_topic = rospy.get_param("~tele_report_topic", "tele_report")
        self._pub = rospy.Publisher(tele_control_topic,
                                    TeleControl,
                                    queue_size=10)
        self._sub = rospy.Subscriber(car_info_topic,
                                     TeleReport,
                                     self._on_info_cb,
                                     queue_size=100)

    def _on_info_cb(self, msg):
        """

        :param msg:
        :type msg: tele_control
        :return:
        """
        self._car_info = msg

    def tele_control(self, msg):
        """
        :param msg:
        :return:
        """
        control_msg = TeleControl()
        control_msg.gear = msg['gear']
        control_msg.upGear = msg['upGear']
        control_msg.downGear = msg['downGear']
        control_msg.driveMode = msg['driveMode']
        control_msg.realWheelAngle = msg['realWheelAngle']
        control_msg.signalLeft = msg['signalLeft']
        control_msg.signalRight = msg['signalRight']
        control_msg.hazardLight = msg['hazardLight']
        control_msg.sideLight = msg['sideLight']
        control_msg.headLight = msg['headLight']
        control_msg.beanLight = msg["beanLight"]
        control_msg.rearFogLamp = msg['rearFogLamp']
        control_msg.horns = msg['horns']
        control_msg.modifyWheelAngle = msg['modifyWheelAngle']
        control_msg.speed = msg["speed"]
        control_msg.brake = msg['brake']
        self._pub.publish(control_msg)

    def car_info(self):
        dc = {
            "gear": self._car_info.gear,
            "direction": self._car_info.direction,
            "wheelAngle": self._car_info.wheelAngle,
            "speed": self._car_info.speed,
            "driveMode": self._car_info.driveMode,
            "accGear": self._car_info.accGear,
            "decGear": self._car_info.decGear,
            "accumulateDistance": self._car_info.accumulateDistance,
            "leftSignal": self._car_info.leftSignal,
            "rightSignal": self._car_info.rightSignal,
            "flashLights": self._car_info.flashLights,
            "parkingLights": self._car_info.parkingLights,
            "dippedHeadLight": self._car_info.dippedHeadLight,
            "highBeam": self._car_info.highBeam,
            "rearFogLight": self._car_info.rearFogLight,
            "horn": self._car_info.horn,
            "enduranceMileage": self._car_info.enduranceMileage,
            "health": self._car_info.health,
            "charge": self._car_info.charge,
            "voltage": self._car_info.voltage
        }
        return dc
