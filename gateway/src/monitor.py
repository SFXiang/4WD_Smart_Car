#!/bin/python
# -*-encoding:utf-8-*-

from monitor_msgs.msg import ModuleHealth, ModuleCheck
import rospy
from common import Timer
from location_msgs.msg import UtmPoint
from nav_msgs.msg import Odometry
from can_msgs.msg import Feedback
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped


class Unit:
    def __init__(self, state, category):
        self._state = state
        self._category = category

    @property
    def state(self):
        return self._state

    @property
    def category(self):
        return self._category

    @state.setter
    def state(self, v):
        if isinstance(v, int):
            self._state = v
        else:
            raise ValueError()

    @category.setter
    def category(self, v):
        if isinstance(v, int):
            self._category = v
        else:
            raise ValueError()


class Monitor:
    def __init__(self, nodes, timer):
        """
        :type nodes: list of str
        :type timer: Timer
        """
        self._nodes = {}
        for node in nodes:
            if node == "/rosout":
                continue
            self._nodes.setdefault(
                node,
                Unit(ModuleHealth.MODULE_NOT_STARTED,
                     ModuleHealth.MODULE_TYPE_GENERAL))
        self._requestId = -1
        self._cur_nodes = dict()
        self._in_check = False
        self._timer = timer
        self._timer_handler = None
        self._car_values = dict()
        self._check_timer_id = None

    def start(self):
        self.pub = rospy.Publisher("ModuleCheck", ModuleCheck, queue_size=10)
        rospy.Subscriber("ModuleHealth",
                         ModuleHealth,
                         self.module_health_cb,
                         queue_size=10)
        rospy.Subscriber("utm", UtmPoint, self.utm_cb)
        rospy.Subscriber("feedback", Feedback, self.feedback_cb)
        # rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("/ndt/current_pose", PoseStamped, self.currentpose_cb)
        self._check_timer_id = "Monitor_handler"
        self._timer.submit_periodical_job(self.check_health, 20,
                                          self._check_timer_id)

    def get_next_request_id(self):
        self._requestId += 1
        return self._requestId

    def module_health_cb(self, msg):
        assert isinstance(msg, ModuleHealth)
        self._update_node(msg.moduleName, msg.state, msg.moduleType)
        if msg.requestId != self._requestId:
            return
        self._cur_nodes.setdefault(msg.moduleName, True)
        self._try_update()

    def _try_update(self):
        if len(self._nodes.keys()) != len(self._cur_nodes.keys()):
            return
        self._timer.remove_job(self._timer_handler)
        self._in_check = False
        self._cur_nodes = {}

    def _update_node(self, nodeName, status, category):
        rospy.loginfo("[monitor] update node {}, {}, {}".format(
            nodeName, status, category))
        self._nodes[nodeName] = Unit(status, category)

    def utm_cb(self, point):
        """
        :type point: UtmPoint
        :return:
        """

    def feedback_cb(self, feedback):
        """
        :type feedback: Feedback
        :return:
        """
        self._car_values["v"] = feedback.cur_speed
        self._car_values["wheelAngle"] = feedback.cur_steer
        self._car_values["gear"] = 1

    def odom_cb(self, odom):
        """
        :type odom: Odometry
        :return:
        """
        # todo set the position's status.
        self._car_values["position"] = {
            "x": odom.pose.pose.position.x,
            "y": odom.pose.pose.position.y,
            "z": odom.pose.pose.position.z,
        }
        # todo set the orientation's status
        orientation = odom.pose.pose.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        roll, pitch, yaw = tr.euler_from_quaternion([x, y, z, w])
        self._car_values["orientation"] = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        }

    def currentpose_cb(self, msg):
        """
        :type msg: PoseStamped
        :return:
        """
        # todo set the position's status.
        self._car_values["position"] = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
        }
        # todo set the orientation's status
        orientation = msg.pose.orientation
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        roll, pitch, yaw = tr.euler_from_quaternion([x, y, z, w])
        self._car_values["orientation"] = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        }

    def check_health(self):
        check_msg = ModuleCheck()
        check_msg.header.stamp = rospy.Time.now()
        check_msg.requestId = self.get_next_request_id()
        check_msg.components = self._nodes.keys()
        self._timer_handler = self._timer.submit_job(self.on_response_timeout,
                                                     10)
        self._in_check = True
        self.pub.publish(check_msg)

    def on_response_timeout(self):
        if self._in_check:
            for node in self._nodes.keys():
                unit = self._cur_nodes.get(node)
                if not unit:  # todo set as timeout in the near future, and if serveral timeout happens, marked as error
                    self._nodes[node].state = ModuleHealth.MODULE_ERROR
        self._in_check = False

    def get_car_status(self):
        status = {}
        status["timestamp"] = int(rospy.Time.now().to_time() * 1000)
        status["pose"] = {
            "position": self._car_values.get("position"),
            "orientation": self._car_values.get("orientation")
        }
        status["v"] = self._car_values.get("v")
        status["wheelAngle"] = self._car_values.get("wheelAngle")
        status["gear"] = self._car_values.get("gear")
        status["nodes"] = []
        for node, unit in self._nodes.items():
            status["nodes"].append({
                "name": node,
                "status": unit.state,
                "type": unit.category
            })

        return status

    def get_modules(self):
        status = []
        for node, unit in self._nodes.items():
            status.append({
                "name": node,
                "status": unit.state,
                "type": unit.category
            })
        return status

    def get_module(self, module_name):
        for node, unit in self._nodes.items():
            if node == module_name:
                return {
                    "name": node,
                    "status": unit.state,
                    "type": unit.category
                }
        return {}
