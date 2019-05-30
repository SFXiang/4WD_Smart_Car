#!/bin/python
# -*-encoding:utf-8-*-
import json
import time

import rospy
from enum import IntEnum, Enum
from track_msgs.msg import LaneSection, ReplaySection, TaskShutdown, TrjSection, SectionTaskState, WayPoint, Station, \
    SiteSection
from can_msgs.msg import ECU
from common import EventType, Event
from smartcar_msgs.msg import Lane, Waypoint, State


# data:
#       task_id: string,
#       speed: double,
#       actions: [{
#           "outset": {
#               "name": str,
#               "x": double,
#               "y": double
#           },
#           "goal": {
#               "name": str,
#               "x": double,
#               "y": double
#           },
#           "v": double,
#           "type": str,
#           "points": [], // for TRAJECTORY
#           "instructions": [], // for Replay
#           "site": {}, // for SiteTask
#        },{
#           "outset": {
#               "name": str,
#               "x": double,
#               "y": double
#           },
#           "goal": {
#               "name": str,
#               "x": double,
#               "y": double
#           },
#           "v": double,
#           "type": str,
#           "points": []
#           "instructions": []
#        }, ...
#       ]
#       type: REPLAY, LANE, TRJ
#       instructions:
#           default
#           [{"motor":, "servo": , "shift":}, ...]
#
#       lane:
#           nothing.
#
#       trajectory:
#           [{points}]
#
#       site:
#           {
#               "outset": {
#               },
#               "goal": {
#               },
#               "v": float,
#               "type": str,
#               "site":{
#                   "fromQr": , // string
#                   "toQr": // string,
#                   "actionScene": // string,
#                   "toBaseX": , // float
#                   "toBaseZ": , // float
#                   "shift": , // int
#                   "curveRadius": , // float
#                   "curveAngle": // float
#                   "fromBaseX": // float
#                   "fromBaseZ": // float
#                   "speed": // float

#               }
#           }
#
#


class ActionType(Enum):
    REPLAY = "REPLAY"
    LANE = "LANE"
    TRAJECTORY = "TRJ"
    SITE = "SITE"
    LIDAR = "LIDAR"


class TaskState(IntEnum):
    CREATED = 0x00
    RUNNING = 0x03
    FINISHED = 0x08
    INTERRUPTED = 0x10
    DEVICE_INTERRUPTED = 0x18
    KILLING = 0x30
    KILLED = 0x31


class SectionTask:
    """
    """

    @staticmethod
    def create_section_task(section_id, action):
        outset = Station()
        outset.name = str(action['outset']['name'])
        outset.x = float(action['outset']['x'])
        outset.y = float(action['outset']['y'])
        goal = Station()
        goal.name = str(action['goal']['name'])
        goal.x = float(action['goal']['x'])
        goal.y = float(action['goal']['y'])
        v = float(action['v'])
        # print action
        # print("action: type: %s action.trj : %s" % (action['type'], str(ActionType.TRAJECTORY)))
        if action['type'] == "REPLAY":
            return ReplaySectionTask(section_id, v, outset, goal,
                                     action['instructions'])
        elif action['type'] == "LANE":
            return LaneSectionTask(section_id, v, outset, goal)
        elif action['type'] == "TRJ":
            return TrjSectionTask(section_id, v, outset, goal,
                                  action['points'])
        elif action['type'] == "SITE":
            return SiteSectionTask(section_id, v, outset, goal, action['site'])
        elif action['type'] == "LIDAR":
            return LidarSectionTask(section_id, v, outset, goal, action['points'])
        return None

    def __init__(self, section_id, speed, outset, goal):
        self.section_id = str(section_id)
        self.v = speed
        self.outset = outset
        self.goal = goal
        self.start_stamp = None
        self.end_stamp = None
        self.status = TaskState.CREATED
        self.percentage = 0

    def get_section_id(self):
        return self.section_id

    def get_speed(self):
        return self.v

    @property
    def start_stamp(self):
        return self.start_stamp

    @start_stamp.setter
    def start_stamp(self, stamp):
        self.start_stamp = stamp

    @property
    def end_stamp(self):
        return self.end_stamp

    @end_stamp.setter
    def end_stamp(self, stamp):
        self.end_stamp = stamp

    @property
    def state(self):
        return self.state

    @state.setter
    def state(self, state):
        self.state = state

    @property
    def percentage(self):
        return self.percentage

    @percentage.setter
    def percentage(self, percentage):
        self.percentage = percentage


class LaneSectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal):
        SectionTask.__init__(self, section_id, speed, outset, goal)

    def to_lane_task(self):
        lane_section = LaneSection()
        lane_section.speed = self.v
        lane_section.sectionId = self.section_id
        lane_section.direction = 1
        lane_section.outset = self.outset
        lane_section.goal = self.goal
        return lane_section


class ReplaySectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal, instructions_json_o):
        SectionTask.__init__(self, section_id, speed, outset, goal)
        self.frequency = 100
        self.ecus = map(self.instruction_to_ecu, instructions_json_o)

    @staticmethod
    def instruction_to_ecu(instruction):
        """
        :type instruction dict
        :return:
        """
        ecu = ECU()
        ecu.shift = int(instruction['shift'])
        ecu.servo = float(instruction['servo'])
        ecu.motor = float(instruction['motor'])
        return ecu

    def to_replay_task(self):
        replay = ReplaySection()
        replay.sectionId = self.section_id
        replay.frequency = self.frequency
        replay.instructions = self.ecus
        return replay


class TrjSectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal, points_json_o):
        SectionTask.__init__(self, section_id, speed, outset, goal)
        self.waypoints = map(self.point_to_waypoint, points_json_o)

    @staticmethod
    def point_to_waypoint(point):
        """
        :type point dict
        :param point:
        :rtype WayPoint
        :return:
        """
        way_point = WayPoint()
        way_point.x = point['x']
        way_point.y = point['y']
        way_point.v = point['v']
        way_point.t = point['t']
        return way_point

    def to_trj_task(self):
        task = TrjSection()
        task.sectionId = self.section_id
        task.waypoints = self.waypoints
        task.outset = self.outset
        task.goal = self.goal
        return task


class LidarSectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal, points_list):
        SectionTask.__init__(self, section_id, speed, outset, goal)
        self.waypoints = points_list

    # @staticmethod
    # def point_to_waypoint(point):
    #     """
    #     :type point dict
    #     :param point:
    #     :rtype WayPoint
    #     :return:
    #     """
    #     way_point = WayPoint()
    #     way_point.x = point['x']
    #     way_point.y = point['y']
    #     way_point.v = point['v']
    #     way_point.t = point['t']
    #     return way_point

    def make_global_path(self):
        global_path = Lane()
        global_path.header.frame_id = "map"
        for p in self.waypoints:
            point = Waypoint()
            point.speed_limit = float(p['speed_limit'])
            point.is_lane = int(p['is_lane'])
            point.lane_id = int(p['lane_id'])
            point.cross_id = int(p['cross_id'])
            point.pose.pose.position.x = float(p['position']['x'])
            point.pose.pose.position.y = float(p['position']['y'])
            point.pose.pose.position.z = float(p['position']['z'])
            point.pose.pose.orientation.x = float(p['orientation']['x'])
            point.pose.pose.orientation.y = float(p['orientation']['y'])
            point.pose.pose.orientation.z = float(p['orientation']['z'])
            point.pose.pose.orientation.w = float(p['orientation']['w'])
            global_path.waypoints.append(point)
        return global_path

class SiteSectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal, attach):
        SectionTask.__init__(self, section_id, speed, outset, goal)
        # print attach
        self._from_qr = str(attach['fromQr'])
        self._from_base_x = float(attach['fromBaseX'])
        self._from_base_z = float(attach['fromBaseZ'])
        self._to_qr = str(attach['toQr'])
        self._to_base_x = float(attach['toBaseX'])
        self._to_base_z = float(attach['toBaseZ'])
        self._action_scene = str(attach['scene'])
        self._shift = int(attach['shift'])
        self._curve_radius = float(attach['curveRadius'])
        self._curve_angle = float(attach['curveAngle'])
        self._speed = float(attach['speed'])

    def to_site_task(self):
        site_section = SiteSection()
        site_section.speed = self._speed
        site_section.sectionId = str(self.section_id)
        site_section.outset = self.outset
        site_section.goal = self.goal
        site_section.fromQr = self._from_qr
        site_section.fromBaseX = self._from_base_x
        site_section.fromBaseZ = self._from_base_z
        site_section.toQr = self._to_qr
        site_section.toBaseX = self._to_base_x
        site_section.toBaseZ = self._to_base_z
        site_section.actionScene = self._action_scene
        site_section.shift = self._shift
        site_section.curveRadis = self._curve_radius
        site_section.curveAngle = self._curve_angle
        print site_section
        return site_section


class DrivingTask:
    def __init__(self, task_id, speed, route_json_o):
        self.task_id = task_id
        self.speed = speed
        # print self.speed
        self.sections = self.to_sections(route_json_o)
        print "route_json_o"
        print route_json_o
        # print self.sections
        self.cur = -1
        self.state = TaskState.CREATED
        print("len(self.sections): %d" % len(self.sections))

    def get_task_id(self):
        return self.task_id

    def to_sections(self, route_json_o):
        """
        :param route_json_o:
        :type route_json_o json object
        :return:
        """
        sections = []
        for ind, action in enumerate(route_json_o):
            section = SectionTask.create_section_task(
                ("%s_%d" % (self.task_id, ind)), action)
            if section:
                sections.append(section)
        return sections

    def get_state(self):
        """"""
        return self.state

    def get_task_state(self):
        """"""
        state = dict()
        state['taskId'] = self.task_id
        state['status'] = int(self.state)
        state['timestamp'] = int(time.time() * 1000)
        if self.cur == len(self.sections):
            self.cur = len(self.sections) - 1
        state['offset'] = self.cur
        state['percentage'] = self.sections[self.cur].percentage
        return state

    def move_next(self):
        self.cur += 1
        if self.cur == len(self.sections):
            return None
        return self.sections[self.cur]

    def get_cur_section_id(self):
        if 0 <= self.cur < len(self.sections):
            return self.sections[self.cur].section_id
        return None

    def set_state(self, state):
        self.state = state

    def set_section_state(self, section_id, section_state):
        """
        :param section_id:
        :type section_id str
        :param section_state:
        :type section_state TaskInfo
        :return:
        """
        for i, section in enumerate(self.sections):
            if section.section_id == section_id:
                section.state = section_state.state
                section.percentage = section_state.percentage
                if section.state == TaskState.FINISHED or section.state == TaskState.KILLED:
                    section.end_stamp = time.time()
                break

    def get_all_info(self):
        """
        collect all the section tasks.
        :return:
        """
        info = {
            "taskId": self.task_id,
            "v": self.speed,
            "state": self.state,
            "actions": []
        }
        for section in self.sections:
            action = {
                "actionId": section.section_id,
                "gmtCreate": section.start_stamp,
                "gmtEnd": section.end_stamp,
                "state": section.state,
                "v": section.get_speed(),
                "percentage": section.percentage
            }
            info["actions"].append(action)
        return info


class DrivingTaskService:
    def __init__(self, event_bus):
        """
        :type event_bus: Queue.Queue
        """
        self._history_task = []
        self._current_task = None
        self._event_bus = event_bus
        self._add_pub_sub()

    def _add_pub_sub(self):
        self._replay_pub = rospy.Publisher("ReplaySection", ReplaySection, queue_size=10)
        self._lane_pub = rospy.Publisher("LaneSection", LaneSection, queue_size=10)
        self._trajectory_pub = rospy.Publisher("TrjSection", TrjSection, queue_size=10)
        self._site_pub = rospy.Publisher("SiteSection", SiteSection, queue_size=10)
        self._stop_pub = rospy.Publisher("TaskShutdown", TaskShutdown, queue_size=10)
        self._ecu_pub = rospy.Publisher("ecu", ECU, queue_size=10)
        self._global_path_pub = rospy.Publisher("global_path", Lane, queue_size=1)
        self._state_change_pub = rospy.Publisher("SmartcarState", State, queue_size=1)
        rospy.Subscriber("SectionTaskState", SectionTaskState, self.on_task_info, queue_size=10)

    def is_free(self):
        return self._current_task is None

    def start_driving_task(self, drivingTask):
        """
        :param drivingTask:
        :type drivingTask DrivingTask
        :return:
        """
        self._current_task = drivingTask
        self._fire_task()
        return True

    def _fire_task(self):
        section_task = self._current_task.move_next()
        rospy.loginfo("fire_task {} ".format(section_task))
        if section_task:
            if isinstance(section_task, ReplaySectionTask):
                replay = section_task.to_replay_task()
                self._replay_pub.publish(replay)
            elif isinstance(section_task, LaneSectionTask):
                lane = section_task.to_lane_task()
                self._lane_pub.publish(lane)
            elif isinstance(section_task, TrjSectionTask):
                trj = section_task.to_trj_task()
                self._trajectory_pub.publish(trj)
                rospy.loginfo("publish trj section")
            elif isinstance(section_task, SiteSectionTask):
                site = section_task.to_site_task()
                self._site_pub.publish(site)
                rospy.loginfo("publish site section")
            elif isinstance(section_task, LidarSectionTask):
                global_path = section_task.make_global_path()
                self._global_path_pub.publish(global_path)
                rospy.loginfo("[LidarSectionTask]: publish global path")
            section_task.start_stamp = time.time()
        else:
            self._current_task.set_state(TaskState.FINISHED)
            self._history_task.append(self._current_task)
            ecu = ECU()
            ecu.motor = 0
            ecu.servo = 0
            ecu.shift = 0
            self._ecu_pub.publish(ecu)
            self.report_finished()
            self._current_task = None

    def on_task_info(self, section_task_state):
        """
        :param section_task_state:
        :type section_task_state SectionTaskState
        :return:
        """
        self._current_task.set_section_state(section_task_state.sectionId,
                                             section_task_state)
        if section_task_state.state == TaskState.FINISHED:  # current is finished and prepared next
            rospy.loginfo("[Switch] received task finished")
            self._fire_task()
        if section_task_state.state == TaskState.KILLED:  # task is killed
            self._current_task.set_state(TaskState.KILLED)
            self.report_finished()
            self._history_task.append(self._current_task)
            self._current_task = None

    def _stop(self):
        stop_msg = TaskShutdown()
        stop_msg.sectionId = self._current_task.get_cur_section_id()
        self._stop_pub.publish(stop_msg)

    def report_finished(self):
        """"""
        self._event_bus.put(Event(EventType.TaskFinishedEvent,
                                  self._current_task.get_task_state()),
                            block=True)

    def task_pause(self):
        if self._current_task:
            state = State()
            state.main_state = "PAUSE"
            self._state_change_pub.publish(state)

    def task_continue(self):
        if self._current_task:
            state = State()
            state.main_state = "RUN"
            self._state_change_pub.publish(state)

    def stop(self):
        if self._current_task:
            self._stop()

    def get_task_status(self, taskId):
        if self._current_task is not None and self._current_task.get_task_id(
        ) == taskId:
            return self._current_task.get_task_state()
        for task in self._history_task:
            if task.taskId == taskId:
                return task.get_task_state()
        return None

    def get_tasks(self):
        tasks = []
        if self._current_task:
            tasks = [self._current_task]
        tasks.extend(self._history_task)
        return tasks

    def get_task(self, task_id):
        if self._current_task and self._current_task.get_task_id == task_id:
            return self._current_task
        for task in self._history_task:
            if task.taskId == task_id:
                return task

    def current_task(self):
        return self._current_task
