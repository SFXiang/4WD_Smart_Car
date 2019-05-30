#!/bin/python
# -*-encoding:utf-8-*-

from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.date import DateTrigger
import datetime
from enum import Enum
import ConfigParser


class Timer:
    def __init__(self):
        self.scheduler = BackgroundScheduler()

    def submit_job(self, func, delay_seconds):
        """
        :param func:
        :param delay_seconds:
        :return:
        :rtype str
        """
        return self.scheduler.add_job(
            func=func,
            trigger=DateTrigger(datetime.datetime.now() +
                                datetime.timedelta(seconds=delay_seconds))).id

    def submit_periodical_job(self, func, period_seconds, id):
        return self.scheduler.add_job(func,
                                      'interval',
                                      seconds=period_seconds,
                                      id=id).id

    def remove_job(self, job_id):
        self.scheduler.remove_job(job_id)

    def start(self):
        self.scheduler.start()

    def shutdown(self):
        self.scheduler.remove_all_jobs()
        self.scheduler.shutdown()


class EventType(Enum):
    InterruptedEvent = 0x10
    ConnectionMadeEvent = 0x11
    ConnectionDisconnectedEvent = 0x12
    DataReceivedEvent = 0x13
    TaskFinishedEvent = 0x20


class Event:
    def __init__(self, eventType, data=None):
        self.eventType = eventType
        self.data = data

    def get_type(self):
        return self.eventType

    def get_data(self):
        return self.data


class Config:
    def __init__(self, config_file):
        cf = ConfigParser.ConfigParser()
        cf.read(config_file)
        self.params = dict()
        self.params['host'] = cf.get("server", "host")
        self.params['port'] = int(cf.get("server", "port"))
        self.params['version'] = cf.get("protocol", "version")
        self.params['vin'] = cf.get("vehicle", "vin")

    @property
    def host(self):
        return self.params['host']

    @host.setter
    def host(self, host):
        self.params['host'] = host

    @property
    def port(self):
        return self.params['port']

    @port.setter
    def port(self, port):
        self.params['port'] = port

    @property
    def version(self):
        return self.params['version']

    @version.setter
    def version(self, version):
        self.params['version'] = version

    @property
    def vin(self):
        return self.params['vin']

    @vin.setter
    def vin(self, vin):
        self.params['vin'] = vin

    def to_json(self):
        return self.params
