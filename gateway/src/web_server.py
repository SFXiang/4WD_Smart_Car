#!/bin/python
# -*- encoding:utf-8 -*-


from flask import Flask, request, Response, jsonify
from monitor import Monitor
from switch import DrivingTaskService
from common import Config
import threading


class WebServer(threading.Thread):
    """
        resources here are dependent on other modules, so dependencies should be registered in constructor.
        Current, we provided several services including task, module state,
        And in the near future, pictures and processed values should be provided and monitored.
        This module, the start method should be called at main-thread.
        todo register dependencies.
    """

    def __init__(self, monitor_service, task_service, config):
        """
        :param monitor_service:
        :type monitor_service: Monitor
        :param task_service:
        :type task_service DrivingTaskService
        :param config
        :type config Config
        """
        threading.Thread.__init__(self)
        self.monitor_service = monitor_service
        self.task_service = task_service
        self.config = config
        self.app = Flask(__name__)
        self.host = "0.0.0.0"
        self.port = 9080
        self._add_uri_handler()

    def _add_uri_handler(self):
        self.app.add_url_rule("/tasks", 'api_tasks', self.api_tasks, methods=['GET'])
        self.app.add_url_rule("/task/<task_id>", 'api_task', self.api_task, methods=['GET'])
        self.app.add_url_rule("/car", "api_car", self.api_car, methods=['GET'])
        self.app.add_url_rule("/modules", "api_modules", self.api_modules, methods=['GET'])
        self.app.add_url_rule("/module/<module_name>", "api_module", self.api_module, methods=["GET"])
        self.app.add_url_rule("/config", "api_config", self.api_config, methods=["GET"])
        self.app.add_url_rule("/shutdown", 'api_shutdown', self.api_shutdown, methods=['POST'])

    def api_tasks(self):  # query all tasks
        return jsonify(self.task_service.get_tasks())

    def api_task(self, task_id):
        return jsonify(self.task_service.get_task(task_id))

    def api_car(self):
        return jsonify(self.monitor_service.get_car_status())

    def api_modules(self):
        return jsonify(self.monitor_service.get_modules())

    def api_module(self, module_name):
        return jsonify(self.monitor_service.get_module(module_name))

    def api_config(self):
        return jsonify(self.config.to_json())

    def run(self):
        self.app.run(self.host, self.port)

    def api_shutdown(self):
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()

    def shutdown(self):
        import requests
        shutdown_url = "http://localhost:%d/shutdown" % self.port
        print "shutdown web_server"
        requests.post(shutdown_url)
