# !/usr/bin/env python
# -*-encoding:utf-8 -*-


class BeanContainer(object):
    """
    stores various objects,
    usually, we stores the (name, object)
    interfaces for getBeansByType(), getBeanByName() is provided.

    How to manage the lifecycle of object, especially the objects'destroy.
    """

    def __init__(self):
        self._objects = []

    def register(self, bean_name, bean):
        """"""

    def get_bean_by_type(self):
        """"""

    def get_bean_by_name(self, bean_name):
        """"""

    def shutdown(self):
        """"""
