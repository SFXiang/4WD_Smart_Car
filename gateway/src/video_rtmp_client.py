# !/usr/bin/python
# -*-encoding:utf-8-*-

import rospy
import subprocess
import signal
from sensor_msgs.msg import Image
import cv2
import cv_bridge


# class RtmpClient:
#     def __init__(self):
#         self.rtmp_url = "rtmp://192.168.1.198:5555/myapp/dev1"
#         self.rtmp_sub = rospy.Subscriber("processed_frame", Image, self.start)
#         self.isFirst = True

#     def init_rmtp(self, sizeStr, fps):
#         command = ['ffmpeg', '-re', '-y',
#                    '-f', 'rawvideo',
#                    '-pix_fmt', 'bgr24',
#                    '-s', sizeStr,
#                    '-r', str(fps),
#                    '-i', '-',
#                    '-pix_fmt', 'yuv420p',
#                    '-preset', 'ultrafast',
#                    '-f', 'flv', self.rtmp_url]
#         self.pipe = subprocess.Popen(command, stdin=subprocess.PIPE, shell=False)

#     def start(self, processed_frame):
#         if self.isFirst:
#             size = (int(processed_frame.shape[1], int(processed_frame.shape[0])))
#             fps = 30
#             self.init_rmtp(size, fps)
#             self.isFirst = False
#         self.push_frame(processed_frame.data)

#     def push_frame(self, frame):
#         if not self.pipe:
#             print "not open rtmp process"
#             return
#         self.pipe.stdin.write(frame.tostring())

#     def __enter__(self):
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         self.close_push()

#     def close_push(self):
#         self.pipe.send_signal(signal.SIGINT)


# # parameters
# # rtmp_address

# # subscriber
# # topic /
# #  Image
# #  /post_image
# #  /LaneImage
# #  /

# class RtmpFileSystem(object):
#     """

#     """

#     def make_client(self, rtmp_address):
#         try:
#             ## allocate
#             ## initial
#             ## assign
#             client = RtmpClient() # client -> memory block
#         except Exception, e:
#             client = None

#         return client

#     def __init__(self, rtmp_address):
#         self.x = 0
#         self._rtmp_address = rtmp_address

#     def open(self):
#         if self._rtmp_address:
#             pass

#     def write(self, frame):
#         pass

#     def __enter__(self):
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         pass

#     def close(self):
#         pass

class App(object):
    # resource management
    def __init__(self):
        self._rtmp_address = None
        self._rtmp_initial = False
        self._br = cv_bridge.CvBridge()

    def setup(self):
        # launch <param name = "rtmp_address" type="str" value="">
        self._rtmp_address = rospy.get_param("rtmp_address",default="rtmp://192.168.1.198:5555/myapp/dev1")
        if self._rtmp_address is None:
            rospy.logerr("rtmp address not set")
            exit(-1)
        rospy.loginfo("rtmp_address: %s" % self._rtmp_address)

        topic = rospy.get_param("image_topic", default="LaneImage")
        self._image_sub = rospy.Subscriber(topic, Image, self._image_cb)
        rospy.loginfo("subscribe image topic: %s" % topic)

    def _image_cb(self, msg):
        """

        :param msg:
        :type msg: Image
        :return:
        """
        if not self._rtmp_initial:
            self._init_rtmp((msg.width, msg.height), 30)
        if self._rtmp_initial:
            frame = self._br.imgmsg_to_cv2(msg)
            self.push_frame(frame)

    def _init_rtmp(self, size, fps):
        """
        :param size:  (width, height)
        :param fps: 
        :return: 
        """

        command = ['ffmpeg', '-re', '-y',
                   '-f', 'rawvideo',
                   '-pix_fmt', 'bgr24',
                   '-s', "%sx%s" % size,
                   '-r', str(fps),
                   '-i', '-',
                   '-pix_fmt', 'yuv420p',
                   '-preset', 'ultrafast',
                   '-f', 'flv', self._rtmp_address]
        self.pipe = subprocess.Popen(command, stdin=subprocess.PIPE, shell=False)

        if self.pipe is None:
            rospy.logerr("rtmp start false.")
        else:
            self._rtmp_initial = True

    def __enter__(self):
        """
        resource allocation
        :return: 
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def cleanup(self):
        # do resource release
        if hasattr(App,"pipe"):
        # pipe = self.__getattribute__("pipe")
        # if pipe:
            self.pipe.send_signal(signal.SIGINT)

    def push_frame(self, frame):
        if not self.pipe:
            print "not open rtmp process"
            return
        self.pipe.stdin.write(frame.tostring())


if __name__ == "__main__":
    rospy.init_node("Video_rtmp_client")
    # resource management
    with App() as app:
        app.setup()
        rospy.spin()




