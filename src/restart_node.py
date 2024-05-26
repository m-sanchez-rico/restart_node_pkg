#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from rosmon_msgs.srv import StartStop, StartStopRequest
import time

class ImageMonitor:
    def __init__(self):
        rospy.init_node('restar_node')
        self.sub = rospy.Subscriber('/image_topic', Image, self.image_callback) # TODO: Change the topic name
        self.service = rospy.ServiceProxy('/service_name', StartStop) # TODO: Change the service name
        self.last_received_time = time.time()
        self.elapsed_time = rospy.get_param('~elapsed_time', 4*60)  # get parameter, default to 4 minutes
        str

    def image_callback(self, data):
        self.last_received_time = time.time()
    
    def restart_node(self, node_name: str):
        try:
            request = StartStop()
            #request.node_name = node_name # TODO: Change the node name
            #request.namespace = namespace # TODO: Change the node namespace
            request.action = StartStopRequest.RESTART
            self.service(request)
            rospy.loginfo("Service called to restart node")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def check_time_and_call_service(self):
        while not rospy.is_shutdown():
            if time.time() - self.last_received_time > self.elapsed_time:
                self.restart_node('node_name')
            rospy.sleep(1)

if __name__ == "__main__":
    monitor = ImageMonitor()
    monitor.check_time_and_call_service()