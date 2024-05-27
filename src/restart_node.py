#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from rosmon_msgs.srv import StartStop, StartStopRequest
import time

class ImageMonitor:
    def __init__(self):
        rospy.init_node('restar_node')
        self.sub = rospy.Subscriber('/robot/top_thermal_ptz_camera/image_raw', Image, self.image_callback) # TODO: Change the topic name
        self.service = rospy.ServiceProxy('/rosmon_sensors/start_stop', StartStop) # TODO: Change the service name
        self.last_received_time = time.time()
        self.elapsed_time = rospy.get_param('~elapsed_time', 3*60)  # get parameter, default to 4 minutes

    def image_callback(self, data):
        self.last_received_time = time.time()

    def restart_node(self, node_name: str) -> bool:
        try:
            self.service.wait_for_service(timeout=0.25)
            request = StartStopRequest()
            #request.node_name = node_name # TODO: Change the node name
            #request.namespace = namespace # TODO: Change the node namespace
            request.action = StartStopRequest.RESTART
            self.service(request)
            rospy.loginfo("Service called to restart node")
        except rospy.ROSException as e:
            rospy.logwarn("Service is not available: %s", str(e))
            return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", str(e))
            return False
        return True

    def check_time_and_call_service(self):
        while not rospy.is_shutdown():
            if time.time() - self.last_received_time > self.elapsed_time:
                result = self.restart_node('node_name')
                if result:
                    rospy.loginfo("Node restarted")
                    rospy.signal_shutdown("Sensor screen restarted.")
            rospy.sleep(1)

if __name__ == "__main__":
    monitor = ImageMonitor()
    monitor.check_time_and_call_service()