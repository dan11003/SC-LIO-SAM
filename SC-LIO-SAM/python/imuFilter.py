#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rospy
import time
import sys
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def callback(data):
    global prev
    
    

class ImuFilter:

  def __init__(self):
    self.sub = rospy.Subscriber("/imu/data", Imu, self.callback)
    self.first = True
    self.pub = rospy.Publisher('/imu/data_repetition_free', Imu, queue_size=10)


  def callback(self,data):
    data_tmp = data
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.stamp)
    now = rospy.get_rostime()
    now.secs = data_tmp.header.stamp.secs
    now.nsecs = data_tmp.header.stamp.nsecs*1000
    data_tmp.header.stamp = now
    if self.first:
      publish = True
      self.prev = now
      self.first = False
    dt = now - self.prev
    print(dt)
    if dt > rospy.Duration(0.0001):
      print("pub")
      self.pub.publish(data_tmp)
      
      
    #rospy.loginfo("Current time %i %i", now.secs, now.nsecs)    

def main(args):
  obc = ImuFilter()
  rospy.init_node('ImuFilterNode', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
