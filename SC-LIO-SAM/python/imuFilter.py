#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rospy
import time
import sys
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class ImuFilter:

  def __init__(self):
    self.sub = rospy.Subscriber("/imu/data", Imu, self.callback)
    self.first = True
    self.pub = rospy.Publisher('/imu/data_repetition_free', Imu, queue_size=10)
    self.dt = rospy.Duration(0)
    

  def callback(self,data):
    data_tmp = data
    #print(data.header.stamp)
    now = rospy.get_rostime()
    now.secs = data_tmp.header.stamp.secs
    now.nsecs = data_tmp.header.stamp.nsecs #*1000
    data_tmp.header.stamp = now
    
    if not self.first:  
      self.dt = now - self.prev
    else:
      self.first = False
    self.prev = now
    
      
      
    #print(self.dt)
    if self.dt > rospy.Duration(0.0001):
      #print("pub")
      self.pub.publish(data_tmp)
    else:
      print("repetition")
      
      
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
