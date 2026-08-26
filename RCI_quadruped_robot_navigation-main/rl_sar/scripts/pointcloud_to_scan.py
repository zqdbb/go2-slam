#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
class CloudScan(Node):
 def __init__(self):
  super().__init__('pointcloud_to_scan'); self.pub=self.create_publisher(LaserScan,'/scan',10); self.sub=self.create_subscription(PointCloud2,'/velodyne_points',self.cb,10)
 def cb(self,m):
  n=720; a0=-math.pi; inc=2*math.pi/n; ranges=[float('inf')]*n
  for x,y,z in point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True):
   if z < -0.1 or z > 1.2: continue
   r=math.hypot(x,y)
   if r<0.2 or r>30: continue
   i=int((math.atan2(y,x)-a0)/inc)
   if 0<=i<n and r<ranges[i]: ranges[i]=r
  s=LaserScan(); s.header=m.header; s.header.frame_id='vlp16'; s.angle_min=a0; s.angle_max=math.pi; s.angle_increment=inc; s.range_min=.2; s.range_max=30.; s.ranges=ranges; self.pub.publish(s)
def main(): rclpy.init(); rclpy.spin(CloudScan()); rclpy.shutdown()
if __name__=='__main__': main()
