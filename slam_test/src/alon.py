#!/usr/bin/env python
import json
import rospy
from os.path import expanduser, join
from datetime import datetime
from custom_msgs.msg import slam_in

count = 1

start_time = str(datetime.now().time()).replace(":", "_").replace(".", "_")

home = expanduser("~")

file_name = start_time+'.txt'

file_path = join(home, file_name)

def callback(msg):
  global count, file_path
  data = {}
  data[count] = [{
  'yellow_x': msg.yellow_x,
  'yellow_y': msg.yellow_y,
  'blue_x':   msg.blue_x,
  'blue_y':   msg.blue_y,
  'pose':     [msg.pos_x, msg.pos_y],
  'normal':   [msg.normal_x, msg.normal_y]
  }]
  count = count + 1
  with open(file_path, 'a') as outfile:
    json.dump(data,outfile)
    outfile.write('\n')

def listener():
  rospy.init_node('alon_debug', anonymous=True)
  topic_in = rospy.get_param('/orb_slam2/points_topic', 'points_map')
  rospy.Subscriber(topic_in, slam_in, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
