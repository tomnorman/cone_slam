#!/usr/bin/env python
import json
import rospy
from os.path import expanduser, join
from datetime import datetime
from custom_msgs.msg import slam_test_msg

count = 1

start_time = str(datetime.now().time()).replace(":", "_").replace(".", "_")

home = expanduser("~")

file_name = start_time+'.txt'

file_path = join(home, file_name)
print 'writing to:\n'
print file_path
print '\n'
def callback(msg):
  global count, file_path
  data = {}
  data[count] = [{
  'yellow_x': msg.yellow_cones_x,
  'yellow_y': msg.yellow_cones_y,
  'blue_x':   msg.blue_cones_x,
  'blue_y':   msg.blue_cones_y,
  'pose':     [msg.pos_x, msg.pos_y],
  'normal':   [msg.normal_x, msg.normal_y]
  }]
  count = count + 1
  with open(file_path, 'a') as outfile:
    json.dump(data,outfile)
    outfile.write('\n')

def listener():
  rospy.init_node('alon_debug', anonymous=True)
  topic_in = rospy.get_param('/cones_map/test_topic', 'slam_test')
  rospy.Subscriber(topic_in, slam_test_msg, callback)
  rospy.spin()

if __name__ == '__main__':
  listener()
