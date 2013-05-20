#! /usr/bin/python

# Copyright: Carnegie Mellon University & Intel Corporation
# Author: Alvaro Collet (acollet@cs.cmu.edu)

import roslib; roslib.load_manifest('moped2')
import rospy
import random

import sys

import subprocess



class TestTiming():
  def test_play(self):
    rospy.sleep(2)
    self.moped = subprocess.Popen("rosrun moped2 moped2_test moped:=m _image_transport:=compressed",shell=True)
    rospy.sleep(10)
    self.recorder = subprocess.Popen("rosbag play `rospack find moped2`/test_data/timing.bag /moped/Image/compressed:=/Image/compressed",shell=True)


    if self.recorder.poll() is not None:
      assert(False)

    self.recorder.wait();
    self.moped.wait();
    print "DONE"


if __name__=="__main__":
  rospy.init_node('foo')                                                                                                                                           
  t=TestTiming();
  t.test_play();




  
