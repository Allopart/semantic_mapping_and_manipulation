#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('semantic_mapping')
import sys
import rospy
import cv2
import message_filters

import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import PIL.Image as pilimg

import caffe

h = std_msgs.msg.Header()

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("sem_img",Image)
    self.depth_pub = rospy.Publisher("sem_depth",Image)
    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image, self.image_callback)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_callback)

  def image_callback(self, rgb_data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Run FCN
    caffe.set_mode_gpu()
    caffe.set_device(0)
    in_ = np.array(cv_image, dtype=np.float32)
    in_ = in_[:,:,::-1]
    in_ -= np.array((104.00698793,116.66876762,122.67891434))
    in_ = in_.transpose((2,0,1))
    # shape for input (data blob is N x C x H x W), set data
    net.blobs['data'].reshape(1, *in_.shape)
    net.blobs['data'].data[...] = in_
		# run net and take argmax for prediction
    net.forward()
    out = net.blobs['score'].data[0].argmax(axis=0)
    # save as image
    palette_base = [i for i in xrange(0, 256, 255 / 3)]
    palette = [(palette_base[i], palette_base[j] , palette_base[k]) for i in xrange(4) for j in xrange(4) for k in xrange(4)]
    colors = np.array(palette, dtype=np.uint8)[out]

    # Create header
    global h
    h.stamp =  rospy.Time.now()
    h.frame_id = 'rgb_optical_frame'

    try:
      final_sem_img=self.bridge.cv2_to_imgmsg(colors, "bgr8")
      final_sem_img.header = h   
      self.image_pub.publish(final_sem_img)
    except CvBridgeError as e:
      print(e)


  def depth_callback(self, depth_data):
    depth_data.header = h
    try:
      self.depth_pub.publish(depth_data)
    except CvBridgeError as e:
      print(e)


def main(args):
  
  rospy.init_node('image_converter', anonymous=True)  
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':

    caffe.set_mode_gpu() #Change to gpu when we have 1070
    caffe.set_device(0)

    # load net
    net = caffe.Net('/home/adrian/git/caffe/models/fcn.berkeleyvision.org/voc-fcn8s/deploy.prototxt', '/home/adrian/git/caffe/models/fcn.berkeleyvision.org/voc-fcn8s/fcn8s-heavy-pascal.caffemodel', caffe.TEST)
    print("Finished loading net")

    main(sys.argv)

