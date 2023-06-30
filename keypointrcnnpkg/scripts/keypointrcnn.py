#!/usr/bin/env python

import sys
import os
import rospy
import torch
import time
from torch.utils.data import Dataset, DataLoader
import cv2
import numpy as np
import torchvision
from torchvision.models.detection.rpn import AnchorGenerator
from torchvision.transforms import functional as F
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('abc')
bridge = CvBridge()
cv_image = None

print(os.getcwd())
#########################################################################################################################################################

def visualize(cv2_img,kp):
  img = np.copy(cv2_img)
  kp = np.round(kp)
  kp = kp.astype(np.int32)
  for i in range(4):
    x1 = kp[0,i]
    y1 = kp[1,i]
    x2 = kp[0,i+1]
    y2 = kp[1,i+1]
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),5)
  cv2.imshow('',img)
  cv2.waitKey(5)

#########################################################################################################################################################

def get_model(num_keypoints, weights_path=None):
    
    anchor_generator = AnchorGenerator(sizes=(32,64, 128, 256, 512), aspect_ratios=(0.25, 0.5, 0.75, 1.0, 2.0, 3.0, 4.0))
    model = torchvision.models.detection.keypointrcnn_resnet50_fpn(pretrained=False,
                                                                   pretrained_backbone=True,
                                                                   num_keypoints=num_keypoints,
                                                                   num_classes = 2, # Background is the first class, object is the second class
                                                                   rpn_anchor_generator=anchor_generator)

    if weights_path:
        state_dict = torch.load(weights_path)
        model.load_state_dict(state_dict)        
        
    return model

#########################################################################################################################################################

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model=torch.load('real_keypoint.pt')
model.to(device)
model.eval()



print(device)

now = time.time()

def sub_callback(ros_img):
  global cv_image
  cv_image = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(ros_img.height, ros_img.width, -1)
  cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)


if __name__ == '__main__':
  while not rospy.is_shutdown():
    sub = rospy.Subscriber('/camera/color/image_raw',Image,callback=sub_callback,queue_size=1)
    time.sleep(0.1)
    sub.unregister()
    img = F.to_tensor(cv_image).to(device)
    output = model([img])
    scores = output[0]['scores'].detach().cpu().numpy()
    #print(scores)
    kp = output[0]['keypoints'][np.where(scores > 0.5)[0].tolist()].detach().cpu().numpy().astype(np.int32)
    kp = kp[0,:,0:2]
    print(kp)
    print("Completed in ",end='')
    print(time.time()-now)
    now = time.time()
    visualize(cv_image,np.array(kp).T)


'''
for i in range(10):
 output = model([img])
 scores = output[0]['scores'].detach().cpu().numpy()
 #print(scores)
 kp = output[0]['keypoints'][np.where(scores > 0.5)[0].tolist()].detach().cpu().numpy().astype(np.int32)
 kp = kp[0,:,0:2]
 print(kp)
 print("Completed in ",end='')
 print(time.time()-now)
 now = time.time()

'''
