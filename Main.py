from NodeManager import NodeManager
from Camera import PS3EyeCamera
import random
import cv2
import os
from pathlib import Path
import glob
import numpy as np
import time
from scipy.optimize import least_squares

mtx = np.array([[ 554.2563,   0,          320],
 [  0,        554.2563,      240],
 [  0,          0,           1,        ]])
dist = np.array([[-0.10771770030260086, 0.1213262677192688,  0.00091733073350042105, 0.00010589254816295579]],dtype="double")







##Program Flow:
## 1. Add all trackers and calibration boards and Cameras
## 2. Choose Calibration method
## 3. Calibrate Colors if needed
## 4. Calibrate extrinsic matrices (global pos and rot of each cam)
## 5. Start Tracking :D
##First order of bussiness, color calibration

## Step 1
manager = NodeManager()

cameras = []
i = 0
CAMDIR = "/dev/v4l/by-path/pci-0000:00:14.0-usbv2-*"
#CAMDIR = "/dev/video*"

for camPath in glob.glob(CAMDIR):
    print(camPath)
    cameras.append(PS3EyeCamera(str(i),camPath))
    cameras[-1].trackSettings()
    cameras[-1].inMatrix = mtx
    cameras[-1].distortCoeff = dist
    i += 1
    
## This will later be replaced with a button click saying "Done adding".
while len(manager.nodes)<1:
    pass


def RGBtoBGR(color):
    return (color[2],color[1],color[0])
## This should   be changed such that:
## 1.It considers the average color in contour instead of the center
## 2.It considers the average color in a series of frames
## 3.It calculates a 3D margin of error plus 5% more error of error.
time.sleep(2)
def colorCalibration():
    calDataPath = Path("./CapturedCalData/")
    calDataPath.mkdir(parents=True,exist_ok=True)
    for _,node in manager.nodes.items():
        nodeCalDataPath = Path(f"./CapturedCalData/{node.color}/")
        nodeCalDataPath.mkdir(parents=True,exist_ok=True)
        node.light_on()
        print(node.name)
        for _,otherNode in manager.nodes.items():
            if node is otherNode:
                continue
            otherNode.light_off()
            print(f"Turn of {otherNode.color}, {otherNode.addr}")
        time.sleep(1)
        for cam in cameras:
            cam.readFrame()
        for cam in cameras:
            col = RGBtoBGR(node.color)
            obsCol,cnt = cam.calibrateColor(col)
            print(f"Cam {cam.name} observed {node.color} as {obsCol}.")
            timg = cam.frame.copy()
            cv2.drawContours(timg, [cnt], -1, (0,255,0), 3)
            cv2.imwrite(f"./CapturedCalData/{node.color}/{cam.name}.png",timg)
    print("Done color cap")
    for _,node in manager.nodes.items():
        node.light_on()
    
    
colorCalibration()


#calMethod = int(input("Cal method? 1 : Board , 2 : none board \n"))
#assert calMethod in [1,2]



###Below is getContourByColor Test code
#while True:
#    for cam in cameras:
#           cam.readFrame()
#    for cam in cameras:
#        img = cam.savedFrame
#        for _,node in manager.nodes.items():
#            col = RGBtoBGR(node.color)
#            f, l, c = cam.getNodeInCamSpace(col)
#            timg = img.copy()
#            cv2.drawContours(timg, [c], 0, (0,255,0), 3)
#            cv2.imshow(f"Cam {cam.name} found color {node.color}",timg)
#            cv2.waitKey(1)
#



## Let's try template-less calibration first.
## First let's gather some data.
## Structure : In which test, for each camera, for each node, what location in cam space did we see the node at.
## cam_calibration_data = {
##    cam_index : {
##        node_name : {
##            cam_name : location_in_cam_space
##        }
##    }
## }
cam_calibration_data = []
for i in range(5):
    cam_calibration_data.append({})
    for cam in cameras:
        cam.readFrame()
    for _,node in manager.nodes.items():
        cam_calibration_data[i][node.name] = {}
        for cam in cameras:
            col = RGBtoBGR(node.color)
            f, l, c = cam.getNodeInCamSpace(col)
            if not f:
                continue
            cam_calibration_data[i][node.name][cam.name] = l   
    time.sleep(2)
    
    
for dataFrame in cam_calibration_data:
    for nodeName,nodeData in dataFrame.items():
        print(f"Node {nodeName} : ")
        for camName1,loc1 in nodeData.items():
            print(f"    Cam {camName1} saw at {loc1}")
            for camName2,loc2 in nodeData.items():
                if camName1 is camName2:
                    continue
                print(f"        With Cam {camName2} at {loc2}")
                


## For now, let's ignore calMethod and implement either one that heart desires.
## INCOMING UNTESTED CODE BELOW!!
# while True:
            
#     results = []
#     for cam1 in cameras:
#         for cam2 in cameras:
#             if cam1 is cam2:
#                 continue
#             f, loc, cont = cam1.getNodeInCamSpace()
            
#             if not f:
#                 continue
    
#             pt1_hom = np.array([[loc[0]], [loc[1]]], dtype=np.float32)
            
#             f, loc, cont = GetTrackerCordFromImge(cam2.lastFrame)
            
#             if not f:
#                 continue
            
#             pt2_hom = np.array([[loc[0]], [loc[1]]], dtype=np.float32)

#             # Perform triangulation
#             point_4d_hom = cv2.triangulatePoints(cam1.projectionMatrix, cam2.projectionMatrix, pt1_hom, pt2_hom)
            
#             # Convert from homogeneous coordinates to (X, Y, Z)
#             point_3d = point_4d_hom[:3] / point_4d_hom[3]

#             results.append(point_3d.flatten())
#     #print(results)
#     results = np.array(results)
#     results = np.average(results,axis=0)
#     print(results)


#while True:
#    pass
#    for node in manager.nodes:
#        pass
        