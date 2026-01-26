from NodeManager import NodeManager
from Camera import PS3EyeCamera
from Visualizer import Visualizer
import random
import cv2
import os
from pathlib import Path
import glob
import numpy as np
import time
import socket
import threading
import struct
import queue
from scipy.optimize import least_squares

mtx = np.array([[554.2563,   0,          320],
                [0,        554.2563,      240],
                [0,          0,           1,]])
dist = np.array([[-0.10771770030260086, 0.1213262677192688,
                0.00091733073350042105, 0.00010589254816295579]], dtype="double")

DEBUG_MODE = True
# Program Flow:
# 1. Add all trackers and calibration boards and Cameras
# 2. Choose Calibration method
# 3. Calibrate Colors if needed
# 4. Calibrate extrinsic matrices (global pos and rot of each cam)
# 5. Start Tracking :D
# First order of bussiness, color calibration

# Step 1
manager = NodeManager()

cameras = []
i = 0
CAMDIR = "/dev/v4l/by-path/pci-0000:00:14.0-usbv2-*"
# CAMDIR = "/dev/video*"

for camPath in glob.glob(CAMDIR):
    print(camPath)
    cameras.append(PS3EyeCamera(str(i), camPath))
    cameras[-1].trackSettings()
    cameras[-1].inMatrix = mtx
    cameras[-1].distortCoeff = dist
    i += 1
    
    
#assert len(cameras) >= 2

def update_visualizer_thread():
    global cameras,manager
    nodes = manager.nodes
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    sock.bind(("127.0.0.1",50406))
    print("Ready")
    sock.listen(5)
    connection, _ = sock.accept()
    print("accepted")
    while(True):
        try:
            for i in range(len(cameras)):
                cam = cameras[i]
                t = cam.t.reshape((-1))
                
                data = struct.pack("Bfff",i,float(t[0]),float(t[1]),float(t[2]))
                print(data)
                connection.send(data)
            for i, key in enumerate(nodes):
                node = nodes[key]
                pos = node.pos.reshape((-1))
                data = struct.pack("BfffBBB",128+i,float(pos[0]),float(pos[1]),float(pos[2]),node.color[0],node.color[1],node.color[2])
                print(data)
                connection.send(data)
            time.sleep(0.5)
        except:
            connection, _ = sock.accept()
            print("accepted")


vis_queue = queue.Queue()
vis = Visualizer(vis_queue)


# This will later be replaced with a button click saying "Done adding".
while len(manager.nodes) < 1:
    pass

cams_to_send = []
nodes_to_send = []
for cam in cameras:
    cams_to_send.append([cam.t.reshape((-1))])
for key,node in manager.nodes.items():
    nodes_to_send.append([node.pos[:3].reshape((-1)),node.color])
vis_queue.put((cams_to_send,nodes_to_send))

def RGBtoBGR(color):
    return (color[2], color[1], color[0])


# TODO : This should   be changed such that:
# 1.It considers the average color in contour instead of the center
# 2.It considers the average color in a series of frames
# 3.It calculates a 3D margin of error plus 5% more error of error.
time.sleep(2)


def colorCalibration():
    calDataPath = Path("./CapturedCalData/")
    calDataPath.mkdir(parents=True, exist_ok=True)
    for _, node in manager.nodes.items():
        nodeCalDataPath = Path(f"./CapturedCalData/{node.color}/")
        nodeCalDataPath.mkdir(parents=True, exist_ok=True)
        node.light_on()
        print(node.name)
        for _, otherNode in manager.nodes.items():
            if node is otherNode:
                continue
            otherNode.light_off()
            print(f"Turn of {otherNode.color}, {otherNode.addr}")
        time.sleep(1)
        for cam in cameras:
            cam.readFrame()
        for cam in cameras:
            col = RGBtoBGR(node.color)
            obsCol, cnt = cam.calibrateColor(col)
            print(f"Cam {cam.name} observed {node.color} as {obsCol}.")
            timg = cam.frame.copy()
            cv2.drawContours(timg, [cnt], -1, (0, 255, 0), 3)
            cv2.imwrite(f"./CapturedCalData/{node.color}/{cam.name}.png", timg)
    print("Done color cap")
    for _, node in manager.nodes.items():
        node.light_on()


colorCalibration()


# calMethod = int(input("Cal method? 1 : Board , 2 : none board \n"))
# assert calMethod in [1,2]


# Below is getContourByColor Test code
# while True:
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


# Let's try template-less calibration first.
# First let's gather some data.
# Structure : In which test, for each camera, for each node, what location in cam space did we see the node at.
# cam_calibration_data = {
# cam_index : {
# node_name : {
# cam_name : location_in_cam_space
# }
# }
# }
cam_calibration_data = []
for i in range(8):
    print(f"Capturing calibration frame {i}")
    time.sleep(1)
    for cam in cameras:
        cam.readFrame()
    for _, node in manager.nodes.items():
        points = {}
        all_see = False
        while not all_see:
            for cam in cameras:
                cam.readFrame()
            all_see = True
            for cam in cameras:
                col = RGBtoBGR(node.color)
                f, l, c, debugImg = cam.getNodeInCamSpace(col)
                timg = debugImg.copy()
                #cv2.drawContours(timg, [c], 0, (0, 255, 0), 3)
                #cv2.circle(timg,(l[0],l[1]), 2, (0,255,0), -1)
                cv2.namedWindow(f"Cam {cam.name} found color {node.color}", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(f"Cam {cam.name} found color {node.color}", 640, 480)
                cv2.imshow(f"Cam {cam.name} found color {node.color}", timg)
                cv2.waitKey(1)
                print(
                    f"Cam {cam.name} sees Node {node.name} at {l}, found: {f}")
                
                if not f:
                    all_see = False
                points[cam.name] = l
            if all_see:
                cam_calibration_data.append(points)


# For now, let's ignore calMethod and implement either one that heart desires.
# Gotta find essential matrix for cam 1 and 2
# triangulate points to find 3D locations
# solvePnP for the rest of the cameras
# from now on, keep the first cam frozen and keep calibrating other cams using bundle adjustment

# Let's try calibrating cam 2 first.
cam1 = cameras[0]
cam2 = cameras[1]

cam1.R = np.eye(3)
cam1.t = np.zeros((3, 1))

pts1 = []
pts2 = []
for data in cam_calibration_data:
    p1 = data[cam1.name]
    p2 = data[cam2.name]
    pts1.append(p1)
    pts2.append(p2)

pts1 = np.array(pts1, dtype=np.float32)
pts2 = np.array(pts2, dtype=np.float32)
#pts1 = cv2.undistortPoints(pts1.reshape(-1, 1, 2),
#                           cam1.inMatrix, cam1.distortCoeff, P=cam1.inMatrix)
#pts2 = cv2.undistortPoints(pts2.reshape(-1, 1, 2),
#                           cam2.inMatrix, cam2.distortCoeff, P=cam2.inMatrix)


E, mask = cv2.findEssentialMat(
    pts1, pts2, cam1.inMatrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
_, R, t, mask = cv2.recoverPose(E, pts1, pts2, cam1.inMatrix)

cam2.R = R  
cam2.t = t  


# Now let's triangulate all points to get their 3D locations
cam1.projectionMatrix = cam1.inMatrix @ np.hstack((cam1.R, cam1.t))
cam2.projectionMatrix = cam2.inMatrix @ np.hstack((cam2.R, cam2.t))
print(f"Cam 1 -> {np.hstack((cam1.R, cam1.t))}")
print(f"Cam 2 -> {np.hstack((cam2.R, cam2.t))}")

world3DPoints = cv2.triangulatePoints(
    cam1.projectionMatrix, cam2.projectionMatrix, pts1.T, pts2.T,)
print(pts1)
print("3D ", world3DPoints)
world3DPoints = world3DPoints / world3DPoints[3, :]
print(world3DPoints)
input()
# Now we can use these 3D points to calibrate the other cameras using solvePnP
for cam in cameras[2:]:
    objectPoints = []
    imagePoints = []
    for idx in range(len(cam_calibration_data)):
        data = cam_calibration_data[idx]
        p = data[cam.name]
        objectPoints.append(world3DPoints[:3, idx].T)
        imagePoints.append(p)
    objectPoints = np.array(objectPoints, dtype=np.float32)
    imagePoints = np.array(imagePoints, dtype=np.float32)
    assert len(objectPoints) >= 4, "Not enough points to calibrate camera!"
    ret, rvec, tvec = cv2.solvePnP(
        objectPoints, imagePoints, cam.inMatrix, cam.distortCoeff)
    R, _ = cv2.Rodrigues(rvec)
    cam.R = R  
    cam.t = tvec  
    print()
    cam.projectionMatrix = cam.inMatrix @ np.hstack((cam.R, cam.t))

print("Calibration done!")
for cam in cameras:
    print(f"{cam.name} : {cam.t} : {cam.R}")



def bundle_adjustment_fun(params, n_cameras, n_points, points_2d, K):
    # Unpack params
    camera_params = params[:n_cameras * 12].reshape((n_cameras, 3,4))
    points_3d = params[n_cameras * 12:].reshape((n_points, 4))
    
    print(f"camera_params : {camera_params}")
    print(f"points_3d : {points_3d}")
    
    # Project
    projmats = (K @ camera_params)
    
    print(f"projmats : {projmats}") 
    
    errors = []
    for i in range(n_points):
        reproj = (projmats @ points_3d[i])[:,:2] - points_2d[i]
        errors.append(reproj.ravel())
   
    # Return the difference (residuals)
    return np.array(errors).ravel()

while True:
    for cam in cameras:
        cam.readFrame()

    
    detects = {}
    for _, node in manager.nodes.items():
        col = RGBtoBGR(node.color)
        detects[node] = {}
        for cam in cameras:
            f, l, c,_ = cam.getNodeInCamSpace(col)
            if f:
                detects[node][cam] = l
        if len(detects) >= 2:
            estimatedPoses = []
            for cam, loc in detects.items():
                for otherCam, otherLoc in detects.items():
                    if cam is otherCam:
                        continue

                    p1 = np.array([loc], dtype=np.float32).reshape(-1, 1, 2)
                    p2 = np.array(
                        [otherLoc], dtype=np.float32).reshape(-1, 1, 2)

                    # Problem : Strange behaviour according to StackOverflow, the undistortPoints does something to the points regarding
                    # the intrinsic matrix that the triangulatePoints function doesnt expect it to do.
                    # It apperantly nomalizes the intrinsic matrix while undistorting, so the resulting points assume an
                    # Identity intrinsic matrix.
                    # Solution 1 : For the time being the original un-undistorted points are give to triangulatePoints
                    # Solution 2 : User suggests doing the undistortion and preforming tirangulation using a projection matrix that uses an identity matrix as
                    # its intrinsic matix.
                    # Solution 3: user says by passing the projection matrix(or the intrinsicMatrix, not sure which) as the P parameter to the undistortPoints function,
                    # the undistorted points can be used as normanl

                    # p1_u = cv2.undistortPoints(p1, cam.inMatrix, cam.distortCoeff, P=cam.inMatrix)
                    # p2_u = cv2.undistortPoints(p2, otherCam.inMatrix, otherCam.distortCoeff, P=otherCam.inMatrix)
                    #print(f"Located at {loc} in cam {cam.name} and at {otherLoc} in cam {otherCam.name}")
                    point3D = cv2.triangulatePoints(
                        cam.projectionMatrix, otherCam.projectionMatrix, p1.reshape(2, 1), p2.reshape(2, 1))
                    point3D = point3D / point3D[3]
                    #print("Point in da third dimansion : ", point3D)
                    #print(f"{cam.name} Project : {cam.projectionMatrix}")
                    estimatedPoses.append(point3D.reshape(4))
            estimatedPoses = np.array(estimatedPoses)
            finalPos = np.mean(estimatedPoses, axis=0)
            node.pos = finalPos
            #print(f"Node {node.name} estimated at {finalPos}")

    observed_2d = []
    x0 =  np.array([])
    for cam in cameras:
        x0 = np.hstack((x0,np.hstack((cam.R, cam.t)).ravel()))
        
    for _, node in manager.nodes.items():
        x0 = np.hstack((x0,node.pos.ravel()))
    print(x0)
    
    for _, node in manager.nodes.items():
        
        temp_arr = np.array([])
        for cam in cameras:
             temp_arr = np.append(temp_arr,(np.array(detects[node][cam])))
        observed_2d.append(temp_arr) 
    print(observed_2d)
    res = least_squares(bundle_adjustment_fun, x0, verbose=2, x_scale='jac', ftol=1e-4, method='trf',
                        args=(len(cameras), len(manager.nodes.items()), np.array(observed_2d), mtx))
    print(res)
            
    if DEBUG_MODE:    
        for cam in cameras:
            img = cam.savedFrame
            for _, node in manager.nodes.items():
                col = RGBtoBGR(node.color)
                f, l, c,_ = cam.getNodeInCamSpace(col)
                timg = img.copy()
                reprojected_point =  cam.projectionMatrix @ node.pos 
                projection_error = np.array(l) - reprojected_point[:2]
                least_squares()
                if(f):
                    cv2.drawContours(timg, [c], 0, (0, 255, 0), 3)
                    cv2.circle(timg,(int(l[0]),int(l[1])), 2, (0,255,0), -1)
                    cv2.circle(timg,(int(reprojected_point[0]),int(reprojected_point[1])),2,(0,0,255),-1)
                cv2.namedWindow(f"Cam {cam.name} found color {node.color}", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(f"Cam {cam.name} found color {node.color}", 640, 480)
                cv2.imshow(f"Cam {cam.name} found color {node.color}", timg)
                cv2.waitKey(1)
    # break
    cams_to_send = []
    nodes_to_send = []
    for cam in cameras:
        cams_to_send.append([cam.t.reshape((-1))])
    for key,node in manager.nodes.items():
        nodes_to_send.append([node.pos[:3].reshape((-1)),node.color])
    vis_queue.put((cams_to_send,nodes_to_send))

