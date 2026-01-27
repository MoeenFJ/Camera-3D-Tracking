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
node_manager = NodeManager()


# This will later be replaced with a button click saying "Done adding".
while len(node_manager.nodes) < 1:
    pass

node_manager.stop_accepting()

cameras = []
i = 0
CAMDIR = "/dev/v4l/by-path/pci-0000:00:14.0-usbv2-*"
# CAMDIR = "/dev/video*"

for camPath in glob.glob(CAMDIR):
    print(camPath)
    cameras.append(PS3EyeCamera(str(i), camPath, node_manager))
    cameras[-1].trackSettings()
    cameras[-1].inMatrix = mtx
    cameras[-1].distortCoeff = dist
    i += 1

#assert len(cameras) >= 2


vis_queue = queue.Queue()
vis = Visualizer(vis_queue)


cams_to_send = []
nodes_to_send = []
for cam in cameras:
    cams_to_send.append([cam.t.reshape((-1))])
for node in node_manager.nodes:
    nodes_to_send.append([node.pos[:3].reshape((-1)),node.color])
vis_queue.put((cams_to_send,nodes_to_send))

def RGBtoBGR(color):
    return (color[2], color[1], color[0])


# TODO : This should   be changed such that:
# 1.It considers the average color in contour instead of the center
# 2.It considers the average color in a series of frames
# 3.It calculates a 3D margin of error plus 5% more error of error.


def colorCalibration():
    calDataPath = Path("./CapturedCalData/")
    calDataPath.mkdir(parents=True, exist_ok=True)

    for node in node_manager.nodes:
        nodeCalDataPath = Path(f"./CapturedCalData/{node.color}/")
        nodeCalDataPath.mkdir(parents=True, exist_ok=True)
        node.light_on()
        print(node.name)
        for otherNode in node_manager.nodes:
            if node is otherNode:
                continue
            otherNode.light_off()
            print(f"Turn of {otherNode.color}, {otherNode.addr}")
        time.sleep(1)
        for cam in cameras:
            cam.readFrame()
        for cam in cameras:
            obsCol, cnt = cam.calibrateColor(node)
            print(f"Cam {cam.name} observed {node.color} as {obsCol}.")
            timg = cam.frame.copy()
            cv2.drawContours(timg, [cnt], -1, (0, 255, 0), 3)
            cv2.imwrite(f"./CapturedCalData/{node.color}/{cam.name}.png", timg)
    print("Done color cap")
    for node in node_manager.nodes:
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
    for node in node_manager.nodes:
        points = {}
        all_see = False
        while not all_see:
            for cam in cameras:
                cam.readFrame()
            all_see = True
            for cam in cameras:
                col = RGBtoBGR(node.color)
                f, l, c, debugImg = cam.getNodeInCamSpace(node)
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
print(R)
print(t)
print("************************************88")


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



def bundle_adjustment_fun(params, cameras, nodes):
    # Unpack params
    n_cameras = len(cameras)-1
    n_points = len(nodes)
    print(n_cameras)
    camera_params = params[:n_cameras * 12].reshape((n_cameras, 3,4))
    points_3d = params[n_cameras * 12:].reshape((n_points, 3))
    points_3d = np.hstack((points_3d,np.ones((n_points,1))))
    #print(f"camera_params : {camera_params}")
    #print(f"points_3d : {points_3d}")
    errors = []
    projmat = cameras[0].projectionMatrix
    for i in range(n_points):
        if nodes[i].id not in cameras[0].nodeLocations:
            continue
        reprojected_point = (projmat @ points_3d[i].T)
        reprojected_point = reprojected_point / reprojected_point[2]
        reproj = reprojected_point[:2] - cameras[0].nodeLocations[nodes[i].id]
        
        #print(f"reproj {reproj} : {(projmat @ points_3d[i].T)[:2]} and {cam.nodeLocations[nodes[i].id]}")
        errors.append(reproj.ravel())
    
    for i in range(n_cameras):
        projmat = (cameras[i+1].inMatrix @ camera_params[i])
    
        #print(f"projmats : {projmats}") 
        
   
        for j in range(n_points):
            if nodes[j].id not in cameras[i+1].nodeLocations:
                continue
            reprojected_point = (projmat @ points_3d[j].T)
            reprojected_point = reprojected_point / reprojected_point[2]
            reproj = reprojected_point[:2] - cameras[i+1].nodeLocations[nodes[j].id]
            
            #print(f"reproj {reproj} : {(projmat @ points_3d[i].T)[:2]} and {cam.nodeLocations[nodes[i].id]}")
            errors.append(reproj.ravel())
   
    # Return the difference (residuals)
    return np.array(errors).ravel()
bundle_adjustment = False
frameCnt = 0
bundle_adjustment_data = []
while True:
    for cam in cameras:
        cam.readFrame()
        
    for cam in cameras:
        cam.processFrame()

    
    for node in node_manager.nodes:
        estimatedPoses = []
        for cam in cameras:
            for otherCam in cameras:
                if cam is otherCam:
                    continue
                if node.id not in cam.nodeLocations or node.id not in otherCam.nodeLocations:
                    continue
                loc =cam.nodeLocations[node.id]
                otherLoc = cam.nodeLocations[node.id]

                p1 = np.array([loc], dtype=np.float32).reshape((-1, 1, 2))
                p2 = np.array(
                    [otherLoc], dtype=np.float32).reshape((-1, 1, 2))

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
                estimatedPoses.append(point3D.reshape(4)[:3])
        if len(estimatedPoses )> 0:
            estimatedPoses = np.array(estimatedPoses)
            finalPos = np.mean(estimatedPoses, axis=0)
            node.set_pos(finalPos)
            #print(f"Node {node.name} estimated at {finalPos}")

    if frameCnt%120==0:
        bundle_adjustment = True
    if bundle_adjustment :
        x0 =  np.array([])
        for i in range(1,len(cameras)):
            x0 = np.hstack((x0,np.hstack((cameras[i].R, cameras[i].t)).ravel()))
            
        for node in node_manager.nodes:
            x0 = np.hstack((x0,node.get_pos().ravel()))
        print(f"x0 : {x0}")
        
    
        res = least_squares(bundle_adjustment_fun, x0, verbose=2, x_scale='jac', ftol=1e-4, method='trf',
                            args=(cameras, node_manager.nodes))
        #print(f"res : {res}")
        params = res.x
        n_cameras = len(cameras)-1
        n_points = len(node_manager.nodes)
        camera_params = params[:n_cameras * 12].reshape((n_cameras, 3,4))
        points_3d = params[n_cameras * 12:].reshape((n_points, 3))
        
        for i in range(n_cameras):
            cameras[i+1].set_R(camera_params[i,:,:3])
            cameras[i+1].set_t(np.array([camera_params[i,:,3]]).T)
    
        for i in range(n_points):
            node_manager.nodes[i].set_pos(points_3d[i])
        bundle_adjustment = False

    if DEBUG_MODE:    
        for cam in cameras:
            img = cam.savedFrame
            for node in node_manager.nodes:
                col = RGBtoBGR(node.color)
                f, l, c,_ = cam.getNodeInCamSpace(node)
                timg = img.copy()
                reprojected_point =  cam.projectionMatrix @ np.append(node.get_pos(),np.array([1]))
                reprojected_point = reprojected_point / reprojected_point[2]
                print(reprojected_point) 
                print(l)
                print("=-=-=-=-=-=-=-")
                if(f):
                    cv2.drawContours(timg, [c], 0, (0, 255, 0), 3)
                    cv2.circle(timg,(int(l[0]),int(l[1])), 2, (0,255,0), -1)
                    cv2.circle(timg,(int(reprojected_point[0]),int(reprojected_point[1])),2,(0,0,255),-1)
                cv2.namedWindow(f"Cam {cam.name} found color {node.color}", cv2.WINDOW_NORMAL)
                cv2.resizeWindow(f"Cam {cam.name} found color {node.color}", 640, 480)
                cv2.imshow(f"Cam {cam.name} found color {node.color}", timg)
                cv2.waitKey(1)
    
    
    ##Update visualizer
    cams_to_send = []
    nodes_to_send = []
    for cam in cameras:
        cams_to_send.append([cam.t.reshape((-1))])
    for node in node_manager.nodes:
        nodes_to_send.append([node.pos[:3].reshape((-1)),node.color])
    vis_queue.put((cams_to_send,nodes_to_send))

    frameCnt += 1

