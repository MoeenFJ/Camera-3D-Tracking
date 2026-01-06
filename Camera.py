import cv2
import os
import numpy as np
import threading
from NodeManager import Node

class Camera:
    
    
    def __init__(self,name,path):
        self.name = ""
        self.path = ""
        self.cap = None
        self.extMat = np.zeros((3,4))
        self.T = None
        self.R = None
        self.tvec = None
        self.rvec = None
        self.calibrated = False
        self.projectionMatrix = None
        self.inMatrix = None
        self.distortCoeff = None
        self.frame = None
        self.frameReaderThread = None
        self.colorMap = {}
        
        self.name = name
        self.cap = cv2.VideoCapture(path,cv2.CAP_V4L2)
        self.path = path
  
    def readFrame(self):
        ret, frame = self.cap.read()
        self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return ret
    
    def getBrightestContour(self):
        found = False
        cx = 0
        cy = 0
        cont = None
        
        imgGray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        imgGray = cv2.medianBlur(imgGray, 5)

        ret, threshed = cv2.threshold(imgGray, 64, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        contours = sorted(contours, key=cv2.contourArea)

        if len(contours) > 0:
            cont = contours[-1]
            area = cv2.contourArea(cont)

            if area > 0:
                found = True
                M = cv2.moments(cont)

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cont = None
                
        return found,np.array([cx,cy]),cont
    
    def getAverageInContour(self,img,cont):
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [cont], -1, 255, -1)

    
        mean_val = cv2.mean(img, mask=mask)

        avg_color = mean_val[:3]
        
        return avg_color
    
    def getContourByColor(self,color):
        found = False
        cx = 0
        cy = 0
        cont = None
        
  
        
        blured_image = cv2.medianBlur(self.frame, 5)
        cv2.cvtColor(blured_image, cv2.COLOR_BGR2RGB)
        
        color  = np.array(color, dtype = np.float32)
        target_unit = color / np.linalg.norm(color)
        
        blured_image = blured_image.astype(np.float32)

        
        dot_product = np.sum(blured_image * target_unit, axis=2)


        gray_result = cv2.normalize(dot_product, None, 0, 255, cv2.NORM_MINMAX)
        gray_result = gray_result.astype(np.uint8)

        ret, threshed = cv2.threshold(gray_result, 200, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        #contours = sorted(contours, key=cv2.contourArea)
        def _contourValue(cnt):
            return self.getAverageInContour(gray_result,cnt)[0]
        contours = sorted(contours, key=_contourValue)

        if len(contours) > 0:
            cont = contours[-1]
            area = cv2.contourArea(cont)

            if area > 0:
                found = True
                M = cv2.moments(cont)

                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cont = None
                
        return found,np.array([cx,cy]),cont
        
    def calibrateColor(self,color):
        f,c,cnt = self.getBrightestContour()
        assert f is not False, "A cam couldnt fine the node!"
        observedColor = self.frame[c[1],c[0]]
        if f:
            self.colorMap[color] = observedColor


            return observedColor,cnt
        else:
            return None
        
    def getNodeInCamSpace(self,color):
        _color = self.colorMap[color]
        return self.getContourByColor(_color)

    def saveframe(self,loc):
        cv2.imwrite(loc,self.frame)

class PS3EyeCamera(Camera):
    def normalSettings(self):
        os.system(f"v4l2-ctl -d {self.path} -c auto_exposure=0")
        os.system(f"v4l2-ctl -d {self.path} -c gain_automatic=0")
        os.system(f"v4l2-ctl -d {self.path} -c white_balance_automatic=0")

    def trackSettings(self):
        os.system(f"v4l2-ctl -d {self.path} -c auto_exposure=1")
        os.system(f"v4l2-ctl -d {self.path} -c gain_automatic=0")
        os.system(f"v4l2-ctl -d {self.path} -c white_balance_automatic=0")
        os.system(f"v4l2-ctl -d {self.path} -c gain=0")
        os.system(f"v4l2-ctl -d {self.path} -c exposure=30")