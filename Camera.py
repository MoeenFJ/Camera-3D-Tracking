import cv2
import os
import threading
import numpy as np

class Camera:
    
    ## TODO : Calculate this based on resolution
    MIN_CONTOUR_AREA = 1000
    
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
        self.savedFrame = None
        self.colorMap = {}
        
        self.name = name
        self.cap = cv2.VideoCapture(path,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.path = path
        
        self.thread = threading.Thread(target=self._update, args=())
        self.thread.daemon = True # Thread dies when main program exits
        self.thread.start()
        
    def _update(self):
        # Constantly grab frames to clear the buffer
        while True:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
                
    def readFrame(self):
        self.savedFrame = self.frame.copy()

    
    def getBrightestContour(self):
        found = False
        cx = 0
        cy = 0
        cont = None
        
        #cv2.imwrite("CapturedCalData/debug_orig.png",self.savedFrame)

        
        blur = cv2.medianBlur(self.savedFrame, 5)
        blur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        brightness = blur[:,:,2]
        #cv2.imwrite("CapturedCalData/debug_gray.png",brightness)


        ret, threshed = cv2.threshold(brightness, 64, 255, cv2.THRESH_BINARY)

        #cv2.imwrite("CapturedCalData/debug_thresh.png",threshed)

        contours, hierarchy = cv2.findContours(
            threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)


        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_CONTOUR_AREA]
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
        
  
        ## TODO : USE HSV HERE INSTEAD OF RGB!!
        blured_image = cv2.medianBlur(self.savedFrame, 5)
        
        color  = np.array(color, dtype = np.float32)
        target_unit = color / np.linalg.norm(color)
        
        blured_image = blured_image.astype(np.float32)

        
        dot_product = np.sum(blured_image * target_unit, axis=2)


        gray_result = cv2.normalize(dot_product, None, 0, 255, cv2.NORM_MINMAX)
        gray_result = gray_result.astype(np.uint8)

        ret, threshed = cv2.threshold(gray_result, 150, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            threshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        def _contourValue(cnt):
            return self.getAverageInContour(gray_result,cnt)[0]
        
        ## TODO : Currently the contourArea fucntion used can give wrong results specially if the contour is a line or self intesecting.
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_CONTOUR_AREA]
        
        ## TODO : CUTOFF HERE INSTEAD OF SORTING
        contours = sorted(contours, key=_contourValue)

        if len(contours) > 0:
            cont = contours[-1]
            
            area = cv2.contourArea(cont)
            ##print(f"{self.name} saw {color} with area {area}") 
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
        observedColor = self.savedFrame[c[1],c[0]]
        if f:
            self.colorMap[color] = observedColor


            return observedColor,cnt
        else:
            return None
        
    def getNodeInCamSpace(self,color):
        _color = self.colorMap[color]
        return self.getContourByColor(_color)

    def saveframe(self,loc):
        cv2.imwrite(loc,self.savedFrame)

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