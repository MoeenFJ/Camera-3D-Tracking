import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import threading
import queue
class Visualizer:
    
    def __init__(self, queue):
        self.queue = queue
        
        
        self.nodes = []
        self.cams = []
        self.camera_speed = 1
        self.sensitivity = 1
        self.rotation_x = 0
        self.rotation_y = 0
        self.camera_position = np.array([0, 0, -5], dtype=np.float32)
  
        
        self.thread = threading.Thread(target=self._visualizer_thread, args=())
        self.thread.daemon = False 
        self.thread.start()
        

    def draw_axes(self):
        # X-axis (Red)
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(1, 0, 0)
        glEnd()

        # Y-axis (Green)
        glColor3f(0.0, 1.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1, 0)
        glEnd()

        # Z-axis (Blue)
        glColor3f(0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1)
        glEnd()
    def update_camera_position(self,keys):
        if keys[K_w]:
            self.camera_position[2] += self.camera_speed  
        if keys[K_s]:
            self.camera_position[2] -= self.camera_speed  
        if keys[K_a]:
            self.camera_position[0] += self.camera_speed  
        if keys[K_d]:
            self.camera_position[0] -= self.camera_speed  
        glPopMatrix()
        glPushMatrix()
        glTranslatef(*self.camera_position)
        
        
 
        rotation_matrix_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.radians(self.rotation_x)), -np.sin(np.radians(self.rotation_x)), 0],
        [0, np.sin(np.radians(self.rotation_x)), np.cos(np.radians(self.rotation_x)), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

        rotation_matrix_y = np.array([
            [np.cos(np.radians(self.rotation_y)), 0, np.sin(np.radians(self.rotation_y)), 0],
            [0, 1, 0, 0],
            [-np.sin(np.radians(self.rotation_y)), 0, np.cos(np.radians(self.rotation_y)), 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        rotation_matrix = np.dot(rotation_matrix_y, rotation_matrix_x)

        glMultMatrixf(rotation_matrix)

    def draw_trackers_cameras(self):
        if(not self.queue.empty()):
            self.cams, self.nodes = self.queue.get()
            
            
        for node in self.nodes:
            r,g,b = node[1]
            glColor3f(r/255, g/255, b/255)
            glPushMatrix()
            glTranslatef(*node[0])  
            glutSolidSphere(0.1, 20, 20)  
            glPopMatrix()
           
        glColor3f(0, 1, 0) 
        for cam in self.cams:
            glPushMatrix()
            glTranslatef(*cam[0]) 
            glutSolidCube(0.1) 
            glPopMatrix()


        
    def handle_mouse_motion(self,x,y):
        delta_x = x - self.last_x
        delta_y = y - self.last_y
        
        self.rotation_x += delta_y * self.sensitivity  
        self.rotation_y += delta_x * self.sensitivity  
        
        self.rotation_x = max(min(self.rotation_x, 90), -90)
        
        self.last_x, self.last_y = x, y

 
        
    def printMat(self):
        matrix = glGetFloatv(GL_MODELVIEW_MATRIX)
    

        print("Modelview Matrix:")
        matrix = np.array(matrix).reshape((4, 4)).T
        for row in matrix:
            print(row)
            
            
    def _visualizer_thread(self):
        pygame.init()
        display = (800, 600)
        self.last_x = display[0]//2
        self.last_y = display[1]//2
        pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
        glutInit()
        
        glLoadIdentity()
        gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)  
        glPushMatrix()
        glTranslatef(*self.camera_position)  
        self.printMat()
           
        clock = pygame.time.Clock()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                elif event.type == pygame.MOUSEMOTION:
                    self.handle_mouse_motion(event.pos[0], event.pos[1])  
                    
            keys = pygame.key.get_pressed()
            

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.draw_axes()
            self.update_camera_position(keys)
            self.draw_trackers_cameras()
        

            pygame.display.flip()
            clock.tick(60)  


