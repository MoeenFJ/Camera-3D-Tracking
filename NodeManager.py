import socket
import threading
import struct
import time
class Node:
    
    PACKET_SIZE = 32
    
    def __init__(self,mac,name,addr,color):
        self.mac = None
        self.addr = ""
        self.name = ""
        self.color = (0,0,0)
        self.sock = None
        self.acc = ()
        self.rot = ()
        self.connected = False
        
        self.mac = mac
        self.name = name
        self.addr = addr
        self.color = color
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.sock.connect((addr[0],50405))
        self.sendToManager(bytearray([0x03]))
        self.light_on()

        
    def set_color(self,color):
        self.color = color
        self.sendToManager(bytearray([0x00,color[0],color[1],color[2]]))
    
    def light_off(self):
        self.sendToManager(bytearray([0x00,0x00,0x00,0x00]))
        
    def light_on(self):
        self.sendToManager(bytearray([0x00,self.color[0],self.color[1],self.color[2]]))
        
    def sendToManager(self,data):
        rem = Node.PACKET_SIZE - (len(data) % Node.PACKET_SIZE)
        data += bytearray([0x00]*rem)
        self.sock.send(data)

    
class NodeManager:
    
    def __init__(self,port=50404):
        self.sock = None
        self.port = None
        self.managerThread = None
        self.nodes_to_reg = {}
        self.nodes = {}
        self.colors = [(255,255,255),(255,255,0),(0,255,255),(255,0,255),(0,255,0),(0,0,255),(255,0,0)]
        
        self.port = port
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(("",self.port))
        self.managerThread = threading.Thread(target=self.manage_nodes)
        self.managerThread.start()
    def manage_nodes(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            message = data.decode("ascii","ignore")
            esp_ip = addr[0]
            espMAC = bytearray(data[0:6]).hex()
            print(f"Received from {esp_ip} : {message}")
            if "TRACKNODE_REG" in message :
                color = self.colors.pop()    
                print(f"TRACKNODE_REG from {esp_ip}")            
                node = Node(espMAC,f"Node{len(self.nodes)+1}",addr,color)
                self.nodes[espMAC] = node
            if "TRACKNODE_DBG" in message:
                print(f"DEBUG from {esp_ip}:")
                print(data)
                print(message)
            if"TRACKNODE_UPD" in message:
                if espMAC in self.nodes.keys():   
                    node = self.nodes[espMAC]
                    data = data[19:]
                    ax = struct.unpack('<f', data[0:4])[0]
                    ay = struct.unpack('<f', data[4:8])[0]
                    az =struct.unpack('<f', data[8:12])[0]
                    rx = round(struct.unpack('<f', data[12:16])[0],3)
                    ry = round(struct.unpack('<f', data[16:20])[0],3)
                    rz = round(struct.unpack('<f', data[20:24])[0],3)
                    node.acc = (ax,ay,az)
                    node.rot = (rx,ry,rz)
                    #print('{0: <5}'.format(rx),'{0: <5}'.format(ry),'{0: <5}'.format(rz))
                
