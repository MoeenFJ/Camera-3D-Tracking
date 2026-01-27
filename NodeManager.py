import socket
from threading import Thread, Lock
import struct
import time
import numpy as np


class Node:

    PACKET_SIZE = 32

    def __init__(self, mac, name, addr, color):
        self.mac = None
        self.addr = ""
        self.name = ""
        self.color = (0, 0, 0)
        self.sock = None
        self.acc = ()
        self.rot = ()
        self.connected = False
        self.pos = np.zeros((4))
        self.lock = Lock()

        self.mac = mac
        self.id = mac #hash later
        self.name = name
        self.addr = addr
        self.color = color

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect()
        
    def set_pos(self,pos):
        with self.lock:
            self.pos = pos
    def get_pos(self):
        with self.lock:
            return self.pos

    def connect(self):

        self.sock.connect((self.addr[0], 50405))
        self.sendToManager(bytearray([0x03]))
        self.light_on()
        self.connected = True

    def set_color(self, color):
        self.color = color
        self.sendToManager(bytearray([0x00, color[0], color[1], color[2]]))

    def light_off(self):
        self.sendToManager(bytearray([0x00, 0x00, 0x00, 0x00]))

    def light_on(self):
        self.sendToManager(
            bytearray([0x00, self.color[0], self.color[1], self.color[2]]))

    def sendToManager(self, data):
        rem = Node.PACKET_SIZE - (len(data) % Node.PACKET_SIZE)
        data += bytearray([0x00]*rem)
        self.sock.send(data)

    def gyro_on(self):
        self.sendToManager(bytearray([0x01]))
        
    def gyro_off(self):
        self.sendToManager(bytearray([0x02]))


class NodeManager:

    def __init__(self, port=50404):
        self.sock = None
        self.port = None
        self.lock = Lock()
        self.managerThread = None
        self.accepting_new_node = True
        self.nodes = []
        self.colors = [(255, 255, 255), (255, 255, 0), (0, 255, 255),
                       (255, 0, 255), (0, 255, 0), (0, 0, 255), (255, 0, 0)]

        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.port))
        self.managerThread = Thread(target=self.manage_nodes)
        self.managerThread.daemon = False
        self.managerThread.start()
        
    def stop_accepting(self):
        self.accepting_new_node = False

    def manage_nodes(self):
        while True:
            data, addr = self.sock.recvfrom(1024) ## Blocking, which is good
            message = data.decode("ascii", "ignore")
            esp_ip = addr[0]
            espMAC = bytearray(data[0:6]).hex()
            print(f"Received from {esp_ip} : {message}")
            if "TRACKNODE_REG" in message:
                res = [node for node in self.nodes if node.mac == espMAC]
                if self.accepting_new_node and len(res) == 0:

                    color = self.colors.pop()
                    print(f"TRACKNODE_REG from {esp_ip}")
                    node = Node(
                        espMAC, f"Node{len(self.nodes)+1}", addr, color)

            
                    self.nodes.append(node)
                else:
                    res[0].connect()
            if "TRACKNODE_DBG" in message:
                print(f"DEBUG from {esp_ip}:")
                print(data)
                print(message)
            if "TRACKNODE_UPD" in message:
                res = [node for node in self.nodes if node.mac == espMAC]
                if len(res) > 0:
                    node = res[0]
                    data = data[19:]
                    ax = struct.unpack('<f', data[0:4])[0]
                    ay = struct.unpack('<f', data[4:8])[0]
                    az = struct.unpack('<f', data[8:12])[0]
                    rx = round(struct.unpack('<f', data[12:16])[0], 3)
                    ry = round(struct.unpack('<f', data[16:20])[0], 3)
                    rz = round(struct.unpack('<f', data[20:24])[0], 3)
                    with node.lock:
                        node.acc = (ax, ay, az)
                        node.rot = (rx, ry, rz)
                    # print('{0: <5}'.format(rx),'{0: <5}'.format(ry),'{0: <5}'.format(rz))
