'''
Listen to broadcasted messages from CreepyTracker

Jiannan Li, 2022

'''
import socket
from typing import Callable

class KinectUdpListener(object):

    def __init__(self, port: int) -> None:
        self._port = port
        self._receive_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._receive_socket.settimeout(1)
        self._receive_socket.bind(('0.0.0.0', int(self._port)))

    def listen(self, handler: Callable[[bytes], None]):
        try:
            data = self._receive_socket.recv(20000)
            handler(data)
        except socket.timeout:
            print(f'Timeout when listening to UDP data at port {self._port}, is there data?')