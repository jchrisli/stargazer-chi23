import socket

class UdpForward(object):
    def __init__(self, target_ip: str, target_port: int) -> None:
        self._sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
        self._forward_ip = target_ip

        self._forward_port = target_port
        

    def forward(self, data: bytes) -> None:
        self._sock.sendto(data, (self._forward_ip, self._forward_port))

    def forward_str(self, msg_str: str) -> None:
        msg_bytes = msg_str.encode('utf-8')
        self.forward(msg_bytes)