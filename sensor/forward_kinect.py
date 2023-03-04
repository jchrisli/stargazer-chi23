'''
Check Kinect to camera calibration precision
'''
from kinect_udp_listener import KinectUdpListener

from kinect_udp_forward import UdpForward


UDP_PORT = 5102

kinect_forward = UdpForward('your.ip.here', 12345)
#listen_to_hand_forward = UdpForward('127.0.0.1', 12347)


def forward_kinect_data(data: bytes) -> None:
    '''
    Forward Kinect data stream to the robot machine. 
    '''
    kinect_forward.forward(data)
    #listen_to_hand_forward.forward(data)



listener = KinectUdpListener(UDP_PORT)

listening = True
while listening:
    try:
        #listener.listen(parse_kinect_data)
        listener.listen(forward_kinect_data)
    except KeyboardInterrupt:
        listening = False
    

print('Stop listening...')

