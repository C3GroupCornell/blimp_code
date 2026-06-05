#############################################################################
# THIS SCRIPT IS FOR OPTITRACK TESTING                                        #
#############################################################################

import sys
from NatNetClient import NatNetClient #OptiTrack's software that receives the multicast data
import signal
import sys

SERVER_IP = "192.168.0.104"   # Motive PC IP (Can be found in OptiTrack software under data streaming panel)
LOCAL_IP  = "192.168.0.105"  # This machine's IP (Can be found using "ip a" in terminal or on TPLink domain 192.168.0.1)

def receiveRigidBodyFrame(id_, position, rotation):
    print(f"Received data: {id_}, {position}, {rotation}")

def natnet_runner():
    try:
        print("Starting blocking run()")
        client.run()
    except Exception as e:
        print("client.run() exception:", repr(e))

def _signal_handler(sig,frame):
    try:
        client.shutdown()
        print("NatNetClient shut down")
    except:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

client = NatNetClient(SERVER_IP,LOCAL_IP)
client.rigidBodyListener = receiveRigidBodyFrame

try:
    natnet_runner()
except:
    print("Error starting NatNetClient")
    pass
while True:
    pass