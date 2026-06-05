#############################################################################
# THIS SCRIPT IS FOR TESTING THE TELEOP RECEIVING                           #
#############################################################################

import threading, time, sys
import socket
import signal
import sys
from collections import deque
import pygame
DEADZONE = 0.1

pygame.init()
pygame.joystick.init()

def clip_cmd(v): #checks deadzone and bounds
    if abs(v) > DEADZONE:
        return v if abs(v) < 1 else v/abs(v)*1
    return 0

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    joystick = None

def start_receiving():
    while True:
        pygame.event.pump()
        if joystick:
            axes = [clip_cmd(joystick.get_axis(i)) for i in range(joystick.get_numaxes())]
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            hat = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
            print(axes, buttons, hat)
        time.sleep(0.01)

'''
AXES:
0: Left Joystick Left/Right (-1 left)
1: Left Joystick Up/Down (-1 up)
2: Right Joystick Left/Right (-1 left)
3: Right Joystick Up/Down (-1 up)

BUTTONS:
0: A
1: B
2: X
3: Y
4: LB
5: RB
'''



def _signal_handler(sig,frame):
    sys.exit(0)

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


start_receiving()