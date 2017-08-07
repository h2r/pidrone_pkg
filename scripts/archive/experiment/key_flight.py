#!/usr/bin/env python
import rospy
from pidrone_pkg.msg import RC
import pygame

rospy.init_node('key_flight', anonymous=True)

pub = rospy.Publisher('/pidrone/commands', RC, queue_size=1)
msg = RC()
msg.roll = 1500
msg.pitch = 1500
msg.yaw = 1500
msg.throttle = 1000
msg.aux1 = 1500
msg.aux2 = 1500
msg.aux3 = 1500
msg.aux4 = 1500
stop = False
pygame.init()
size = width, height = 320, 240
screen = pygame.display.set_mode(size)
while not rospy.is_shutdown() and not stop:
    print([msg.roll, msg.pitch, msg.yaw, msg.throttle, msg.aux4])
    pub.publish(msg)
    msg.aux4 = 1500
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            print(event.key)
            if event.key == pygame.K_w:
                msg.roll = 1600
            elif event.key == pygame.K_s:
                msg.roll = 1400
            elif event.key == pygame.K_a:
                msg.pitch = 1400
            elif event.key == pygame.K_d:
                msg.pitch = 1600
            elif event.key == pygame.K_q:
                msg.yaw = 1400
            elif event.key == pygame.K_e:
                msg.yaw = 1600
            elif event.key == pygame.K_u:
                if msg.throttle <= 1900:
                    msg.throttle += 100
            elif event.key == pygame.K_j:
                if msg.throttle >= 1100:
                    msg.throttle -= 100
            elif event.key == pygame.K_h:
                msg.aux4 = 1600
            elif event.key == pygame.K_n:
                msg.aux4 = 1400
