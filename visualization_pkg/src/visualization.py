#!/usr/bin/env python3

import rospy
import math
import pygame
import sys
import struct
from datetime import datetime
from pygame.locals import *
from visualization_pkg.msg import display_info


pygame.init()

Red = (255, 0, 0)
Blue = (0, 0, 255)
Black = (0, 0, 0)
Gray = (140, 140, 140)
Orange = (255, 165, 0)
Green = (0, 255, 0)
White = (220, 220, 220)

#now = datetime.now()
#datetime = now.strftime("%Y. %m. %d %H:%M:%S:%MS")

time_font = pygame.font.Font(pygame.font.get_default_font(), 20)
data_font = pygame.font.Font(pygame.font.get_default_font(), 15)

SCREEN = pygame.display.set_mode((1000,1000))
SCREEN.fill(Gray)

pygame.display.set_caption("Object detection visualization")

def visu_callback(data):
    
	    
    if len(data.pontok_x) > 0:

        SCREEN.fill(Gray)
        drawGrid()
        drawTurtlebot()
        drawText()

        for i in range(len(data.pontok_x)):

            if data.pontok_x[i] != 0:  

                x = data.pontok_x[i] / 2 * -1000
                y = data.pontok_y[i] / 2 * -1000

                if (data.kanyarodas == False) & (data.angular_vel == 0):

                    if ((500+y) > 450) & ((500+y) < 550) & ((600+x) < 600):

                        if data.acc_on_off == True:

                            pygame.draw.circle(SCREEN,Green,(500+y,600+x),4)

                        else:

                            pygame.draw.circle(SCREEN,Red,(500+y,600+x),4)

                    else:

                        pygame.draw.circle(SCREEN,Red,(500+y,600+x),4)

                else:

                    fordulokor_x = 0
                    if data.jobbra_kanyarodas == True:

                        fordulokor_y = data.fordulokor_sugar 
                        
                    else:

                        fordulokor_y = data.fordulokor_sugar

                    distance = math.sqrt(math.pow((data.pontok_x[i] - fordulokor_x), 2) + math.pow((data.pontok_y[i] - fordulokor_y), 2))

                    if (distance < (data.fordulokor_sugar + 0.1)) & (distance > (data.fordulokor_sugar - 0.1)):

                        if data.acc_on_off == True:

                            if data.jobbra_kanyarodas == True:

                                pygame.draw.circle(SCREEN,Green,(500+y,600+x),4)
                            
                            else:

                                pygame.draw.circle(SCREEN,Green,(500-y,600+x),4)

                        else:

                            if data.jobbra_kanyarodas == True:

                                pygame.draw.circle(SCREEN,Red,(500+y,600+x),4)
                            
                            else:

                                pygame.draw.circle(SCREEN,Red,(500-y,600+x),4)

                    else:

                        pygame.draw.circle(SCREEN,Red,(500+y,600+x),4)

                if i > 0:

                    z = data.pontok_x[i-1] / 2 * -1000
                    v = data.pontok_y[i-1] / 2 * -1000

                    if((z-x < 20) & (z-x > -20)) & ((v-y < 20) & (v-y > -20) & data.acc_on_off == True):

                        pygame.draw.line(SCREEN, Orange, (500+y,600+x), (500+v,600+z), 1)

        if data.acc_on_off == True:

            for i in range(len(data.objektum_kozeppont_x)):

                x = data.objektum_kozeppont_x[i] / 2 * -1000
                y = data.objektum_kozeppont_y[i] / 2 * -1000
                #pygame.draw.circle(SCREEN,Orange,(500+y,600+x),12)

        if (data.kanyarodas == False) & (data.angular_vel == 0):

            pygame.draw.line(SCREEN, Blue, (450, 600), (450, 0), 2)
            pygame.draw.line(SCREEN, Blue, (550, 600), (550, 0), 2)

        else:

            if(data.jobbra_kanyarodas == False):

                pygame.draw.circle(SCREEN, Blue, (500 + (data.fordulokor_sugar*1000/2), 600), data.fordulokor_sugar*1000/2+50, 2)
                pygame.draw.circle(SCREEN, Blue, (500 + (data.fordulokor_sugar*1000/2), 600), data.fordulokor_sugar*1000/2-50, 2)

            else:

                pygame.draw.circle(SCREEN, Blue, (500 - (data.fordulokor_sugar*1000/2), 600), data.fordulokor_sugar*1000/2+50, 2)
                pygame.draw.circle(SCREEN, Blue, (500 - (data.fordulokor_sugar*1000/2), 600), data.fordulokor_sugar*1000/2-50, 2)

        text_surface = data_font.render("Linear vel.:          ", False, Black, Gray)
        SCREEN.blit(text_surface, dest=(1,51))
        text_surface = data_font.render(str(round(data.linear_vel, 2)), False, Black, Gray)
        SCREEN.blit(text_surface, dest=(120,51))

        text_surface = data_font.render("Angular vel.:          ", False, Black, Gray)
        SCREEN.blit(text_surface, dest=(1,71))
        text_surface = data_font.render(str(round(data.angular_vel, 2)), False, Black, Gray)
        SCREEN.blit(text_surface, dest=(120,71))
        

        text_surface = data_font.render("ACC:          ", False, Black, Gray)
        SCREEN.blit(text_surface, dest=(1,91))
        if data.acc_on_off == True:
            text_surface = data_font.render("ON", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,91))
        else:
            text_surface = data_font.render("OFF", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,91))

        text_surface = data_font.render("Controll:          ", False, Black, Gray)
        SCREEN.blit(text_surface, dest=(1,111))
        if data.lock_on == True:
            text_surface = data_font.render("ON", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,111))
        else:
            text_surface = data_font.render("OFF", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,111))

        if (data.kanyarodas == False) & (data.angular_vel == 0):
            text_surface = data_font.render("Going forward", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(1,141))
            text_surface = data_font.render("Velocity: ", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(1,161))
            text_surface = data_font.render(str(round(data.veh_vel_measured, 2)), False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,161))
            if (data.lock_on == True) & (data.object_detected == False):
                text_surface = data_font.render("Object search", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,701))
            if (data.lock_on == True) & (data.object_detected == True):
                text_surface = data_font.render("Object detected", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,801))
                text_surface = data_font.render("Object rel. vel.: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,821))
                text_surface = data_font.render(str(round(data.avg_rel_vel_of_closest_interesting_object, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,821))
                text_surface = data_font.render("Distance from object: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,841))
                text_surface = data_font.render(str(round(data.closest_interesting_object_x_range, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,841))
                text_surface = data_font.render("Linear controll: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,891))
                text_surface = data_font.render(str(round(data.veh_vel_control_lin, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,891))
                text_surface = data_font.render("Angular controll: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,911))
                text_surface = data_font.render(str(round(data.veh_vel_control_ang, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,911))


        else:
            if data.jobbra_kanyarodas == True:
                text_surface = data_font.render("Turning right", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,141))
            else:
                text_surface = data_font.render("Turning left", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,141))
            text_surface = data_font.render("Velocity: ", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(1,161))
            text_surface = data_font.render(str(round(data.veh_vel_measured, 2)), False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,161))
            text_surface = data_font.render("Turning radius: ", False, Black, Gray)
            SCREEN.blit(text_surface, dest=(1,181))
            text_surface = data_font.render(str(round(math.sqrt(math.pow(data.fordulokor_sugar, 2)), 2)), False, Black, Gray)
            SCREEN.blit(text_surface, dest=(120,181))
            if (data.lock_on == True) & (data.object_detected == False):
                text_surface = data_font.render("Object search", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,801))
            if (data.lock_on == True) & (data.object_detected == True):
                text_surface = data_font.render("Object detected", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,801))
                text_surface = data_font.render("Object turn. rad.: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,821))
                text_surface = data_font.render(str(round(data.object_turning_radius, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,821))
                text_surface = data_font.render("Distance from object: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,841))
                text_surface = data_font.render(str(round(data.closest_object_range, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,841))
                text_surface = data_font.render("Linear controll: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,891))
                text_surface = data_font.render(str(round(data.veh_vel_control_lin, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,891))
                text_surface = data_font.render("Angular controll: ", False, Black, Gray)
                SCREEN.blit(text_surface, dest=(1,911))
                text_surface = data_font.render(str(round(data.veh_vel_control_ang, 2)), False, Black, Gray)
                SCREEN.blit(text_surface, dest=(170,911))


        pygame.display.update()


def drawGrid():

    for x in range(20):

        for y in range(20):

                pygame.draw.rect(SCREEN, Black, pygame.Rect(x*50, y*50, 50, 50), 1)
    
    pygame.draw.rect(SCREEN, Gray, pygame.Rect(0, 0, 250, 50))

    pygame.draw.rect(SCREEN, Gray, pygame.Rect(0, 50, 250, 150))
    
    pygame.draw.rect(SCREEN, Gray, pygame.Rect(0, 800, 200, 150))



def drawTurtlebot():

    pygame.draw.rect(SCREEN, (50,50,50), pygame.Rect(465, 565, 71, 71), 0 , 15)
    pygame.draw.rect(SCREEN, (0,0,0), pygame.Rect(465, 565, 71, 71), 2 , 15)
    pygame.draw.rect(SCREEN, Black, pygame.Rect(456, 565, 9, 33), 0)
    pygame.draw.rect(SCREEN, Black, pygame.Rect(535, 565, 9, 33), 0)
    pygame.draw.circle(SCREEN,(0, 0, 0),(500,600), 17)

def drawText():

        now = datetime.now()
        current_time = now.strftime("%Y. %m. %d. %H:%M:%S")
        text_surface = time_font.render(current_time, False, Black, Gray)
        SCREEN.blit(text_surface, dest=(1,1))





def drawer():

    rospy.init_node('pygame_visualization', anonymous=True)
    rospy.Subscriber('/pygame_visu', display_info, visu_callback)
    rospy.spin()


if __name__ == '__main__':

    try:

        drawer()

    except rospy.ROSInterruptException:

        pass
