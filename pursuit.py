'''
@author: Pedro Belin Castellucci
This is an implementantion for visualization of the mice pursuit problem.
Each mouse starts at a different corner of a square. Each one is going to pursuit
the mouse at its right with constant velocity. It is known that the path of
each mouse is a logaritmic spiral.
'''

import sys
import pygame


class MovingObject2D:
    '''
    I think pygame object do not support non integer positions so I made this
    to make handle the positions.
    '''

    def __init__(self, initialPosition):
        self.x = initialPosition[0]  # Vertical
        self.y = initialPosition[1]  # Horizontal

    def setPosition(self, newPosition):
        self.x, self.y = newPosition

    def getPosition(self):
        return self.x, self.y

    def getIntPosition(self):
        return int(self.x), int(self.y)

    def move(self, speed):
        self.x += speed[0]
        self.y += speed[1]


def normalizeVector2D(vec):
    normVec = float(vec[0]**2 + vec[1]**2)**0.5
    if normVec <= 10e-4:  # Just preventing division by zero
        normVec = 10e-4
    return float(vec[0])/normVec, float(vec[1])/normVec

pygame.init()

size = width, height = 400, 400
screen = pygame.display.set_mode(size)
window_title = "Pursuit demo"
pygame.display.set_caption(window_title)
black = 0, 0, 0


# Object one

r0_obj1 = 0, height/2  # Initial position
obj1 = MovingObject2D(r0_obj1)
pygameObj1 = pygame.draw.circle(screen, (255, 0, 0), r0_obj1, 2)  # Drawing

r0_obj2 = width/2, height - 1
obj2 = MovingObject2D(r0_obj2)
pygameObj2 = pygame.draw.circle(screen, (255, 255, 0), r0_obj2, 2)  # Drawing

r0_obj3 = width - 1, height/2 - 1
obj3 = MovingObject2D(r0_obj3)
pygameObj2 = pygame.draw.circle(screen, (0, 255, 0), r0_obj3, 2)  # Drawing

r0_obj4 = width/2 - 1, 0
obj4 = MovingObject2D(r0_obj4)
pygameObj2 = pygame.draw.circle(screen, (0, 255, 255), r0_obj4, 2)  # Drawing

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

    # move object
    pos1 = obj1.getPosition()
    pos2 = obj2.getPosition()
    pos3 = obj3.getPosition()
    pos4 = obj4.getPosition()
    speed_obj1 = normalizeVector2D((pos2[0] - pos1[0], pos2[1] - pos1[1]))
    speed_obj2 = normalizeVector2D((pos3[0] - pos2[0], pos3[1] - pos2[1]))
    speed_obj3 = normalizeVector2D((pos4[0] - pos3[0], pos4[1] - pos3[1]))
    speed_obj4 = normalizeVector2D((pos1[0] - pos4[0], pos1[1] - pos4[1]))
    obj1.move(speed_obj1)
    obj2.move(speed_obj2)
    obj3.move(speed_obj3)
    obj4.move(speed_obj4)
    pygame.time.delay(100)

    # update image
#    screen.fill(black)
    pygame.draw.circle(screen, (255, 0, 0), obj1.getIntPosition(), 2)  # Drawing obj1
    pygame.draw.circle(screen, (255, 255, 0), obj2.getIntPosition(), 2)  # Drawing obj1
    pygame.draw.circle(screen, (0, 255, 0), obj3.getIntPosition(), 2)  # Drawing obj1
    pygame.draw.circle(screen, (0, 255, 255), obj4.getIntPosition(), 2)  # Drawing obj1
    pygame.display.flip()
