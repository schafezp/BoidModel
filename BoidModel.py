import pygame, sys
from pygame.locals import *
import random as rand
import math
from utils import *

#Number of frames per second
FPS = 20

###Sets size of grid

WINDOWWIDTH = 640
WINDOWHEIGHT = 480
isBigWindow = False
if isBigWindow:
    WINDOWWIDTH = 1920
    WINDOWHEIGHT = 1080

CELLSIZE = 10

#Check to see if the width and height are multiples of the cell size.
assert WINDOWWIDTH % CELLSIZE == 0, "Window width must be a multiple of cell size"
assert WINDOWHEIGHT % CELLSIZE == 0, "Window height must be a multiple of cell size"

#Determine number of cells in horizonatl and vertical plane
CELLWIDTH = WINDOWWIDTH / CELLSIZE # number of cells wide
CELLHEIGHT = WINDOWHEIGHT / CELLSIZE # Number of cells high
MAXX = CELLWIDTH/2
MAXY = CELLHEIGHT/2
MINX = -MAXX
MINY = -MAXY

MAXVELOCITY = 1
MINVELOCITY = .1
# set up the colours
BLACK =    (0,  0,  0)
WHITE =    (255,255,255)
DARKGRAY = (40, 40, 40)
GREEN =    (0,255,0)

#Configurations
BIRDCOLOR = BLACK
BIRDCOUNT = 10
BIRDTOLERANCE = 20
BIRDTOUCHINGTOLERANCE = 5
#How much weight it gives to various parts of it's life

#How much it wants to preserve it's initial velocity
weightMoveOriginalOrientation = 1
#How much it wants to move towards the center of the cluster
weightMoveTowardsCenter = 0
#How much it wants to move away from other birds
globalweightMoveAwayFromOthers = 0
#How much it tries to orientate itself the same way all other birds are orientatd
weightMoveTowardsTotalOrientation = 0
#How much it tries to orientate itself the same way birds around it are oreintated
weightMoveTowardsLocalOrientation = 1

#If modifying orientation is set to false all birds will fly away
#in their initial direction
modifyingOrientation = True
modifyingVelocity = True

#This influences how large the boids will be
step = 10


#Options are circle or polygon
BIRDSHAPE = "circle"
#BIRDSHAPE = "polygon"
#BIRDSHAPE = "polygonrotate"
class Position(object):
    def __init__(self,x,y,isNull=False):
        self.x = x
        self.y = y
        self.isNull = isNull
    def __repr__(self):
        return "I am a position at (%d,%d)"%(self.x,self.y)


class Vector(object):
    def __init__(self,x,y,isNull=False):
        self.x = x
        self.y = y
    def __str__(self):
        return "I am a velocity at (%d,%d)"%(self.x,self.y)


class BoidArray(object):
    """A boid array holds all of the boids which live in the universe """
    def __init__(self,SimilarDirection=False,BoidInitList=[]):
        self.boidArray = []
        self.numberOfBoids = BIRDCOUNT
        if len(BoidInitList) > 0:
            self.boidArray = BoidInitList
        else:
            for i in range(self.numberOfBoids):
                if SimilarDirection:
                    #print "Init with similar direction"
                    newBoid = Boid(orientation = rand.uniform(0,2*math.pi))
                else:
                    newBoid = Boid()
                self.boidArray.append(newBoid)


    def append(self,boid):
        self.boidArray.append(boid)
    def len(self):
        return len(self.boidArray)

    def getCenterPointOfAll(self):
        return getCenterPoint(self.boidArray)
    def getCenterPointOfAllExcept(self,boidExcept):
        boidArrayWithoutBoid = []
        for boid in self.boidArray:
            if boid != boidExcept:
                boidArrayWithoutBoid.append(boid)

        return getCenterPoint(boidArrayWithoutBoid)
    def getVector(self,p1,p2):
        #Vector from p1 to p2

        return Vector(p2.x-p1.x,p2.y-p1.y)
    def getVectorToCenter(self,boid):
        centerpoint = self.getCenterPointOfAllExcept(boid)
        return self.getVector(boid.position,centerpoint)
    def getOrientationOfAll(self):
        #here o is the total orientation. We're averaging them all together
        return getOrientationOfBoids(self.boidArray)

    
    def getLocalCenter(self,selfboid,tolerance):
        #Here centerBoid is the boid we are looking to find the local neighborhood of
        cp = selfboid.position
        localBoids = []
        for boid in self.boidArray:
            p = boid.position
            dist = distance(p,cp)
            isSelf = dist > 0.1
            if dist < tolerance and isSelf:
                localBoids.append(boid)
        if len(localBoids) == 0:
            centerPoint = Position(0,0,isNull=True)
        else:
            centerPoint = getCenterPoint(localBoids)
            
        return centerPoint
    def getLocalVelocity(self,selfBoid,tolerance):
        #Here centerBoid is the boid we are looking to find the local neighborhood of
        #
        cv = selfBoid.velocity
        localBoids = self.getBoidsAround(selfBoid,tolerance)
        avgV = 0;
        for boid in localBoids:
            avgV = avgV + boid.velocity

        numberOfBoids =len(localBoids)
        if numberOfBoids == 0:
            print "No boids getLocalVelocity"
            return

        return avgV/numberOfBoids


        return centerPoint
    def getBoidsAround(self,selfBoid,tolerance):
        nearbyeBoids = []
        cp = selfBoid.position
        for boid in self.boidArray:
            p = boid.position
            if distance(p,cp) < tolerance:
                nearbyeBoids.append(boid)
        return nearbyeBoids

    def drawBoids(self):
        for boid in self.boidArray:
            boid.draw_self()
    def isUniquePoint(self,position):
        isUnique = 1
        for boid in self.boidArray:
            if (boid.position.x  == position.x and boid.position.y  == position.y):
                isUnique = 0

        return isUnique
    def tick(self):
        cp = getCenterPoint(self.boidArray)
        usingCenter = False
        usingVelocity = True
        if(usingCenter):
            #In this case all of them will head towards the center of mass of the birds
            for boid in self.boidArray:
                p = boid.position
                if abs(cp.x - p.x) > abs(cp.y - p.y):
                    if cp.x > p.x:
                        boid.position.x = p.x+1 if self.isUniquePoint(Position(p.x+1,p.y)) else p.x
                    else:
                        boid.position.x = p.x-1 if self.isUniquePoint(Position(p.x-1,p.y)) else p.x
                else:
                    if cp.y > p.y:
                        boid.position.y = p.y+1 if self.isUniquePoint(Position(p.x,p.y+1)) else p.y
                    else:
                        boid.position.y = p.y-1 if self.isUniquePoint(Position(p.x,p.y-1)) else p.y
        if(usingVelocity):
            for boid in self.boidArray:
                p = boid.position
                #v1 is current velocity vector
                v1 = boid.velocity
                v2 = self.getVectorToCenter(boid)

                #boid.velocity = addVector(v1,v2)
                boid.velocity = v1
                print boid.velocity
                boid.position = sumVectorPosition(boid.position,boid.velocity)
                
                print boid
                print boid.velocity.x
                print boid.velocity.y
                ## p.x = p.x + newX if self.isUniquePoint(Position(p.x + newX,p.y)) else p.x
                ## p.y = p.y + newY if self.isUniquePoint(Position(p.x,p.y + newY)) else p.y


                #print boid
                if p.x > MAXX or p.y > MAXY or p.x < MINX or p.y < MINY:
                    print "A poor boid died today"
                    self.boidArray.remove(boid)
        #The last thing that we do in our tick is render the boids.
        #Before we render we choose to wipe the screen blank.
        DISPLAYSURF.fill(WHITE)
        self.drawBoids()
class Boid(object):
    """ A boid is an independent entity which moves thorugh the world"""
    #Velocity will be in speed/time
    #Orientation starts at 0pi on unit circle and radians

    def __init__(self,velocity=None,position=None):
        if position==None:
            x = rand.randrange(MINX,MAXX)
            y = rand.randrange(MINY,MAXY)
            p = Position(x,y)
            print p
            self.position = p
        else:
            self.position = position

        if velocity==None:
            vx = rand.uniform(MINVELOCITY,MAXVELOCITY)
            vy = rand.uniform(MINVELOCITY,MAXVELOCITY)
            self.velocity = Vector(vx,vy)
        else:
            self.velocity = velocity
        
    def __repr__(self):
        return 'I am a boid centered at %d,%d going (%d,%d)' %(self.position.x,self.position.y,self.velocity.x,self.velocity.y)
    def draw_self(self):
        xRenderPos = math.trunc((self.position.x+MAXX)*CELLSIZE)
        yRenderPos = math.trunc((self.position.y+MAXY)*CELLSIZE)

        if BIRDSHAPE == "polygonrotate":
            center = (xRenderPos,yRenderPos)

            tp1 = (step,0)
            tp2 = (0,step)
            tp3 = (0,-step)
            #print "Orientation is %f" %(self.orientation)
            tp1 = transform(tp1,self.orientation,center)
            tp2 = transform(tp2,self.orientation,center)
            tp3 = transform(tp3,self.orientation,center)
            trianglePoints = (tp1,tp2,tp3)

            pygame.draw.polygon(DISPLAYSURF, BIRDCOLOR,trianglePoints, CELLSIZE/2)
        elif BIRDSHAPE == "polygon":
            pygame.draw.polygon(DISPLAYSURF, BIRDCOLOR,\
                            ((xRenderPos+step,yRenderPos),\
                            (xRenderPos,yRenderPos+step),\
                            (xRenderPos,yRenderPos-step)), CELLSIZE/2)
        elif BIRDSHAPE == "circle":
            pygame.draw.circle(DISPLAYSURF, BIRDCOLOR,(xRenderPos,yRenderPos), CELLSIZE/2)


#main function
def main():
    pygame.init()
    global DISPLAYSURF
    FPSCLOCK = pygame.time.Clock()
    DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT))
    pygame.display.set_caption('Boid Modeling')

    DISPLAYSURF.fill(WHITE)
    #When no parameters are passed BoidArray is initialized with 15 boids
    #boidArray = BoidArray(SimilarDirection=True)
    boidInit = []
    #boidInit.append(Boid(.4,.3 + math.pi ,Position(55,15)))    
    #boidArray = BoidArray(BoidInitList = boidInit)
    boidArray = BoidArray()
    boidArray.drawBoids()

    pygame.display.update()
    #NumberOfBoids = 15
    while True: #main game loop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        #runs a tick
        boidArray.tick()


        pygame.display.update()
        FPSCLOCK.tick(FPS)

        #print boidDict[(1,1)]

if __name__=='__main__':
    main()
