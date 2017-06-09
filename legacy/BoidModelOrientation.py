import pygame, sys
from pygame.locals import *
import random as rand
import math
from BoidModelOrientationUtils import *

#Number of frames per second
FPS = 30

###Sets size of grid

WINDOWWIDTH = 640
WINDOWHEIGHT = 480
#Set this to true if you want to view full screened. 
isBigWindow = True
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
MAXX = CELLWIDTH
MAXY = CELLHEIGHT

MAXVELOCITY = 1
MINVELOCITY = .1
# set up the colours
BLACK =    (0,  0,  0)
WHITE =    (255,255,255)
DARKGRAY = (40, 40, 40)
GREEN =    (0,255,0)

#Configurations
BIRDCOLOR = BLACK
BIRDCOUNT = 50
BIRDTOLERANCE = 20
BIRDTOUCHINGTOLERANCE = 5

#How much weight it gives to various parts of it's life

#How much it wants to preserve it's initial velocity
weightMoveOriginalOrientation = 1
#How much it wants to move towards the center of the cluster
weightMoveTowardsCenter = 1
#How much it wants to move away from other birds
globalweightMoveAwayFromOthers = 1
#How much it tries to orientate itself the same way all other birds are orientatd
weightMoveTowardsTotalOrientation = 1
#How much it tries to orientate itself the same way birds around it are oreintated
weightMoveTowardsLocalOrientation = 1

#If modifying orientation is set to false all birds will fly away
#in their initial direction
modifyingOrientation = True
modifyingVelocity = True

#This influences how large the boids will be drawn (purely cosmetic)
step = 10


#Options are simply circles or rotating polygons
#BIRDSHAPE = "circle"
#BIRDSHAPE = "polygon"
BIRDSHAPE = "polygonrotate"
class Position(object):
    def __init__(self,x,y,isNull=False):
        self.x = x
        self.y = y
        self.isNull = isNull

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
                    #print ("Init with similar direction")
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
            print ("No boids getLocalVelocity")
            return

        return avgV/numberOfBoids


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
        usingMomentum = True
        #usingCenter = True
        #usingMomentum = False
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
        if(usingMomentum):
            totalCenterPoint= getCenterPoint(self.boidArray)
            for boid in self.boidArray:
                p = boid.position
                o = boid.orientation

                if modifyingOrientation:
                    
                    #The angle to the center of mass of all birds
                    #orientationToCenter = getOrientationToPosition(p,totalCenterPoint)
                    orientationToCenter = getOrientationToPosition(totalCenterPoint,p)
                    
                    #The angle to the center of mass of all the birds within a certain tolerance
                    localCenterPoint = self.getLocalCenter(boid,BIRDTOUCHINGTOLERANCE)
                    weightMoveAwayFromOthers = globalweightMoveAwayFromOthers
                    if localCenterPoint.isNull:
                        #In this case we don't use localCenter becaues local center is only ourselves
                        weightMoveAwayFromOthers = 0
                        orientationAwayFromLocalCenter = 0
                    else:
                        orientationToLocalCenter = getOrientationToPosition(localCenterPoint,p)
                        #Negate the previous angle to give the angle going away from the center of the birds
                        orientationAwayFromLocalCenter = -orientationToLocalCenter
                    #The average orientation
                    orientationOfAll = self.getOrientationOfAll()

                    orientationOfLocal = getOrientationOfBoids(self.getBoidsAround(boid,BIRDTOLERANCE))
                    print ("Orientation around me is %f"%(orientationOfLocal))
                    totalWeight = weightMoveOriginalOrientation + weightMoveTowardsCenter + weightMoveAwayFromOthers + weightMoveTowardsTotalOrientation + weightMoveTowardsLocalOrientation
                    newOrientation = (weightMoveOriginalOrientation*o + weightMoveTowardsCenter*orientationToCenter + weightMoveAwayFromOthers * orientationAwayFromLocalCenter + weightMoveTowardsTotalOrientation*orientationOfAll + weightMoveTowardsLocalOrientation*orientationOfLocal)/totalWeight
                    #used for going away from walls
                    
                    print ("------------------")
                    print ("New orientation %f" %(newOrientation))
                    print ("Orientation of all %f" % (orientationOfAll))
                    #print (boid)
                    boid.orientation = newOrientation
                if modifyingVelocity:
                    localVelocity = self.getLocalVelocity(boid,BIRDTOLERANCE)
                    boid.velocity = (localVelocity + boid.velocity)/2
                v = boid.velocity
                newX = math.cos(o)*v
                newY = math.sin(o)*v

                p.x = p.x + newX if self.isUniquePoint(Position(p.x + newX,p.y)) else p.x
                p.y = p.y + newY if self.isUniquePoint(Position(p.x,p.y + newY)) else p.y


                #print (boid)
                if p.x > MAXX or p.y > MAXY or p.x < 0 or p.y < 0:
                    print ("A poor boid died today")
                    self.boidArray.remove(boid)
        #The last thing that we do in our tick is render the boids.
        #Before we render we choose to wipe the screen blank.
        DISPLAYSURF.fill(WHITE)
        self.drawBoids()
class Boid(object):
    """ A boid is an independent entity which moves thorugh the world"""
    #Velocity will be in speed/time
    #Orientation starts at 0pi on unit circle and radians

    def __init__(self,velocity=None,orientation=None,position=None):
        if position==None:
            x = rand.randrange(0,MAXX)
            y = rand.randrange(0,MAXY)
            p = Position(x,y)
            self.position = p
        else:
            self.position = position

        if velocity==None:
            v = rand.uniform(MINVELOCITY,MAXVELOCITY)
            print (v)
            self.velocity = v
        else:
            self.velocity = velocity

        if orientation==None:
            o= rand.uniform(0,2*math.pi)
            self.orientation = o
        else:
            self.orientation = orientation

    def __repr__(self):
        return 'I am a boid centered at %d,%d with orientation %f' %(self.position.x,self.position.y,self.orientation)
    def draw_self(self):
        #pygame.draw.circle(DISPLAYSURF, BIRDCOLOR, (math.trunc(self.position.x*CELLSIZE), math.trunc(self.position.y*CELLSIZE)), CELLSIZE//2)
        xRenderPos = math.trunc(self.position.x*CELLSIZE)
        yRenderPos = math.trunc(self.position.y*CELLSIZE)

        if BIRDSHAPE == "polygonrotate":
            center = (xRenderPos,yRenderPos)
            
            tp1 = (step,0)
            tp2 = (0,step)
            tp3 = (0,-step)
            #print ("Orientation is %f" %(self.orientation))
            tp1 = transform(tp1,self.orientation,center)
            tp2 = transform(tp2,self.orientation,center)
            tp3 = transform(tp3,self.orientation,center)
            trianglePoints = (tp1,tp2,tp3)

            pygame.draw.polygon(DISPLAYSURF, BIRDCOLOR,trianglePoints, CELLSIZE//2)
        elif BIRDSHAPE == "polygon":
            pygame.draw.polygon(DISPLAYSURF, BIRDCOLOR,\
                            ((xRenderPos+step,yRenderPos),\
                            (xRenderPos,yRenderPos+step),\
                            (xRenderPos,yRenderPos-step)), CELLSIZE/2)
        elif BIRDSHAPE == "circle":
            pygame.draw.circle(DISPLAYSURF, BIRDCOLOR,(xRenderPos,yRenderPos), CELLSIZE//2)


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
    boidInit.append(Boid(.4,math.pi/2,Position(40,15)))
    boidInit.append(Boid(.4,.0 ,Position(26,15)))
    boidInit.append(Boid(.4,.5 + math.pi,Position(43,20)))
    boidInit.append(Boid(.4,.5 + math.pi,Position(35,22)))
    #------BOID ARRAY INITIALIZATION OPTIONS------
    #If you chose to 
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

        #print (boidDict[(1,1)])

if __name__=='__main__':
    main()
