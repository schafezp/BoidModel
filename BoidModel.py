import pygame, sys
from pygame.locals import *
import random as rand
import math
from utils import *


#Number of frames per second
FPS = 25

###Sets size of grid

WINDOWWIDTH = 840
WINDOWHEIGHT = 680
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
RED =      (255,0,0)
BLUE =      (0,0,255)
GREEN =    (0,255,0)

#Configurations
BIRDCOLOR = BLACK
BIRDCOUNT = 20
BIRDTOLERANCE = 20
BIRDTOUCHINGTOLERANCE = 5

COLLISIONAVOIDANCE = 0 # 0 is ours, 1 is from paper
MAX_SEE_AHEAD = 10
#How much weight it gives to various parts of it's life
dampenMovementOfAllBoids = 1.00
dampenMovementTowardsCenter = 100
dampenForceRepellingBoidsfromOtherBoids = 20
dampenAveragingVelocityEffect = 30
dampenWillOfBoidstoDie = 1

velocityLimit = 2
#This influences how large the boids will be
step = 10


#Options are circle or polygon
#BIRDSHAPE = "circle"
#BIRDSHAPE = "polygon"
BIRDSHAPE = "polygonrotate"
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
        return "I am a velocity at (%f,%f)"%(self.x,self.y)
    def divideby(self,divisor):
        self.x = self.x/divisor
        self.y = self.y/divisor
        return self
    def multiplyby(self,multiplier):
        self.x = self.x*multiplier
        self.y = self.y*multiplier
        return self
    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)
    def unitize(self):
        return self.divideby(self.magnitude())
    def clone(self):
        return Vector(self.x,self.y)
    def negate(self):
        return self.multiplyby(-1)
    
class Obstacle(object):
    def __init__(self,position,radius,color):
        self.position = position
        self.radius = radius
        self.color = color
    def draw_self(self):
        xRenderPos = math.trunc((self.position.x+MAXX)*CELLSIZE)
        yRenderPos = math.trunc((self.position.y+MAXY)*CELLSIZE)
        pygame.draw.circle(DISPLAYSURF, self.color,(xRenderPos,yRenderPos), self.radius)
        
class ObstacleArray(object):
    def __init__(self,obstacleArray=[]):
        self.obstacleArray = obstacleArray
    def drawObstacles(self):
        for obstacle in self.obstacleArray:
            obstacle.draw_self()
    def boidInObstacle(self,boid):
        return positionInObstacle(boid.position)
    def positionInObstacle(self,position):
        for obstacle in self.obstacleArray:
            if distance(position,obstacle.position) <= obstacle.radius/CELLSIZE:
                return True
        return False
    def getObstacles(self):
        return self.obstacleArray
    
class BoidArray(object):
    """A boid array holds all of the boids which live in the universe """
    def __init__(self,SimilarDirection=False,BoidInitList=[],ObstacleInitList=ObstacleArray()):
        self.boidArray = []
        self.numberOfBoids = BIRDCOUNT
        self.obstacleArray = ObstacleInitList
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
    def getAllBoidsExcept(self,boidExcept):
        boidArrayWithoutBoid = []
        for boid in self.boidArray:
            if boid != boidExcept:
                boidArrayWithoutBoid.append(boid)
        return boidArrayWithoutBoid
    def getCenterPointOfAll(self):
        return getCenterPoint(self.boidArray)
    def getCenterPointOfAllExcept(self,boidExcept):
        boidArrayWithoutBoid = self.getAllBoidsExcept(boidExcept)
        return getCenterPoint(boidArrayWithoutBoid)
    # ---------------Velocity Model Functions---------
    #
    def getVectorToCenter(self,boid):
        centerpoint = self.getCenterPointOfAllExcept(boid)
        return getVectorBetweenPoints(boid.position,centerpoint)

    def getVectorAwayFromBoid(self,selfboid):
        #v is the vector we will return
        v = Vector(0,0)
        #TODO: Maybe this line will run slow because we create lots of new lists
        for boid in self.getAllBoidsExcept(selfboid):
            if distance(selfboid.position, boid.position) < BIRDTOUCHINGTOLERANCE:
                dispv = vectorSubtract(selfboid.position,boid.position)
                v = addVectors(v,dispv)
                
        return v

    def getVelocityOfOthersAround(self,selfboid):
        v = Vector(0,0)
        otherBoids = self.getAllBoidsExcept(selfboid)
        for boid in otherBoids:
            v = addVectors(v,boid.velocity)

        if len(otherBoids) == 0:
            return v
        
        v.divideby(len(otherBoids))
        #Add the difference between these velocity vectors
        v = vectorSubtract(v,selfboid.velocity)
        return v

    def boundPosition(self,selfboid):
        v = Vector(0,0)
        pullBackVelocity = .5
        ## xspan = MAXX-MINX
        ## yspan = MAXY-MINY
        ## if selfboid.position.x < MINX:
        ##     v.x = pullBackVelocity + (selfboid.position.x - MINX) / xspan
        ## elif selfboid.position.x > MAXX:
        ##     v.x = -(pullBackVelocity + (selfboid.position.x - MAXX) / xspan)
        ## if selfboid.position.y < MINY:
        ##     v.y = pullBackVelocity + (selfboid.position.y - MINY) / yspan
        ## elif selfboid.position.y > MAXY:
        ##     v.y = -(pullBackVelocity + (selfboid.position.y - MAXY) / yspan)
        if selfboid.position.x < MINX:
            v.x = pullBackVelocity
        elif selfboid.position.x > MAXX:
            v.x = -pullBackVelocity
        if selfboid.position.y < MINY:
            v.y = pullBackVelocity
        elif selfboid.position.y > MAXY:
            v.y = -pullBackVelocity

        return v

    def getVectorAwayFromObstacles(self,selfboid):
        v = Vector(0,0)
        if COLLISIONAVOIDANCE == 0:
            obstacles = self.obstacleArray.getObstacles()
            for obstacle in obstacles:
                vbetween = getVectorBetweenPoints(obstacle.position,selfboid.position)
                vToRemove = vbetween.clone().unitize().multiplyby(obstacle.radius)
                v = addVectors(vbetween,vToRemove.negate())
                dbetween = v.magnitude()
                #d = (math.exp((MAXX-MINX)/dbetween))
                #d = (math.exp((MAXX-MINX)/dbetween))
                d = dbetween**2
                print 'dbetween is %f modified d is %f' % (dbetween,d)
                v = addVectors(v,v.multiplyby(dampenWillOfBoidstoDie/d))
                #v = v.multiplyby(dampenWillOfBoidstoDie/(dbetween*dbetween))
                print 'Vector away is %s' %v
        elif COLLISIONAVOIDANCE == 1:
            #from http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
            vahead = selfboid.velocity.clone().unitize().multiplyby(MAX_SEE_AHEAD)
            vahead2 = selfboid.velocity.clone().unitize().multiplyby(MAX_SEE_AHEAD/2)
            pahead = sumVectorPosition(selfboid.position,vahead)
            if self.obstacleArray.positionInObstacle(pahead):
                
            
        return v


    def killBoidIfInObstacle(self,boid):
        if self.obstacleArray.boidInObstacle(boid):
                self.boidArray.remove(boid)
    #
    # ---------------Velocity Model Functions---------

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
                v3 = self.getVectorAwayFromBoid(boid)
                v4 = self.getVelocityOfOthersAround(boid)
                v5 = self.boundPosition(boid)
                v6 = self.getVectorAwayFromObstacles(boid)
                #v6 is a heavy wind
                #v6 = Vector(-0.03,0)
                #Apply parameter weightings
                #print "Vi is"
                print v1
                v1.divideby(dampenMovementOfAllBoids)
                v2.divideby(dampenMovementTowardsCenter)
                v3.divideby(dampenForceRepellingBoidsfromOtherBoids)
                v4.divideby(dampenAveragingVelocityEffect)
                
                boid.velocity = addVectors(v1,v2,v3,v4,v5,v6)
                boid.limit_velocity()
                #boid.velocity = addVectors(v1,v2,v3,v4,v5)
                #boid.velocity = addVectors(v1)
                #boid.velocity = v1
                print "--------------------"
                
                
                boid.position = sumVectorPosition(boid.position,boid.velocity)
                self.killBoidIfInObstacle(boid)
                print boid
                ## p.x = p.x + newX if self.isUniquePoint(Position(p.x + newX,p.y)) else p.x
                ## p.y = p.y + newY if self.isUniquePoint(Position(p.x,p.y + newY)) else p.y
                #Kill boids if they outside of the bounds
                ## if p.x > MAXX or p.y > MAXY or p.x < MINX or p.y < MINY:
                ##     print "A poor boid died today"
                ##     self.boidArray.remove(boid)
        #The last thing that we do in our tick is render the boids.
        #Before we render we choose to wipe the screen blank.
        DISPLAYSURF.fill(WHITE)
        self.drawBoids()
        self.obstacleArray.drawObstacles()
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
        return 'I am a boid centered at %d,%d going (%f,%f)' %(self.position.x,self.position.y,self.velocity.x,self.velocity.y)
    def limit_velocity(self):
        vMagnitude = self.velocity.magnitude()
        if  vMagnitude > velocityLimit:
            self.velocity = (self.velocity.divideby(vMagnitude)).multiplyby(velocityLimit)
    def draw_self(self):
        xRenderPos = math.trunc((self.position.x+MAXX)*CELLSIZE)
        yRenderPos = math.trunc((self.position.y+MAXY)*CELLSIZE)

        if BIRDSHAPE == "polygonrotate":
            center = (xRenderPos,yRenderPos)
            orientation = getOrientationFromVector(self.velocity)
            tp1 = (step,0)
            tp2 = (0,step)
            tp3 = (0,-step)
            #print "Orientation is %f" %(self.orientation)
            tp1 = transform(tp1,orientation,center)
            tp2 = transform(tp2,orientation,center)
            tp3 = transform(tp3,orientation,center)
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
    #boidInit.append(Boid(Vector(0.5,0.5),Position(55,15)))    
    #boidArray = BoidArray(BoidInitList = boidInit)
    p = 20
    radius = 30
    ## o1 = Obstacle(Position(0,10),radius,GREEN)
    ## o2 = Obstacle(Position(10,0),radius,DARKGRAY)
    ## obsticleArray = ObstacleArray([o1,o2])
    o1 = Obstacle(Position(-p,-p),radius,GREEN)
    o2 = Obstacle(Position(-p,p),radius,GREEN)
    o3 = Obstacle(Position(p,p),radius,GREEN)
    o4 = Obstacle(Position(p,-p),radius,GREEN)

    ## obsticleArray = ObstacleArray([o1,o3])
    obsticleArray = ObstacleArray([o1,o4])
    boidArray = BoidArray(ObstacleInitList=obsticleArray)
    #boidArray = BoidArray()
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
