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
isBigWindow = True
if isBigWindow:
    WINDOWWIDTH = 1920
    WINDOWHEIGHT = 1080

CELLSIZE = 10
CELLSIZE = 8

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

MAXVELOCITY = 1.0
MINVELOCITY = .1
LIMITVELOCITY = 1.5
# set up the colours
BLACK =    (0,  0,  0)
WHITE =    (255,255,255)
DARKGRAY = (40, 40, 40)
RED =      (255,0,0)
BLUE =      (0,0,255)
GREEN =    (0,255,0)

#Configurations
BOIDCOLOR = BLACK
DEADBOIDCOLOR = RED
FRIGHTENEDBOIDCOLOR = BLUE
BOIDCOUNT = 60
BOID_FLOCK_VISION_TOLERANCE = 20
BOID_TOUCHING_TOLERANCE = 5
DISPLAY_TEXT = True
#NUMBER_OF_OBSTACLES_TO_GENERATE = 10
NUMBER_OF_OBSTACLES_TO_GENERATE = 10
MAX_OBSTACLE_SIZE_TO_GENERATE = (MAXX-MINX)/20*CELLSIZE
MIN_OBSTACLE_SIZE_TO_GENERATE=(MAXX-MINX)/40*CELLSIZE

COLLISIONAVOIDANCE = 1 # 0 is our original, 1 is from paper

#Determines if easilyfrightened boids are afraid of their peers
FRIGHTENED_BOIDS_AFRAID_OF_OTHERS = False
#from 0 to 1 based on how more afraid boids are of other others than normal. Smaller numbers more force
REDUCE_dampenForceRepellingBoidsfromOtherBoids = 0.5
#Determines if easilyfrightened boids are afraid of obstacles
FRIGHTENED_BOIDS_AFRAID_OF_OBSTACLES = True
FACTOR_BOIDS_MORE_AFRAID_OF_OBSTACLES = 3

#this is the BOID_TOUcHING_TOLERANCE for freightinted boids
FRIGHTENED_BOIDS_AVOID_BOIDS_IN_RADIUS = BOID_TOUCHING_TOLERANCE
#FRIGHTENED_BOIDS_AVOID_BOIDS_IN_RADIUS = 10

#How much more afraid? largr
#Determines how boids avoid obstacles
MAX_SEE_AHEAD = 15
MAX_AVOID_FORCE = 1

#Bounds the boid to the window
FORCE_RETURN_TO_WINDOW = .3

#How much weight it gives to various parts of it's life
dampenMovementOfAllBoids = 1.00
dampenMovementTowardsCenter = 200
dampenForceRepellingBoidsfromOtherBoids = 40
dampenAveragingVelocityEffect = 30
dampenWillOfBoidstoDie = 1


#This influences how large the boids will be
step = 10


#Options are circle or polygon
#BOIDSHAPE = "circle"
#BOIDSHAPE = "polygon"
BOIDSHAPE = "polygonrotate"

class Position(object):
    def __init__(self,x,y,isNull=False):
        self.x = x
        self.y = y
        self.isNull = isNull
    def __repr__(self):
        return "I am a position at (%d,%d)"%(self.x,self.y)
    def negate(self):
        self.x = -self.x
        self.y = -self.y
        return self
    def clone(self):
        return Position(self.x,self.y)
    def isOutOfBounds(self):
        return self.x>MAXX and self.x<MINX and self.y>MAXY and self.y<MAXY


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
    def positionInObstacle(self,position):
        return distance(position,self.position) <= self.radius/CELLSIZE
    
class ObstacleArray(object):
    def __init__(self,obstacleArray=[],generateObstacles=False,numtoGenerate=NUMBER_OF_OBSTACLES_TO_GENERATE,minRadius=MIN_OBSTACLE_SIZE_TO_GENERATE,maxRadius=MAX_OBSTACLE_SIZE_TO_GENERATE):
        self.obstacleArray = obstacleArray
        if generateObstacles:
            for i in range(numtoGenerate):
                o = Obstacle(randomPosition(), rand.randint(minRadius,maxRadius),GREEN)
                self.obstacleArray.append(o)
        
    def drawObstacles(self):
        for obstacle in self.obstacleArray:
            obstacle.draw_self()
    def boidInObstacle(self,boid):
        return self.positionInObstacles(boid.position)
    def positionInObstacles(self,position):
        for obstacle in self.obstacleArray:
            if obstacle.positionInObstacle(position):
                return True
        return False
    def getObstacles(self):
        return self.obstacleArray
    
class BoidArray(object):
    """A boid array holds all of the boids which live in the universe """
    def __init__(self,SimilarDirection=False,BoidInitList=[],ObstacleInitList=ObstacleArray(),DeadBoids=[],PercentageOfBoidsBornAfraid=0):
        self.boidArray = []
        self.numberOfBoids = BOIDCOUNT
        self.obstacleArray = ObstacleInitList
        self.deadBoids = DeadBoids
        self.percentageOfBoidsBornAfraid = PercentageOfBoidsBornAfraid
        self.numberOfFrightenenedBoids = math.ceil(self.numberOfBoids*self.percentageOfBoidsBornAfraid)

        if len(BoidInitList) > 0:
            self.boidArray = BoidInitList
        else:
            for i in range(self.numberOfBoids):
                if SimilarDirection:
                    #print "Init with similar direction"
                    newBoid = Boid(orientation = rand.uniform(0,2*math.pi))
                else:
                    newBoid = Boid()
                    
                    if i < self.numberOfFrightenenedBoids:
                        isFrightened = True
                    else:
                        isFrightened = False
                    while self.obstacleArray.positionInObstacles(newBoid.position):
                        newBoid = Boid()
                        
                    newBoid.isEasilyFrightened = isFrightened
                self.boidArray.append(newBoid)
        

    def getFrightened(self):
        boids = []
        for boid in self.boidArray:
            if boid.isEasilyFrightened:
                boids.append(boid)
        return boids
    def getNotFrightened(self):
        boids = []
        for boid in self.boidArray:
            if not boid.isEasilyFrightened:
                boids.append(boid)
        return boids
    
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
        if selfboid.isEasilyFrightened :
            boid_toucing_tolerance = FRIGHTENED_BOIDS_AVOID_BOIDS_IN_RADIUS 
        else:
            boid_toucing_tolerance = BOID_TOUCHING_TOLERANCE
        for boid in self.getAllBoidsExcept(selfboid):
            if distance(selfboid.position, boid.position) < boid_toucing_tolerance:
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
        
        ## xspan = MAXX-MINX
        ## yspan = MAXY-MINY
        ## if selfboid.position.x < MINX:
        ##     v.x = FORCE_RETURN_TO_WINDOW + (selfboid.position.x - MINX) / xspan
        ## elif selfboid.position.x > MAXX:
        ##     v.x = -(FORCE_RETURN_TO_WINDOW + (selfboid.position.x - MAXX) / xspan)
        ## if selfboid.position.y < MINY:
        ##     v.y = FORCE_RETURN_TO_WINDOW + (selfboid.position.y - MINY) / yspan
        ## elif selfboid.position.y > MAXY:
        ##     v.y = -(FORCE_RETURN_TO_WINDOW + (selfboid.position.y - MAXY) / yspan)
        if selfboid.position.x < MINX:
            v.x = FORCE_RETURN_TO_WINDOW
        elif selfboid.position.x > MAXX:
            v.x = -FORCE_RETURN_TO_WINDOW
        if selfboid.position.y < MINY:
            v.y = FORCE_RETURN_TO_WINDOW
        elif selfboid.position.y > MAXY:
            v.y = -FORCE_RETURN_TO_WINDOW

        return v

    def getVectorAwayFromObstacles(self,selfboid):
        v = Vector(0,0)
        if COLLISIONAVOIDANCE == 0:
            obstacles = self.obstacleArray.getObstacles()
            for obstacle in obstacles:
                vbetween = getVectorBetweenPoints(obstacle.position,selfboid.position)
                vToRemove = vbetween.clone().unitize().multiplyby(obstacle.radius/CELLSIZE)
                v = addVectors(vbetween,vToRemove.negate())
                dbetween = v.magnitude()
                #d = (math.exp((MAXX-MINX)/dbetween))
                #d = (math.exp((MAXX-MINX)/dbetween))
                #d = dbetween**2
                d = math.exp(dbetween)
                print 'dbetween is %f modified d is %f' % (dbetween,d)
                v = v.multiplyby(dampenWillOfBoidstoDie/d)
                #v = v.multiplyby(dampenWillOfBoidstoDie/(dbetween*dbetween))
                print 'Vector away is %s' %v
        elif COLLISIONAVOIDANCE == 1:
            #from http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
            dynamic_length = selfboid.velocity.magnitude()/MAXVELOCITY
            dynamic_length = dynamic_length*CELLSIZE*1.5
            vahead = selfboid.get_vahead(dynamic_length)
            print "Dynamic length is %d" %dynamic_length
            print selfboid.position
            print vahead
            print sumVectorPosition(selfboid.position,vahead)
            
            vahead2 = selfboid.get_vahead(dynamic_length/2)
            vahead3 = selfboid.get_vahead(dynamic_length/6)
            #vahead = selfboid.velocity.clone().unitize().multiplyby(dynamic_length)
            #vahead2 = selfboid.velocity.clone().unitize().multiplyby(dynamic_length/2)
            #vahead3 = selfboid.velocity.clone().unitize().multiplyby(dynamic_length/6)
            #vahead = selfboid.velocity.clone().unitize().multiplyby(MAX_SEE_AHEAD)
            #vahead2 = selfboid.velocity.clone().unitize().multiplyby(MAX_SEE_AHEAD/2)
            pahead = sumVectorPosition(selfboid.position,vahead)
            pahead2 = sumVectorPosition(selfboid.position,vahead2)
            pahead3 = sumVectorPosition(selfboid.position,vahead3)
            obstacles = self.obstacleArray.getObstacles()
            blockingObstacles = []
            
            for obstacle in obstacles:
                if obstacle.positionInObstacle(pahead) or obstacle.positionInObstacle(pahead2) or obstacle.positionInObstacle(pahead3):
                    blockingObstacles.append(obstacle)

            if len(blockingObstacles) == 0:
                return v
            else:
                minDistance = distance(selfboid.position, blockingObstacles[0].position)
                blockingObstacle = blockingObstacles[0]
                #fine closests obstacle
                for obstacle in blockingObstacles:
                    #TODO: We might want to have this distance by pahead2
                    d = distance(pahead, obstacle.position)
                    if d <minDistance:
                        blockingObstacle = obstacle
                        minDistance = d

                paheadInBlocking = obstacle.positionInObstacle(pahead)
                pahead2InBlocking = obstacle.positionInObstacle(pahead2)
                pahead3InBlocking = obstacle.positionInObstacle(pahead3)
                closenessFactor = 1
                if pahead2InBlocking:
                    closenessFactor = 1.5
                if pahead3InBlocking:
                    closenessFactor = 2

                if selfboid.isEasilyFrightened and FRIGHTENED_BOIDS_AFRAID_OF_OBSTACLES:
                    #closenessFactor = (closenessFactor+1)**2
                    closenessFactor = closenessFactor*FACTOR_BOIDS_MORE_AFRAID_OF_OBSTACLES
                
                if selfboid.position.x > blockingObstacle.position.x:
                    v.x = pahead.x - blockingObstacle.position.x 
                else:
                    v.x = blockingObstacle.position.x - pahead.x  
                if selfboid.position.y > blockingObstacle.position.y:
                    v.y = pahead.y - blockingObstacle.position.y 
                else:
                    v.y = blockingObstacle.position.y - pahead.y  
                ## if selfboid.position.x > blockingObstacle.position.x:
                ##     v.x = blockingObstacle.position.x + vahead.x
                ## else:
                ##     v.x = vahead.x + blockingObstacle.position.x
                ## if selfboid.position.y > blockingObstacle.position.y:
                ##     v.y = blockingObstacle.position.y +  vahead.y 
                ## else:
                ##     v.y = vahead.y +  blockingObstacle.position.y
                ## if selfboid.position.x > blockingObstacle.position.x:
                ##     v.x = blockingObstacle.position.x - vahead.x
                ## else:
                ##     v.x = vahead.x - blockingObstacle.position.x
                ## if selfboid.position.y > blockingObstacle.position.y:
                ##     v.y = blockingObstacle.position.y - vahead.y 
                ## else:
                ##     v.y = vahead.y - blockingObstacle.position.y
                #v.x = vahead.x - blockingObstacle.position.x
                print v.x
                #v.y = vahead.y - blockingObstacle.position.y
                print v.y
                #v.unitize().multiplyby(MAX_AVOID_FORCE).multiplyby(2*selfboid.velocity.magnitude()/MAXVELOCITY)
                
                
                v.unitize().multiplyby(MAX_AVOID_FORCE*closenessFactor)
                print "COLLISIONAVOIDANCE v"
                print v
                #exit()
                
                
            
        return v


    def killBoidIfInObstacle(self,boid):
        if self.obstacleArray.boidInObstacle(boid):
            boid.isDead=True
            self.deadBoids.append(boid)
            self.boidArray.remove(boid)
    #
    # ---------------Velocity Model Functions---------

    def getVectorToLocalCenter(self, selfboid):
        #tolerance = BOID_TOUCHING_TOLERANCE
        tolerance = BOID_FLOCK_VISION_TOLERANCE
        cp = selfboid.position
        localBoids = []
        for boid in self.boidArray:
            p = boid.position
            dist = distance(p,cp)
            isNotSelf = dist > 0
            if dist < tolerance and isNotSelf:
                localBoids.append(boid)
        if len(localBoids) == 0:
            return Vector(0,0)
        else:
            centerPoint = getCenterPoint(localBoids)
            if centerPoint.isNull:
                return Vector(0,0)
            else:
                return getVectorBetweenPoints(selfboid.position,centerPoint)
        
    def getMinVectorToWall(self,selfboid):
        #TODO: Do this
        return Vector(0,0)
    
    def drawBoids(self):
        #draw all live boids
        for boid in self.boidArray:
            boid.draw_self()
            
        #draw all dead boids
        for boid in self.deadBoids:
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
                #v2 = self.getVectorToCenter(boid)
                v2 = self.getVectorToLocalCenter(boid)
                v3 = self.getVectorAwayFromBoid(boid)
                v4 = self.getVelocityOfOthersAround(boid)
                v5 = self.boundPosition(boid)
                v6 = self.getVectorAwayFromObstacles(boid)
                v7 = self.getMinVectorToWall(boid)
                #v6 is a heavy wind
                #v6 = Vector(-0.03,0)
                #Apply parameter weightings
                #print "Vi is"
                print v1
                v1.divideby(dampenMovementOfAllBoids)
                v2.divideby(dampenMovementTowardsCenter)
                
                if FRIGHTENED_BOIDS_AFRAID_OF_OTHERS:
                    afraidOfBoids = REDUCE_dampenForceRepellingBoidsfromOtherBoids if boid.isEasilyFrightened else 1
                else:
                    afraidOfBoids =1
                    
                v3.divideby(dampenForceRepellingBoidsfromOtherBoids*afraidOfBoids)
                v4.divideby(dampenAveragingVelocityEffect)

                if not boid.isDead:
                    if v6.magnitude() > 0.3:
                        boid.velocity = addVectors(v1,v3,v5,v6)
                        print "Greater than 1"
                    else:
                        print v6
                        #boid.velocity = addVectors(v1,v2,v3,v4,v5,v6)
                        boid.velocity = addVectors(v1,v2,v3,v4,v5,v6)
                        #print "Less than 1"
                    boid.limit_velocity()
                    boid.position = sumVectorPosition(boid.position,boid.velocity)
                print "--------------------"                
                
                
                #Kill boids if they run into obstacles
                self.killBoidIfInObstacle(boid)
                print boid
                ## p.x = p.x + newX if self.isUniquePoint(Position(p.x + newX,p.y)) else p.x
                ## p.y = p.y + newY if self.isUniquePoint(Position(p.x,p.y + newY)) else p.y


        #The last thing that we do in our tick is render the boids.
        #Before we render we choose to wipe the screen blank.
        DISPLAYSURF.fill(WHITE)
        self.drawBoids()
        self.obstacleArray.drawObstacles()
        font = pygame.font.SysFont("comicsansms", 30)
        if DISPLAY_TEXT:
            if self.percentageOfBoidsBornAfraid > 0:
                textafraid = font.render("Number of Frightened Boids Alive: "+str(len(self.getFrightened())), 1, (10, 0, 0))
                textnotafraid = font.render("Number of Not Frightened Boids Alive: "+str(len(self.getNotFrightened())), 1, (10, 0, 0))
                textdead = font.render("Number of Boids Dead: "+str(len(self.deadBoids)), 1, (10, 0, 0))
                DISPLAYSURF.blit(textafraid,(20, 20))
                DISPLAYSURF.blit(textnotafraid,(20, 45))
                DISPLAYSURF.blit(textdead,(20, 65))
            else:
                textalive = font.render("Number of Boids Alive: "+str(len(self.boidArray)), 1, (10, 10, 10))
                textdead = font.render("Number of Boids Dead: "+str(len(self.deadBoids)), 1, (10, 0, 0))
                DISPLAYSURF.blit(textalive,(20,20))
                DISPLAYSURF.blit(textdead,(20, 45))
        
        
class Boid(object):
    """ A boid is an independent entity which moves thorugh the world"""
    #Velocity will be in speed/time
    #Orientation starts at 0pi on unit circle and radians

    def __init__(self,velocity=None,position=None,isDead=False,isEasilyFrightened=False):
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

        self.isDead = isDead
        self.isEasilyFrightened=isEasilyFrightened
        
    def __repr__(self):
        return 'I am a boid centered at %d,%d going (%f,%f)' %(self.position.x,self.position.y,self.velocity.x,self.velocity.y)
    def limit_velocity(self):
        vMagnitude = self.velocity.magnitude()
        if  vMagnitude > LIMITVELOCITY:
            self.velocity = (self.velocity.divideby(vMagnitude)).multiplyby(LIMITVELOCITY)
    def get_vahead(self,multby):
        return self.velocity.clone().unitize().multiplyby(multby)
    def draw_self(self):
        xRenderPos = math.trunc((self.position.x+MAXX)*CELLSIZE)
        yRenderPos = math.trunc((self.position.y+MAXY)*CELLSIZE)
        if self.isDead:
            birdcolor = DEADBOIDCOLOR
        if self.isEasilyFrightened :
            birdcolor = FRIGHTENEDBOIDCOLOR
        elif not self.isDead:
            birdcolor = BOIDCOLOR
            
            
        if self.isDead:
            #pygame.draw.circle(DISPLAYSURF, DEADBOIDCOLOR,(xRenderPos,yRenderPos), 2*CELLSIZE)
            pygame.draw.circle(DISPLAYSURF, birdcolor,(xRenderPos,yRenderPos), 2*CELLSIZE)
        else:
            if BOIDSHAPE == "polygonrotate":
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

                pygame.draw.polygon(DISPLAYSURF, birdcolor,trianglePoints, CELLSIZE/2)
            elif BOIDSHAPE == "polygon":
                pygame.draw.polygon(DISPLAYSURF, birdcolor,\
                                    ((xRenderPos+step,yRenderPos),\
                                     (xRenderPos,yRenderPos+step),\
                                    (xRenderPos,yRenderPos-step)), CELLSIZE/2)
            elif BOIDSHAPE == "circle":
                pygame.draw.circle(DISPLAYSURF, birdcolor,(xRenderPos,yRenderPos), CELLSIZE/2)


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
    radius = 20
    
    ## o1 = Obstacle(Position(-p,-p),radius,GREEN)
    ## o2 = Obstacle(Position(-p,p),radius,GREEN)
    ## o3 = Obstacle(Position(p,p),radius,GREEN)
    ## o4 = Obstacle(Position(p,-p),radius,GREEN)
    o1 = Obstacle(Position(-20,-10),radius,GREEN)
    o2 = Obstacle(Position(-30,24),radius,DARKGRAY)
    o3 = Obstacle(Position(35,-20),radius,BLUE)
    oA1 = ObstacleArray([o1,o2,o3])
    
    o4  = Obstacle(Position(-20,-10),radius,GREEN)
    oA2 = ObstacleArray(generateObstacles=True)
    
    #boidArray = BoidArray(ObstacleInitList=oA2)
    boidArray = BoidArray(ObstacleInitList=oA2,PercentageOfBoidsBornAfraid=0.5)
    
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

        #printbo idDict[(1,1)]

if __name__=='__main__':
    main()
