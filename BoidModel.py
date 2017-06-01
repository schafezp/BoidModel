import math
from utils import *
import pygame, sys
from pygame.locals import *
import random as rand
#TODO: Implement the following features:
#Model:
# Make boids not hit walls and if they hit wall they should lose all velocity
# Make frightened boid model not the default
# Seek to remove unnecessary constants, all others should be made vectorized for input
# Train on vectorized coefficients
#UI:
# Add UI to switch between regular and frightened model
# Allow click in the ui to add obstacle

#Performance:
# Most complicated operations performed are mults and divs which numpy doesn't make faster
# Consider effect of using numpy matrix for transform required in animating polygons.
# Perhaps the rotated polygon can be drawn from it's coordinates directly and shouldn't be transformed
# Generating random numbers from numpy is fast? http://www.shocksolution.com/2009/01/optimizing-python-code-for-fast-math/





#Number of frames per second
FPS = 25

#If debug is true then print values
DEBUG = True
#PRINT_TRAILS is true then print the trails of the boids instead of redrawing the animation each time
PRINT_TRAILS = False

###Sets size of grid

WINDOWWIDTH = 840
WINDOWHEIGHT = 680
#isBigWindow = True
isBigWindow = False
if isBigWindow:
    WINDOWWIDTH = 1920
    WINDOWHEIGHT = 1080

CELLSIZE = 10
#CELLSIZE = 8

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
#Move towards boids in this area
BOID_FLOCK_VISION_TOLERANCE = 20
#Move away from  boids in this area
BOID_FLOCK_TOUCHING_TOLERANCE = 5

#this is the BOID_FLOCK_TOUCHING_TOLERANCE for freightinted boids
FRIGHTENED_BOIDS_AVOID_BOIDS_IN_RADIUS = BOID_FLOCK_TOUCHING_TOLERANCE

DISPLAY_TEXT = True
NUMBER_OF_OBSTACLES_TO_GENERATE = 10

MAX_OBSTACLE_SIZE_TO_GENERATE = (MAXX-MINX)/20*CELLSIZE
MIN_OBSTACLE_SIZE_TO_GENERATE=(MAXX-MINX)/40*CELLSIZE

COLLISIONAVOIDANCE = 1 # 0 is low quality , 1 is high quality
#What percentage of boids are born afraid?
PERCENTAGE_OF_BOIDS_BORN_AFRAID = 0.5
#Determines if easilyfrightened boids are afraid of their peers
FRIGHTENED_BOIDS_AFRAID_OF_OTHERS = False
#Multiplied by the vector which drives boids apart
FACTOR_FRIGHTENED_BOIDS_AFRAID_OF_OTHERS = 5
#Determines if easilyfrightened boids are afraid of obstacles
FRIGHTENED_BOIDS_AFRAID_OF_OBSTACLES = True
FACTOR_BOIDS_MORE_AFRAID_OF_OBSTACLES = 5
#FACTOR_BOIDS_MORE_AFRAID_OF_OBSTACLES = 3


#FRIGHTENED_BOIDS_AVOID_BOIDS_IN_RADIUS = 10

#Determines how boids avoid obstacles
MAX_SEE_AHEAD = 30
MAX_AVOID_FORCE = 1

#Bounds the boid to the window
FORCE_RETURN_TO_WINDOW = .3

#How much weight it gives to various parts of it's life

DAMPEN_MOVEMENT_OF_ALL_BOIDS = 1
DAMPEN_MOVEMENT_TOWARDS_CENTER = 200
DAMPEN_FORCE_REPELLING_BOIDS_FROM_OTHER_BOIDS = 40
DAMPEN_AVERAGING_VELOCITY_EFFECT = 30
DAMPEN_WILL_OF_BOIDS_TO_DIE = 1


#This influences how large the boids will be
BOID_ANIMATION_SIZE = 6


#Options are circle or polygon
#BOID_SHAPE = "circle"
#BOID_SHAPE = "polygon"
BOID_SHAPE = "polygonrotate"

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
        return distance(position,self.position) <= self.radius//CELLSIZE

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
                    #print ("Init with similar direction")
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
            boid_toucing_tolerance = BOID_FLOCK_TOUCHING_TOLERANCE
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
                d = dbetween**2
                #possible instead use exp
                #d = math.exp(dbetween)
                if DEBUG:
                    print ('dbetween is %f modified d is %f' % (dbetween,d))
                v = v.multiplyby(DAMPEN_WILL_OF_BOIDS_TO_DIE/d)
                if DEBUG:
                    print ('Vector away is %s' %v)
        elif COLLISIONAVOIDANCE == 1:
            #from http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777 initially. modified since then
            dynamic_length = selfboid.velocity.magnitude()/MAXVELOCITY
            dynamic_length = dynamic_length*CELLSIZE*1.0
            vahead = selfboid.get_vahead(dynamic_length)
            if DEBUG:
                print ("Dynamic length is %d" %dynamic_length)
                print (selfboid.position)
                print ("Vahead:")
                print (vahead)

            vahead2 = selfboid.get_vahead(dynamic_length/2)
            vahead3 = selfboid.get_vahead(dynamic_length/6)
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

                paheadInBlocking = blockingObstacle.positionInObstacle(pahead)
                pahead2InBlocking = blockingObstacle.positionInObstacle(pahead2)
                pahead3InBlocking = blockingObstacle.positionInObstacle(pahead3)
                ## paheadInBlocking = obstacle.positionInObstacle(pahead)
                ## pahead2InBlocking = obstacle.positionInObstacle(pahead2)
                ## pahead3InBlocking = obstacle.positionInObstacle(pahead3)
                closenessFactor = 1
                if pahead2InBlocking:
                    closenessFactor = 1.5
                if pahead3InBlocking:
                    closenessFactor = 2

                if selfboid.isEasilyFrightened and FRIGHTENED_BOIDS_AFRAID_OF_OBSTACLES:
                    #closenessFactor = (closenessFactor+1)**2
                    closenessFactor = closenessFactor*FACTOR_BOIDS_MORE_AFRAID_OF_OBSTACLES
                #the vector from the obstacle to pahead1
                chosenpahead = pahead
                chosenvahead = vahead
                if pahead2InBlocking:
                    chosenpahead = pahead2
                    chosenvahead = vahead2
                elif pahead3InBlocking:
                    chosenpahead = pahead3
                    chosenvahead = vahead3
                    
                o_to_pahead = getVectorBetweenPoints(blockingObstacle.position, chosenpahead)
                if DEBUG:
                    print ("O to pahead")
                    print (o_to_pahead)

                chosenvahead.unitize()
                o_to_pahead.unitize()
                v.x = chosenvahead.x + o_to_pahead.x
                v.y = chosenvahead.y + o_to_pahead.y
                ## v.x = vahead.x - blockingObstacle.position.x
                ## v.x = vahead.x - blockingObstacle.position.x
                ## if selfboid.position.x > blockingObstacle.position.x:
                ##     v.x = pahead.x - blockingObstacle.position.x
                ## else:
                ##     v.x = blockingObstacle.position.x - pahead.x
                ## if selfboid.position.y > blockingObstacle.position.y:
                ##     v.y = pahead.y - blockingObstacle.position.y
                ## else:
                ##     v.y = blockingObstacle.position.y - pahead.y
                #v.y = vahead.y - blockingObstacle.position.y
                v.unitize().multiplyby(MAX_AVOID_FORCE*closenessFactor)
                if DEBUG:
                    print ("COLLISIONAVOIDANCE v")
                    print (v)


        return v


    def killBoidIfInObstacle(self,boid):
        if self.obstacleArray.boidInObstacle(boid):
            boid.isDead=True
            self.deadBoids.append(boid)
            self.boidArray.remove(boid)
    #
    # ---------------Velocity Model Functions---------

    def getVectorToLocalCenter(self, selfboid):
        #tolerance = BOID_FLOCK_TOUCHING_TOLERANCE
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
        #TODO: Implement
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
                v1.divideby(DAMPEN_MOVEMENT_OF_ALL_BOIDS)
                v2.divideby(DAMPEN_MOVEMENT_TOWARDS_CENTER)

                if FRIGHTENED_BOIDS_AFRAID_OF_OTHERS:
                    afraidOfBoids = FACTOR_FRIGHTENED_BOIDS_AFRAID_OF_OTHERS if boid.isEasilyFrightened else 1
                else:
                    afraidOfBoids =1


                v3.divideby(DAMPEN_FORCE_REPELLING_BOIDS_FROM_OTHER_BOIDS).multiplyby(afraidOfBoids)
                v4.divideby(DAMPEN_AVERAGING_VELOCITY_EFFECT)
                if not boid.isDead:
                    boid.velocity = addVectors(v1,v2,v3,v4,v5,v6)
                        #print "Less than 1"
                    boid.limit_velocity()
                    boid.position = sumVectorPosition(boid.position,boid.velocity)

                if DEBUG:
                    print ("--------------------")


                #Kill boids if they run into obstacles
                self.killBoidIfInObstacle(boid)
                if DEBUG:
                    print (boid)


        #The last thing that we do in our tick is render the boids.
        #Before we render we choose to wipe the screen blank.
        if not PRINT_TRAILS:
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
            if DEBUG:
                print (p)
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
            boidcolor = DEADBOIDCOLOR
        if self.isEasilyFrightened :
            boidcolor = FRIGHTENEDBOIDCOLOR
        elif not self.isDead:
            boidcolor = BOIDCOLOR


        if self.isDead:
            #pygame.draw.circle(DISPLAYSURF, DEADBOIDCOLOR,(xRenderPos,yRenderPos), 2*CELLSIZE)
            pygame.draw.circle(DISPLAYSURF, boidcolor,(xRenderPos,yRenderPos), 2*CELLSIZE)
        else:
            if BOID_SHAPE == "polygonrotate":
                center = (xRenderPos,yRenderPos)
                orientation = getOrientationFromVector(self.velocity)
                tp1 = (BOID_ANIMATION_SIZE,0)
                tp2 = (0,BOID_ANIMATION_SIZE)
                tp3 = (0,-BOID_ANIMATION_SIZE)
                #print ("Orientation is %f" %(self.orientation))
                tp1 = transform(tp1,orientation,center)
                tp2 = transform(tp2,orientation,center)
                tp3 = transform(tp3,orientation,center)
                trianglePoints = (tp1,tp2,tp3)

                pygame.draw.polygon(DISPLAYSURF, boidcolor,trianglePoints, CELLSIZE//2)
            elif BOID_SHAPE == "polygon":
                pygame.draw.polygon(DISPLAYSURF, boidcolor,\
                                    ((xRenderPos+BOID_ANIMATION_SIZE,yRenderPos),\
                                     (xRenderPos,yRenderPos+BOID_ANIMATION_SIZE),\
                                    (xRenderPos,yRenderPos-BOID_ANIMATION_SIZE)), CELLSIZE//2)
            elif BOID_SHAPE == "circle":
                pygame.draw.circle(DISPLAYSURF, boidcolor,(xRenderPos,yRenderPos), CELLSIZE//2)


#main function
def main():
    pygame.init()
    global DISPLAYSURF
    FPSCLOCK = pygame.time.Clock()
    DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT))
    pygame.display.set_caption('Boid Modeling')

    DISPLAYSURF.fill(WHITE)
    
    boidInit = []
    radius = 20
    o1 = Obstacle(Position(-20,-10),radius,GREEN)
    o2 = Obstacle(Position(-30,24),radius,DARKGRAY)
    o3 = Obstacle(Position(35,-20),radius,BLUE)
    oA1 = ObstacleArray([o1,o2,o3])

    o4  = Obstacle(Position(-20,-10),radius,GREEN)
    oA2 = ObstacleArray(generateObstacles=True)

    #boidArray = BoidArray(ObstacleInitList=oA2)
    boidArray = BoidArray(ObstacleInitList=oA2,PercentageOfBoidsBornAfraid=PERCENTAGE_OF_BOIDS_BORN_AFRAID)

    #boidArray = BoidArray()
    boidArray.drawBoids()


    pygame.display.update()
    while True: #main game loop
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        #runs a tick
        boidArray.tick()


        pygame.display.update()
        FPSCLOCK.tick(FPS)


if __name__=='__main__':
    main()
