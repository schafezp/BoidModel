import math
import random as rand

from BoidModel import Position, Boid, Vector, MAXX, MINX, MAXY,MINY
def getCenterPoint(boidArray):
        totalX = 0
        totalY = 0
        count = len(boidArray)
        if count == 0:
            #TODO: FIX THIS
            return Position(0,0,isNull=True)
        for boid in boidArray:
            p = boid.position
            totalX = totalX + p.x
            totalY = totalY + p.y
        avgX = totalX / count
        avgY = totalY / count
        return Position(avgX,avgY)
def getOrientationOfBoids(boidArray):
    o = 0
    for boid in boidArray:
        o = o + boid.orientation

    if len(boidArray) != 0:
        return o % (math.pi*2)
    else:
        return 0
def getOrientationToPosition(positionStart,positionEnd):
        p1 = positionStart
        p2 = positionEnd
        
        orientation = math.atan2(p2.y-p1.y,p2.x-p1.x) + math.pi
        return orientation

def distance(p1,p2):
        return math.sqrt((p2.y-p1.y)**2 + (p2.x-p1.x)**2)

#Transform is called many times since it run in the tick loop for each alive boid.
#TODO: consider matrix multiple for transform instead of writing out operations by hand
def transform(xypair,orientation,center):
    x = xypair[0]
    y = xypair[1]
    xstart = x
    ystart = y
    x = xstart*math.cos(orientation) - ystart*math.sin(orientation)
    y = xstart*math.sin(orientation) + ystart*math.cos(orientation)
    #print ("x changed %f y changed %f" %(xstart-x,ystart-y))
    return (x+center[0],y+center[1])
def xycof(boid,MAXX, MAXY):
    #Return a tuple with (xcof,ycof)
    p = boid.position
    a = MAXX-p.x
    aprime = p.x
    b = MAXY-p.y
    bprime = p.y

    if a > aprime:
        cofx = a/aprime
    else:
        cofx = aprime/a

    if b > bprime:
        cofy = b/bprime
    else:
        cofy = bprime/b
    return (cofx,cofy)
def sumVectorPosition(p,v):
 return Position(p.x+v.x, p.y + v.y)


def addVectors(*vs):
    """ Takes a list of vectors and returns their sum as a vector"""
    xsum = 0
    ysum = 0
    #print ("Vs")
    #print (vs)
    for v in vs:
        #print (v)
        xsum += v.x
        ysum += v.y
    return Vector(xsum,ysum)
def vectorSubtract(v1,v2):
    """Return v1-v2"""
    vx = v1.x-v2.x
    vy = v1.y-v2.y
    return Vector(vx,vy)

def getVectorBetweenPoints(p1,p2):
    """ Returns the vector that takes us from  p1 to p2 """
    return Vector(p2.x-p1.x,p2.y-p1.y)
def getOrientationFromVector(v):
    #TODO: return atan thing
    return math.atan2(v.y,v.x)
def dotProduct(v1,v2):
    return v1.x*v2.x + v1.y*v2.y
def randomPosition():
    return Position(rand.randint(MINX,MAXX),rand.randint(MINY,MAXY))

# log dbug
def dblog(strn, DEBUG):
        if DEBUG:
                print (strn)
                
