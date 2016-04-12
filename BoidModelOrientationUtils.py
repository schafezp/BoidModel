import math
from BoidModelOrientation import Position, Boid
## class Position(object):
##     def __init__(self,x,y,isNull=False):
##         self.x = x
##         self.y = y
##         self.isNull = isNull
def getCenterPoint(boidArray):
        totalX = 0
        totalY = 0
        count = len(boidArray)
        if count == 0:
            #TODO: FIX THIS
            return Position(0,0)
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

def transform(xypair,orientation,center):
    x = xypair[0]
    y = xypair[1]
    xstart = x
    ystart = y
    x = xstart*math.cos(orientation) - ystart*math.sin(orientation)
    y = xstart*math.sin(orientation) + ystart*math.cos(orientation)
    #print "x changed %f y changed %f" %(xstart-x,ystart-y)
    return (x+center[0],y+center[1])
