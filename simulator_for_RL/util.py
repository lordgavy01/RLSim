import numpy as np
from math import *
import pygame
import os

os.environ["SDL_VIDEO_CENTERED"] = "1"
np.random.seed(1)
class Background(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)  #call Sprite initializer
        self.image = pygame.image.load(image_file)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location

INF=1e18

image_name='geomap.png'
mapBackground=Background(image_name,[0,0])
length=50
breadth=30
Polygons=[]
positions=[]
goals=[]
maxDistance=100
indexToAct={}
VMAX=0.5
WMAX=radians(20)
vsamples=[-VMAX,0,VMAX]
wsamples=[-WMAX,-WMAX/2,0,WMAX/2,WMAX]
index=0
for v in vsamples:
    for w in wsamples:
        indexToAct[index]=(v,w)
        index+=1

def addObject(source,goal):
    positions.append(source)
    goals.append(goal)

def initMap():
    # Read list of Polygons from out.txt where one line in that file is list of points of one polygon
    with open('out.txt') as f:
        for line in f:
            Polygon=[]
            lst=[]
            comma=False
            Fir,Sec='',''
            Cent=(-1,-1)
            for p in line:
                if p==' ' or p=='(':
                    continue
                if p==',':
                    comma=True
                    continue
                if p==')':
                    if Cent[0]==-1:
                        Cent=(float(Fir),float(Sec))
                    else:
                        lst.append((float(Fir),float(Sec)))
                    Fir,Sec='',''
                    comma=False
                    continue
                if comma:
                    Sec+=p
                else:
                    Fir+=p              
            if len(lst):
                Polygon=[Cent,lst]
                Polygons.append(Polygon)


def euclidean(A,B):
  return sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)

def getCorners(theta,X,Y): 
    l=length/2
    b=breadth/2
    C1=(X+l*cos(theta)+b*cos(pi/2+theta),Y+l*sin(theta)+b*sin(pi/2+theta))
    C2=(X-l*cos(theta)+b*cos(pi/2+theta),Y-l*sin(theta)+b*sin(pi/2+theta))
    C3=(X-l*cos(theta)-b*cos(pi/2+theta),Y-l*sin(theta)-b*sin(pi/2+theta))
    C4=(X+l*cos(theta)-b*cos(pi/2+theta),Y+l*sin(theta)-b*sin(pi/2+theta))
    return C1,C2,C3,C4

def normalAngle(theta):
  return atan2(sin(theta),cos(theta))

def getLinesegmentsIntersection(line1,line2):
    ((x1,y1),(x2,y2))=line1
    ((x3,y3),(x4,y4))=line2
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if denom == 0: # parallel
        return False
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
    if ua < 0 or ua > 1: # out of range
        return False
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
    if ub < 0 or ub > 1: # out of range
        return False
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x,y)

def kinematic_equation(oldPose,v,w,dT):
    xOld,yOld,thetaOld=oldPose
    if w==0:
        xNew=xOld+v*dT*cos(thetaOld)
        yNew=yOld+v*dT*sin(thetaOld)
        thetaNew=thetaOld
        return (xNew,yNew,thetaNew)
    dTheta=w*dT
    L=v/w
    ICCx=xOld-L*sin(thetaOld)
    ICCy=yOld+L*cos(thetaOld)
    xNew=(cos(dTheta)*(xOld-ICCx))-(sin(dTheta)*(yOld-ICCy))+ICCx
    yNew=(sin(dTheta)*(xOld-ICCx))+(cos(dTheta)*(yOld-ICCy))+ICCy
    thetaNew=thetaOld+dTheta
    thetaNew=atan2(sin(thetaNew),cos(thetaNew))
    return (xNew,yNew,thetaNew)

def addForces(F1,F2):
    magnitude1,theta1=F1
    magnitude2,theta2=F2
    x_comp=magnitude1*cos(theta1)+magnitude2*cos(theta2)
    y_comp=magnitude1*sin(theta1)+magnitude2*sin(theta2)
    magnitudeRes=sqrt(x_comp**2+y_comp**2)
    thetaRes=normalAngle(atan2(y_comp,x_comp))
    return (magnitudeRes,thetaRes)

def getLinesegmentCircleIntersection(line1,circle):
    p1,p2=line1
    center,radius=circle
    
    Q=pygame.math.Vector2(center[0],center[1])
    r=radius
    P1=pygame.math.Vector2(p1[0],p1[1])
    V=pygame.math.Vector2(p2[0]-p1[0],p2[1]-p1[1])
    a = V.dot(V)
    b = 2 * V.dot(P1 - Q)
    c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2
    disc = b**2 - 4 * a * c
    if disc < 0:
        return False
    sqrt_disc = sqrt(disc)
    t1 = (-b + sqrt_disc) / (2 * a)
    t2 = (-b - sqrt_disc) / (2 * a)
    if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
        return False
    inter_p1=P1+t1*V
    inter_p2=P1+t2*V
    if euclidean(p1,inter_p1)<euclidean(p1,inter_p2):
        return inter_p1
    return inter_p2
