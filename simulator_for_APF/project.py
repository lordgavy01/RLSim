import numpy as np
from math import *
import pygame
import cv2
from collections import defaultdict
import heapq

np.random.seed(1)
pygame.init()
class Background(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)  #call Sprite initializer
        self.image = pygame.image.load(image_file)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location

image_name='geomap.png'
Background=Background(image_name,[0,0])
length=50
breadth=30
# set title of the window
pygame.display.set_caption("Reactive Multi-Robot Navigation")

Sources=[]
Goals=[]
FGoals=[]
Paths=[]

def Euclidean(A,B,wt=1):
  return sqrt((A[0]-B[0])**2+(A[1]-B[1])**2+wt*min((A[2]-B[2])**2,(2*pi-abs(A[2]-B[2]))**2))
 
def get_corners(theta,X,Y): 
    l=length/2
    b=breadth/2
    C1=(X+l*cos(theta)+b*cos(pi/2+theta),Y+l*sin(theta)+b*sin(pi/2+theta))
    C2=(X-l*cos(theta)+b*cos(pi/2+theta),Y-l*sin(theta)+b*sin(pi/2+theta))
    C3=(X-l*cos(theta)-b*cos(pi/2+theta),Y-l*sin(theta)-b*sin(pi/2+theta))
    C4=(X+l*cos(theta)-b*cos(pi/2+theta),Y+l*sin(theta)-b*sin(pi/2+theta))
    return C1,C2,C3,C4

Polygons=[]
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

StaticGrid=[[1]*Background.image.get_width() for _ in range(Background.image.get_height())]
for i in range(Background.image.get_height()):
    for j in range(Background.image.get_width()):
        if Background.image.get_at((j,i))[0]==0:
            StaticGrid[i][j]=0

def line_intersect(Points):
    Line1=Points[:2]
    Line2=Points[2:]
    dx0 = Line1[1][0]-Line1[0][0]
    dx1 = Line2[1][0]-Line2[0][0]
    dy0 = Line1[1][1]-Line1[0][1]
    dy1 = Line2[1][1]-Line2[0][1]
    p0 = dy1*(Line2[1][0]-Line1[0][0]) - dx1*(Line2[1][1]-Line1[0][1])
    p1 = dy1*(Line2[1][0]-Line1[1][0]) - dx1*(Line2[1][1]-Line1[1][1])
    p2 = dy0*(Line1[1][0]-Line2[0][0]) - dx0*(Line1[1][1]-Line2[0][1])
    p3 = dy0*(Line1[1][0]-Line2[1][0]) - dx0*(Line1[1][1]-Line2[1][1])
    return (p0*p1<=0) & (p2*p3<=0)

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def check(Point, poly):
    center=poly[0]
    PointList=poly[1]
    
    n = len(PointList)
    p1x, p1y = PointList[0]
    for i in range(1,n + 1):
        p2x, p2y = PointList[i % n]
        if ccw(Point, (p1x, p1y), (p2x, p2y))!=ccw(center, (p1x, p1y), (p2x, p2y)):
            return False
        p1x, p1y = p2x, p2y
    return True

def check_inside(Poly1, Poly2):
    flag=1
    for i in range(len(Poly1[1])):
        flag&=check(Poly1[1][i],Poly2) 
    return flag
    
def check_intersect(Poly1, Poly2):
    if check_inside(Poly1, Poly2) or check_inside(Poly2, Poly1):
        return "One is inside the other"

    for i in range(len(Poly1[1])):
        for j in range(len(Poly2[1])):
            Points=[Poly1[1][i],Poly1[1][(i+1)%len(Poly1)],Poly2[1][j],Poly2[1][(j+1)%len(Poly2)]]
            if line_intersect(Points):
                return "Yes"
    return "No"

def dont_intersect(Pol1,Pol2):
    Poly1=Pol1[1]
    Poly2=Pol2[1]
    # Get the bounding box of the two polygons
    x1=min(Poly1,key=lambda x:x[0])[0]
    x2=max(Poly1,key=lambda x:x[0])[0]
    y1=min(Poly1,key=lambda x:x[1])[1]
    y2=max(Poly1,key=lambda x:x[1])[1]
    x3=min(Poly2,key=lambda x:x[0])[0]
    x4=max(Poly2,key=lambda x:x[0])[0]
    y3=min(Poly2,key=lambda x:x[1])[1]
    y4=max(Poly2,key=lambda x:x[1])[1]
    # Check if the bounding boxes overlap
    if x1>x4 or x2<x3 or y1>y4 or y2<y3:
        return True
    return False

def check_safe(theta,X,Y):
    C1,C2,C3,C4=get_corners(theta,X,Y)
    Current=[(X,Y),[C1,C4,C3,C2]]
    if C1[0]>=0 and C1[1]>=0 and C2[0]>=0 and C2[1]>=0 and C3[0]>=0 and C3[1]>=0 and C4[0]>=0 and C4[1]>=0 and C1[0]<Background.image.get_width() and C1[1]<Background.image.get_height() and C2[0]<Background.image.get_width() and C2[1]<Background.image.get_height() and C3[0]<Background.image.get_width() and C3[1]<Background.image.get_height() and C4[0]<Background.image.get_width() and C4[1]<Background.image.get_height():
        if StaticGrid[int(C1[1])][int(C1[0])]==0 or StaticGrid[int(C2[1])][int(C2[0])]==0 or StaticGrid[int(C3[1])][int(C3[0])]==0 or StaticGrid[int(C4[1])][int(C4[0])]==0:
            return False
        for poly in Polygons:
            if dont_intersect(Current,poly):
                continue
            if check_intersect(Current,poly)!="No":
                return False
        
        return True
    return False

def check_feasible(A,B,givePath=False,eps=0.1):
    L=0
    if not givePath:
        eps=1/Euclidean(A,B,0)
    # check if the line segment connecting A and B is feasible
    Flag1,Flag2=True,True
    Pts=[]
    thetas1=[]
    thetas2=[]
    while L<=1:
        # using A and B to find the point Pt
        Pt=(A[0]*L+B[0]*(1-L),A[1]*L+B[1]*(1-L))
        Pts.append(Pt)
        theta1=(A[2]*L+B[2]*(1-L))
        theta2=(A[2]*L+(-2*pi+B[2])*(1-L))
        thetas1.append(theta1)
        thetas2.append(theta2)
        # check if the point is safe
        if Flag1 and check_safe(theta1,Pt[0],Pt[1])==False:
            Flag1=False
        if Flag2 and check_safe(theta2,Pt[0],Pt[1])==False:
            Flag2=False
        L+=eps
    
    if not givePath:
        if Flag1==True or Flag2==True:
            return True

        return False
    else:
        prefer=1
        if Flag1==True and Flag2==True:
            if abs(B[2]-A[2])>abs(2*pi+A[2]-B[2]):
                prefer=2
        elif Flag2==True:
            prefer=2
        Path=[]
        if prefer==1:
            for i in range(len(Pts)):
                Path.append((Pts[i][0],Pts[i][1],thetas1[i]))
        else:
            for i in range(len(Pts)):
                Path.append((Pts[i][0],Pts[i][1],thetas2[i]))
        return Path

def insertObs(image,A,B):
    global Paths
    if A[0]==B[0]:
        R1=list(range(A[1],B[1],8))
        R2=[A[0]]*len(R1)
        Paths.append(list(zip(R2,R1)))
    else:
        R1=list(range(A[0],B[0],8))
        R2=[A[1]]*len(R1)
        Paths.append(list(zip(R1,R2)))
    cv2.line(image,A,B,(0,0,255),2)

def addRobot(Source,Goal):
    Sources.append(Source)
    Goals.append([Goal])
    FGoals.append(Goal)

addRobot((676,519,0),(88,55,0))
addRobot((136,372,0),(739,188,0))
addRobot((62,191,0),(282,542,0))
addRobot((492,542,0),(401,45,0))
# addRobot((677,29,0),(49,540,0))

GoalIndex=[0]*len(Goals)
image=cv2.imread(image_name)
insertObs(image,(362,120),(362,310))
insertObs(image,(186,468),(388,468))
insertObs(image,(540,300),(540,420))

for i in range(len(Sources)):
    S=Sources[i]
    C1,C2,C3,C4=get_corners(0,S[0],S[1])
    col=(0,0,255)
    # write text inside the rectangle formed by C1,C2,C3 and C4 points
    cv2.putText(image,str(i+1),(int(S[0]),int(S[1])),cv2.FONT_HERSHEY_SIMPLEX,0.5,col,2)

    cv2.line(image,(int(C1[0]),int(C1[1])),(int(C4[0]),int(C4[1])),col,2)
    cv2.line(image,(int(C2[0]),int(C2[1])),(int(C3[0]),int(C3[1])),col,2)
    cv2.line(image,(int(C1[0]),int(C1[1])),(int(C2[0]),int(C2[1])),col,2)
    cv2.line(image,(int(C3[0]),int(C3[1])),(int(C4[0]),int(C4[1])),col,2)

for i in range(len(Goals)):
    G=Goals[i]
    D=G[0]
    C1,C2,C3,C4=get_corners(0,D[0],D[1])
    col=(0,255,)
    # write text inside the rectangle formed by C1,C2,C3 and C4 points
    cv2.putText(image,str(i+1),(int(D[0]),int(D[1])),cv2.FONT_HERSHEY_SIMPLEX,0.5,col,2)
    cv2.line(image,(int(C1[0]),int(C1[1])),(int(C4[0]),int(C4[1])),col,2)
    cv2.line(image,(int(C2[0]),int(C2[1])),(int(C3[0]),int(C3[1])),col,2)
    cv2.line(image,(int(C1[0]),int(C1[1])),(int(C2[0]),int(C2[1])),col,2)
    cv2.line(image,(int(C3[0]),int(C3[1])),(int(C4[0]),int(C4[1])),col,2)

cv2.imwrite('map.png',image)
# exit(0)

N=200
configPoints=[]
image2=image.copy()

for i in range(N):
    while 1:
        X=np.random.randint(0,Background.image.get_width())
        Y=np.random.randint(0,Background.image.get_height())
        theta=np.random.uniform(0,2*pi)
        if check_safe(theta,X,Y):
            configPoints.append((X,Y,theta))
            C1,C2,C3,C4=get_corners(theta,X,Y)
            cv2.line(image2,(int(C1[0]),int(C1[1])),(int(C4[0]),int(C4[1])),(255,0,0),2)
            # draw line from C2 to C3
            cv2.line(image2,(int(C2[0]),int(C2[1])),(int(C3[0]),int(C3[1])),(255,0,0),2)
            # draw line from C1 to C2
            cv2.line(image2,(int(C1[0]),int(C1[1])),(int(C2[0]),int(C2[1])),(255,0,0),2)
            # draw line from C3 to C4
            cv2.line(image2,(int(C3[0]),int(C3[1])),(int(C4[0]),int(C4[1])),(255,0,0),2)
            break

for S in Sources:
    configPoints.append((S[0],S[1],0))
for G in Goals:
    D=G[0]
    configPoints.append((D[0],D[1],0))

cv2.imwrite('RoadMap.png',image2)

adjMatrix=[[0]*500 for _ in range(500)]
image3=image2.copy()

k=10
for i in range(0,len(configPoints)):
    lst=[]
    for j in range(0,len(configPoints)):
        if i!=j:
            lst.append((dist(configPoints[i],configPoints[j]),j,configPoints[j]))

    lst.sort(key=lambda x:x[0])
    for j in range(0,k):
        if check_feasible(configPoints[i],lst[j][2]):
            adjMatrix[i][lst[j][1]]=adjMatrix[lst[j][1]][i]=1
            cv2.line(image3,(int(configPoints[i][0]),int(configPoints[i][1])),(int(lst[j][2][0]),int(lst[j][2][1])),(255,0,255),1)

cv2.imwrite('RoadmapNetwork.png',image3)

def PRMPath(S,G):
    global adjMatrix
    if S not in configPoints:
        configPoints.append(S)
    if G not in configPoints:
        configPoints.append(G)
    
    SIndex=configPoints.index(S)
    GIndex=configPoints.index(G)
    lst1=[]
    lst2=[]
    for j in range(0,len(configPoints)):
        lst1.append((dist(S,configPoints[j]),j,configPoints[j]))
        lst2.append((dist(G,configPoints[j]),j,configPoints[j]))
    lst1.sort(key=lambda x:x[0])
    lst2.sort(key=lambda x:x[0])
    for j in range(0,k):
        if S!=lst1[j][2] and check_feasible(S,lst1[j][2]):
            adjMatrix[SIndex][lst1[j][1]]=adjMatrix[lst1[j][1]][SIndex]=1
        if  G!=lst2[j][2] and check_feasible(G,lst2[j][2]):
            adjMatrix[GIndex][lst2[j][1]]=adjMatrix[lst2[j][1]][GIndex]=1

    
    heap=[]
    n=len(configPoints)
    d=[inf]*n
    par=[-1]*n

    heapq.heappush(heap,(0,S,SIndex))
    d[SIndex]=0
    while len(heap)>0:
        (dis,node,j)=heapq.heappop(heap)
        if j==GIndex:
            print('Path found')
            break
        
        for i in range(0,n):
            if adjMatrix[j][i]:
                if dis+int(Euclidean(node,configPoints[i],50))<d[i]:
                    d[i]=dis+int(Euclidean(node,configPoints[i],50))
                    par[i]=j
                    heapq.heappush(heap,(d[i],configPoints[i],i))

    Path=[]
    Cur=GIndex
    while Cur!=-1:
        Path.append(configPoints[Cur])
        Cur=par[Cur]
    Path.reverse()
    return Path


Bots=Sources.copy()
temp=Paths.copy()
for i in range(len(temp)):
    temp2=temp[i].copy()
    temp[i]=temp2[:-1]
    temp2.reverse()
    temp[i]+=temp2[:-1]

Paths=[]
for i in range(len(Sources)):
    path=PRMPath(Sources[i],Goals[i][GoalIndex[i]])
    Paths.append(path)
Paths+=temp
Index=[0]*len(Paths)


R=32
def checkInside(Pos,C):
    return (dist(Pos,C)<=R)

def collision_checker(ind):
    # ind - Bot ID
    for pt in range(Index[ind],min(Index[ind]+4,len(Paths[ind]))):
        Pos=Paths[ind][pt]
        C1,C2,C3,C4=get_corners(Pos[2],Pos[0],Pos[1])
        Current=[(Pos[0],Pos[1]),[C1,C4,C3,C2]]
        for i in range(len(Paths)):
            if i!=ind:
                if i>=len(Sources):
                    OtherPos=Paths[i][(Index[i]+pt-Index[ind]+len(Paths[i]))%len(Paths[i])]
                    for d in Current[1]:
                        if checkInside(d,OtherPos):
                            return True
                elif Index[i]+pt-Index[ind]<len(Paths[i]):
                    OtherPos=Paths[i][Index[i]+pt-Index[ind]]
                    C1,C2,C3,C4=get_corners(OtherPos[2],OtherPos[0],OtherPos[1])
                    Other=[(OtherPos[0],OtherPos[1]),[C1,C4,C3,C2]]
                    if check_intersect(Current,Other)!="No":
                        return True

    return False

def getSubGoal(ind):
    maxi=-inf
    GG=None
    for i in range(-100,100):
        for j in range(-100,100):
            G=(Bots[ind][0]+i,Bots[ind][1]+j)
            if G[0]>=0 and G[0]<Background.image.get_width() and G[1]>=0 and G[1]<Background.image.get_height():
                if StaticGrid[int(G[1])][int(G[0])]==1:
                    if not check_safe(0,G[0],G[1]):
                        continue
                    cur=inf
                    for k in range(len(Paths)):
                        if k!=ind: 
                            cur=min(cur,dist((Paths[k][Index[k]][0],Paths[k][Index[k]][1]),G))
                    
                    cte=0
                    for h in range(Index[ind],min(Index[ind]+10,len(Paths[ind]))):
                            cte=min(cte,dist((Paths[ind][h][0],Paths[ind][h][1]),G))

                    val=cur*0.8-0.2*dist(G,(Goals[ind][GoalIndex[ind]][0],Goals[ind][GoalIndex[ind]][1]))-0.05*cte
                    if val>maxi:
                        maxi=val
                        GG=G
    return (GG[0],GG[1],0)



running=True
key=0
Background.image=pygame.image.load('map.png')
paused=True
screen=pygame.display.set_mode((Background.image.get_width(),Background.image.get_height()))

while running:
    for events in pygame.event.get():
        if events.type==pygame.QUIT:
            running=False
            break
        if events.type==pygame.KEYDOWN:
            if events.key==pygame.K_SPACE:
                paused^=True
    if paused:
        continue
    key+=1
    screen.blit(Background.image, Background.rect)

    if key%200==0:
        for i in range(len(Paths)):
            if i>=len(Sources):
                Index[i]=(Index[i]+1)%len(Paths[i])
            else:
                if (collision_checker(i)):
                    subG=getSubGoal(i)
                    Paths[i]=PRMPath(Bots[i],subG)
                    Goals[i]=[subG,FGoals[i]]
                    GoalIndex[i]=0
                    Index[i]=0
                    continue
                if Index[i]+1<len(Paths[i]):
                    Index[i]=Index[i]+1
                    Bots[i]=Paths[i][Index[i]]
                elif GoalIndex[i]+1<len(Goals[i]):
                    GoalIndex[i]=GoalIndex[i]+1
                    Paths[i]=PRMPath(Bots[i],Goals[i][GoalIndex[i]])
                    Index[i]=0
    
    for i in range(len(Bots)):
        Bot=Bots[i]
        C1,C2,C3,C4=get_corners(Bot[2],Bot[0],Bot[1])
        pygame.draw.polygon(screen,	(128, 0, 255),[C1,C2,C3,C4])
        # write text inside the polygon
        font = pygame.font.Font(None, 22)
        text = font.render(str(i+1), True, (255, 0, 0))
        textRect = text.get_rect()
        textRect.centerx = Bot[0]
        textRect.centery = Bot[1]
        screen.blit(text, textRect)

    for i in range(len(Paths)):
        if i>=len(Sources):
            pygame.draw.circle(screen,(0,0,255),(Paths[i][Index[i]][0],Paths[i][Index[i]][1]),R)
            continue
        for j in range(Index[i]+1,len(Paths[i])):
            pygame.draw.circle(screen,(255,140,0),(Paths[i][j][0],Paths[i][j][1]),3)

    pygame.display.update()