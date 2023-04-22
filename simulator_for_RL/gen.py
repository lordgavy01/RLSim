from time import sleep
import pygame
import math
# make pygame window with the functionality to draw heaxagons wherever the mouse is clicked
# make a function that draws a hexagon at the mouse position
pygame.init()

screen=pygame.display.set_mode((800,600))
running=True


Poly=[]
def draw_hexagon(screen,x,y,side_length):
    global Poly
    col=(0,0,0)

    # get list of points of hexagon whose centre is at (x,y)
    points=[]
    angle=math.pi/12
    for i in range(6):
        angle+=math.pi/3
        points.append((x+side_length*math.cos(angle),y+side_length*math.sin(angle)))
    # draw the hexagon
    pygame.draw.polygon(screen,col,points)
    Poly.append(((x,y),points))

green=(0,255,0)
flag=0
font = pygame.font.Font('freesansbold.ttf', 32)
screen.fill((255,255,255))
while running:

    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
        if event.type==pygame.MOUSEBUTTONDOWN:
            pos=pygame.mouse.get_pos()
            x,y=pos
            side_length=20
            print(pos)
            draw_hexagon(screen,x,y,side_length)
            # print(Poly[-1])
            # pygame.draw.polygon(screen,(255,0,0),[(x,y),(x+100,y-50),(x+100,y+50),(x,y+100),(x-100,y+50),(x-100,y-50)])
        
    pygame.display.flip()

# n=input()
# if n==7:
#     exit(0)
pygame.image.save(screen,"cir.png")

# write list of points in Poly to file out.txt
with open("out.txt","w") as f:
    for i in Poly:
        f.write(str(i[0])+" ")
        for j in i[1]:
            f.write(str(j)+" ")
        f.write("\n")
    f.write("\n")
print(Poly)