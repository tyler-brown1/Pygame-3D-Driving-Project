"""
3d Driving Game
Tyler Brown 6/1/2023
Incorporates physics and graphics 
"""

import pygame
from math import *
import time

WIDTH,HEIGHT = 600,600
WIN = pygame.display.set_mode((WIDTH,HEIGHT))

pygame.display.set_caption("racing")

black = (0,0,0)
blue = (50,50,255)
red = (230,60,50)
green = (50,255,50)
grey = (200,200,200)
white = (255,255,255)
purple = (255,0,255)
pink = (200,60,60)
pinky = (190,80,80)
darkred = (150,20,20)
redtop = (200,30,30)
dullred = (160,60,60)
window = (170,170,170)
window2 = (155,155,155)
road = (60,60,60)
water = (20,200,200)
boost = (255,150,0)
jump = (255,0,200)
slip = (255,255,0)

fov = 100 * pi/180 #field of view

px,py,pz = 2,0,.5 # player
cx,cy,cz = 0,0,0 # camera

velocity = 0
angularvel = 0
acceleration,angularacc = 2,4
deaccel =.998
yv = 0 # yvelocity
g = 9.8 # gravity
jumpv = 8 #jump velocity
fp = 0 #first person
slippy = .5# amount slipped
lastslipped = 0 # time since last slipped
chunkrad = 3

yaw = 0 #xz rotation
rad = WIDTH/1.3 #stretch screen by
cameraRoll = 0

FPS = 75

lines = []
triangles = []
t,w1,w2,w3,w4 = [],[],[],[],[] # car + wheels
r = 0 # reverse
track = dict() # dict of track

skid = [] #skid mark list

chunks = dict()


cameraDist = 2.5 # default camera distance to player
closer  =0 # get closer to player when accelerating

cameraYaw,playerYaw,cameraPitch,playerPitch = 0,0,-.4,0


def draw_window():
    WIN.fill(grey)
    plotLines()
    plotTriangles()
    drawChar()
    pygame.display.update()

def drawChar(): # draw character
    global velocity,yv,angularvel,lastslipped
    if not fp:
        p = (px,py,pz)

        for item in t:
            Triangle(playRotate(item[0]),playRotate(item[1]),playRotate(item[2]),item[3],fill=True).plot()
        
        if r == 0:
            if cameraYaw>-.12:
                for item in w1:
                    Triangle(playRotate(item[0]),playRotate(item[1]),playRotate(item[2]),item[3],fill=True).plot()
            if cameraYaw>-.08:
                for item in w2:
                    Triangle(playRotate(add(item[0],[-.01,0,0])),playRotate(add(item[1],[-.01,0,0])),playRotate(add(item[2],[-.01,0,0])),item[3],fill=True).plot()
            if cameraYaw<.12:
                for item in w3:
                    Triangle(playRotate(item[0]),playRotate(item[1]),playRotate(item[2]),item[3],fill=True).plot()
            if cameraYaw<.08:
                for item in w4:
                    Triangle(playRotate(add(item[0],[.01,0,0])),playRotate(add(item[1],[.01,0,0])),playRotate(add(item[2],[.01,0,0])),item[3],fill=True).plot()
        else:
            if cameraYaw<.12:
                for item in w1:
                    Triangle(playRotate(item[0]),playRotate(item[1]),playRotate(item[2]),item[3],fill=True).plot()
            if cameraYaw<.08:
                for item in w2:
                    Triangle(playRotate(add(item[0],[-.01,0,0])),playRotate(add(item[1],[-.01,0,0])),playRotate(add(item[2],[-.01,0,0])),item[3],fill=True).plot()
            if cameraYaw>-.12:
                for item in w3:
                    Triangle(playRotate(item[0]),playRotate(item[1]),playRotate(item[2]),item[3],fill=True).plot()
            if cameraYaw>-.06:
                for item in w4:

                    Triangle(playRotate(add(item[0],[.01,0,0])),playRotate(add(item[1],[.01,0,0])),playRotate(add(item[2],[.01,0,0])),item[3],fill=True).plot()

    bl = getSurface(playRotate((-.1,0,-.35),False))
    br = getSurface(playRotate((.1,0,-.35),False))
    tr = getSurface(playRotate((.1,0,0),False))
    tl = getSurface(playRotate((-.1,0,0),False))

    if py == 0:
        if bl == 0:
            velocity *= .994
        if br == 0:
            velocity *= .994
        if tl == 0:
            velocity *= .994
        if tr == 0:
            velocity *= .994
        
        if bl == 2:
            velocity += acceleration/FPS
        if br == 2:
            velocity += acceleration/FPS
        if tl == 2:
            velocity += acceleration/FPS
        if tr == 2:
            velocity += acceleration/FPS

        if (bl == 3) or (br == 3) or (tr == 3) or (tl == 3):
            yv=jumpv
        if (bl == 4) or (br == 4) or (tr == 4) or (tl == 4):
            if time.time()-lastslipped>3.5:
                angularvel+= slippy*sign(cos(playerYaw))*max(1,velocity)
                lastslipped = time.time()
                velocity *= .6



def getSurface(A): # get surface of track below player
    try:
        a = track[(floor(A[0]),floor(A[2]))] 
        if (a == 4):
            if (.35<A[0]%1<.65) and (.35<A[2]%1<.65): 
                return 4
            else:
                return 1
        else:
            return a
    except(KeyError):
        return -1

    

def playRotate(A,sub = True): # rotate the space around the character
    if sub:
        A = subtract(A,[0,0,.55])
    return add((px,py,pz),[A[0]*cos(playerYaw)+A[2]*sin(playerYaw),A[1],-A[0]*sin(playerYaw)+A[2]*cos(playerYaw)])


def plotLines(): # plot every line
    global skid
    for i in range(-chunkrad//2,chunkrad//2+1):
        for j in range(-chunkrad//2,chunkrad//2+1):
            for line in chunks[(floor((cx+5)/10)+i,floor((cz+5)/10)+j)]:
                v1 = rotate(line[0])
                v2 = rotate(line[1])
                if inView(v1) and inView(v2):
                    pygame.draw.line(WIN,line[2],proj(v1),proj(v2),line[3])

    new_skid = []
    skidtime = time.time()
    for line in skid:
        v1 = rotate(line[0])
        v2 = rotate(line[1])

        if inView(v1) and inView(v2):
            pygame.draw.line(WIN,line[2],proj(v1),proj(v2),line[3])
        if skidtime -line[4]< 4: new_skid.append(line)
    skid = new_skid


def plotTriangles(): # plot every triangle
    for triangle in triangles:
        triangle.plot()

class Triangle(): #triangle class graphics

    def __init__(self,v1,v2,v3,color,width = 1,cull = True,l1=True,l2=True,l3=True,normal = True,fill = False,place = triangles):
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.color = color
        self.width = width
        self.cull = cull
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.normal = normal
        self.fill = fill
        self.place = place


    def create(self):
        self.place.append(self)

    def plot(self):

        vert1 = rotate(self.v1)
        vert2 = rotate(self.v2)
        vert3 = rotate(self.v3)
        
        if dist(self.v1,(cx,cy,cz))< 10*chunkrad/2:
            if inView(vert1) and inView(vert2) and inView(vert3):
                if not self.cull or (dot(subtract(self.v1,(cx,cy,cz)),cross(subtract(self.v2,self.v1),subtract(self.v3,self.v1)))>=0) == self.normal: # backface culling
                    if not self.fill:
                        if self.l1:
                            pygame.draw.line(WIN,self.color,proj(vert1),proj(vert2),self.width)
                        if self.l2:
                            pygame.draw.line(WIN,self.color,proj(vert2),proj(vert3),self.width)
                        if self.l3:
                            pygame.draw.line(WIN,self.color,proj(vert3),proj(vert1),self.width)
                    else:
                            pygame.draw.polygon(WIN,self.color,[proj(vert2),proj(vert3),proj(vert1)])


def init_stuff(): #initalizations for car and map
    global t,w1,w2,w3,w4,track,chunks
    v1 = (-.08,.06,.03) # points for car
    v2 = (.08,.06,.03)
    v3 = (-.06,.02,.053)
    v4 = (.06,.02,.05)
    v5 = (.1,0,.2)
    v6 = (.08,.2,.2)
    v7 = (-.08,.2,.2)
    v8 = (-.1,0,.2)
    v9 = (-.1,.01,.2)
    v10 = (-.08,.23,.45)
    v11 = (-.08,.01,.45)
    v12 = (.1,.01,.2)
    v13 = (.08,.23,.45)
    v14 = (.08,.01,.45)
    v15 = (-.08,.01,.45)
    v16 = (-.08,.14,.51)
    v17 = (.08,0,.45)
    v18 = (.08,.14,.51)
    v19 = (.07,.11,.035)
    v20 = (-.07,.11,.035)
    v21 = (-.08,.12,.65)
    v22 = (-.08,.05,.65)
    v23 = (.08,.12,.65)
    v24 = (.08,.05,.65)
    
    t = [[v11,v16,v21,pinky],[v22,v11,v21,pinky],[v18,v14,v23,pinky],[v14,v24,v23,pinky],[v18,v23,v21,red],[v16,v18,v21,red],[v2,v1,v3,darkred],[v3,v4,v2,darkred],[v2,v4,v5,pink],[v5,v6,v2,pink],[v1,v7,v8,pink],[v8,v3,v1,pink],[v1,v19,v20,dullred],[v1,v2,v19,dullred],[v1,v20,v7,pinky],[v2,v6,v19,pinky],[v19,v6,v7,red],[v19,v7,v20,red],[v9,v7,v10,pink],[v11,v9,v10,pinky],[v6,v12,v13,pink],[v12,v14,v13,pinky]]
    t += [[v6,v13,v10,redtop],[v7,v6,v10,redtop],[v15,v10,v16,pink],[v13,v17,v18,pink]]
    t+= [[v24,v22,v21,darkred],[v23,v24,v21,darkred],[v13,v18,v10,window],[v18,v16,v10,window2]]
    w1 = [[(-.12,.08,.2),(-.12,.05,.23),(-.12,0,.24),black],[(-.12,0,.16),(-.1,0,.16),(-.1,.08,.20),black],[(-.12,.08,.20),(-.12,0,.16),(-.1,.08,.20),black],[(-.12,0,.24),(-.12,0,.16),(-.12,.08,.20),black],[(-.12,0,.16),(-.12,.05,.17),(-.12,.08,.2),black],[(-.1,0,.24),(-.12,0,.24),(-.1,.08,.20),black],[(-.12,0,.24),(-.12,.08,.20),(-.1,.08,.20),black]]
    w2 = [[(-.1,.08,.45),(-.1,.05,.48),(-.1,0,.49),black],[(-.1,0,.41),(-.08,0,.41),(-.08,.08,.45),black],[(-.1,.08,.45),(-.1,0,.41),(-.08,.08,.45),black],[(-.1,0,.49),(-.1,0,.41),(-.1,.08,.45),black],[(-.1,0,.41),(-.1,.05,.42),(-.1,.08,.45),black],[(-.08,0,.49),(-.1,0,.49),(-.08,.08,.45),black],[(-.1,0,.49),(-.1,.08,.45),(-.08,.08,.45),black]]
    w3 = [[(.12,.05,.23),(.12,.08,.2),(.12,0,.24),black],[(.1,0,.16),(.12,0,.16),(.1,.08,.20),black],[(.12,0,.16),(.12,.08,.20),(.1,.08,.20),black],[(.12,0,.16),(.12,0,.24),(.12,.08,.20),black],[(.12,.05,.17),(.12,0,.16),(.12,.08,.2),black],[(.12,0,.24),(.1,0,.24),(.1,.08,.20),black],[(.12,.08,.20),(.12,0,.24),(.1,.08,.20),black]]
    w4 = [[(.1,.05,.48),(.1,.08,.45),(.1,0,.49),black],[(.08,0,.41),(.1,0,.41),(.08,.08,.45),black],[(.1,0,.41),(.1,.08,.45),(.08,.08,.45),black],[(.1,0,.41),(.1,0,.49),(.1,.08,.45),black],[(.1,.05,.42),(.1,0,.41),(.1,.08,.45),black],[(.1,0,.49),(.08,0,.49),(.08,.08,.45),black],[(.1,.08,.45),(.1,0,.49),(.08,.08,.45),black]]

    roadx = [0,1,0,1,0,1,1,2,2,3,3,4,7,10,13,14,13,16,19,21,16,18,20,21,20,21,21,22,23,24,26,28,23,24,26,28,31,31,34,34,37,37,40,40,43,43,46,45,46,47,48,49,47,48,49,50,53,54,57,58,61,62,63,64,65,65,66,62,63,64,65,66,66,67,67,67,67,67,67,66,66,65,62,59,65,56,54,53,52,51,50,49,46,47,43,40,37,34,31,28,25,22,21,21,20,20,19,18,16,13,10,9,6,3,2,1,1]
    roady = [0,0,3,3,6,6,9,9,12,12,13,14,14,14,14,15,15,15,16,17,16,17,18,18,21,21,22,25,28,30,31,32,25,28,30,31,31,32,31,32,31,32,31,32,31,32,31,30,29,28,27,26,29,28,27,26,26,25,25,24,24,23,22,21,19,17,14,24,23,22,21,11,15,8,9,6,3,-3,-6,-9,-12,-14,-15,-16,-13,-17,-18,-19,-21,-23,-24,-25,-26,-26,-26,-26,-27,-27,-28,-28,-27,-26,-24,-21,-18,-15,-12,-11,-10,-10,-10,-9,-9,-8,-7,-4,-1]

    for i in range(-5,75):
        for j in range(-35,40):
            track[(i,j)] = 0
    
    for i in range(len(roadx)):
        for j in range(3):
            for k in range(3):
                track[(roadx[i]+j,roady[i]+k)] = 1

    specials = dict()

    #2 speed
    #3 jump
    #4 banana
    specials[(67,2)] = 3
    specials[(68,2)] = 3
    specials[(69,2)] = 3
    specials[(67,3)] = 2
    specials[(68,3)] = 2
    specials[(69,3)] = 2
    specials[(33,33)] = 4
    specials[(3,3)] = 4
    specials[(30,-26)] = 2
    specials[(56,26)] = 2
    specials[(19,-9)] = 4

    for i,j in track.items():
        addSurface(i,j)
    for i,j in specials.items():
        track[(i[0],i[1])] = j
        addSurface(i,j)

    for i in range(30):
        for j in range(30):
            chunks[(i-10,j-15)] = []

    for line in lines:
        chunks[(floor(line[0][0]/10),floor(line[0][2]/10))].append(line)

def addSurface(i,j): # add a tile
        if j == 0: # water
            lines.append([(i[0]+1,-.05,i[1]),(i[0],-.05,i[1]+1),water,1])
            
        if j == 1: #road
            lines.append([(i[0],0,i[1]),(i[0]+1,0,i[1]),road,2])
            lines.append([(i[0]+1,0,i[1]),(i[0]+1,0,i[1]+1),road,2])
            lines.append([(i[0]+1,0,i[1]+1),(i[0],0,i[1]+1),road,2])
            lines.append([(i[0],0,i[1]+1),(i[0],0,i[1]),road,2])
            lines.append([(i[0],0,i[1]+.5),(i[0]+.5,0,i[1]+1),road,1])
            lines.append([(i[0],0,i[1]),(i[0]+1,0,i[1]+1),road,1])
            lines.append([(i[0]+.5,0,i[1]),(i[0]+1,0,i[1]+.5),road,1])
            lines.append([(i[0],0,i[1]+.25),(i[0]+.75,0,i[1]+1),road,1])
            lines.append([(i[0]+.25,0,i[1]),(i[0]+1,0,i[1]+.75),road,1])
            lines.append([(i[0],0,i[1]+.75),(i[0]+.25,0,i[1]+1),road,1])
            lines.append([(i[0]+.75,0,i[1]),(i[0]+1,0,i[1]+.25),road,1])
        if j == 2: # boost pad
            lines.append([(i[0],0,i[1]),(i[0]+1,0,i[1]+1),boost,2])
            lines.append([(i[0]+1,0,i[1]),(i[0],0,i[1]+1),boost,2])
        if j == 3: #jump pad
            lines.append([(i[0]+1/3,0,i[1]),(i[0]+1/3,0,i[1]+1),jump,2])
            lines.append([(i[0]+2/3,0,i[1]),(i[0]+2/3,0,i[1]+1),jump,2])
            lines.append([(i[0],0,i[1]+1/3),(i[0]+1,0,i[1]+1/3),jump,2])
            lines.append([(i[0],0,i[1]+2/3),(i[0]+1,0,i[1]+2/3),jump,2])
        if j == 4: #banana
            lines.append([(i[0]+.5,.3,i[1]+.5),(i[0]+.5,.35,i[1]+.5),black,3])

            Triangle((i[0]+.5,0,i[1]+.3),(i[0]+.55,.05,i[1]+.45),(i[0]+.45,.05,i[1]+.45),slip,4,l2 = False,fill = True).create()
            Triangle((i[0]+.5,0,i[1]+.7),(i[0]+.55,.05,i[1]+.55),(i[0]+.45,.05,i[1]+.55),slip,4,l2 = False,normal = False,fill = True).create()
            Triangle((i[0]+.3,0,i[1]+.5),(i[0]+.45,.05,i[1]+.45),(i[0]+.45,.05,i[1]+.55),slip,4,l2 = False,fill = True).create()
            Triangle((i[0]+.7,0,i[1]+.5),(i[0]+.55,.05,i[1]+.45),(i[0]+.55,.05,i[1]+.55),slip,4,l2 = False,normal = False,fill = True).create()
            Triangle((i[0]+.5,.3,i[1]+.5),(i[0]+.55,.05,i[1]+.45),(i[0]+.45,.05,i[1]+.45),slip,4,l2 = False,normal = False,fill = True).create()
            Triangle((i[0]+.5,.3,i[1]+.5),(i[0]+.55,.05,i[1]+.55),(i[0]+.45,.05,i[1]+.55),slip,4,l2 = False,fill = True).create()
            Triangle((i[0]+.5,.3,i[1]+.5),(i[0]+.45,.05,i[1]+.45),(i[0]+.45,.05,i[1]+.55),slip,4,l2 = False,normal = False,fill = True).create()
            Triangle((i[0]+.5,.3,i[1]+.5),(i[0]+.55,.05,i[1]+.45),(i[0]+.55,.05,i[1]+.55),slip,4,l2 = False,fill = True).create()



def updateCamera(): # update position of camera every frame to rotate around player
    global cx,cy,cz,yaw,pitch,r,fp
    keys = pygame.key.get_pressed()
    r = 0
    yaw = cameraYaw+playerYaw
    if keys[pygame.K_SPACE]: 
        yaw = yaw + pi
        r = -1
    pitch = cameraPitch+playerPitch
    if fp == 0:
        cx = px-(cameraDist-closer) * sin(yaw)*cos(pitch)
        cy = py-(cameraDist-closer) *sin(pitch)
        cz = pz-(cameraDist-closer) *cos(yaw)*cos(pitch)
    else:
        cx,cy,cz = px+.35*sin(yaw),py+.5,pz+.35*cos(yaw)

    if keys[pygame.K_f]: 
        fp = 1
    if keys[pygame.K_r]:
        fp = 0


def proj(A): # project 2d to 3d
    return (WIDTH/2+rad*A[0]/(A[2]*tan(fov/2)),HEIGHT/2-rad*A[1]/(A[2]*tan(fov/2)))

def rotate(A): # perform all 3d rotations
    return rollRotate(pitchRotate(yawRotate(translate(A))))

def inView(A): # decide to render or not
    return A[2] > 0

def yawRotate(A): # rotate around xz axis (spin car around)
    return (A[0]*cos(yaw)-A[2]*sin(yaw),A[1],A[0]*sin(yaw)+A[2]*cos(yaw))

def pitchRotate(A): # rotate around yz axis (look up)
    return (A[0],A[1]*cos(pitch)-A[2]*sin(pitch),A[1]*sin(pitch)+A[2]*cos(pitch))

def rollRotate(A): # rotaate around xy axis (turn head to side)
    return (A[0]*cos(cameraRoll)-A[1]*sin(cameraRoll),A[0]*sin(cameraRoll)+A[1]*cos(cameraRoll),A[2])

def translate(A): # translate point in respect to player
    return subtract(A,(cx,cy,cz))

def mult(A,a): # multiply a vector by a scalar
    return (A[0]*a,A[1]*a,A[2]*a)

def add(A,B): # add two vectors
    return (A[0]+B[0],A[1]+B[1],A[2]+B[2])
 
def subtract(A,B): # subtract two vectors
    return (A[0]-B[0],A[1]-B[1],A[2]-B[2])

def norm(A): # norm of a vector
    return sqrt(A[0]**2+A[1]**2+A[2]**2)

def dot (X,Y): # dot product of two vectors
    return (X[0]*Y[0]+X[1]*Y[1]+X[2]*Y[2])

def cross(A,B): # cross product of two vectors
    return(A[1]*B[2]-A[2]*B[1],A[2]*B[0]-A[0]*B[2],A[0]*B[1]-A[1]*B[0])


def sign(x): #sign of a value
    if x>0: return 1
    if x<0: return -1
    return 0

def ctrls(): # inputs from arrow keys to move camera and player
    global fov,px,pz,py,cameraYaw,cameraPitch,velocity,angularvel,playerPitch,playerYaw,closer,yv,cameraRoll

    keys_pressed = pygame.key.get_pressed()

    if keys_pressed[pygame.K_a]:
        angularvel -= angularacc/FPS
        cameraYaw -= .015
        cameraRoll += velocity/2500
        
    if keys_pressed[pygame.K_d]:
        angularvel += angularacc/FPS
        cameraYaw += .015
        cameraRoll -= velocity/2500

    if keys_pressed[pygame.K_w]:
        if py==0:
            velocity += acceleration/FPS
            skid.append([(px-.1*cos(playerYaw),py,pz+.1*sin(playerYaw)),(px-.07*cos(playerYaw),py,pz+.07*sin(playerYaw)),black,1,time.time()])
            skid.append([(px+.1*cos(playerYaw),py,pz-.1*sin(playerYaw)),(px+.07*cos(playerYaw),py,pz-.07*sin(playerYaw)),black,1,time.time()])
            closer += 1.1/FPS


    if keys_pressed[pygame.K_s]:
        velocity -= acceleration/FPS
    
    if keys_pressed[pygame.K_t]:
        py += .01

    if keys_pressed[pygame.K_g]:
        py -= .01


    if keys_pressed[pygame.K_RIGHT]:
        cameraYaw += .02
        
    if keys_pressed[pygame.K_LEFT]:
        cameraYaw -= .02

    if keys_pressed[pygame.K_UP]:
        cameraPitch += .01

    if keys_pressed[pygame.K_DOWN]:
        cameraPitch -= .01
    
    if cameraPitch>-.2:
        cameraPitch = -.2
    if cameraPitch<-.9:
        cameraPitch = -.9
    
    if cameraYaw> .8:
        cameraYaw = .8
    if cameraYaw<-.8:
        cameraYaw = -.8

    playerYaw += angularvel/FPS
    angularvel *= .97
    closer *= .985
    
    px += velocity*sin(playerYaw)/FPS
    pz += velocity*cos(playerYaw)/FPS

    velocity *= deaccel

    cameraYaw *= .98
    cameraRoll *= .98

    py += yv/FPS
    py = max(py,0)
    if py>0:
        yv -= g/FPS
    else:
        yv = 0
    
    
    if keys_pressed[pygame.K_1]:
        fov = 60 * pi/180
    if keys_pressed[pygame.K_2]:
        fov = 80 * pi/180
    if keys_pressed[pygame.K_3]:
        fov = 90 * pi/180
    if keys_pressed[pygame.K_4]:
        fov = 100 * pi/180
    if keys_pressed[pygame.K_5]:
        fov = 120 * pi/180
    if keys_pressed[pygame.K_6]:
        fov = 130 * pi/180


def main():
    ct = 0
    start = time.time()
    clock = pygame.time.Clock()
    init_stuff()

    run = True
    while run:
        
        clock.tick(FPS)
        ct+=1
        
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                run = False
        
        updateCamera()
        draw_window()
        ctrls()

    pygame.quit()

if __name__ == "__main__":
    main()

