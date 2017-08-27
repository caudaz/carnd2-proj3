### UDACITY Lesson 14 -implementation of Particle filter

# Transform observations into the map's C.S. 
# from the point of view of the particle

import math

def transfTrans(xp,yp,theta,xobs,yobs):
    xm = xp + math.cos(theta)*xobs + math.sin(theta)*yobs
    ym = yp - math.sin(theta)*xobs + math.cos(theta)*yobs
    return xm, ym

print("TRANSFORMED OBSERVATIONS INTO GLOBAL SPACE, SEEN FROM PARTICLE \n")

#OBS1
xtobs1,ytobs1=transfTrans(4,5,+math.pi/2,2,2)
print("TOBS1=",xtobs1,ytobs1)

#OBS2
xtobs2,ytobs2=transfTrans(4,5,+math.pi/2,3,-2)
print("TOBS2=",xtobs2,ytobs2)

#OBS3
xtobs3,ytobs3=transfTrans(4,5,+math.pi/2,0,-4)
print("TOBS3=",xtobs3,ytobs3)

print("\n")

L = [[5,3],[2,1],[6,1],[7,4],[4,7]]

print("TOBS1 distance to Landmarks(smallest value is the correspondent landmark")
for loc in L:
    dist = math.sqrt((loc[0]-xtobs1)**2+(loc[1]-ytobs1)**2)
    print(dist)

print("\n")
print("TOBS2 distance to Landmarks(smallest value is the correspondent landmark")
for loc in L:
    dist = math.sqrt((loc[0]-xtobs2)**2+(loc[1]-ytobs2)**2)
    print(dist)

print("\n")
print("TOBS2 distance to Landmarks(smallest value is the correspondent landmark")
for loc in L:
    dist = math.sqrt((loc[0]-xtobs3)**2+(loc[1]-ytobs3)**2)
    print(dist)


print("\n")    
    
def P(x,y,sx,sy,ux,uy):
    P=1/(2*math.pi*sx*sy) * math.exp(- ( ((x-ux)**2)/(2*sx**2) + ((y-uy)**2)/(2*sy**2)))
    return P

Pobs1 = P(xtobs1,ytobs1,0.3,0.3,L[0][0],L[0][1])
print("Pobs1=",Pobs1)
    
Pobs2 = P(xtobs2,ytobs2,0.3,0.3,L[1][0],L[1][1])
print("Pobs2=",Pobs2)

Pobs3 = P(xtobs3,ytobs3,0.3,0.3,L[1][0],L[1][1])
print("Pobs3=",Pobs3)

Part_final_weight = Pobs1 * Pobs2 * Pobs3
print("Part_final_weight = Pobs1 * Pobs2 * Pobs3 = ",Part_final_weight)