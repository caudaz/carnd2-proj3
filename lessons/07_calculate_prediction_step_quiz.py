import math

def xf(x0,v,theta0,thetadot,dt):
    xf = x0 + v/thetadot * (math.sin(theta0+thetadot*dt)-math.sin(theta0))
    return xf

def yf(y0,v,theta0,thetadot,dt):
    yf = y0 + v/thetadot * (math.cos(theta0) - math.cos(theta0+thetadot*dt))
    return yf

def thetaf(theta0,thetadot,dt):
    thetaf = theta0 + thetadot * dt
    return thetaf

x0 = 102.0
y0 = 65.0
theta0= 5*math.pi/8

v=110
thetadot = math.pi/8
dt = 0.1


xf = xf(x0,v,theta0,thetadot,dt)

yf = yf(y0,v,theta0,thetadot,dt)

thetaf = thetaf(theta0,thetadot,dt)