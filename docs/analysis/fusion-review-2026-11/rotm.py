import sys; sys.path.insert(0,'.')
from load import *
def rot(q,v):
    w,x,y,z=q
    # R(q) v
    r00=1-2*(y*y+z*z); r01=2*(x*y-w*z); r02=2*(x*z+w*y)
    r10=2*(x*y+w*z); r11=1-2*(x*x+z*z); r12=2*(y*z-w*x)
    r20=2*(x*z-w*y); r21=2*(y*z+w*x); r22=1-2*(x*x+y*y)
    return (r00*v[0]+r01*v[1]+r02*v[2], r10*v[0]+r11*v[1]+r12*v[2], r20*v[0]+r21*v[1]+r22*v[2])
def wrap(a):
    while a>180:a-=360
    while a<-180:a+=360
    return a
