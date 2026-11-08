import sys, math, glob
sys.path.insert(0,'.')
from load import load
import numpy as np
G=9.80665
def qmul(a,b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])
def qn(q): return q/np.linalg.norm(q)
def qfromacc(a):
    a=a/np.linalg.norm(a)
    r=math.atan2(a[1],a[2]); p=math.atan2(-a[0],math.hypot(a[1],a[2]))
    cr,sr,cp,sp=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2)
    return qn(np.array([cr*cp, sr*cp, cr*sp, -sr*sp]))
def integ(q,w,dt):
    th=np.linalg.norm(w)*dt
    if th<1e-6: dq=np.array([1,*(0.5*w*dt)])
    else: dq=np.array([math.cos(th/2),*(math.sin(th/2)/th*w*dt)])
    return qn(qmul(q,dq))
def dcm(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def tilt(q):
    c=-dcm(q)[2,0]; return math.degrees(math.acos(max(-1,min(1,c))))
def blend_fw(q,a,g):
    qa=qfromacc(-a)
    if np.dot(q,qa)<0: qa=-qa
    return qn((1-g)*q+g*qa)
def blend_tilt(q,a,g):
    # rotate so predicted body 'up' moves fraction g toward measured specific-force direction; heading untouched
    C=dcm(q); up_b=C.T@np.array([0,0,-1.0])   # nav up in body
    m=a/np.linalg.norm(a)
    ax=np.cross(up_b,m); s=np.linalg.norm(ax); 
    if s<1e-9: return q
    ang=math.atan2(s,np.dot(up_b,m))*g
    ax/=s
    # body-frame rotation that takes up_b toward m: rotate body vectors by -ang about ax  => q*exp(-ang/2 ax)
    dq=np.array([math.cos(ang/2),*(-math.sin(ang/2)*ax)])
    return qn(qmul(q,dq))
def run(path, mode, gate_g=0.15, win=5, gain=0.02):
    hdr,cols,rows=load(path)
    t=np.array([r['time_ms'] for r in rows])/1000.
    A=np.array([[r['accel_x_g'],r['accel_y_g'],r['accel_z_g']] for r in rows])*G
    W=np.radians(np.array([[r['gyro_x_dps'],r['gyro_y_dps'],r['gyro_z_dps']] for r in rows]))
    st=np.array([r['flight_state'] for r in rows]); lt=np.array([r['tilt_deg'] for r in rows])
    launch=float(hdr.get('Launch detect time','0'))/1000.
    pad=(t<launch-0.3)
    if pad.sum()<5: pad=np.arange(len(t))<5
    bias=W[pad].mean(0)
    q=qfromacc(-A[pad].mean(0))
    an=np.abs(np.linalg.norm(A,axis=1)/G-1)
    out=np.zeros(len(t)); ok=np.zeros(len(t),bool)
    for i in range(len(t)):
        if i>0:
            dt=t[i]-t[i-1]
            if 0<dt<0.2: q=integ(q,W[i]-bias,dt)
        inflight=t[i]>=launch
        if not inflight:
            q=blend_fw(q,A[i],gain)
        elif mode!='none':
            j0=max(0,i-win+1)
            g_ok=(an[j0:i+1]<=gate_g).all() and st[i]>=4  # descent states only (see below)
            ok[i]=g_ok
            if g_ok: q=(blend_fw if mode=='fw' else blend_tilt)(q,A[i],gain)
        out[i]=tilt(q)
    return dict(t=t,A=A,W=W,st=st,lt=lt,est=out,ok=ok,hdr=hdr,launch=launch)
