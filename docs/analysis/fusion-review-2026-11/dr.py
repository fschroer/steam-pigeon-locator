import sys; sys.path.insert(0,'.')
from load import *
from rotm import rot, wrap
R=6371000.0
def ne(r,lat0,lon0): return ((r['lat_deg']-lat0)*math.pi/180*R, (r['lon_deg']-lon0)*math.pi/180*R*math.cos(lat0*math.pi/180))
def run(nm,t0,t1,off_window,sgn=1,vzsrc='baro'):
    f=[x for x in files() if nm in x][0]; h,c,rows=load(f)
    lat0,lon0=rows[0]['lat_deg'],rows[0]['lon_deg']
    # offset calibration
    offs=[]
    for r in rows:
        t=r['time_ms']/1000
        if off_window[0]<=t<=off_window[1] and r['fix_type']==3:
            nv=rot((r['q_w'],r['q_x'],r['q_y'],r['q_z']),(1,0,0))
            az=math.degrees(math.atan2(nv[1],nv[0]))*sgn
            crs=math.degrees(math.atan2(r['gps_vel_e_mps'],r['gps_vel_n_mps']))
            offs.append(wrap(crs-az))
    sx=sum(math.cos(math.radians(o)) for o in offs); sy=sum(math.sin(math.radians(o)) for o in offs)
    off=math.degrees(math.atan2(sy,sx))
    start=None; pos=None; hold=None; cv=None; prevt=None
    out=[]
    for r in rows:
        t=r['time_ms']/1000
        if t<t0: continue
        if start is None:
            start=ne(r,lat0,lon0); pos=list(start); v0=(r['gps_vel_n_mps'],r['gps_vel_e_mps']); cv=list(start); prevt=t; continue
        dt=t-prevt; prevt=t
        nv=rot((r['q_w'],r['q_x'],r['q_y'],r['q_z']),(1,0,0))
        az=math.radians(math.degrees(math.atan2(nv[1],nv[0]))*sgn+off)
        tilt=math.radians(min(r['tilt_deg'],80))
        vz=max(-50,min(400,r['raw_baro_vel_mps']))
        vh=max(0,vz)*math.tan(tilt)
        pos[0]+=vh*math.cos(az)*dt; pos[1]+=vh*math.sin(az)*dt
        cv[0]+=v0[0]*dt; cv[1]+=v0[1]*dt
        if t>=t1: break
    truth=ne(r,lat0,lon0)
    d=lambda a:math.hypot(a[0]-truth[0],a[1]-truth[1])
    print(f"{nm[:20]} outage {t0}-{t1}s offset={off:.1f} (n={len(offs)}): truth move={math.hypot(truth[0]-start[0],truth[1]-start[1]):.0f} m | err hold={d(start):.0f} constV={d(cv):.0f} tilt-DR={d(pos):.0f}  DR range={math.hypot(pos[0]-start[0],pos[1]-start[1]):.0f}")
if __name__=='__main__':
    run('Red Ryder',5,20,(3,5))
    run('Red Ryder',3,12,(2.5,3))
    run('Red Ryder',8,21,(6,8))
    run('Shane',7.0,20.5,(19.5,20.5))   # oracle offset from reacquisition
    run('Shane',7.0,20.5,(7,8))         # real-time offset from last pre-outage fixes
