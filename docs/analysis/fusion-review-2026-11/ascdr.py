"""#72 ascent dead reckoning: does it work on the app's inputs (about 1 Hz
telemetry with lost packets) as well as on the locator's 20 Hz loop?

Horizontal speed = max(0, baro v_up) * tan(tilt), along the nose azimuth from
the strapdown quaternion plus an offset learned from GPS course over the last
2 s of trusted fixes before the outage.  Trusted = 3D fix, >= 6 SV, ground speed
>= 10 m/s (so the course means something), and GPS v_up within max(25 m/s,
15 %) of baro (the #70 cross-check).  Outages start every 0.5 s in Launched,
Burnout and Noseover, and are scored only where the fix at the end is trusted.

"hybrid" dead-reckons only while Launched (thrust), and otherwise holds a
velocity: the last trusted GPS velocity if the outage began in coast, else
dead reckoning's own velocity at the last thrust step.  On the 2026 corpus
tilt dead reckoning beat holding velocity only under thrust."""
import sys, math, random, bisect; sys.path.insert(0,'.')
from load import *
from rotm import rot, wrap
R=6371000.0
APP_TRUST=False   # True: the app's view, no GPS vertical speed, so no #70 cross-check
def trusted(r, judge=False):
    if not (r['fix_type']==3 and r['num_sv']>=6): return False
    if APP_TRUST and not judge:
        return r['gps_vel_n_mps']==r['gps_vel_n_mps'] and r['lat_deg']==r['lat_deg']
    vn,ve,vd=r['gps_vel_n_mps'],r['gps_vel_e_mps'],r['gps_vel_d_mps']
    if vn!=vn or vd!=vd or r['lat_deg']!=r['lat_deg']: return False
    vb=r['raw_baro_vel_mps']
    return abs(-vd-vb)<=max(25,0.15*abs(vb))
def nose(r): return rot((r['q_w'],r['q_x'],r['q_y'],r['q_z']),(1,0,0))
def az_deg(r): n=nose(r); return math.degrees(math.atan2(n[1],n[0]))
def tilt_rad(r): n=nose(r); return math.acos(max(-1,min(1,-n[2])))
def run(L=(3,6,10), loss=0.2, seed=1, verbose=False):
    rnd=random.Random(seed)
    res={l:{'hold':[],'cv':[],'dr20':[],'dr1':[],'hy20':[],'hy1':[]} for l in L}
    for f in files():
        h,c,rows=load(f)
        if 'gps_vel_n_mps' not in c or 'q_w' not in c: continue
        asc=[r for r in rows if 1<=int(r['flight_state'])<=3]
        if len(asc)<60: continue
        lat0=asc[0]['lat_deg'] if asc[0]['lat_deg']==asc[0]['lat_deg'] else rows[0]['lat_deg']
        lon0=asc[0]['lon_deg'] if asc[0]['lon_deg']==asc[0]['lon_deg'] else rows[0]['lon_deg']
        if lat0!=lat0: continue
        cl=math.cos(math.radians(lat0))
        P=lambda r:((r['lat_deg']-lat0)*math.pi/180*R,(r['lon_deg']-lon0)*math.pi/180*R*cl)
        T=[r['time_ms']/1000 for r in asc]
        # the app's packet stream: one per second, some lost
        keep=[]; nxt=T[0]
        for i,t in enumerate(T):
            if t>=nxt:
                nxt+=1.0
                if rnd.random()>=loss: keep.append(i)
        kset=set(keep)
        tst=asc[0]['time_ms']/1000
        for i0 in range(len(asc)):
            t0=T[i0]
            if (t0-tst)%0.5>0.06 and i0 not in kset: continue
            r0=asc[i0]
            if not trusted(r0) or math.hypot(r0['gps_vel_n_mps'],r0['gps_vel_e_mps'])<10: continue
            def offset(idx):
                o=[]
                for j in idx:
                    r=asc[j]
                    if t0-2<=T[j]<=t0 and trusted(r) and math.hypot(r['gps_vel_n_mps'],r['gps_vel_e_mps'])>=10:
                        crs=math.degrees(math.atan2(r['gps_vel_e_mps'],r['gps_vel_n_mps']))
                        o.append(math.radians(wrap(crs-az_deg(r))))
                if not o: return None
                return math.atan2(sum(map(math.sin,o)),sum(map(math.cos,o)))
            off20=offset(range(max(0,i0-60),i0+1))
            off1=offset([j for j in keep if j<=i0 and j>=i0-60])
            if off20 is None: continue
            for l in L:
                j1=bisect.bisect_left(T,t0+l)
                if j1>=len(asc) or abs(T[j1]-(t0+l))>0.06 or not trusted(asc[j1],judge=True): continue
                p0=P(r0); pt=P(asc[j1])
                def hy(idx,off):
                    x,y=p0; prev=t0; last=r0
                    hv=(r0['gps_vel_n_mps'],r0['gps_vel_e_mps']) if int(r0['flight_state'])>=2 else None
                    def step(last,dt):
                        nonlocal hv,x,y
                        if int(last['flight_state'])==1:
                            vz=max(0,min(400,last['raw_baro_vel_mps'])); a=math.radians(az_deg(last))+off
                            vh=vz*math.tan(min(tilt_rad(last),math.radians(80)))
                            v=(vh*math.cos(a),vh*math.sin(a)); hv=v
                        else:
                            v=hv if hv is not None else (0,0)
                        x+=v[0]*dt; y+=v[1]*dt
                    for j in idx:
                        step(last,T[j]-prev); prev=T[j]; last=asc[j]
                    step(last,t0+l-prev)
                    return x,y
                def dr(idx,off):
                    x,y=p0; prev=t0; last=r0
                    for j in idx:
                        dt=T[j]-prev
                        # integrate with the previous packet's values held (what the app has)
                        vz=max(0,min(400,last['raw_baro_vel_mps'])); a=math.radians(az_deg(last))+off
                        vh=vz*math.tan(min(tilt_rad(last),math.radians(80)))
                        x+=vh*math.cos(a)*dt; y+=vh*math.sin(a)*dt; prev=T[j]; last=asc[j]
                    dt=t0+l-prev
                    vz=max(0,min(400,last['raw_baro_vel_mps'])); a=math.radians(az_deg(last))+off
                    vh=vz*math.tan(min(tilt_rad(last),math.radians(80)))
                    x+=vh*math.cos(a)*dt; y+=vh*math.sin(a)*dt
                    return x,y
                e=lambda q: math.hypot(q[0]-pt[0],q[1]-pt[1])
                res[l]['hold'].append(e(p0))
                res[l]['cv'].append(e((p0[0]+r0['gps_vel_n_mps']*l,p0[1]+r0['gps_vel_e_mps']*l)))
                res[l]['dr20'].append(e(dr(range(i0+1,j1+1),off20)))
                res[l]['hy20'].append(e(hy(range(i0+1,j1+1),off20)))
                if off1 is not None:
                    res[l]['dr1'].append(e(dr([j for j in keep if i0<j<=j1],off1)))
                    res[l]['hy1'].append(e(hy([j for j in keep if i0<j<=j1],off1)))
    pc=lambda v,p: sorted(v)[min(len(v)-1,int(p*len(v)))] if v else float('nan')
    print(f"ascent outages, app packet loss {loss:.0%}")
    for l in L:
        s=res[l]; n=len(s['hold'])
        print(f"  {l:2d} s  n={n:4d}  hold {pc(s['hold'],.5):5.0f}/{pc(s['hold'],.95):5.0f}  constV {pc(s['cv'],.5):5.0f}/{pc(s['cv'],.95):5.0f}"
              f"  DR 20 Hz {pc(s['dr20'],.5):5.0f}/{pc(s['dr20'],.95):5.0f}  DR app {pc(s['dr1'],.5):5.0f}/{pc(s['dr1'],.95):5.0f}"
              f"  hybrid 20 Hz {pc(s['hy20'],.5):5.0f}/{pc(s['hy20'],.95):5.0f}  hybrid app {pc(s['hy1'],.5):5.0f}/{pc(s['hy1'],.95):5.0f} m (app n={len(s['dr1'])})")
    return res
if __name__=='__main__':
    for loss in (0.0,0.2): run(loss=loss)
    APP_TRUST=True; print("without the #70 cross-check (what TelemetryData allows):")
    run(loss=0.2)
