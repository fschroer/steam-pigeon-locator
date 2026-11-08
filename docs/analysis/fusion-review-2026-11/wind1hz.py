"""#72 descent coast at the app's inputs: ~1 Hz telemetry with packet loss.
For each descent, simulate outages: at outage start the app holds the mean GPS
N/E velocity of the packets it received in the previous 5 s (gps ok = 3D fix),
then carries the last position forward.  Compare with holding the last fix,
and score an error radius of h_acc + k*t for k = 2..5 m/s (4 m/s covered
95-98 % at 2-30 s on the 2026 corpus)."""
import sys, math, random; sys.path.insert(0,'.')
from load import *
R=6371000.0
K=(2.0,3.0,4.0,5.0)
def run(loss, rate_hz=1.0, window_s=5.0, seed=1):
    rnd=random.Random(seed)
    L=[2,5,10,20,30]; err={l:([],[]) for l in L}; rad={l:{k:[] for k in K} for l in L}
    for f in files():
        h,c,rows=load(f)
        if 'gps_vel_n_mps' not in c: continue
        d=[r for r in rows if 4<=int(r['flight_state'])<=7 and r['fix_type']==3 and r['num_sv']>=6
           and r['lat_deg']==r['lat_deg'] and r['gps_vel_n_mps']==r['gps_vel_n_mps']]
        if len(d)<200: continue
        lat0=d[0]['lat_deg']; lon0=d[0]['lon_deg']; cl=math.cos(math.radians(lat0))
        P=lambda r:((r['lat_deg']-lat0)*math.pi/180*R,(r['lon_deg']-lon0)*math.pi/180*R*cl)
        # telemetry stream: one row per 1/rate s, then drop packets
        step=1000.0/rate_hz; tel=[]; nxt=d[0]['time_ms']
        for r in d:
            if r['time_ms']>=nxt:
                nxt+=step
                if rnd.random()>=loss: tel.append(r)
        tt=[r['time_ms']/1000 for r in d]
        import bisect
        for i,r in enumerate(tel):
            t0=r['time_ms']/1000
            if i%3: continue
            win=[q for q in tel[max(0,i-12):i+1] if t0-q['time_ms']/1000<=window_s]
            if len(win)<2: continue
            vn=sum(q['gps_vel_n_mps'] for q in win)/len(win); ve=sum(q['gps_vel_e_mps'] for q in win)/len(win)
            # spread of the window's velocities: an honest growth rate
            sd=math.sqrt(sum((q['gps_vel_n_mps']-vn)**2+(q['gps_vel_e_mps']-ve)**2 for q in win)/len(win))
            p0=P(r)
            for l in L:
                j=bisect.bisect_left(tt,t0+l)
                if j>=len(d) or abs(tt[j]-(t0+l))>0.2: continue
                pt=P(d[j])
                hold=math.hypot(pt[0]-p0[0],pt[1]-p0[1])
                co=math.hypot(pt[0]-(p0[0]+vn*l),pt[1]-(p0[1]+ve*l))
                err[l][0].append(hold); err[l][1].append(co)
                for k in K: rad[l][k].append(co<=r['gps_h_acc_m']+k*l)
    pc=lambda v,p: sorted(v)[min(len(v)-1,int(p*len(v)))]
    print(f"loss {loss:.0%}  rate {rate_hz} Hz  window {window_s} s")
    for l in L:
        H,C=err[l]
        if not H: continue
        print(f"  {l:3d} s  n={len(H):4d}  hold {pc(H,.5):5.0f}/{pc(H,.95):5.0f} m   coast {pc(C,.5):5.0f}/{pc(C,.95):5.0f} m   inside h_acc+k*t, k=2/3/4/5 m/s: "+" ".join(f"{100*sum(rad[l][k])/len(rad[l][k]):3.0f}%" for k in K))
if __name__=='__main__':
    for loss in (0.0,0.2,0.5): run(loss)
