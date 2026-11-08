import sys; sys.path.insert(0,'.')
from load import *
from rotm import rot
G=9.80665
def a_up(r):
    fn=rot((r['q_w'],r['q_x'],r['q_y'],r['q_z']),(r['accel_x_g'],r['accel_y_g'],r['accel_z_g']))
    return (-fn[2]-1.0)*G
def run(rows,tau=0.5,vmax=230.0,boost_sigma=20.0,baro_sigma=1.0,gps=False,accel=True):
    """2-state (h,v) vertical filter. Accel bias learned on the pad and frozen at launch.
    Inputs: strapdown-rotated specific force in boost/coast (until a tumble is seen);
    random-walk accel otherwise. Baro: delayed by tau, ignored above vmax (transonic),
    ignored 1 s after a broadside/tumble signature, de-weighted under thrust (lag).
    Optional GPS v_up fused when it agrees with the filter (cross-check)."""
    h=rows[0]['raw_baro_agl_m']; v=0.0; P=[[1.0,0.0],[0.0,1.0]]
    bias=0.0; nb=0; D=0.0; hist=[]; block=-1; tumble=-1; tumbled=False
    out=[]; prev=rows[0]['time_ms']/1000
    for r in rows:
        t=r['time_ms']/1000; dt=max(0.0,t-prev); prev=t; st=int(r['flight_state'])
        gyro=math.sqrt(r['gyro_x_dps']**2+r['gyro_y_dps']**2+r['gyro_z_dps']**2)
        lat=math.hypot(r['accel_y_g'],r['accel_z_g'])
        pq=math.hypot(r['gyro_y_dps'],r['gyro_z_dps'])
        if 1<=st<=7 and (pq>400 or lat>4): block=t+1.0; tumbled=True
        au=a_up(r)
        mag=math.sqrt(r['accel_x_g']**2+r['accel_y_g']**2+r['accel_z_g']**2)
        if st==0 and abs(mag-1)<0.05 and gyro<5:
            nb+=1; bias+= (au-bias)/min(nb,40)
        if accel and (st in (1,2) or (st==0 and mag>1.5)) and not tumbled:
            a=au-bias; sa=1.0
        elif st==0:
            a=0.0; sa=0.3
        else:
            a=0.0; sa=3.0
        if dt>0:
            dh=v*dt+0.5*a*dt*dt; D+=dh; h+=dh; v+=a*dt
            p00,p01,p11=P[0][0],P[0][1],P[1][1]
            p00=p00+2*dt*p01+dt*dt*p11; p01=p01+dt*p11
            q=sa*sa; p00+=q*dt**4/4; p01+=q*dt**3/2; p11+=q*dt*dt
            P=[[p00,p01],[p01,p11]]
        # baro
        R=baro_sigma**2
        tl=math.cos(math.radians(min(r['tilt_deg'],75)))
        if st in (1,2) and abs(v)/tl>vmax and not tumbled: R=1e6
        if tumbled and st in (1,2) and R<1e5: R=max(R,4.0)
        if t<block: R=1e6
        if st==1 and R<1e5: R=max(R,boost_sigma**2)
        hist.append((t,D))
        while len(hist)>2 and hist[1][0]<=t-tau: hist.pop(0)
        hp=h-(D-hist[0][1])
        y=r['raw_baro_agl_m']-hp; S=P[0][0]+R
        K0=P[0][0]/S; K1=P[0][1]/S
        h+=K0*y; v+=K1*y
        P=[[P[0][0]-K0*P[0][0], P[0][1]-K0*P[0][1]],[P[0][1]-K0*P[0][1], P[1][1]-K1*P[0][1]]]
        # GPS vertical velocity, cross-checked
        if gps and r['fix_type']==3 and r['gps_vel_d_mps']==r['gps_vel_d_mps'] and r['num_sv']>=6:
            g=-r['gps_vel_d_mps']; Rg=max(1.0,(0.5+0.3*r['gps_h_acc_m']))**2
            yv=g-v; Sv=P[1][1]+Rg
            if yv*yv<9*Sv and abs(g)<320:
                K0=P[0][1]/Sv; K1=P[1][1]/Sv
                h+=K0*yv; v+=K1*yv
                P=[[P[0][0]-K0*P[0][1], P[0][1]-K0*P[1][1]],[P[0][1]-K0*P[1][1], P[1][1]-K1*P[1][1]]]
        out.append((t,h,v,bias))
    return out
