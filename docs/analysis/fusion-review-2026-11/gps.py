import sys; sys.path.insert(0,'.')
from load import *
R=6371000.0
def ne(r,lat0,lon0):
    return ((r['lat_deg']-lat0)*math.pi/180*R, (r['lon_deg']-lon0)*math.pi/180*R*math.cos(lat0*math.pi/180))
def phase(s):
    s=int(s)
    return 'pad' if s==0 else 'boost' if s==1 else 'coast' if s==2 else 'landed' if s>=9 else 'descent'
out=[]
for f in files():
    h,c,rows=load(f)
    if 'gps_vel_d_mps' not in c: continue
    name=os.path.basename(f)[:28]
    lat0,lon0=rows[0]['lat_deg'],rows[0]['lon_deg']
    st={}
    prev=None; prevt=None
    worst_jump=(0,0)
    for i,r in enumerate(rows):
        ph=phase(r['flight_state'])
        d=st.setdefault(ph,{'n':0,'nofix':0,'lowsv':0,'hacc10':0,'vd_bad':0,'vd_n':0,'jump':0})
        d['n']+=1
        if r['fix_type']!=3: d['nofix']+=1
        if r['num_sv']<6: d['lowsv']+=1
        if r['gps_h_acc_m']>10: d['hacc10']+=1
        bv=r['raw_baro_vel_mps']
        if abs(bv)<200 and r['fix_type']==3:
            d['vd_n']+=1
            if abs(-r['gps_vel_d_mps']-bv)>15: d['vd_bad']+=1
        # position jump vs reported velocity, on new fixes
        p=ne(r,lat0,lon0)
        if prev is not None and (p!=prev):
            dt=(r['time_ms']-prevt)/1000
            if dt>0:
                sp=math.hypot(p[0]-prev[0],p[1]-prev[1])/dt
                vh=math.hypot(r['gps_vel_n_mps'],r['gps_vel_e_mps'])
                if abs(sp-vh)>max(20,0.5*vh) and dt<=0.25: d['jump']+=1
            prev=p; prevt=r['time_ms']
        elif prev is None: prev=p; prevt=r['time_ms']
    print(name)
    for ph in ['pad','boost','coast','descent','landed']:
        if ph in st:
            d=st[ph]; print(f"   {ph:8s} n={d['n']:5d} nofix={d['nofix']:4d} sv<6={d['lowsv']:4d} hacc>10={d['hacc10']:4d} |gpsVz-baroVz|>15: {d['vd_bad']}/{d['vd_n']} posjump={d['jump']}")
