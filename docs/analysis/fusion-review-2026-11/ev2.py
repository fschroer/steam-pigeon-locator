import sys; sys.path.insert(0,'.')
from load import *
from vkf2 import run
def stable_mask(rows):
    m=[]; good_since=None
    for r in rows:
        t=r['time_ms']/1000
        g=r['fix_type']==3 and r['num_sv']>=10 and r['gps_h_acc_m']<2.5 and r['gps_vel_d_mps']==r['gps_vel_d_mps'] and abs(r['gps_vel_d_mps'])<320
        if g:
            if good_since is None: good_since=t
        else: good_since=None
        m.append(g and t-good_since>=2.0)
    return m
def pct(v,p): v=sorted(v); return v[min(len(v)-1,int(p*len(v)))] if v else float('nan')
def ev(nm,**kw):
    f=[x for x in files() if nm in x][0]; h,c,rows=load(f)
    out=run(rows,**kw); m=stable_mask(rows)
    res={}
    for ph,sts in (('coast',(2,)),('desc',(4,5,6,7))):
        E={'raw':[],'ekf':[],'vkf':[]}
        for r,o,ok in zip(rows,out,m):
            if not ok or int(r['flight_state']) not in sts: continue
            g=-r['gps_vel_d_mps']
            E['raw'].append(abs(r['raw_baro_vel_mps']-g)); E['ekf'].append(abs(r['fused_vspeed_mps']-g)); E['vkf'].append(abs(o[2]-g))
        res[ph]=E
    # apogee timing: GPS zero crossing within stable mask
    tg=None
    for i in range(1,len(rows)):
        if m[i] and m[i-1] and rows[i-1]['gps_vel_d_mps']<0<=rows[i]['gps_vel_d_mps'] and rows[i]['time_ms']>2000:
            tg=rows[i]['time_ms']/1000; break
    def zc(series):
        for i in range(1,len(series)):
            if series[i-1][1]>0>=series[i][1] and series[i][0]>2: return series[i][0]
    traw=zc([(r['time_ms']/1000,r['raw_baro_vel_mps']) for r in rows])
    tvkf=zc([(o[0],o[2]) for o in out])
    tekf=zc([(r['time_ms']/1000,r['fused_vspeed_mps']) for r in rows])
    s=lambda v:f"{pct(v,.5):5.1f}/{pct(v,.95):5.1f}" if v else "   -  /  -  "
    def d(x): return f"{x-tg:+5.2f}" if (x and tg) else "  -  "
    print(f"{nm[:24]:24s} coast n={len(res['coast']['raw']):4d} raw {s(res['coast']['raw'])} ekf {s(res['coast']['ekf'])} vkf {s(res['coast']['vkf'])} | desc n={len(res['desc']['raw']):5d} raw {s(res['desc']['raw'])} ekf {s(res['desc']['ekf'])} vkf {s(res['desc']['vkf'])} | apogee vs GPS({tg}): raw {d(traw)} ekf {d(tekf)} vkf {d(tvkf)}")
    return out
NAMES=['Ken_6','Ken_Locator_2026-09-05_132857','Mike_5_Locator_2026-09-06','Mike_5_Locator_2026-09-07','Mike_6','Mike_8','Mike_9_Locator_2026-09-06','Mike_9_Locator_2026-09-07','Nike_Smoke_2026-09-27_11-30-20.csv','Red Ryder','Shane']
if __name__=='__main__':
    kw=eval('dict('+(sys.argv[1] if len(sys.argv)>1 else '')+')')
    print('median/p95 |v - GPS v_up| m/s;', kw)
    for nm in NAMES: ev(nm,**kw)
