from tc import *
import numpy as np
from load import BASE as _B; B=_B+'/'
for name,f in [('Red Ryder','2026_09_26 Gerlach NV BALLS/Red Ryder Revisited_2026-09-26_09-07-35.csv'),('Shane','2026_09_26 Gerlach NV BALLS/Shane_2026-09-26_11-06-57.csv'),('Ken_6','2026_09_04 Pasco WA Sod Blaster/Ken_6_Locator_2026-09-06_133020.csv'),('Mike_5','2026_09_04 Pasco WA Sod Blaster/Mike_5_Locator_2026-09-06_180444.csv')]:
    hdr,cols,rows=load(B+f)
    t=np.array([r['time_ms'] for r in rows])/1000.; st=np.array([r['flight_state'] for r in rows])
    A=np.array([[r['accel_x_g'],r['accel_y_g'],r['accel_z_g']] for r in rows])
    an=np.abs(np.linalg.norm(A,axis=1)-1)
    pas=np.array([ (an[max(0,i-4):i+1]<=0.15).all() for i in range(len(t))])
    for s in sorted(set(st.astype(int))):
        m=st==s
        if s in (1,2,3): 
            U=A[m]/np.linalg.norm(A[m],axis=1)[:,None]
            print(name,'state',s,'n',m.sum(),'pass %.0f%%'%(pas[m].mean()*100), 'passing accel dir vs +X (deg) median %s'%(np.round(np.median(np.degrees(np.arccos(np.clip(U[pas[m],0],-1,1)))),0) if pas[m].any() else '-'))
