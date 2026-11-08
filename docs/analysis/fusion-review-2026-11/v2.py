from tc import *
import tc
from v1 import F,B
def ref_tilt(A,t,half=2.5):
    U=A/np.linalg.norm(A,axis=1)[:,None]
    cs=np.cumsum(np.vstack([np.zeros(3),U]),0)
    out=np.zeros(len(t))
    j0=0;j1=0;n=len(t)
    for i in range(n):
        while t[j0]<t[i]-half: j0+=1
        while j1<n and t[j1]<=t[i]+half: j1+=1
        m=(cs[j1]-cs[j0]); m/=np.linalg.norm(m)
        out[i]=math.degrees(math.acos(max(-1,min(1,m[0]))))
    return out
def inst(A):
    U=A/np.linalg.norm(A,axis=1)[:,None]; return np.degrees(np.arccos(np.clip(U[:,0],-1,1)))
res={}
for k,f in F.items():
    for mode in ['none','fw','tilt']:
        r=run(B+f,mode)
        t,e,st=r['t'],r['est'],r['st']
        ref=ref_tilt(r['A'],t)
        hdr=r['hdr']
        for ph,(s0,s1) in {'drogue':(4,5),'main':(6,7)}.items():
            m=(st>=s0)&(st<=s1)
            idx=np.where(m)[0]
            if len(idx)==0: continue
            m2=m&(t>=t[idx[0]]+10)   # skip first 10 s of the phase (deployment shock, convergence)
            err=e[m2]-ref[m2]
            steps=np.abs(np.diff(e[m]))
            print(f"{k:10s} {mode:5s} {ph:6s} n={m2.sum():4d} gated={r['ok'][m].mean()*100:4.0f}% med|err|={np.median(np.abs(err)):6.1f} p90={np.percentile(np.abs(err),90):6.1f} medref={np.median(ref[m2]):5.1f} medest={np.median(e[m2]):5.1f} maxstep={steps.max():5.1f}")
