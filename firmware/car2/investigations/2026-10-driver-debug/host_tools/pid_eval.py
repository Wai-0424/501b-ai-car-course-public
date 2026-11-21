import csv, sys, numpy as np
rows=list(csv.DictReader(open(sys.argv[1]),delimiter='\t'))
ph={}
for r in rows: ph.setdefault(int(float(r['phase'])),[]).append({k:float(v) for k,v in r.items()})
phases=sorted(k for k in ph if k>=0)
print('step            final(L/R rpm) | rise90 s  | overshoot(+)/undershoot(-) % | settle(+-5%) s | ripple std (L/R)')
prev=None
for k in phases:
    rs=ph[k]; tgt=rs[0]['tgtL']
    t0=rs[0]['t']
    tail=[r for r in rs if r['t']>=rs[-1]['t']-1.0]
    fin=[np.mean([abs(r['vL']) for r in tail]), np.mean([abs(r['vR']) for r in tail])]
    sd=[np.std([abs(r['vL']) for r in tail]), np.std([abs(r['vR']) for r in tail])]
    pr=prev if prev is not None else [0.0,0.0]
    out=[]
    for i,col in enumerate(('vL','vR')):
        v=np.array([abs(r[col]) for r in rs]); t=np.array([r['t']-t0 for r in rs])
        f=fin[i]
        up = f>=pr[i]
        # rise: time to cover 90 % of the move
        lvl=pr[i]+0.9*(f-pr[i])
        idx=np.where(v>=lvl)[0] if up else np.where(v<=lvl)[0]
        rise=t[idx[0]] if len(idx) else float('nan')
        dev = (v.max()-f)/f*100 if up else (f-v.min())/f*100 if f>8 else float('nan')
        # for a down step the 'undershoot' = how far below final it dips
        tol=max(0.05*f,2.0)
        bad=np.where(np.abs(v-f)>tol)[0]
        settle = t[bad[-1]+1] if len(bad) and bad[-1]+1<len(t) else (0.0 if not len(bad) else float('nan'))
        out.append((rise,dev,settle))
    prev=fin
    print(f'{"up" if fin[0]>=(pr[0]) else "down"} to {tgt:3.0f}%  {fin[0]:6.1f}/{fin[1]:6.1f} | {out[0][0]:4.1f}/{out[1][0]:4.1f} | {out[0][1]:5.1f}/{out[1][1]:5.1f} | {out[0][2]:4.1f}/{out[1][2]:4.1f} | {sd[0]:3.1f}/{sd[1]:3.1f}')
