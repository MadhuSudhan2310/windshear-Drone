import numpy as np, quad_sim as q
print("Study C: position-loop gain sets, Ug=8")
for gx in [(0.10,0.01,0.15,10.0),(0.23,0.03,0.245,10.0),(0.5,0.08,0.4,10.0),(1.0,0.2,0.6,10.0)]:
    t,d=q.run(gains={'x':gx},tf=50); m=q.metrics(t,d)
    # step response overshoot/settling in calm air
    t2,d2=q.run(on=False,gains={'x':gx},xcmd=2.0,tf=40)
    ov=(d2['x'].max()-2)/2*100; ts=t2[np.where(abs(d2['x']-2)>0.04)[0][-1]]
    # settle time of wind recovery (|x|<0.05 after peak)
    tw=t[np.where(abs(d['x'])>0.05)[0][-1]]-q.W['t0']
    print(gx[:3],"x_dev %.2f th_max %.1f | step os %.1f%% ts %.1fs | wind recovery %.1fs"%(m['x_dev'],m['th_max'],ov,ts,tw))
