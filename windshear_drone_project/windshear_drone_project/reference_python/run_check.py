import numpy as np, quad_sim as q
print("hover thrust mg = %.2f N"%(q.P['m']*q.P['g']))
# calm sanity: step response
t,d=q.run(on=False,xcmd=2.0,zcmd=1.0,tf=30)
print("step: x_end %.3f z_end %.3f  x_os %.1f%% z_os %.1f%%"%(d['x'][-1],d['z'][-1],(d['x'].max()-2)/2*100,(d['z'].max()-1)*100))
# Study A: PD vs PID
for name,gn in [("PID (Ki on)",{}),("PD only (Ki=0)",{'z':(13.5,0,7.2,20.0),'x':(0.23,0,0.245,10.0)})]:
    t,d=q.run(on=True,gains=gn,tf=50); m=q.metrics(t,d)
    print(name,{k:round(float(v),2) for k,v in m.items()})
t,d=q.run(on=False); print("calm",{k:round(float(v),3) for k,v in q.metrics(t,d).items()})
print("intensity sweep (Ug, Aw=Ug*3/8)")
for Ug in [0,4,8,12,16,20]:
    for name,gn in [("PID",{}),("PD",{'z':(13.5,0,7.2,20.0),'x':(0.23,0,0.245,10.0)})]:
        t,d=q.run(gains=gn,wk=dict(Ug=Ug,Aw=Ug*3/8),tf=50); m=q.metrics(t,d)
        print(Ug,name,"x_dev %.2f z_dev %.2f th %.1f Tmax %.1f x_end %.2f z_end %.2f"%(m['x_dev'],m['z_dev'],m['th_max'],m['T_max'],m['x_end'],m['z_end']))
