import numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, quad_sim as q
pd={'z':(13.5,0,7.2,20.0),'x':(0.23,0,0.245,10.0)}
cases=[("Calm air",False,{}),("Shear, PID",True,{}),("Shear, PD only",True,pd)]
fig,ax=plt.subplots(5,1,figsize=(9,11),sharex=True)
for n,on,gn in cases:
    t,d=q.run(on=on,gains=gn,tf=50)
    ax[1].plot(t,d['x'],label=n);ax[2].plot(t,d['z']);ax[3].plot(t,np.rad2deg(d['th']));ax[4].plot(t,d['T'])
t,d=q.run(on=True,tf=50); ax[0].plot(t,d['uw'],label="horizontal wind [m/s]");ax[0].plot(t,d['ww'],label="vertical wind [m/s]");ax[0].legend()
for a,l in zip(ax,["Wind [m/s]","x [m]","z dev [m]","pitch [deg]","Thrust [N]"]): a.set_ylabel(l);a.grid(alpha=.3)
ax[1].legend();ax[4].set_xlabel("t [s]");fig.suptitle("Quadrotor under wind shear (reference simulation)")
fig.tight_layout();fig.savefig("../reference_drone_shear_response.png",dpi=130)
