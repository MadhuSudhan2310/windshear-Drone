"""Reference simulation of the planar quadrotor + cascaded PID under wind shear.
Mirrors the Simulink model (same equations, PID with filtered derivative on error,
output clamping + clamping anti-windup)."""
import numpy as np
from scipy.integrate import solve_ivp

P = dict(m=1.5, g=9.81, J=0.03, kdx=0.30, kdz=0.40, kq=0.02, tau_m=0.03,
         taumax=1.5, z0=30.0, alphas=0.2, thmax=np.deg2rad(25))
G = dict(  # PID gains: (Kp, Ki, Kd, N)
    z=(13.5, 5.0, 7.2, 20.0),
    x=(0.23, 0.03, 0.245, 10.0),
    th=(6.75, 0.0, 0.63, 40.0))
W = dict(Ug=8.0, Aw=3.0, t0=10.0, T=8.0)

def wind(t, z, on=True, Ug=None, Aw=None, t0=None, T=None, alphas=None):
    Ug = W['Ug'] if Ug is None else Ug; Aw = W['Aw'] if Aw is None else Aw
    t0 = W['t0'] if t0 is None else t0; T = W['T'] if T is None else T
    alphas = P['alphas'] if alphas is None else alphas
    s = np.clip((t - t0) / T, 0, 1)
    uenv = Ug * (1 - np.cos(np.pi * s)) / 2
    uw = uenv * ((P['z0'] + z) / P['z0']) ** alphas     # horizontal wind with height gradient
    ww = -Aw * np.sin(np.pi * s) ** 2                   # vertical wind, negative = downdraft
    return (uw, ww) if on else (0.0, 0.0)

def pid(e, I, yf, K, lo, hi):
    """Filtered-derivative PID with clamping anti-windup. Returns u_sat, dI, dyf."""
    Kp, Ki, Kd, N = K
    D = Kd * N * (e - yf)
    u = Kp * e + Ki * I + D
    us = np.clip(u, lo, hi)
    sat = (u != us) and (np.sign(u) == np.sign(e))
    dI = 0.0 if sat else e
    return us, dI, N * (e - yf)

def run(on=True, gains=None, tf=40.0, xcmd=0.0, zcmd=0.0, wk=None, pk=None):
    g = {**G, **(gains or {})}; p = {**P, **(pk or {})}; wk = wk or {}
    m, gr = p['m'], p['g']
    def f(t, s):
        x, xd, z, zd, th, q, Ix, yx, Iz, yz, It, yt, T_d, tau_a = s
        uw, ww = wind(t, z, on, **wk)
        # altitude loop -> thrust delta command
        dT, dIz, dyz = pid(zcmd - z, Iz, yz, g['z'], -gr*m, gr*m)
        # position loop -> pitch command
        thc, dIx, dyx = pid(xcmd - x, Ix, yx, g['x'], -p['thmax'], p['thmax'])
        # attitude loop -> torque command
        tc, dIt, dyt = pid(thc - th, It, yt, g['th'], -p['taumax'], p['taumax'])
        T = m*gr + T_d          # motor-lagged thrust delta state T_d
        xdd = (T*np.sin(th) - p['kdx']*(xd - uw)) / m
        zdd = (T*np.cos(th) - m*gr - p['kdz']*(zd - ww)) / m
        qd = (tau_a - p['kq']*q) / p['J']
        return [xd, xdd, zd, zdd, q, qd, dIx, dyx, dIz, dyz, dIt, dyt,
                (dT - T_d)/p['tau_m'], (tc - tau_a)/p['tau_m']]
    t = np.linspace(0, tf, int(tf/0.01)+1)
    s = solve_ivp(f, [0, tf], np.zeros(14), t_eval=t, max_step=0.01, method='RK45')
    y = s.y
    Tact = m*gr + y[12]
    return s.t, dict(x=y[0], z=y[2], th=y[4], T=Tact, tau=y[13],
                     uw=np.array([wind(tt, zz, on, **wk)[0] for tt, zz in zip(s.t, y[2])]),
                     ww=np.array([wind(tt, zz, on, **wk)[1] for tt, zz in zip(s.t, y[2])]))

def metrics(t, d):
    return dict(x_dev=np.abs(d['x']).max(), z_dev=np.abs(d['z']).max(),
                th_max=np.rad2deg(np.abs(d['th']).max()),
                T_max=d['T'].max(), x_end=d['x'][-1], z_end=d['z'][-1])
