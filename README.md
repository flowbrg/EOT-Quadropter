Answer to <strong>[this](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/blob/main/projects/Energy-Optimal%20Trajectory%20Planning%20for%20Multirotor%20Drones/README.md)</strong> project.



<table>
<td><img src="https://gist.githubusercontent.com/robertogl/e0115dc303472a9cfd52bbbc8edb7665/raw/UAV_plan.png"  width=500 /></td>
<td><p><h1>Energy-Optimal Trajectory Planning for Multirotor Drones</h1></p>
</table>

MATLAB implementation of an optimization-based trajectory planner for a battery-powered quadrotor. The planner computes the rotor speeds that bring the drone from an initial state to a target while minimizing the electrical energy drawn from the battery, subject to the quadrotor, brushless DC motor (BLDCM) and battery models.

The repository addresses MathWorks Challenge Project #237 (*Energy-Optimal Trajectory Planning for Multirotor Drones*) and is inspired by Schacht-Rodriguez et al. (2018), *Path planning generation algorithm for a class of UAV multirotor based on state of health of lithium polymer battery*, J. Intell. Robot. Syst. 91(1), 115-131 (`references/`).


## 1. Overview

> **Scope.** Open-loop trajectory generation by direct collocation, solved as a nonlinear program (NLP) with a free final time $T_f$. The battery is a static Thevenin model (no thermal model, no state-of-health model).

The NLP uses a 16-state model $[x\ y\ z\ v_x\ v_y\ v_z\ \phi\ \theta\ \psi\ p\ q\ r\ \omega_1\ldots\omega_4]$ with the rotor accelerations $\dot\omega_i$ as inputs, discretized on $N$ intervals (`nlp.N`, set in `src/init_params.m`). The full derivation (dynamics, motor, battery, energy criterion, collocation, Hessian) is in `notebook/`; this README does not repeat it.

Four solver scripts are provided:

| Script | Scenario | Solver | Hessian |
|---|---|---|---|
| `src/nlp_solv_fmincon.m` | Simple trajectory, no obstacles | `fmincon` (interior-point) | L-BFGS approximation |
| `src/nlp_solv_fmincon_2.m` | Trajectory with obstacles (buildings) | `fmincon` (interior-point) | Exact, from symbolic local blocks (`gen_hess_loc.m`); L-BFGS selectable via `use_exact_hessian` |
| `src/old/nlp_solv_casadi.m` | Same as `nlp_solv_fmincon.m` | CasADi + IPOPT | Handled by CasADi |
| `src/old/nlp_solv_casadi_2.m` | Same as `nlp_solv_fmincon_2.m` | CasADi + IPOPT | Handled by CasADi |

## 2. Repository structure

```
.
├── README.md
├── references/                 Papers used (Schacht-Rodriguez et al. 2018; morbidi2016.pdf)
├── figures/                    Figures (.jpg) and their draw.io sources (figures/src/)
├── notebook/                   Complete notebook: equations, models, trajectory-solving details
│   ├── main.mlx / main.pdf     Notebook and its PDF export
│   ├── main_script.m           Script version (fmincon, exact Hessian)
│   └── fmincon_objective.m, fmincon_constraints.m, hess_loc.m   Auxiliary files used by the notebook
└── src/                        All the code
    ├── init_params.m           Creates data/params.mat: UAV, motor, battery and NLP parameters (e.g. nlp.N)
    ├── init_scenario.m         Creates data/scenario.mat: obstacles, target, initial point (shared by all runs)
    ├── data/                   params.mat, scenario.mat (+ generated_fmincon/, git-ignored)
    ├── nlp_solv_fmincon.m      Obstacle-free trajectory, fmincon + L-BFGS
    ├── nlp_solv_fmincon_2.m    Trajectory with obstacles, fmincon + exact Hessian
    ├── gen_hess_loc.m          Symbolic generation of the exact Hessian blocks (model/hess_loc.m, data/generated_fmincon/)
    ├── plot_nlp.m              Plots obstacle-free solutions (fmincon or CasADi: uncomment the line at the top)
    ├── plot_nlp_2.m            Plots solutions with obstacles (fmincon or CasADi: uncomment the line at the top)
    ├── plot_scenario.m         Plots the flight space and the obstacles
    ├── test_aug.m              Test script for dyn_uav.m (augmented model)
    ├── test_motor_reduction.m  Test: planner model vs full motor + battery model
    ├── compare_models.m        Test: simple 12-state model (dyn) vs augmented 21-state model (dyn_uav)
    ├── model/                  Model functions (dyn, dyn_vec, dyn_uav, dyn_bldcm, dutycycle, Pm_rows, hess_loc)
    ├── old/                    CasADi versions of the two fmincon solver scripts
    ├── sols/                   Saved solutions (sol_fmincon, sol_fmincon_2, sol_casadi, sol_casadi_2 .mat)
    └── misc/                   Early separate models and Simulink tests (battery, bldcm, drone_dynamics, uav_simulink), kept for reference
```


<!--
Implementation notes (not rendered):
- dyn.m must use the spin convention of Section 1.3: replace
  [0;0;w1-w2+w3-w4] by [0;0;-w1+w2-w3+w4] in the gyroscopic term.
- K_E, R, J_r, D_f (and L, T_f if used) are not yet in init_params.m.
- simu.m: rename the variables `deg2rad` and `eps`, which shadow built-in functions.
-->

# Theory: energy-optimal trajectory planning for a quadrotor


The following repo aims at addressing the topic above.
It shall compare two strategies:
- Resolving a NLP (Open-loop)
- Following a polynomial trajectory (Closed-loop)

This project is heavily inspired of the paper [Path Planning Generation Algorithm for a Class of UAV Multirotor
Based on State of Health of Lithium Polymer Battery](https://doi.org/10.1007/s10846-018-0870-0) but takes some liberties were taken.
The battery features no thermodynamics model, nor degradation models as it serves no purpose in our case.

A table of all parameters and their value can be found at the end of this page.

## 0. Notation and conventions

| Symbol | Meaning | Domain / unit |
|---|---|---|
| $E_0$ | Inertial frame, $z_0$ upward | - |
| $E_b$ | Body frame at the centre of mass; $x_b$ through rotor 1, $y_b$ through rotor 2, $z_b$ upward | - |
| $\xi=[x\ y\ z]^\top$, $v$ | Position, velocity in $E_0$ | $\mathbb R^3$, m, m/s |
| $\eta=[\phi\ \theta\ \psi]^\top$ | Roll, pitch, yaw (ZYX) | $\mathbb R^3$, rad |
| $\bar\omega=[p\ q\ r]^\top$ | Body angular rates in $E_b$ | $\mathbb R^3$, rad/s |
| $\omega_i$ | Speed of rotor $i$ | rad/s |
| $\hat a$ | Skew-symmetric matrix, $\hat a\,b=a\wedge b$ | $\mathbb R^{3\times3}$ |

Rotors are numbered clockwise seen from above, at $(+l,0,0)$, $(0,-l,0)$, $(-l,0,0)$, $(0,+l,0)$ in $E_b$.

> **Assumptions.**
> **(H1)** Rigid body, constant mass $m$, constant diagonal inertia $I=\mathrm{diag}(J_{xx},J_{yy},J_{zz})$ (frame symmetry).
> **(H2)** Centre of mass at the geometric centre.
> **(H3)** Thrust $k_T\omega_i^2$ and reaction torque $k_Q\omega_i^2$ of each rotor are quadratic in $\omega_i$.
> **(H4)** Body aerodynamic drag neglected.
> **(H5)** Rotor gyroscopic torque $J_r\omega_G(\ldots)$ and rotor-acceleration torque $J_r\dot\omega_G$ neglected (see 1.3).

## 1. Quadrotor dynamics

### 1.1 Newton-Euler equations

$$
\dot\xi=v,\qquad m\dot v=R_b\,E(3)\,T-mg\,E(3),\qquad \dot R_b=R_b\,\hat{\bar\omega},\qquad I\dot{\bar\omega}=-\bar\omega\wedge I\bar\omega+\Gamma,
$$

with $E(3)=[0\ 0\ 1]^\top$, $g=9.81\, m/s^2$, $T$ the total thrust [N] and $\Gamma\in E_b$ the torque [N·m]. $R_b\in SO(3)$ maps $E_b$ to $E_0$:

$$
R_b=\begin{bmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi-s_\psi c_\phi & c_\psi s_\theta c_\phi+s_\psi s_\phi\\
s_\psi c_\theta & s_\psi s_\theta s_\phi+c_\psi c_\phi & s_\psi s_\theta c_\phi-c_\psi s_\phi\\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix}.
$$

### 1.2 Rotor forces and torques

A thrust $k_T\omega_i^2$ applied at $(x_i,y_i,0)$ gives the torque $(y_ik_T\omega_i^2,\,-x_ik_T\omega_i^2)$ about $(x_b,y_b)$. With the rotor positions above:

$$
T=k_T\sum_{i=1}^4\omega_i^2,\qquad
\Gamma=\begin{bmatrix}l\,k_T(-\omega_2^2+\omega_4^2)\\ l\,k_T(-\omega_1^2+\omega_3^2)\\ k_Q(\omega_1^2-\omega_2^2+\omega_3^2-\omega_4^2)\end{bmatrix}.
$$

Units: $k_T$ [N/(rad/s)$^2$], $k_Q$ [N·m/(rad/s)$^2$], $l$ [m]. The yaw torque corresponds to rotors 1, 3 spinning clockwise and rotors 2, 4 counter-clockwise seen from above.

### 1.3 Neglected rotor terms (H5)

The rotor angular momentum $J_r\omega_G$ ($\omega_G=-\omega_1+\omega_2-\omega_3+\omega_4$) would add the gyroscopic torque $J_r\omega_G[-q\ \ p\ \ 0]^\top$, and its variation the yaw torque $J_r\dot\omega_G$. Both are **not** in the model (`dyn.m`). Order of magnitude for one rotor accelerating by 20 rad/s over the open-loop motor time constant (23 ms, section 3.3): $J_r\dot\omega\approx3\times10^{-5}\cdot20/0.023=2.6\times10^{-2}$ N·m, against $\approx6\times10^{-3}$ N·m for the corresponding variation of the reaction torque $2k_Q\omega_h\Delta\omega$. The yaw dynamics are therefore inaccurate during fast speed transients.

### 1.4 State-space model (12 states)

$$
X=[x\ y\ z\ v_x\ v_y\ v_z\ \phi\ \theta\ \psi\ p\ q\ r]^\top\in\mathbb R^{12},\qquad \omega=[\omega_1\dots\omega_4]^\top\in\mathbb R^4 .
$$

$$
\begin{aligned}
\dot x&=v_x,\quad \dot y=v_y,\quad \dot z=v_z,\\
\dot v_x&=\tfrac{T}{m}(c_\psi s_\theta c_\phi+s_\psi s_\phi),\quad
\dot v_y=\tfrac{T}{m}(s_\psi s_\theta c_\phi-c_\psi s_\phi),\quad
\dot v_z=\tfrac{T}{m}c_\theta c_\phi-g,\\
\dot\phi&=p+s_\phi t_\theta q+c_\phi t_\theta r,\quad
\dot\theta=c_\phi q-s_\phi r,\quad
\dot\psi=\tfrac{s_\phi q+c_\phi r}{c_\theta},\\
\dot p&=\tfrac{(J_{yy}-J_{zz})qr+\Gamma_\phi}{J_{xx}},\quad
\dot q=\tfrac{(J_{zz}-J_{xx})rp+\Gamma_\theta}{J_{yy}},\quad
\dot r=\tfrac{(J_{xx}-J_{yy})pq+\Gamma_\psi}{J_{zz}}.
\end{aligned}
$$

The kinematics are exact (no small-angle approximation). This is `dyn.m`.

> **Classification.** Nonlinear, time-invariant, continuous-time, MIMO (12 states, 4 inputs), deterministic.
> **Validity.** The Euler parametrization is singular at $\theta=\pm\pi/2$ ($t_\theta$, $1/c_\theta$). The NLP enforces $|\phi|,|\theta|\le35^\circ$ (section 5). Quaternions would remove the singularity.

### 1.5 Hover

For $\phi=\theta=0$, zero velocities and rates, $\omega_i=\omega_h$: $\Gamma=0$ and $T=mg$, hence

$$
\omega_h=\sqrt{\frac{mg}{4k_T}}=\sqrt{\frac{0.5\times9.81}{4\times3\times10^{-5}}}\approx202.18\ \mathrm{rad/s}\ (\approx1931\ \mathrm{rpm}).
$$

Linearization around hover ($\psi^*=0$ without loss of generality): $A$ is nilpotent ($A^4=0$), all 12 eigenvalues are zero in Jordan blocks of sizes $(4,4,2,2)$. Hover is therefore **not asymptotically stable** in open loop (Lyapunov unstable, polynomial divergence), and $(A,B)$ is in Brunovsky form with controllability indices $(4,4,2,2)$ summing to 12, hence controllable. This is not used by the planner; it only supports the LQR hover tests in `src/misc/`.

## 2. Battery model

### 2.1 Static Thevenin model

> **Assumptions.**
> **(B1)** 3S LiPo pack ($N_s=3$ cells), $E_{OCV}=V_{oc}(1)=10.6$ V.
> **(B2)** Constant temperature; $R_{int}$ and $C_{batt}$ independent of temperature, SoC and ageing (no thermal, no SoH model).
> **(B3)** Current positive in discharge.
> **(B4)** The diffusion branch $R_d\parallel C_d$ is neglected ($V_d=0$).

The model is then algebraic: the terminal voltage reacts instantaneously to the current, whereas the full circuit would show a transient with time constant $R_dC_d$.

$$
V_{oc}(\mathrm{SoC})=V_{term}+R_{int}\,I,\qquad I=\sum_{k=1}^4I_{BLDCM,k}.
$$

### 2.2 Open-circuit voltage

$$
V_{oc}(s)=\sum_{i=0}^4\lambda_i s^i+s\ln s,\qquad
[\lambda_0\ \lambda_1\ \lambda_2\ \lambda_3\ \lambda_4]=[0,\ 67.6,\ -155,\ 153,\ -55]
$$

(`batt.Pol_ocv=[-55 153 -155 67.6 0]` in descending order for `polyval`). Check: $V_{oc}(1)=67.6-155+153-55=10.6$ V. The term $s\ln s$ tends to 0 as $s\to0^+$, but `log(0)*0` evaluates to NaN: the model is valid for $\mathrm{SoC}>0$.

### 2.3 State of charge

$$
\dot{\mathrm{SoC}}=-\frac{I}{C_{batt}}=\frac{V_{term}-V_{oc}}{R_{int}C_{batt}},
$$

with $C_{batt}$ in A·s ($x$ mAh $=3.6x$ A·s) and $\mathrm{SoC}\in[0,1]$. Pure Coulomb counting: unit coulombic efficiency, capacity independent of the discharge rate (no Peukert effect).

> **Capacity.** The physical value is $C_{batt}=1.3\times3600=4680$ A·s (1300 mAh, `src/init_params.m`). `main.mlx` deliberately uses $4680/10=468$ A·s: on the short trajectory considered the pack loses less than 1 % of its charge, and the reduced capacity makes the SoC variation 10 times larger, hence visible in the plots. This affects only the SoC state (and thus $V_{oc}(\mathrm{SoC})$) of the 21-state simulation, not the NLP.

## 3. Brushless DC motor (BLDCM)

### 3.1 Equations

Each rotor is an equivalent DC machine:

$$
L\,\dot i_i=v_i-R\,i_i-K_E\omega_i,\qquad
J_r\dot\omega_i=K_E\,i_i-d\,\omega_i^2-D_f\omega_i-T_f\,\mathrm{sign}(\omega_i),
$$

with $R$ [Ω], $L$ [H], $K_E$ [V·s/rad] (back-EMF and torque constant), $J_r$ [kg·m$^2$], $d=k_Q$ the propeller load, $D_f$ the viscous and $T_f$ the dry friction.

> **Assumptions.**
> **(M1)** $D_f=T_f=0$.
> **(M2)** Quasi-static current, $L\dot i_i\simeq0$ (the code uses $L=0$); valid when $L/R\ll\tau_m$, with $\tau_m$ defined in 3.3.

$$
\boxed{\dot\omega_i=\frac1{J_r}\left(K_E\,i_i-k_Q\,\omega_i^2\right)},\qquad i_i=\frac{V_{term}D_{c,i}-K_E\omega_i}{R}.
$$

The ESC applies $\bar v_i=V_{term}D_{c,i}$ (PWM, $D_{c,i}\in[0,1]$) and the battery supplies $I_{BLDCM,i}=D_{c,i}\,i_i$.

### 3.2 Algebraic loop battery / motors

$V_{term}$ is needed for the currents $i_k$, and all $i_k$ are needed for $V_{term}$. Substituting $i_k$ in $V_{term}=V_{oc}-R_{int}\sum_kD_{c,k}i_k$, with

$$
a_D=\sum_{k=1}^4D_{c,k}^2,\qquad b_D=K_E\sum_{k=1}^4D_{c,k}\omega_k,
$$

gives $V_{term}(1+R_{int}a_D/R)=V_{oc}+R_{int}b_D/R$, i.e.

$$
\boxed{V_{term}=\frac{R\,V_{oc}+R_{int}\,b_D}{R+R_{int}\,a_D}},\qquad i_k=\frac{V_{term}D_{c,k}-K_E\omega_k}{R}.
$$

This closed form (`dyn_bldcm.m`) removes the algebraic loop (the Simulink loop warning).

### 3.3 Motor time constant

Linearizing 3.1 at constant $D_c$ and $\omega=\omega_h$ gives $\partial\dot\omega/\partial\omega=-\frac1{J_r}\left(\frac{K_E^2}{R}+2k_Q\omega_h\right)$, hence the open-loop mechanical time constant

$$
\tau_m=\frac{J_r}{K_E^2/R+2k_Q\omega_h}=\frac{3\times10^{-5}}{1.303\times10^{-3}}\approx23.0\ \mathrm{ms}.
$$

With the PI feedback of 3.4 the effective constant is $J_r/(K_E^2/R+2k_Q\omega_h+K_EE_{OCV}K_p/R)\approx13.0$ ms (computed from the parameters of section 4.2).

### 3.4 Duty cycle: feedforward + PI

**Feedforward.** In steady state ($\dot\omega=0$): $K_Ei=k_Q\omega^2$. Assuming $V_{term}\simeq E_{OCV}$:

$$
D_{ff,k}=\frac{K_E\,\omega_{k,ref}+R\frac{k_Q}{K_E}\omega_{k,ref}^2}{E_{OCV}}\qquad(\approx0.22\ \text{at hover}).
$$

For a single motor this is accurate; in the augmented model the battery voltage drops under the load of the four motors: at $\omega_{ref}=202.2$ rad/s the achieved $\omega_1=183.1$ rad/s, a $9.4\,\%$ error. A refinement using the quadratic equation in $V_{term}$ (all four motors, $V_{oc}\simeq E_{OCV}$) performed equally poorly and is not retained.

**PI feedback.**

$$
D_{c,k}=\mathrm{clip}_{[0,1]}\!\left(D_{ff,k}+K_p\,e_k+K_i\,z_k\right),\qquad e_k=\omega_{k,ref}-\omega_k,\quad \dot z_k=e_k,
$$

with $K_p=9.4\times10^{-4}$ s/rad and $K_i=9.42\times10^{-2}$ rad$^{-1}$ (dimensionless product since $z$ is in rad). The gains were tuned with MATLAB's PID Tuner on the motor model linearized around $D_c=0$, $\omega=0$ (transient-behavior setting 0.6, response time 0.03 s). With $V_{term}\simeq E_{OCV}$, and since $d\,\omega^2$ has zero slope at $\omega=0$, the linearized plant is first order:

$$
G(s)=\frac{\delta\omega}{\delta D_c}=\frac{K_EE_{OCV}/(RJ_r)}{s+K_E^2/(RJ_r)}=\frac{3.53\times10^{4}}{s+33.3}.
$$

Check of the final gains, computed analytically on this plant (not exported from the tuner), for $C(s)=K_p+K_i/s$: crossover $\approx58$ rad/s, phase margin $\approx60^\circ$, 10-90 % rise time $\approx25$ ms, overshoot $\approx14\,\%$ of the closed loop $CG/(1+CG)$. At the hover operating point the pole moves to $(K_E^2/R+2k_Q\omega_h)/J_r=43.4$ rad/s and the margin becomes $\approx67^\circ$.

## 4. Augmented model (simulation, 21 states)

### 4.1 Equations

$$
X_{aug}=\big[\underbrace{X_{1:12}}_{\text{body}}\ \big|\ \underbrace{\omega_1\dots\omega_4}_{13:16}\ \big|\ \underbrace{\mathrm{SoC}}_{17}\ \big|\ \underbrace{z_1\dots z_4}_{18:21}\big]^\top\in\mathbb R^{21},\qquad u=\omega_{ref}\in\mathbb R^4,
$$

$$
\dot X_{aug}=\begin{bmatrix}\text{dyn}(X_{1:12},\,\omega)\\ \dot\omega\\ -I_{batt}/C_{batt}\\ \omega_{ref}-\omega\end{bmatrix},
$$

with $D_c$ from 3.4, $V_{oc}$ from 2.2, and $(I_{batt},\dot\omega)$ from 3.1-3.2 (`dyn_uav.m`). Nonlinear, time-invariant, continuous-time, MIMO, deterministic.

### 4.2 Parameters

| Group | Parameter | Value | Unit |
|---|---|---|---|
| Drone | $m$ | 0.5 | kg |
| | $k_T$ | $3\times10^{-5}$ | N/(rad/s)$^2$ |
| | $k_Q$ | $7.5\times10^{-7}$ | N·m/(rad/s)$^2$ |
| | $l$ | 0.25 | m |
| | $J_{xx},J_{yy},J_{zz}$ | $5\times10^{-3},\,5\times10^{-3},\,9\times10^{-3}$ | kg·m$^2$ |
| Battery | $R_{int}$ | 0.05 | Ω |
| | $E_{OCV}$ | 10.6 | V |
| | $C_{batt}$ | 4680 (468 in `main.mlx`, for SoC visibility, see 2.3) | A·s |
| BLDCM | $R$ | 0.1 | Ω |
| | $K_E$ | 0.01 | V·s/rad |
| | $J_r$ | $3\times10^{-5}$ | kg·m$^2$ |
| | $d=k_Q$ | $7.5\times10^{-7}$ | N·m/(rad/s)$^2$ |
| | $L$ | 0 | H |
| | $K_p$ / $K_i$ | $9.4\times10^{-4}$ / $9.42\times10^{-2}$ | s/rad / rad$^{-1}$ |

## 5. Trajectory optimization (NLP)

### 5.1 Problem and variables

Open-loop optimal control problem with free final time $T_f$: find the rotor-speed profile that brings the drone from $(\xi_0,v_0)$ to $(\xi_f,v_f)$ at minimum electrical energy, subject to the dynamics and bounds. It is transcribed by direct collocation on $N$ intervals ($N+1$ nodes).

The planning model has 16 states, the rotor speeds being promoted to states and their accelerations being the inputs:

$$
X=[\,\underbrace{x\ y\ z\ v_x\ v_y\ v_z\ \phi\ \theta\ \psi\ p\ q\ r}_{X_{1:12}}\ \omega_1\ \omega_2\ \omega_3\ \omega_4\,]^\top\in\mathbb R^{16},\qquad
U=[\dot\omega_1\dots\dot\omega_4]^\top\in\mathbb R^4,
$$

$$
\dot X=F(X,U)=\begin{bmatrix}\text{dyn}(X_{1:12},\,X_{13:16})\\ U\end{bmatrix}\qquad(\texttt{dyn\_vec.m}).
$$

Decision vector $z=[T_f;\ \mathrm{vec}(X);\ \mathrm{vec}(U)]\in\mathbb R^{n_z}$ with $X\in\mathbb R^{16\times(N+1)}$, $U\in\mathbb R^{4\times(N+1)}$ and $n_z=1+20(N+1)$ (2021 for $N=100$, 1221 for $N=60$).

> **Classification.** Finite-dimensional, nonlinear, non-convex NLP (nonlinear equality constraints from the dynamics, quartic cost). Only a **local** optimum is returned: the initial guess matters (5.7).

### 5.2 Why a mechanical-only planning model

The battery (SoC) and the PI integrators are **not** states of the NLP; they appear only in the 21-state simulation (5.9). The motor and battery dynamics are neglected on the assumption that the planned rotor-speed references are smooth, i.e. slower than the motor closed-loop bandwidth (in the Bode diagrams of `src/compare_models.m`, the 12-state mechanical model and the 21-state augmented model start to differ around 100 rad/s; this is the same order as the corner frequencies $1/23\,\mathrm{ms}\approx43$ and $1/13\,\mathrm{ms}\approx77$ rad/s given by the time constants of 3.3). `src/test_motor_reduction.m` quantifies the approximation on an obstacle-free trajectory: the position error is 3.3 cm, i.e. $\approx0.05\,\%$ of the $50\sqrt2\approx70.7$ m path (motor alone, ideal battery). This test has no obstacles; it is used qualitatively, to show that the reduction is acceptable for smooth trajectories, and is not a bound for the trajectories with obstacles. A residual difference between the ideal and the simulated trajectory remains, and is visible in the final simulation (5.9).

### 5.3 Cost function

Electrical power of the four motors, from 3.1 with $\omega$ and $\dot\omega=U$ (with $a_k=J_r\dot\omega_k+k_Q\omega_k^2=K_Ei_k$ the electromagnetic torque of motor $k$ [N·m], from the mechanical equation of 3.1):

$$
P_m=\sum_{k=1}^4V_{term}D_{c,k}i_k=\sum_{k=1}^4\left(R\,i_k^2+K_E\omega_ki_k\right)
=\sum_{k=1}^4\left[\frac{R}{K_E^2}a_k^2+\omega_ka_k\right],\qquad i_k=\frac{a_k}{K_E}.
$$

Expanded:

$$
P_m=\sum_{k}\Big(k_Q\omega_k^3+\tfrac{Rk_Q^2}{K_E^2}\omega_k^4+\tfrac{RJ_r^2}{K_E^2}\dot\omega_k^2\Big)+\sum_k\Big(J_r\omega_k\dot\omega_k+\tfrac{2RJ_rk_Q}{K_E^2}\omega_k^2\dot\omega_k\Big).
$$

The second sum is an exact derivative: $\int_0^{T_f}(\cdot)\,dt=\sum_k\big[\tfrac{J_r}{2}\omega_k^2+\tfrac{2RJ_rk_Q}{3K_E^2}\omega_k^3\big]_0^{T_f}$, which **vanishes because $\omega(0)=\omega(T_f)=\omega_h$** (boundary conditions `init` and `rot_f`, 5.5). The remaining terms give the "reduced" criterion $\int(k_Q\omega^3+\frac{Rk_Q^2}{K_E^2}\omega^4+\frac{RJ_r^2}{K_E^2}\dot\omega^2)dt$.

Battery. From $V_{oc}=V_{term}+R_{int}I$ and $P_m=V_{term}I$: $R_{int}I^2-V_{oc}I+P_m=0$; the physical (low-current) root gives

$$
V_{oc}I=P_m+\frac{R_{int}}{V_{oc}^2}P_m^2+O(R_{int}^2).
$$

The criterion keeps only the zeroth order, $J\simeq\int P_m\,dt$. At hover ($P_m=28.6$ W, $V_{oc}\approx E_{OCV}=10.6$ V) the neglected relative term is $R_{int}P_m/V_{oc}^2\approx1.3\,\%$ (and $4R_{int}P_m/V_{oc}^2\approx0.05$, which justifies the expansion). The term grows with the power, so the neglected fraction is larger in the climb phases.

**Implemented discrete criterion** (`Pm_rows.m`: the **full** $P_m$ including the cross terms is evaluated, not the reduced form), Simpson's rule on each interval with $h=T_f/N$:

$$
\boxed{J=\frac{T_f}{6\,N\,P_{ref}}\sum_{k=1}^{N}\big[P_k+4P_{k+\frac12}+P_{k+1}\big]+c_T\,T_f}
$$

with $P_k=P_m(\omega_k,U_k)$, $P_{k+\frac12}=P_m\big(\tfrac{\omega_k+\omega_{k+1}}2,\tfrac{U_k+U_{k+1}}2\big)$ and $P_{ref}=P_m(\omega_h\mathbf 1,0)\approx28.6$ W (hover power). The energy term has the unit of a time (hover-equivalent seconds), like $T_f$, so $c_T$ [-] directly weights the arrival time against the energy. With $c_T=0$ the arrival time is not penalized and is only limited by $[T_{f,min},T_{f,max}]$.

> **Remark.** The alternative exact form $J=C_{batt}[\Phi(\mathrm{SoC}_0)-\Phi(\mathrm{SoC}_f)]$, $\Phi(s)=\sum_i\frac{\lambda_i}{i+1}s^{i+1}+\frac12\big(s^2\ln s-\frac{s^2}2\big)$, requires the SoC at every node and is not used.

### 5.4 Dynamic constraints: trapezoidal collocation

$$
d_k=X_{k+1}-X_k-\frac h2\big(F(X_k,U_k)+F(X_{k+1},U_{k+1})\big)=0,\qquad k=1,\dots,N,\quad h=\frac{T_f}{N},
$$

i.e. $16N$ equality constraints. Each $d_k$ depends only on $T_f$ and on the variables of nodes $k$ and $k+1$ (41 variables), which gives the sparsity exploited in section 6. The trapezoidal rule is second order (local error $O(h^3)$, global error $O(h^2)$ for a smooth solution).

### 5.5 Bounds and boundary conditions

| Quantity | Bound | Note |
|---|---|---|
| $x,y$ | $[0,55]$ m | flight volume |
| $z$ | $[0,30]$ m | |
| $\phi,\theta$ | $[-35^\circ,35^\circ]$ | keeps the Euler parametrization away from its singularity |
| $v,\psi,p,q,r$ | free | |
| $\omega_i$ | $[0.1,\,1.8]\,\omega_h=[20.2,\,363.9]$ rad/s | max thrust-to-weight ratio $1.8^2=3.24$ |
| $\dot\omega_i=U_i$ | $[-500,500]$ rad/s$^2$ | |
| $T_f$ | $[2,40]$ s | |

Boundary conditions (31 equalities): at $t=0$, $X_1=[\xi_0;\,v_0;\,0;0;0;\,0;0;0;\,\omega_h\mathbf1]$ (16); at $t=T_f$, $\xi=\xi_f$ (3), $v=v_f$ (3), $\phi=\theta=0$ (2), $p=q=r=0$ (3), $\omega_i=\omega_h$ (4). The final yaw $\psi_f$ is free. Scenario: $\xi_0=(0,0,0)$, $\xi_f=(50,50,0)$, $v_0=v_f=0$ (soft landing).

### 5.6 Obstacle avoidance: super-ellipsoids

Each building $b$ is an axis-aligned box $[x_b\pm d_{x}]\times[y_b\pm d_{y}]\times[0,h_z]$. With the safety margin $d_{safe}=1$ m, semi-axes $a=(d_x+d_{safe},\,d_y+d_{safe},\,h_z+d_{safe})$ and an even exponent $n=4$ (`p_sq`), the constraint at every node $k$ ($N+1$ per building) is

$$
s_b(\xi_k)=\Big(\frac{x_k-x_b}{a_x}\Big)^n+\Big(\frac{y_k-y_b}{a_y}\Big)^n+\Big(\frac{z_k}{a_z}\Big)^n\ \ge\ 1 .
$$

A larger $n$ approaches the box but worsens the conditioning of the NLP.

> **Limits.** (i) The constraint is imposed at the nodes only: collisions between two nodes are not excluded; the notebook checks $\min s_b\ge1$ a posteriori, which is the same condition. (ii) A super-ellipsoid with $n=4$ does **not** contain the box. Example, building B1 ($d_x=3$, $d_y=4$, $h_z=12$): at its top corner $s_b=(3/4)^4+(4/5)^4+(12/13)^4=1.45>1$, i.e. that corner is **not** in the forbidden region. The margin $d_{safe}$ therefore does not guarantee clearance at the corners; an additional check against the true box is needed.

> **Remedies.** The set $\{s_b<1\}$ is convex, so it contains the box as soon as it contains its 8 vertices; the worst one is the top corner, and the condition is $(d_x/a_x)^n+(d_y/a_y)^n+(h_z/a_z)^n<1$ for each building. Two levers:
> - a larger exponent, at the price of a worse conditioned NLP. With $d_{safe}=1$ m, the worst corner value over the six buildings of the notebook is $s_b=1.67$ for $n=4$, $1.26$ for $n=6$, $0.96$ for $n=8$ (B5, the least favourable) and $0.74$ for $n=10$: $n=8$ is the smallest even exponent that covers all the corners;
> - a larger margin at $n=4$: the worst value is $1.67$, $1.03$, $0.70$ for $d_{safe}=1,2,3$ m, so $d_{safe}\ge2.25$ m (0.25 m grid) is required.
>
> Whatever $n$, the clearance at the centre of each face is exactly $d_{safe}$.

Scenario buildings $[x_c,y_c,d_x,d_y,h_z]$ (m): B1 $[10,10,3,4,12]$, B2 $[30,15,5,3,7]$, B3 $[27,40,4,4,15]$, B4 $[35,25,3,6,9]$, B5 $[10,25,5,5,11]$, and B6 $[23,23,4,4,7]$ in the notebook but $[25,25,2,2,5]$ in `src/init_scenario.m` (to be unified).

### 5.7 Initial guess

Kinematic only (not dynamically consistent): horizontal "smoothstep" $s(\tau)=3\tau^2-2\tau^3$, $\tau\in[0,1]$, from start to target (zero speed at both ends); altitude "plateau": ascent over the first 15 % of the time, cruise at $z_{cruise}=\max_bh_{z,b}+2d_{safe}=17$ m, descent over the last 15 %; $\omega=\omega_h$, $U=0$, attitude and rates zero, $T_f^{init}=10$ s ($20$ s in `nlp_solv_fmincon.m`). Requires $z_{cruise}<z_{max}$.

### 5.8 Solver

`fmincon`, interior-point algorithm. Objective and constraint gradients by reverse-mode automatic differentiation (`prob2struct`); the Hessian of the Lagrangian is supplied through `HessianFcn` (section 6). Options: `ScaleProblem=true`, `MaxIterations=3000`, `OptimalityTolerance=ConstraintTolerance=1e-5`, `MaxFunctionEvaluations=1e7` (`src/nlp_solv_fmincon_2.m` additionally sets `StepTolerance=1e-3`). The CasADi/IPOPT versions in `src/old/` solve the same problem.

| Parameter | `main.mlx` | `nlp_solv_fmincon_2.m` | `nlp_solv_fmincon.m` |
|---|---|---|---|
| $N$ | 60 | `nlp.N` = 100 | `nlp.N` = 100 |
| $c_T$ | 1 | 5 | 0 |
| $T_f^{init}$, $[T_{f,min},T_{f,max}]$ | 10, [2, 40] | 10, [2, 40] | 20, [8, 40] |
| Hessian | exact | exact (L-BFGS selectable) | L-BFGS |

### 5.9 Validation on the 21-state model

The reference $\omega_{ref}$ is $X_{13:16}$ of the solution, held **constant on each interval** (value at node $k$ on $[t_k,t_{k+1}]$), and the 21-state model of section 4 is integrated with `ode45` ($\mathrm{RelTol}=10^{-5}$, $\mathrm{AbsTol}=10^{-7}$) from $X_1$, $\mathrm{SoC}_0=1$, $z=0$. Reported: terminal position, speed and attitude errors, and final SoC. The ideal (NLP) and simulated trajectories **do differ**: the NLP ignores the motor and battery dynamics (5.2) and the piecewise-constant hold differs from the piecewise-smooth profile assumed by the NLP. The terminal errors printed by the notebook quantify this gap; it is the expected consequence of the reduction, and the reason why the final simulation is compared with the ideal solution (3D view, top view, states, rotor speeds, SoC).

> **Limit of the open-loop approach.** Any disturbance is not detected and propagates to the end of the trajectory.

## 6. Exact Hessian of the Lagrangian

The interior-point algorithm of `fmincon` uses second-order information. It can be a quasi-Newton approximation (BFGS, L-BFGS; `nlp_solv_fmincon.m`, and `nlp_solv_fmincon_2.m` with `use_exact_hessian=false`) or the exact Hessian of the Lagrangian supplied through `HessianFcn` (`nlp_solv_fmincon_2.m`). The script prints the time per iteration for both options so that they can be compared; no timing is reported here.

### 6.1 Lagrangian and conventions

For $\min_zf(z)$ subject to $c_{eq}(z)=0$ and $c_{ineq}(z)\le0$,

$$
\mathcal L(z,\lambda)=f(z)+\lambda_{eq}^\top c_{eq}(z)+\lambda_{ineq}^\top c_{ineq}(z),\qquad
\nabla^2_{zz}\mathcal L=\nabla^2f+\sum_i\lambda_{eq,i}\nabla^2c_{eq,i}+\sum_j\lambda_{ineq,j}\nabla^2c_{ineq,j},
$$

(same sign for both types in the `fmincon` interior-point convention, with $\lambda_{ineq}\ge0$). The multipliers change at every iteration, so the Hessian must be re-evaluated and must include the constraints, not only the cost.

In this problem ($z\in\mathbb R^{n_z}$, $n_z=1+20(N+1)$):

| Part | Expression | Hessian |
|---|---|---|
| Cost | Simpson sum $\sum_kf_k$ (5.3) | nonzero, interval by interval |
| Nonlinear equalities | collocation defects $d_k$, $\lambda_{eq}$ ordered as $\mathrm{vec}(d)$, column-major $16\times N$ | nonzero, interval by interval |
| Nonlinear inequalities | $c_{b,k}=1-s_b(\xi_k)\le0$, $\lambda_{ineq}$ ordered $[\text{obst}_1\,(N{+}1);\ \text{obst}_2\,(N{+}1);\dots]$ | nonzero, diagonal |
| Bounds, boundary conditions, $c_TT_f$ | affine in $z$ | **zero** |

### 6.2 Interval-wise decomposition

For interval $k$ the local vector is

$$
z_k^{loc}=[\,T_f;\ X_k;\ U_k;\ X_{k+1};\ U_{k+1}\,]\in\mathbb R^{41}\quad(1+2\cdot(16+4)),\qquad z_k^{loc}=S_kz,
$$

with $S_k\in\{0,1\}^{41\times n_z}$ a selection matrix. Both the Simpson contribution $f_k=\frac{T_f}{6NP_{ref}}\big[P_k+4P_{k+\frac12}+P_{k+1}\big]$ and the defect $d_k$ (5.4) depend on $z$ **only through** $z_k^{loc}$. With the local multipliers $\lambda_k\in\mathbb R^{16}$,

$$
\mathcal L_k=f_k(z_k^{loc})+\lambda_k^\top d_k(z_k^{loc}),\qquad
\mathcal L=\sum_{k=1}^N\mathcal L_k+c_TT_f+(\text{affine terms})+(\text{obstacle terms}).
$$

For a function $g(S_kz)$ the chain rule gives $\nabla^2_{zz}g=S_k^\top(\nabla^2g)S_k$. Hence, with $H_k=\nabla^2_{z_k^{loc}}\mathcal L_k\in\mathbb R^{41\times41}$,

$$
\boxed{\nabla^2_{zz}\mathcal L=\underbrace{\sum_{k=1}^NS_k^\top H_kS_k}_{H_{colloc}}+H_{obst}}
$$

Intervals $k$ and $k+1$ share the variables of node $k+1$, and all intervals share $T_f$: their contributions overlap and **must be added**. The resulting matrix is symmetric and sparse (each node couples only to its two neighbours, and $T_f$ couples to everything).

### 6.3 Symbolic generation of the local block

`src/gen_hess_loc.m` (Symbolic Math Toolbox) declares the symbols $T_f,x_a,u_a,x_b,u_b,\lambda$, builds $d=x_b-x_a-\frac{T_f}{2N}(F(x_a,u_a)+F(x_b,u_b))$ and $f_k$ with `dyn_vec` and `Pm_rows`, computes `hessian(f_k + lam.'*d, zl)` and exports it with `matlabFunction(..., 'Optimize', true)` as `model/hess_loc.m`, a function `Hloc = hess_loc(zl, lam)` ($\mathbb R^{41}\times\mathbb R^{16}\to\mathbb R^{41\times41}$). The symbolic computation is done once; during the optimization the generated function is only evaluated numerically.

> **Validity.** $N$, $P_{ref}$, the `drone` and `mot` parameters are embedded in the generated function ($T_f$ remains symbolic). `data/generated_fmincon/hess_loc_meta.mat` stores them and `nlp_solv_fmincon_2.m` asserts that they are unchanged, otherwise it stops and requests a new run of `gen_hess_loc.m`. The constant $g$ has no influence on second derivatives (additive constant of $\dot v_z$).

### 6.4 Sparse assembly

The global index matrix `LOC` ($41\times N$) gives, for each interval, the positions of $z_k^{loc}$ in $z$ (obtained with `varindex(prob)`): `LOC(:,k)=[idx.Tf; IX(:,k); IU(:,k); IX(:,k+1); IU(:,k+1)]`. The row and column indices `hs.I`, `hs.J` of all $41^2N$ entries are computed once. At each call, the $N$ blocks are evaluated, stored column-wise in `V(:,k)=Hk(:)` and assembled with

```matlab
H = sparse(hs.I, hs.J, V(:), hs.nz, hs.nz);
```

`sparse` **sums the duplicated (i, j) entries**, which implements exactly $\sum_kS_k^\top H_kS_k$ without ever forming a dense matrix. For $N=100$: $n_z=2021$, i.e. $n_z^2\approx4.08\times10^6$ entries dense, against at most $41^2N=168{,}100$ stored triplets ($\le4.1\,\%$; fewer after merging duplicates). The per-call cost is $N$ evaluations of a $41\times41$ block plus the sparse assembly, independent of $n_z^2$.

### 6.5 Obstacle contribution

The constraint $c_{b,k}=1-s_b(\xi_k)$ involves only the position of node $k$ and $s_b$ is a sum of three single-variable terms, so the Hessian is **diagonal**: with $r\in\{x,y,z\}$, centre $r_c$ ($z_c=0$) and semi-axis $a_r$ (margin included),

$$
\frac{\partial^2c_{b,k}}{\partial r_k^2}=-\frac{n(n-1)\,(r_k-r_c)^{n-2}}{a_r^{\,n}},\qquad\text{mixed derivatives}=0 .
$$

For $n=4$: $-12(r_k-r_c)^2/a_r^4$. The exponent is even so that $(r-r_c)^{n-2}$ is a polynomial (twice continuously differentiable). Summing over the buildings,

$$
(H_{obst})_{r_k,r_k}=\sum_b\lambda_{ineq,(b,k)}\,\frac{\partial^2c_{b,k}}{\partial r_k^2}.
$$

Since $\lambda_{ineq}\ge0$ and $\partial^2c/\partial r^2\le0$, each entry is $\le0$: $H_{obst}\preceq0$, and it is nonzero only for the active constraints near a building. In the code the diagonal is accumulated in a vector `dg` and added with `spdiags`.

### 6.6 Terms with zero Hessian

Variable bounds, the 31 boundary conditions and $c_TT_f$ are affine in $z$ (they are passed to `fmincon` as bounds or linear constraints by `prob2struct`) and contribute nothing. The total is $H=H_{colloc}+H_{obst}$, $H\in\mathbb R^{n_z\times n_z}$, symmetric.

### 6.7 Numerical validation

Before solving (`check_hessian=true`), `check_hess_fd` compares $Hv$ with a central finite difference of the Lagrangian gradient, itself built from the automatic-differentiation gradients of `prob2struct`:

$$
\nabla\mathcal L=\nabla f+\nabla c_{ineq}\,\lambda_{ineq}+\nabla c_{eq}\,\lambda_{eq},\qquad
Hv\approx\frac{\nabla\mathcal L(z+\epsilon v)-\nabla\mathcal L(z-\epsilon v)}{2\epsilon},\ \ \epsilon=10^{-6}.
$$

The test is made at a randomly perturbed point ($z=z_0+0.01(1+|z_0|)\odot\mathrm{randn}$), with 3 random directions $v$, in four cases that isolate the source of an error: cost alone ($\lambda=0$), collocation alone, obstacles alone, all together. It also prints the asymmetry $\|H-H^\top\|_F/\|H\|_F$ and $\mathrm{nnz}(H)$. The run stops if the maximum relative error exceeds $10^{-4}$; an $O(1)$ error points to a wrong order or sign of the multipliers (`lambda.eqnonlin`, `lambda.ineqnonlin`) or to a wrong local block. The script's own comment expects errors of $10^{-7}$ or less.
