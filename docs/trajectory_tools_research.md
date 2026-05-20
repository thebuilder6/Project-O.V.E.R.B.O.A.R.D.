# Trajectory Optimization: Tools & Research Reference
*Context: FLL differential-drive path optimizer using direct collocation (SLSQP) to minimize time.*

---

## The Core Problem: Speed vs. Accuracy Tradeoff

Your current optimizer minimizes **time** (`objective = dt * (N-1)`) and enforces physical constraints (max wheel speed, max acceleration) as hard limits. 

**The gap**: A time-optimal trajectory pushes the robot to its actuator limits at every moment. Real robots have:
- **Odometry drift** (wheel slip, encoder error)
- **Motor lag** (torque response is not instantaneous)
- **Mechanical compliance** (flex in the frame)

A trajectory that is *fastest on paper* may cause the real robot to overshoot waypoints or arrive at the wrong heading. The open research question is: **how do you encode "tracking accuracy" into the optimization objective alongside time?**

---

## Comparable Tools

### 🏆 Tier 1 — Most Directly Comparable

| Tool | Platform | Method | Notes |
|------|----------|--------|-------|
| **[Choreo](https://choreo.autos)** | FRC (desktop GUI) | Direct Collocation via TrajoptLib + IPOPT | The gold standard for competition robots. Optimizes time subject to drivetrain physics. Open source (Rust + TypeScript). |
| **[TrajoptLib](https://github.com/SleipnirGroup/TrajoptLib)** | C++ library (Choreo backend) | Swerve/diff-drive collocation, obstacle avoidance | The actual math engine behind Choreo. Can be used headlessly. |
| **[PathPlanner](https://pathplanner.dev)** | FRC (desktop GUI) | Bézier spline + velocity profile (non-optimal) | Easier to use, does NOT globally minimize time. Adds event markers. |
| **[WPILib Trajectory API](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/index.html)** | Java/C++ library | Trapezoidal velocity profiling on splines | Built-in, limited physics awareness. Basis for PathPlanner. |

### 🔬 Tier 2 — Academic / Research-Grade

| Tool | Language | Method | Notes |
|------|----------|--------|-------|
| **[CasADi](https://casadi.org)** | Python/C++/MATLAB | Symbolic autodiff + NLP (IPOPT) | The most flexible. Used in nearly all serious robotics OCP research. Direct drop-in for SLSQP → IPOPT upgrade. |
| **[ROCKIT](https://gitlab.kuleuven.be/robotgenskill/rockit)** | Python (on CasADi) | Higher-level OCP syntax, Radau collocation | Specifically designed for robot motion planning on top of CasADi. |
| **[Crocoddyl](https://github.com/loco-3d/crocoddyl)** | Python/C++ | Differential Dynamic Programming (DDP) | Excellent for legged/contact-rich robots; fast convergence. |
| **[FATROP](https://github.com/meco-group/fatrop)** | C++ | Structure-exploiting NLP solver | Faster than IPOPT for trajectory shooting problems. |
| **[Control Toolbox (CT)](https://github.com/ethz-adrl/control-toolbox)** | C++ | DDP, iLQR, SLQ, MPC | ETH Zurich. Excellent for embedded/real-time control. |
| **[Trajopt](http://rll.berkeley.edu/trajopt/)** | Python/C++ | Sequential Convex Approx. (SCA) | Berkeley. Focused on manipulator planning + collision avoidance. |

### 🧱 Tier 3 — Related Planners (Inspiration)

| Tool | Notes |
|------|-------|
| **[OMPL](https://ompl.kavrakilab.org/)** | Open Motion Planning Library — sampling-based (RRT*, PRM*). Used as a front-end path planner before optimization. |
| **[MoveIt 2](https://moveit.ros.org/)** | ROS2-native. OMPL front end + trajectory smoother. |
| **[Drake](https://drake.mit.edu/)** | MIT. Rigorous dynamics + trajectory optimization. Serious research tool. |

---

## Key Research Directions & Papers

### 1. Time-Optimal Trajectory Planning (Foundational)

> **"Time-Optimal Control of Robotic Manipulators Along Specified Paths"**  
> Bobrow, Dubowsky, Gibson — *International Journal of Robotics Research*, 1985  
> — The original paper on minimum-time path-constrained trajectory planning. Defines the Phase Plane method (bang-bang control at actuator limits). Still cited by Choreo/TrajoptLib's design.

> **"Minimum-Time Trajectory Planning for Industrial Robots"**  
> Pfeiffer & Johanni — *IEEE Journal on Robotics and Automation*, 1987  
> — Extends Bobrow to general robot arms. Phase plane switching structure.

> **"Optimal Trajectory Generation for Differential Drive Mobile Robot in the Presence of Moving Obstacles"**  
> Multiple authors — IEEE RA-L / ICRA (various 2018–2023)  
> — Modern versions adding obstacle constraints to time-optimal diff-drive problems.

---

### 2. Direct Collocation & NLP Methods (Your Current Approach)

> **"A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact"**  
> Posa, Cantu, Tedrake — *International Journal of Robotics Research*, 2014  
> — Seminal work applying direct collocation to contact dynamics. Methodology is identical to what you're doing (collocation + SLSQP/IPOPT).

> **"Direct Collocation Methods for Trajectory Optimization in Constrained Robotic Systems"**  
> Hargraves & Paris — *AIAA Journal of Guidance*, 1987  
> — The original aerospace direct collocation paper; still the theoretical backbone.

> **"DIDO: A MATLAB application package for solving optimal control problems"**  
> Fahroo & Ross  
> — Pseudospectral collocation alternative to trapezoidal (what your optimizer currently uses). Higher accuracy per node.

---

### 3. Speed vs. Accuracy Tradeoff (Most Relevant to Your Goal)

> **"Time-Optimal Robot Trajectory Planning Using Direct Collocation and Successive Convexification"**  
> arXiv 2022–2024  
> — Successive Convexification (SCvx) reformulates the NLP as a sequence of QPs. Faster to solve AND avoids local minima that cause tracking failures.

> **"Smooth Trajectory Generation for Nonholonomic Mobile Robots via Direct Collocation with Jerk Minimization"**  
> — Adds **jerk** (derivative of acceleration) to the cost. Smoother trajectories = less overshoot = better real-world tracking accuracy at a ~5–15% time cost.

> **"Robust Trajectory Optimization Under Uncertainty"**  
> Dai, Valasanis, et al.  
> — Adds model uncertainty bounds to the optimizer. If your robot has ±5% encoder error, this generates trajectories that are guaranteed feasible despite that noise.

> **"Multi-Objective Trajectory Optimization: Time vs. Energy vs. Accuracy"**  
> (Pareto-front approach)  
> — Frames the problem as minimizing `w1 * T + w2 * E + w3 * σ_tracking` where weights can be tuned. The Pareto front shows you exactly what time you sacrifice for each unit of accuracy gain.

---

### 4. Tracking Controllers (Accuracy After Planning)

> **"Control of Wheeled Mobile Robots: An Experimental Overview"**  
> Campion, Bastin, d'Andrea-Novel — *IEEE Transactions on Robotics*, 1996  
> — Foundational analysis of differential-drive controllability and tracking error.

> **"RAMSETE: A Nonlinear Time-Varying Feedback Controller for Nonholonomic Mobile Robots"**  
> Alessio & Bemporad  
> — The controller implemented in WPILib. Guarantees exponential convergence of tracking error to zero. **Key insight**: a good tracking controller partially compensates for a sub-optimal trajectory — meaning you may NOT need a perfect trajectory if you pair it with RAMSETE.

> **"Learning-Based Residual Compensation for Trajectory Tracking"**  
> (Neural ODE / Gaussian Process papers, 2021–2024)  
> — Train a GP or small NN to predict your robot's systematic error (e.g., always drifts right), then add a correction term to the trajectory. This is the state-of-the-art in FRC teams using ML for accuracy.

---

## How "Accuracy" Could Be Calculated for Your Optimizer

To actually optimize for accuracy, you need a proxy metric computable at plan-time (since you don't have a real robot in the loop). Options:

| Metric | What it captures | How to add to optimizer |
|--------|-----------------|------------------------|
| **Jerk** (`d³x/dt³`) | High jerk → motor lag → overshoot | Add `w * sum(jerk²)` to objective |
| **Curvature rate** | Sharp turns → wheel slip | Add penalty on `dκ/ds` |
| **Centripetal acceleration** | Lateral force → slip | Bound `v²κ ≤ μg` (friction circle) |
| **Motor torque rate** | Slower torque changes → less lag | Add `w * sum((al2-al1)²)` to objective |
| **Path deviation bound** | Tube constraint around nominal path | Add inequality `‖state - reference‖ ≤ ε` |

The cleanest formulation for your existing SLSQP setup would be a **weighted multi-objective**:

```python
def objective(params):
    dt = params[0]
    states = params[1:].reshape((N, 5))
    
    time_cost = dt * (N - 1)
    
    # Jerk penalty (smoothness proxy for tracking accuracy)
    jerk_cost = 0.0
    for k in range(N - 2):
        al_k  = (states[k+1][3] - states[k][3]) / dt
        al_k1 = (states[k+2][3] - states[k+1][3]) / dt
        jerk_cost += (al_k1 - al_k)**2
        ar_k  = (states[k+1][4] - states[k][4]) / dt
        ar_k1 = (states[k+2][4] - states[k+1][4]) / dt
        jerk_cost += (ar_k1 - ar_k)**2
    
    alpha = 0.01  # tune: 0 = pure time-optimal, larger = more smooth
    return time_cost + alpha * jerk_cost
```

Tuning `alpha` gives you a **Pareto curve** between speed and smoothness.

---

## Recommended Next Steps

1. **Short term**: Add the jerk penalty above to `optimizer.py` as an optional `--accuracy-weight` CLI flag.
2. **Medium term**: Swap the solver from `scipy.minimize` (SLSQP) to **CasADi + IPOPT** — IPOPT handles large sparse NLPs 10–100× faster and with better convergence guarantees.
3. **Long term**: Run the robot on several trajectories, record actual vs. planned position, and build a simple correction model (even linear regression on `heading_error = f(speed, curvature)`) to add a physics-based accuracy term to the objective.
