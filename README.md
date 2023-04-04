# Quantitative Resilience of Generalized Integrators

This projects contains the codes used in the paper [Quantitative Resilience of Generalized Integrators](https://arxiv.org/abs/2111.04163) to be published soon in the IEEE Transactions of Automatic Control. 


## Quantitative Resilience Framework

This work focuses on quantifying the resilience of linear systems of nominal dynamics
$$\dot{x}(t) = \bar{B} \bar{u}(t), \qquad x(0) = x_0 \in \mathbb{R}^n, \qquad \bar{u}(t) \in \bar{\mathcal{U}},$$
where $\bar{B} \in \mathbb{R}^{n \times (m+p)}$ is a constant matrix. After a *loss of control authority* over $p$ of the initial $m+p$ actuators of the system, we split control matrix $\bar{B}$ into two submatrices $B \in \mathbb{R}^{n \times m}$ and $C \in \mathbb{R}^{n \times p}$ representing respectively the controlled and uncontrolled actuators. Similarly, the input signal $\bar{u}$ and its constraint set $\bar{\mathcal{U}}$ are split between the admissible control signal $u$ belonging to compact set $\mathcal{U}$, and the uncontrolled and possibly undesirable input signal $w$ taking values in compact set $\mathcal{W}$. Then, the dynamics of the malfunctioning system can be written as
$$\dot{x}(t) = Bu(t) + Cw(t), \qquad x(0) = x_0 \in \mathbb{R}^n, \qquad u(t) \in \mathcal{U}, \quad w(t) \in \mathcal{W}.$$
A target set $\mathcal{T} \subseteq \mathbb{R}^n$ is *resiliently reachable* if for all undesirable input $w$ there exists a control input $u$ driving the state of the malfunctioning system from $x_0$ to $\mathcal{T}$ in a finite time.

The nominal system is *resilient* to this partial loss of control authority if any target set is resiliently reachable.
However, because of the malfunction the system might need excessively longer to reach the same target compared to the nominal dynamics. To quantify the maximal delay due to the partial loss of control authority we introduce the *nominal reach time*

$$T_N^*(x_g) := \underset{\bar{u}(t) \\, \in \\, \bar{\mathcal{U}} }{\inf} \left\\{ T : x_g - x_0 = \int_0^T \bar{B} \bar{u}(t) dt \right\\},$$

and the *malfunctioning reach time*

$$T_M^*(x_g) := \underset{w(t) \\, \in \\, \mathcal{W} }{\sup} \left\\{ \underset{u(t) \\, \in \\, \mathcal{U} }{\inf} \left\\{ T : x_g - x_0 = \int_0^T Bu(t) + Cw(t) dt \right\\} \right\\}.$$

The *quantitative resilience* of this system is then defined as

$$r_q := \underset{x_g \\, \in \\, \mathbb{R}^n}{\inf} \frac{T_N^* (x_g)}{T_M^* (x_g)}.$$

Note that $r_q$ is the solution of a nonlinear optimization consisting of four nested optimization problems, three of which being over infinite dimensional function sets. Relying on time-optimal control theory we simplified the expressions of the reach times $T_N^*$ and $T_M^*$. Then, we established the [Maximax Minimax Quotient Theorem](https://github.com/Jean-BaptisteBouvier/Maximax-Minimax) to solve the nonlinear optimization over $x_g$ in $r_q$.




## File Structure

1. The code `actuator_dynamics.m` calculates and plots the input and vertical velocity and position depending on the type of inputs chosen.
2. The code `translational_dynamics.m` calculates the resilience and time ratios for the translation dynamics in the case of the loss of control authority over each one of the propellers.
3. The code `rotational_dynamics.m` calculates the resilience and time ratios for the rotation dynamics in the case of the loss of control authority over each one of the propellers.
4. The functions `solution_unperturbed.m` and `solution_perturbed.m` solve linear optimization problem to calculate nominal and malfunctioning reach times for driftless dynamics as defined in the paper.
5. The function `Octorotor.m` generates the dynamics matrices for the translational and rotational dynamics of the octocopter.






## Citation

```
@article{bouvier2023quantitative,  
  title = {Quantitative Resilience of Generalized Integrators},   
  author = {Jean-Baptiste Bouvier, Kathleen Xu and Melkior Ornik},    
  journal = {IEEE Transactions on Automatic Control},    
  year = {2023},   
  volume = {},  
  number = {},  
  pages = {}  
}
```

## Contributors

- [Jean-Baptiste Bouvier](https://jean-baptistebouvier.github.io/)
- [Kathleen Xu](https://scholar.google.com/citations?user=d-zoJD0AAAAJ&hl=en)
- [Melkior Ornik](https://mornik.web.illinois.edu/)


