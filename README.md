# Quantitative-Resilience

This projects contains the codes used in the paper [Quantitative Resilience of Generalized Integrators](https://arxiv.org/abs/2111.04163) to be published soon in the IEEE Transactions of Automatic Control.


**File Structure**
---

1. The code `actuator_dynamics.m` calculates and plots the input and vertical velocity and position depending on the type of inputs chosen.
2. The code `translational_dynamics.m` calculates the resilience and time ratios for the translation dynamics in the case of the loss of control authority over each one of the propellers.
3. The code `rotational_dynamics.m` calculates the resilience and time ratios for the rotation dynamics in the case of the loss of control authority over each one of the propellers.
4. The functions `solution_unperturbed.m` and `solution_perturbed.m` solve linear optimization problem to calculate nominal and malfunctioning reach times for driftless dynamics as defined in the paper.
5. The function `Octorotor.m` generates the dynamics matrices for the translational and rotational dynamics of the octocopter.



**Running**
---




**Citation**
---
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

**Contributors**
---
- [Jean-Baptiste Bouvier](https://github.com/Jean-BaptisteBouvier)
- [Kathleen Xu](https://scholar.google.com/citations?user=d-zoJD0AAAAJ&hl=en)
- [Melkior Ornik](https://mornik.web.illinois.edu/)


