# mknap_pso
“Copyright 2021 Patricia Wong”

Particle swarm optimisation (PSO) used to solve a multi-dimensional knapsack problem (classic NP-Hard combinatorial optimisition problem).

### Introduction 
The Particle Swarm Optimization (PSO) is a swarm algorithm based of the flocking behavior of birds, where a swarm of birds seek for food. PSO was traditionally designed for continuous optimization problems. However, in this coursework PSO was used to solve a discrete problem, a multi-dimensional knapsack problem. This report describes the multi-dimensional knapsack problem and the adaption to PSO. The algorithm design was reference from the paper titled “Solving the Multidimensional Knapsack Problem using a CUDA accelerated PSO” (D. Zan and J. Jaros, 2014). This program is written in C Programming Language. 

### Discrete version of the PSO 
To use PSO for discrete problem, there are some changes made to the algorithm. Unlike in the continuous space, the multidimensional solution vector (particle position) is expressed in discrete binary values (0/1) – binary encoding. The velocity of a particle is expressed as a vector of probabilities, the velocity formula remains the same, but the calculation of the new particle position changes slightly. To limit the particle velocity to stay within an interval of [0,1], the velocity value is normalized using a sigmoid transformation function.

### Algorithm Design Implementation 

The flow diagram below shows the implemented algorithm. 

![image](https://user-images.githubusercontent.com/78403270/190069630-c2f3b5b7-9d1e-403c-8a47-0bac0d597e80.png)
