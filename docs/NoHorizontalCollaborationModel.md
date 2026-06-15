# Formulation

## Objective Function

The objective function of the HMDVRRP aims to minimize the total cost, comprising both travel and fixed costs. We define three sets of binary decision variables: let $x_{ij}^k$ and $y_{ij}^k$ equal $1$ if arc $(i,j) \in A$ is traversed by truck $k$ with and without the robot, respectively, and $0$ otherwise. Similarly, let $z_{ij}^k$ equal $1$ if the arc is traversed by the robot associated with truck $k$. The travel cost for a truck on arc $(i,j)$ is denoted by $c_{ij}^v$ (irrespective of the robot's presence), while the travel cost for a robot is denoted by $c_{ij}^r$. Additionally, a fixed cost $c_f$ is incurred for each truck utilized. The objective is formulated as follows:

$$
\min \sum_{k\in K} \sum_{(i,j)\in A} \left(c_{ij}^v x_{ij}^k + c_{ij}^v y_{ij}^k + c_{ij}^r z_{ij}^k\right) + \sum_{d\in V_D} \sum_{k\in K_d} \sum_{(i,d)\in A_D} c_f x_{id}^k
$$

Equation label: `eq:objective`

## Constraints

### 1. Arc Constraints

The arc constraints are used to govern the connectivity of nodes. To indicate whether a node functions as a TP, DP, and MP, we define three auxiliary binary decision variables. Let $\bar{k}\in K\backslash K_d=\overline{K_d}$ denote trucks that do not belong to depot $d$. Define the binary variable $\zeta_i^{\bar{k}}$ to be one if truck $\bar{k}$ receives transferred goods at node $i$. Let $\xi_i^k$ be a binary variable equal to one if truck $k$ deploys a robot on node $i$, and $\delta_i^k$ be a binary variable equal to one if truck $k$ retrieves a robot at node $i$.

$$
\sum_{(d,j)\in A} x_{dj}^k \leq 1, \quad \forall d\in V_D, k\in K_d
$$

Equation label: `eq:c2`

$$
\sum_{(i,j)\in A} (x_{ij}^k + y_{ij}^k) \leq 1, \quad \forall k\in K, j\in V
$$

Equation label: `eq:c3`

$$
x_{ij}^k + y_{ij}^k + z_{ij}^k \leq \sum_{(d,l)\in A} x_{dl}^k, \quad \forall d\in V_D, (i,j)\in A, k\in K_d
$$

Equation label: `eq:c4`

$$
\sum_{k\in K} \sum_{(i,j)\in A_C} (x_{ij}^k + y_{ij}^k + z_{ij}^k) = 1 + \sum_{\bar{k}\in\overline{K_d}} \zeta_j^{\bar{k}} + \sum_{k\in K} \delta_j^k, \quad \forall d\in V_D, j\in V_C^d
$$

Equation label: `eq:c5`

$$
\sum_{k\in K} \sum_{(i,j)\in A_P} (x_{ij}^k + y_{ij}^k + z_{ij}^k) = \max\left\{2\sum_{k\in K} \delta_j^k, \sum_{k\in K} \xi_j^k\right\}, \quad \forall j\in V_P
$$

Equation label: `eq:c6`

$$
\sum_{(i,j)\in A} x_{ij}^k = \sum_{(j,i)\in A} x_{ji}^k - \delta_j^k + \xi_j^k, \quad \forall j\in V, k\in K
$$

Equation label: `eq:c7`

$$
\sum_{(i,j)\in A} y_{ij}^k = \sum_{(j,i)\in A} y_{ji}^k + \delta_j^k - \xi_j^k, \quad \forall j\in V, k\in K
$$

Equation label: `eq:c8`

$$
\sum_{(i,j)\in A} z_{ij}^k = \sum_{(j,i)\in A} z_{ji}^k + \delta_j^k - \xi_j^k \leq 1, \quad \forall j\in V, k\in K
$$

Equation label: `eq:c9`

$$
\sum_{k\in K_d} \sum_{(i,j)\in A} (x_{ij}^k + y_{ij}^k + z_{ij}^k) \leq 1 + \sum_{k\in K_d} \delta_j^k, \quad \forall d\in V_D, j\in V_d
$$

Equation label: `eq:c10`

$$
\sum_{\bar{k}\in\overline{K_d}} \zeta_j^{\bar{k}} \leq \sum_{k\in K_d} \sum_{(i,j)\in A_{CP}} (x_{ij}^k + y_{ij}^k + z_{ij}^k), \quad \forall d\in V_D, j\in V_C^d
$$

Equation label: `eq:c11`

$$
\zeta_j^{\bar{k}} \leq \sum_{(i,j)\in A} (x_{ij}^{\bar{k}} + y_{ij}^{\bar{k}} + z_{ij}^{\bar{k}}), \quad \forall d\in V_D, j\in V_d\backslash V_P^d, \bar{k}\in\overline{K_d}
$$

Equation label: `eq:c12`

$$
2\delta_j^k \leq \sum_{(i,j)\in A_{CP}} (x_{ij}^k + y_{ij}^k + z_{ij}^k), \quad \forall j\in V_{CP}, k\in K
$$

Equation label: `eq:c13`

$$
\xi_j^k \leq \sum_{(i,j)\in A_P} x_{ij}^k, \quad \forall j\in V_P, k\in K
$$

Equation label: `eq:c14`

$$
\sum_{k\in K} (\xi_i^k + \delta_i^k) + \sum_{\bar{k}\in\overline{K_d}} \zeta_i^{\bar{k}} \leq 1, \quad \forall d\in V_D, i\in V_C^d
$$

Equation label: `eq:c15`

$$
\sum_{k\in K} (\xi_i^k + \delta_i^k) \leq 2, \quad \forall d\in V_D, i\in V_P^d
$$

Equation label: `eq:c16`

$$
\sum_{k\in K} \xi_i^k, \sum_{k\in K} \delta_i^k \leq 1, \quad \forall i\in V
$$

Equation label: `eq:c17`

$$
\xi_i^k + \delta_i^{k'} \leq 1, \quad \forall i\in V_P, k, k'\in K, k\neq k'
$$

Equation label: `eq:c18`

$$
\sum_{\bar{k}\in\overline{K_d}} \zeta_i^{\bar{k}} \leq 1, \quad \forall d\in V_D, i\in V_C^d
$$

Equation label: `eq:c19`

$$
\zeta_d^{\bar{k}} = \sum_{(d,j)\in A} x_{dj}^{\bar{k}}, \quad \forall d\in V_D, \bar{k}\in\overline{K_d}
$$

Equation label: `eq:c20`

$$
\sum_{(i,j)\in A_P} z_{ij}^k = 0, \quad \forall k\in K
$$

Equation label: `eq:c21`

$$
\zeta_i^k = 0, \quad \forall k\in K, i\in V_C''\cup V_P
$$

Equation label: `eq:c24`

$$
\xi_i^k = 0, \quad \forall k\in K, i\in V_D\cup V_C
$$

Equation label: `eq:c25`

$$
\delta_i^k = 0, \quad \forall k\in K, i\in V_C''\cup V_D
$$

Equation label: `eq:c26`

$$
x_{ij}^k, y_{ij}^k, z_{ij}^k \in \{0,1\}, \quad \forall (i,j)\in A, k\in K
$$

Equation label: `eq:c22`

$$
\zeta_i^k, \xi_i^k, \delta_i^k \in \{0,1\}, \quad \forall k\in K, i\in V
$$

Equation label: `eq:c23`

Constraint `eq:c2` imposes that each truck departs from the depot at most once, while Constraint `eq:c3` prevents any truck from visiting a node more than once. Constraint `eq:c4` guarantees that all routes originate from depots and that each arc is traversed at most once.

Constraints `eq:c5` and `eq:c6` govern the visit frequency for customer and parking nodes. Specifically, each customer must be served at least once. However, if a customer node functions as a TP or MP, it necessitates one additional visit. A parking node serving as an MP requires one visit from a robot and one from a truck; conversely, if it serves as a DP, it requires one visit from a truck.

Constraints `eq:c7`-`eq:c9` enforce arc flow balance. Constraint `eq:c10` restricts trucks and robots to accessing only those nodes associated with their assigned depot. Constraint `eq:c11` stipulates that each CTP must be visited at least once by a truck from the same depot, whereas Constraint `eq:c12` requires each TP to be visited at least once by a truck from a different depot.

Constraints `eq:c3` and `eq:c13` ensure that each MP is accessed once by a robot and once by a truck, irrespective of the truck's robot-carrying state. Constraint `eq:c14` mandates that each DP is visited at least once by a truck carrying a robot. Constraint `eq:c15` enforces the mutual exclusivity of customer node roles, ensuring a node acts as at most one of DP, MP, or TP.

Constraints `eq:c16` and `eq:c17` allow parking nodes to function as both a DP and an MP, while Constraint `eq:c18` ensures that each parking node is exclusively utilized by at most one truck. Constraint `eq:c19` ensures that at most one route receives goods at each CTP. Constraint `eq:c20` guarantees that trucks load goods when visiting foreign depots. Finally, Constraint `eq:c21` prohibits robot travel between parking nodes. Constraints `eq:c24`-`eq:c26` are role-restriction constraints that eliminate infeasible node-role assignments. Specifically, Constraint `eq:c24` excludes transfer reception at robot-only customers and parking nodes; Constraint `eq:c25` restricts robot deployment to parking nodes; and Constraint `eq:c26` excludes robot retrieval at depots and robot-only customers.

Constraints `eq:c22`-`eq:c23` define the binary domains of the routing and auxiliary variables.

### 2. Load Constraints

The following set of constraints governs the flow of goods along both truck and robot routes, encompassing transfer, deployment, and retrieval operations. The continuous variables $Q_{idk}^v$ and $Q_{idk}^r$ represent the quantity of goods originating from depot $d$ that are transported by truck $k$ and robot $k$, respectively, upon departing from node $i$. The variable $q_i^T$ denotes the volume of goods transferred at node $i$. Key model parameters include the demand $q_i$ at each customer node $i$ (assumed to be zero at depots and parking nodes), the maximum truck capacity $U_v$, and the robot capacity $U_r$. Finally, the constant $q^{\max}$ represents the maximum customer demand, defined as $q^{\max} = \max_{j\in V}(q_j)$.

$$
\begin{split}
Q_{jdk}^v + q_j + q_j^T \leq Q_{idk}^v + (2U_v + q^{\max})(1 - x_{ij}^k - y_{ij}^k + \xi_j^k + \delta_j^k), \\
\qquad \forall d\in V_D, k\in K_d, i\in V, j\in V_{CP}^d
\end{split}
$$

Equation label: `eq:c27`

$$
\begin{split}
Q_{jd\bar{k}}^v + (1 - \zeta_j^{\bar{k}})q_j - q_j^T \leq Q_{id\bar{k}}^v + (U_v + q^{\max})(1 - x_{ij}^{\bar{k}} - y_{ij}^{\bar{k}} + \xi_j^{\bar{k}} + \delta_j^{\bar{k}}), \\
\qquad \forall d\in V_D, \bar{k}\in\overline{K_d}, i\in V, j\in V_d
\end{split}
$$

Equation label: `eq:c28`

$$
Q_{jdk}^v \leq Q_{idk}^v + U_v(1 - x_{ij}^k - y_{ij}^k + \xi_j^k + \delta_j^k), \quad \forall d\in V_D, k\in K, i\in V, j\in V\setminus V_d
$$

Equation label: `eq:c29`

$$
Q_{jdk}^r \leq U_v\left(1 - \sum_{(i,j)\in A}(x_{ij}^k + y_{ij}^k) + \xi_j^k\right), \quad \forall d\in V_D, k\in K, j\in V
$$

Equation label: `eq:c30`

$$
Q_{jdk}^r + q_j \leq Q_{idk}^r + (U_r + q^{\max})(1 - z_{ij}^k + \xi_j^k + \delta_j^k), \quad \forall d\in V_D, k\in K, i\in V, j\in V_{CP}^d
$$

Equation label: `eq:c31`

$$
Q_{jdk}^r \leq Q_{idk}^r + U_r(1 - z_{ij}^k + \xi_j^k + \delta_j^k), \quad \forall d\in V_D, k\in K, i\in V, j\in V\setminus V_d
$$

Equation label: `eq:c32`

$$
Q_{jdk}^v \leq U_v\left(1 - \sum_{(i,j)\in A} z_{ij}^k + \delta_j^k\right), \quad \forall d\in V_D, k\in K, j\in V
$$

Equation label: `eq:c33`

$$
Q_{jdk}^v + Q_{jdk}^r \leq Q_{idk}^v + (U_v + U_r)(2 - x_{ij}^k - \xi_j^k + \delta_j^k), \quad \forall d\in V_D, i\in V, j\in V_P, k\in K
$$

Equation label: `eq:c34`

$$
Q_{jdk}^v + q_j \leq Q_{idk}^v + Q_{ldk}^r + (U_v + q^{\max})(2 - y_{ij}^k - z_{lj}^k), \quad \forall d\in V_D, i, l\in V, j\in V_{CP}, k\in K
$$

Equation label: `eq:c35`

$$
Q_{jdk}^v + Q_{jdk}^r \leq Q_{idk}^v + Q_{ldk}^r + (U_v + U_r)(2 - x_{ij}^k - z_{lj}^k), \quad \forall d\in V_D, i, l\in V, j\in V_P, k\in K
$$

Equation label: `eq:c36`

$$
\sum_{d\in V_D\setminus\{i\}} (Q_{idk}^v + Q_{idk}^r) \leq 0, \quad \forall i\in V_D, k\in K_i
$$

Equation label: `eq:c37`

$$
Q_{ddk}^r \leq 0, \quad \forall d\in V_D, k\in K_d
$$

Equation label: `eq:c38`

$$
\sum_{d\in V_D} Q_{idk}^v \leq U_v, \quad \forall k\in K, i\in V
$$

Equation label: `eq:c39`

$$
\sum_{d\in V_D} Q_{idk}^r \leq U_r, \quad \forall k\in K, i\in V
$$

Equation label: `eq:c40`

$$
0 \leq q_i^T \leq U_v \zeta_i^{\bar{k}}, \quad \forall d\in V_D, i\in V_C^d, \bar{k}\in\overline{K_d}
$$

Equation label: `eq:c41`

$$
Q_{idk}^v, Q_{idk}^r \geq 0, \quad \forall d\in V_D, i\in V, k\in K
$$

Equation label: `eq:c42`

Constraints `eq:c27`-`eq:c30` govern the load flow conservation for trucks traversing an arc $(i,j)\in A$, provided the destination node $j$ is neither a DP nor an MP. Specifically, Constraint `eq:c27` accounts for load changes when a truck visits a node belonging to its own depot, whereas Constraint `eq:c28` addresses cases where the truck and the visited node belong to different depots. Constraints `eq:c29` and `eq:c30` ensure the invariance of load variables for goods that are not involved in the transaction at the current node.

Similarly, Constraints `eq:c31`-`eq:c33` describe the load evolution for robots, excluding arcs ending at an MP. Special attention is given to robot interaction points: Constraint `eq:c34` handles load transfers at nodes functioning exclusively as DPs, where goods are transferred from the truck to the robot. Conversely, Constraint `eq:c35` defines load changes at nodes functioning exclusively as MPs, while Constraint `eq:c36` covers nodes serving simultaneously as both a DP and an MP.

Constraint `eq:c37` stipulates that trucks must not carry goods from other depots upon leaving their origin. Constraints `eq:c38` and `eq:c37` initialize robot routes with an empty load. Constraints `eq:c39` and `eq:c40` enforce capacity limitations for both trucks and robots. Finally, Constraints `eq:c41` and `eq:c42` define the domains of the continuous load variables.

### 3. Tour Size Limitation

In the HMDVRRP, operational range and energy limitations are imposed on both trucks and robots, though modeled through different mechanisms.
For trucks, the constraint is distance-based: the maximum travel distance per tour is restricted by a range limit $\mathscr{D}$, which acts as a proxy for energy consumption and driver shift durations. Since inventory can be replenished via transfers from other depots or trucks, a truck's operation is primarily bound by this distance limit rather than cargo capacity. The impact of truck range on horizontal collaboration is further examined in the subsequent analysis.
Conversely, for robots, the constraint is energy-based: each route is strictly limited by the battery capacity $\mathscr{E}$ and the consumption rate $\varepsilon$. To track this, a continuous variable $e_i^k$ is defined to represent the remaining battery level of the robot $k$ upon arrival at node $i$. The constant $e$ represents the maximum energy consumption for any single leg of a robot's route, defined as $e = \max_{(i,j)\in A}(d_{ij})\varepsilon$.

$$
\sum_{(i,j)\in A} d_{ij}(x_{ij}^k + y_{ij}^k) \leq \mathscr{D}, \quad \forall k\in K
$$

Equation label: `eq:c43`

$$
e_j^k + d_{ij}\varepsilon \leq e_i^k + (\mathscr{E} + e)(1 - z_{ij}^k + \delta_j^k), \quad \forall k\in K, (i,j)\in A_{CP}
$$

Equation label: `eq:c44`

$$
d_{ij}\varepsilon \leq e_i^k + e(2 - z_{ij}^k - \delta_j^k), \quad \forall k\in K, (i,j)\in A_{CP}
$$

Equation label: `eq:c45`

$$
0 \leq e_i^k \leq \mathscr{E}, \quad \forall k\in K, i\in V\setminus V_D
$$

Equation label: `eq:c46`

Constraint `eq:c43` restricts the truck's total travel distance. Constraint `eq:c44` tracks robot energy consumption and eliminates subtours. Constraint `eq:c45` ensures the robot retains sufficient battery to reach the MP. Constraint `eq:c46` defines the domain of the continuous variables $e_i^k$.

### 4. Route Interaction Coordination

As routes interact through CTPs, the timing of truck access must be temporally feasible to ensure proper synchronization. Temporal constraints are applied to guarantee that the truck transferring goods arrives at the CTP before the truck receiving the goods. Let $t_{ik}$ be a continuous variable representing the time when truck $k$ leaves node $i$. Let $\tau_{ij}$ be the travel time for a truck from node $i$ to node $j$.

$$
x_{ij}^k + y_{ij}^k = 1 \Rightarrow t_{ik} + \tau_{ij} \leq t_{jk}, \quad \forall d\in V_D, k\in K_d, i\in V, j\in V\setminus\{d\}
$$

Equation label: `eq:c47`

$$
\zeta_i^{\bar{k}} = 1 \Rightarrow t_{ik} \leq t_{i\bar{k}}, \quad \forall d\in V_D, k\in K_d, \bar{k}\in\overline{K_d}, i\in V_C^d
$$

Equation label: `eq:c48`

$$
t_{ik} \geq 0, \quad \forall k\in K, i\in V
$$

Equation label: `eq:c49`

Constraint `eq:c47` governs the progression of time along a route. Constraint `eq:c48` ensures that the transferring truck departs from the CTP before the receiving truck arrives. Constraint `eq:c49` defines the domain of temporal variables $t_{ik}$.

### 5. Constraints Linearization

Constraint `eq:c6` involves a non-linear maximum function. To preserve the model's linearity, we reformulate it by introducing two auxiliary binary variables, $\phi_j$ and $\psi_j$, for each $j\in V_P$. The original constraint is equivalently replaced by the set of linear constraints `eq:c50`-`eq:c55`.

$$
2\sum_{k\in K} \delta_j^k - \sum_{k\in K} \sum_{(i,j)\in A_P} (x_{ij}^k + y_{ij}^k + z_{ij}^k) \leq 0, \quad \forall j\in V_P
$$

Equation label: `eq:c50`

$$
\sum_{k\in K} \xi_j^k - \sum_{k\in K} \sum_{(i,j)\in A_P} (x_{ij}^k + y_{ij}^k + z_{ij}^k) \leq 0, \quad \forall j\in V_P
$$

Equation label: `eq:c51`

$$
\sum_{k\in K} \sum_{(i,j)\in A_P} (x_{ij}^k + y_{ij}^k + z_{ij}^k) - 2\sum_{k\in K} \delta_j^k \leq 2(1 - \phi_j), \quad \forall j\in V_P
$$

Equation label: `eq:c52`

$$
\sum_{k\in K} \sum_{(i,j)\in A_P} (x_{ij}^k + y_{ij}^k + z_{ij}^k) - \sum_{k\in K} \xi_j^k \leq 2(1 - \psi_j), \quad \forall j\in V_P
$$

Equation label: `eq:c53`

$$
\phi_j + \psi_j \geq 1, \quad \forall j\in V_P
$$

Equation label: `eq:c54`

$$
\phi_j, \psi_j \in \{0,1\}, \quad \forall j\in V_P
$$

Equation label: `eq:c55`
