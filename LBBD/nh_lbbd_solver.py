from __future__ import annotations

import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

import gurobipy as gp
import matplotlib.pyplot as plt
from gurobipy import GRB


Arc = Tuple[int, int]
TruckArcKey = Tuple[int, int, int]
NodeKey = Tuple[int, int]
INT_TOL = 0.5
FRAC_SUPPORT_TOL = 1e-4
FRAC_CUT_TOL = 1e-6


@dataclass
class DepotContext:
    instance_name: str
    depot: int
    nodes: List[int]
    customers: List[int]
    parking: List[int]
    cp_nodes: List[int]
    regular_customers: List[int]
    robot_only_customers: List[int]
    trucks: List[int]
    a_v: List[Arc]
    a_r: List[Arc]
    truck_in_arcs: Dict[int, List[Arc]]
    truck_out_arcs: Dict[int, List[Arc]]
    robot_in_arcs: Dict[int, List[Arc]]
    robot_out_arcs: Dict[int, List[Arc]]
    q: Sequence[float]
    veh_dist_matrix: Sequence[Sequence[float]]
    rob_power_matrix: Sequence[Sequence[float]]
    veh_cost_matrix: Sequence[Sequence[float]]
    rob_cost_matrix: Sequence[Sequence[float]]
    veh_time_matrix: Sequence[Sequence[float]]
    uv: float
    ur: float
    qjmax: float
    max_travel_time: float
    robot_ub: float
    veh_dist_ub: float
    fixed_cost: float
    big_m: float
    e_max: float


@dataclass
class TruckMasterSolution:
    truck: int
    active: bool
    x: Dict[Arc, int]
    y: Dict[Arc, int]
    z: Dict[Arc, int]
    xi: Dict[int, int]
    delta: Dict[int, int]
    selected_truck_arcs: List[Arc]
    selected_robot_arcs: List[Arc]
    truck_in_count: Dict[int, int]
    robot_in_count: Dict[int, int]
    signature: Tuple[int, ...]


@dataclass
class FarkasFeasibilityCut:
    min_activity: float
    constant_term: float
    x_coeffs: Dict[Arc, float]
    y_coeffs: Dict[Arc, float]
    z_coeffs: Dict[Arc, float]
    xi_coeffs: Dict[int, float]
    delta_coeffs: Dict[int, float]


@dataclass
class SubproblemCheckResult:
    feasible: bool
    farkas_cut: Optional[FarkasFeasibilityCut] = None


@dataclass
class DepotSolveResult:
    depot: int
    status: int
    status_name: str
    objective: Optional[float]
    best_bound: Optional[float]
    mip_gap: Optional[float]
    runtime: float
    x: Dict[TruckArcKey, float]
    y: Dict[TruckArcKey, float]
    z: Dict[TruckArcKey, float]
    xi: Dict[NodeKey, float]
    delta: Dict[NodeKey, float]
    subproblem_calls: int
    warm_start_used: bool
    warm_start_objective: Optional[float]
    root_truck_cuts: int
    truck_subtour_cuts: int
    robot_subtour_cuts: int
    no_good_cuts: int
    farkas_cuts: int
    screening_rejections: int


def _status_name(status: int) -> str:
    status_map = {
        GRB.OPTIMAL: "OPTIMAL",
        GRB.TIME_LIMIT: "TIME_LIMIT",
        GRB.INFEASIBLE: "INFEASIBLE",
        GRB.INTERRUPTED: "INTERRUPTED",
        GRB.SUBOPTIMAL: "SUBOPTIMAL",
        GRB.SOLUTION_LIMIT: "SOLUTION_LIMIT",
    }
    return status_map.get(status, str(status))


def _basic_infeasibility_reason(context: DepotContext) -> Optional[str]:
    total_demand = sum(context.q[customer] for customer in context.customers)
    total_capacity = len(context.trucks) * context.uv
    if total_demand > total_capacity + 1e-9:
        return "total_demand_exceeds_total_capacity"
    if any(context.q[customer] > context.uv + 1e-9 for customer in context.customers):
        return "customer_demand_exceeds_truck_capacity"

    robot_arc_set = set(context.a_r)
    for customer in context.robot_only_customers:
        if context.q[customer] > context.ur + 1e-9:
            return f"robot_only_customer_{customer}_exceeds_robot_capacity"
        feasible = False
        for parking in context.parking:
            if (parking, customer) not in robot_arc_set or (customer, parking) not in robot_arc_set:
                continue
            total_power = (
                context.rob_power_matrix[parking][customer]
                + context.rob_power_matrix[customer][parking]
            )
            if total_power <= context.robot_ub + 1e-9:
                feasible = True
                break
        if not feasible:
            return f"robot_only_customer_{customer}_has_no_feasible_parking"

    return None


def _screened_depot_result(
    context: DepotContext,
    status: int,
    runtime: float = 0.0,
) -> DepotSolveResult:
    return DepotSolveResult(
        depot=context.depot,
        status=status,
        status_name=_status_name(status),
        objective=None,
        best_bound=None,
        mip_gap=None,
        runtime=runtime,
        x={},
        y={},
        z={},
        xi={},
        delta={},
        subproblem_calls=0,
        warm_start_used=False,
        warm_start_objective=None,
        root_truck_cuts=0,
        truck_subtour_cuts=0,
        robot_subtour_cuts=0,
        no_good_cuts=0,
        farkas_cuts=0,
        screening_rejections=0,
    )


def _format_float(value: Optional[float]) -> str:
    return "None" if value is None else f"{value:.2f}"


def _strongly_connected_components(nodes: Sequence[int], arcs: Sequence[Arc]) -> List[List[int]]:
    if not arcs:
        return []

    active_nodes = set()
    adjacency = {node: [] for node in nodes}
    reverse = {node: [] for node in nodes}
    for i, j in arcs:
        adjacency.setdefault(i, []).append(j)
        reverse.setdefault(j, []).append(i)
        active_nodes.add(i)
        active_nodes.add(j)

    visited = set()
    order: List[int] = []

    def forward_dfs(node: int) -> None:
        visited.add(node)
        for nxt in adjacency.get(node, []):
            if nxt not in visited:
                forward_dfs(nxt)
        order.append(node)

    for node in active_nodes:
        if node not in visited:
            forward_dfs(node)

    visited.clear()
    components: List[List[int]] = []

    def reverse_dfs(node: int, component: List[int]) -> None:
        visited.add(node)
        component.append(node)
        for prev in reverse.get(node, []):
            if prev not in visited:
                reverse_dfs(prev, component)

    for node in reversed(order):
        if node in active_nodes and node not in visited:
            component: List[int] = []
            reverse_dfs(node, component)
            components.append(component)

    return components


def _cycle_components(nodes: Sequence[int], arcs: Sequence[Arc]) -> List[List[int]]:
    components = []
    for component in _strongly_connected_components(nodes, arcs):
        component_set = set(component)
        internal_edge_count = sum(1 for i, j in arcs if i in component_set and j in component_set)
        if len(component) > 1 and internal_edge_count >= len(component):
            components.append(sorted(component))
    return components


def _build_incidence(nodes: Sequence[int], arcs: Sequence[Arc]) -> Tuple[Dict[int, List[Arc]], Dict[int, List[Arc]]]:
    incoming = {node: [] for node in nodes}
    outgoing = {node: [] for node in nodes}
    for i, j in arcs:
        outgoing[i].append((i, j))
        incoming[j].append((i, j))
    return incoming, outgoing


def _build_depot_contexts(my_instance) -> List[DepotContext]:
    vp_set = set(my_instance.VP)
    vc1_set = set(my_instance.VC1)
    vc2_set = set(my_instance.VC2)
    q = my_instance.demand
    qjmax = max(q) if q else 0.0
    arc_block_value = 1e5

    contexts: List[DepotContext] = []
    for depot in my_instance.VD:
        nodes = list(my_instance.depot_with_node[depot])
        customers = list(my_instance.cus_with_depot[depot])
        parking = [node for node in nodes if node in vp_set]
        cp_nodes = customers + parking
        regular_customers = [node for node in customers if node in vc1_set]
        robot_only_customers = [node for node in customers if node in vc2_set]
        trucks = list(my_instance.veh_with_depot[depot])

        parking_set = set(parking)

        a_v = [
            (i, j)
            for i in nodes
            for j in nodes
            if i != j
            and my_instance.vehicle_cost_matrix[i][j] < arc_block_value
            and my_instance.vehicle_time_matrix[i][j] < arc_block_value
            and my_instance.veh_dist_matrix[i][j] <= my_instance.veh_dist_ub
            and (j not in cp_nodes or my_instance.vehicle_time_matrix[i][j] <= my_instance.max_travel_time)
        ]
        a_r = [
            (i, j)
            for i in cp_nodes
            for j in cp_nodes
            if i != j
            and my_instance.robot_cost_matrix[i][j] < arc_block_value
            and my_instance.rob_power_matrix[i][j] <= my_instance.robot_ub
            and not (i in parking_set and j in parking_set)
        ]

        truck_in_arcs, truck_out_arcs = _build_incidence(nodes, a_v)
        robot_in_arcs, robot_out_arcs = _build_incidence(cp_nodes, a_r)
        e_max = max((my_instance.rob_power_matrix[i][j] for i, j in a_r), default=0.0)

        contexts.append(
            DepotContext(
                instance_name=my_instance.name,
                depot=depot,
                nodes=nodes,
                customers=customers,
                parking=parking,
                cp_nodes=cp_nodes,
                regular_customers=regular_customers,
                robot_only_customers=robot_only_customers,
                trucks=trucks,
                a_v=a_v,
                a_r=a_r,
                truck_in_arcs=truck_in_arcs,
                truck_out_arcs=truck_out_arcs,
                robot_in_arcs=robot_in_arcs,
                robot_out_arcs=robot_out_arcs,
                q=q,
                veh_dist_matrix=my_instance.veh_dist_matrix,
                rob_power_matrix=my_instance.rob_power_matrix,
                veh_cost_matrix=my_instance.vehicle_cost_matrix,
                rob_cost_matrix=my_instance.robot_cost_matrix,
                veh_time_matrix=my_instance.vehicle_time_matrix,
                uv=my_instance.veh_cap,
                ur=my_instance.rob_cap,
                qjmax=qjmax,
                max_travel_time=my_instance.max_travel_time,
                robot_ub=my_instance.robot_ub,
                veh_dist_ub=my_instance.veh_dist_ub,
                fixed_cost=my_instance.cf,
                big_m=my_instance.very_big,
                e_max=e_max,
            )
        )
    return contexts


def _solve_single_depot(
    context: DepotContext,
    feasibility_cut_mode: str,
    solve_mode: str,
    heuristic_time_limit: Optional[float],
) -> DepotSolveResult:
    solver = SingleDepotLBBD(
        context,
        feasibility_cut_mode=feasibility_cut_mode,
        solve_mode=solve_mode,
        heuristic_time_limit=heuristic_time_limit,
    )
    return solver.solve()


def _solve_depot_contexts(
    contexts: Sequence[DepotContext],
    parallel_depots: bool,
    max_parallel_depots: Optional[int],
    feasibility_cut_mode: str,
    solve_mode: str,
    heuristic_time_limit: Optional[float],
) -> List[DepotSolveResult]:
    if not contexts:
        return []

    screened = {context.depot: _basic_infeasibility_reason(context) for context in contexts}
    if any(reason is not None for reason in screened.values()):
        return [
            _screened_depot_result(
                context,
                GRB.INFEASIBLE if screened[context.depot] is not None else GRB.INTERRUPTED,
            )
            for context in contexts
        ]

    if not parallel_depots or len(contexts) <= 1:
        return [
            _solve_single_depot(context, feasibility_cut_mode, solve_mode, heuristic_time_limit)
            for context in contexts
        ]

    worker_cap = max_parallel_depots if max_parallel_depots is not None else (os.cpu_count() or 1)
    max_workers = max(1, min(len(contexts), worker_cap))
    if max_workers <= 1:
        return [
            _solve_single_depot(context, feasibility_cut_mode, solve_mode, heuristic_time_limit)
            for context in contexts
        ]

    results: List[DepotSolveResult] = []
    with ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="LBBDDepot") as executor:
        futures = {
            executor.submit(
                _solve_single_depot,
                context,
                feasibility_cut_mode,
                solve_mode,
                heuristic_time_limit,
            ): context.depot
            for context in contexts
        }
        for future in as_completed(futures):
            results.append(future.result())

    return sorted(results, key=lambda result: result.depot)


class TruckFeasibilitySubproblem:
    def __init__(self, context: DepotContext, truck: int):
        self.context = context
        self.truck = truck
        self.env = gp.Env(empty=True)
        self.env.setParam("OutputFlag", 0)
        self.env.start()

        self.model = gp.Model(
            name=f"{context.instance_name}-d{context.depot}-k{truck}-subproblem",
            env=self.env,
        )
        self.model.setParam("OutputFlag", 0)
        self.model.setParam("Threads", 1)
        self.model.setParam("Method", 1)
        self.model.setParam("DualReductions", 0)
        self.model.setParam("InfUnbdInfo", 1)

        self.qv = self.model.addVars(context.nodes, lb=0.0, ub=context.uv, name="QV")
        self.qr = self.model.addVars(context.nodes, lb=0.0, ub=context.ur, name="QR")
        self.e = self.model.addVars(context.cp_nodes, lb=0.0, ub=context.robot_ub, name="E")
        self.tv = self.model.addVars(context.nodes, lb=0.0, ub=context.max_travel_time, name="TV")

        self.c27: List[Tuple[gp.Constr, int, int]] = []
        self.c30: List[Tuple[gp.Constr, int]] = []
        self.c31: List[Tuple[gp.Constr, int, int]] = []
        self.c33: List[Tuple[gp.Constr, int]] = []
        self.c34: List[Tuple[gp.Constr, int, int]] = []
        self.c35: List[Tuple[gp.Constr, int, int, int]] = []
        self.c36: List[Tuple[gp.Constr, int, int, int]] = []
        self.c44: List[Tuple[gp.Constr, int, int]] = []
        self.c45: List[Tuple[gp.Constr, int, int]] = []
        self.c47: List[Tuple[gp.Constr, int, int]] = []

        for i in context.nodes:
            for j in context.cp_nodes:
                if i != j:
                    constr = self.model.addConstr(self.qv[j] - self.qv[i] <= 0.0, name=f"c27[{i},{j}]")
                    self.c27.append((constr, i, j))

        for j in context.cp_nodes:
            constr = self.model.addConstr(self.qr[j] <= 0.0, name=f"c30[{j}]")
            self.c30.append((constr, j))

        for i, j in context.a_r:
            constr = self.model.addConstr(self.qr[j] - self.qr[i] <= 0.0, name=f"c31[{i},{j}]")
            self.c31.append((constr, i, j))

        for j in context.cp_nodes:
            constr = self.model.addConstr(self.qv[j] <= 0.0, name=f"c33[{j}]")
            self.c33.append((constr, j))

        for i in context.nodes:
            for j in context.parking:
                if i != j:
                    constr = self.model.addConstr(
                        self.qv[j] + self.qr[j] - self.qv[i] <= 0.0,
                        name=f"c34[{i},{j}]",
                    )
                    self.c34.append((constr, i, j))

        for i in context.nodes:
            for l in context.cp_nodes:
                for j in context.regular_customers:
                    if i != j and l != j:
                        constr = self.model.addConstr(
                            self.qv[j] - self.qv[i] - self.qr[l] <= 0.0,
                            name=f"c35[{i},{l},{j}]",
                        )
                        self.c35.append((constr, i, l, j))

        for i in context.nodes:
            for l in context.cp_nodes:
                for j in context.parking:
                    if i != j and l != j:
                        constr = self.model.addConstr(
                            self.qv[j] + self.qr[j] - self.qv[i] - self.qr[l] <= 0.0,
                            name=f"c36[{i},{l},{j}]",
                        )
                        self.c36.append((constr, i, l, j))

        self.c38 = self.model.addConstr(self.qr[context.depot] <= 0.0, name="c38")

        for i, j in context.a_r:
            constr = self.model.addConstr(self.e[j] - self.e[i] <= 0.0, name=f"c44[{i},{j}]")
            self.c44.append((constr, i, j))

        for i, j in context.a_r:
            constr = self.model.addConstr(-self.e[i] <= 0.0, name=f"c45[{i},{j}]")
            self.c45.append((constr, i, j))

        for i, j in context.a_v:
            if j in context.cp_nodes:
                constr = self.model.addConstr(self.tv[i] - self.tv[j] <= 0.0, name=f"c47[{i},{j}]")
                self.c47.append((constr, i, j))

        self.model.setObjective(0.0, GRB.MINIMIZE)
        self.model.update()

    def _minimum_aggregated_activity(self, aggregated_coeffs: Dict[gp.Var, float]) -> float:
        min_activity = 0.0
        for var, coeff in aggregated_coeffs.items():
            if abs(coeff) <= 1e-9:
                continue
            if coeff > 0:
                min_activity += coeff * var.LB
            else:
                min_activity += coeff * var.UB
        return min_activity

    def _build_farkas_feasibility_cut(
        self,
        solution: TruckMasterSolution,
    ) -> Optional[FarkasFeasibilityCut]:
        tol = 1e-9
        context = self.context
        c27_m = context.uv + context.qjmax
        c31_m = context.ur + context.qjmax
        c34_m = context.uv + context.ur
        c35_m = context.uv + context.qjmax
        c44_m = context.robot_ub + context.e_max
        c47_m = context.big_m

        aggregated_coeffs: Dict[gp.Var, float] = {}
        x_coeffs: Dict[Arc, float] = {}
        y_coeffs: Dict[Arc, float] = {}
        z_coeffs: Dict[Arc, float] = {}
        xi_coeffs: Dict[int, float] = {}
        delta_coeffs: Dict[int, float] = {}
        constant_term = 0.0

        def add_coeff(store, key, value):
            if abs(value) <= tol:
                return
            store[key] = store.get(key, 0.0) + value
            if abs(store[key]) <= tol:
                del store[key]

        def accumulate_row(constr: gp.Constr) -> float:
            lam = constr.FarkasDual
            if abs(lam) <= tol:
                return 0.0
            row = self.model.getRow(constr)
            for idx in range(row.size()):
                var = row.getVar(idx)
                aggregated_coeffs[var] = aggregated_coeffs.get(var, 0.0) + lam * row.getCoeff(idx)
            return lam

        accumulate_row(self.c38)

        for constr, i, j in self.c27:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (c27_m - context.q[j])
            add_coeff(x_coeffs, (i, j), -lam * c27_m)
            add_coeff(y_coeffs, (i, j), -lam * c27_m)
            add_coeff(xi_coeffs, j, lam * c27_m)
            add_coeff(delta_coeffs, j, lam * c27_m)

        for constr, j in self.c30:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * context.ur
            for i, _ in context.truck_in_arcs[j]:
                add_coeff(x_coeffs, (i, j), -lam * context.ur)
                add_coeff(y_coeffs, (i, j), -lam * context.ur)
            add_coeff(xi_coeffs, j, lam * context.ur)

        for constr, i, j in self.c31:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (c31_m - context.q[j])
            add_coeff(z_coeffs, (i, j), -lam * c31_m)
            add_coeff(xi_coeffs, j, lam * c31_m)
            add_coeff(delta_coeffs, j, lam * c31_m)

        for constr, j in self.c33:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * context.uv
            for i, _ in context.robot_in_arcs[j]:
                add_coeff(z_coeffs, (i, j), -lam * context.uv)
            add_coeff(delta_coeffs, j, lam * context.uv)

        for constr, i, j in self.c34:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (2.0 * c34_m)
            add_coeff(x_coeffs, (i, j), -lam * c34_m)
            add_coeff(xi_coeffs, j, -lam * c34_m)
            add_coeff(delta_coeffs, j, lam * c34_m)

        for constr, i, l, j in self.c35:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (2.0 * c35_m - context.q[j])
            add_coeff(y_coeffs, (i, j), -lam * c35_m)
            add_coeff(z_coeffs, (l, j), -lam * c35_m)

        for constr, i, l, j in self.c36:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (2.0 * c34_m)
            add_coeff(x_coeffs, (i, j), -lam * c34_m)
            add_coeff(z_coeffs, (l, j), -lam * c34_m)

        for constr, i, j in self.c44:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (c44_m - context.rob_power_matrix[i][j])
            add_coeff(z_coeffs, (i, j), -lam * c44_m)
            add_coeff(delta_coeffs, j, lam * c44_m)

        for constr, i, j in self.c45:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (2.0 * context.e_max - context.rob_power_matrix[i][j])
            add_coeff(z_coeffs, (i, j), -lam * context.e_max)
            add_coeff(delta_coeffs, j, -lam * context.e_max)

        for constr, i, j in self.c47:
            lam = accumulate_row(constr)
            if lam == 0.0:
                continue
            constant_term += lam * (c47_m - context.veh_time_matrix[i][j])
            add_coeff(x_coeffs, (i, j), -lam * c47_m)
            add_coeff(y_coeffs, (i, j), -lam * c47_m)

        min_activity = self._minimum_aggregated_activity(aggregated_coeffs)

        current_value = constant_term
        for arc, coeff in x_coeffs.items():
            current_value += coeff * solution.x.get(arc, 0)
        for arc, coeff in y_coeffs.items():
            current_value += coeff * solution.y.get(arc, 0)
        for arc, coeff in z_coeffs.items():
            current_value += coeff * solution.z.get(arc, 0)
        for node, coeff in xi_coeffs.items():
            current_value += coeff * solution.xi.get(node, 0)
        for node, coeff in delta_coeffs.items():
            current_value += coeff * solution.delta.get(node, 0)

        if current_value >= min_activity - 1e-6:
            return None

        return FarkasFeasibilityCut(
            min_activity=min_activity,
            constant_term=constant_term,
            x_coeffs=x_coeffs,
            y_coeffs=y_coeffs,
            z_coeffs=z_coeffs,
            xi_coeffs=xi_coeffs,
            delta_coeffs=delta_coeffs,
        )

    def check_feasibility(self, solution: TruckMasterSolution) -> SubproblemCheckResult:
        context = self.context
        c27_m = context.uv + context.qjmax
        c31_m = context.ur + context.qjmax
        c34_m = context.uv + context.ur
        c35_m = context.uv + context.qjmax
        c44_m = context.robot_ub + context.e_max
        c47_m = context.big_m

        for constr, i, j in self.c27:
            rhs = c27_m * (
                1
                - solution.x.get((i, j), 0)
                - solution.y.get((i, j), 0)
                + solution.xi.get(j, 0)
                + solution.delta.get(j, 0)
            ) - context.q[j]
            constr.RHS = rhs

        for constr, j in self.c30:
            rhs = context.ur * (
                1
                - solution.truck_in_count.get(j, 0)
                + solution.xi.get(j, 0)
            )
            constr.RHS = rhs

        for constr, i, j in self.c31:
            rhs = c31_m * (
                1
                - solution.z.get((i, j), 0)
                + solution.xi.get(j, 0)
                + solution.delta.get(j, 0)
            ) - context.q[j]
            constr.RHS = rhs

        for constr, j in self.c33:
            rhs = context.uv * (
                1
                - solution.robot_in_count.get(j, 0)
                + solution.delta.get(j, 0)
            )
            constr.RHS = rhs

        for constr, i, j in self.c34:
            rhs = c34_m * (
                2
                - solution.x.get((i, j), 0)
                - solution.xi.get(j, 0)
                + solution.delta.get(j, 0)
            )
            constr.RHS = rhs

        for constr, i, l, j in self.c35:
            rhs = c35_m * (
                2
                - solution.y.get((i, j), 0)
                - solution.z.get((l, j), 0)
            ) - context.q[j]
            constr.RHS = rhs

        for constr, i, l, j in self.c36:
            rhs = c34_m * (
                2
                - solution.x.get((i, j), 0)
                - solution.z.get((l, j), 0)
            )
            constr.RHS = rhs

        for constr, i, j in self.c44:
            rhs = c44_m * (
                1
                - solution.z.get((i, j), 0)
                + solution.delta.get(j, 0)
            ) - context.rob_power_matrix[i][j]
            constr.RHS = rhs

        for constr, i, j in self.c45:
            rhs = context.e_max * (
                2
                - solution.z.get((i, j), 0)
                - solution.delta.get(j, 0)
            ) - context.rob_power_matrix[i][j]
            constr.RHS = rhs

        for constr, i, j in self.c47:
            rhs = c47_m * (
                1
                - solution.x.get((i, j), 0)
                - solution.y.get((i, j), 0)
            ) - context.veh_time_matrix[i][j]
            constr.RHS = rhs

        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            return SubproblemCheckResult(feasible=True)
        if self.model.Status == GRB.INFEASIBLE:
            return SubproblemCheckResult(
                feasible=False,
                farkas_cut=self._build_farkas_feasibility_cut(solution),
            )
        return SubproblemCheckResult(feasible=False)

    def dispose(self) -> None:
        if hasattr(self.model, "dispose"):
            self.model.dispose()
        if hasattr(self.env, "dispose"):
            self.env.dispose()

class SingleDepotLBBD:
    def __init__(
        self,
        context: DepotContext,
        feasibility_cut_mode: str = "exact",
        solve_mode: str = "exact",
        heuristic_time_limit: Optional[float] = None,
    ):
        self.context = context
        allowed_cut_modes = {"exact", "xyz_only"}
        if feasibility_cut_mode not in allowed_cut_modes:
            raise ValueError(
                f"Unsupported feasibility_cut_mode={feasibility_cut_mode!r}. "
                f"Choose from {sorted(allowed_cut_modes)}."
            )
        allowed_solve_modes = {"exact", "heuristic"}
        if solve_mode not in allowed_solve_modes:
            raise ValueError(
                f"Unsupported solve_mode={solve_mode!r}. "
                f"Choose from {sorted(allowed_solve_modes)}."
            )
        self.feasibility_cut_mode = feasibility_cut_mode
        self.solve_mode = solve_mode
        self.heuristic_time_limit = heuristic_time_limit
        self.env = gp.Env(empty=True)
        self.env.setParam("OutputFlag", 0)
        self.env.start()

        self.master = gp.Model(name=f"{context.instance_name}-depot-{context.depot}-LBBD", env=self.env)
        self.log_file = None
        self.master.setParam("TimeLimit", 10800)
        self.master.setParam("Threads", 1)
        self.master.setParam("OutputFlag", False)
        self.master.setParam("Method", 1)
        self.master.setParam("MIPGap", 0.0001)
        self.master.setParam("IntegralityFocus", 1)
        self.master.setParam("LazyConstraints", 1)
        self.master.setParam("PreCrush", 1)
        if self.solve_mode == "heuristic":
            self.master.setParam("MIPFocus", 1)
            self.master.setParam("Heuristics", 0.95)
            self.master.setParam("RINS", 20)
            self.master.setParam("PumpPasses", 10)

        self.truck_subtour_cuts = 0
        self.root_truck_cuts = 0
        self.robot_subtour_cuts = 0
        self.no_good_cuts = 0
        self.farkas_cuts = 0
        self.screening_rejections = 0
        self.subproblem_calls = 0
        self.root_truck_cut_cache: Dict[int, Set[Tuple[int, ...]]] = {
            truck: set() for truck in context.trucks
        }
        self.feasibility_cache: Dict[int, Dict[Tuple[int, ...], bool]] = {
            truck: {} for truck in context.trucks
        }

        self.free_xi_nodes = list(context.parking)
        self.free_delta_nodes = [node for node in context.cp_nodes if node not in context.robot_only_customers]
        self.truck_arc_set = set(context.a_v)
        self.warm_start_used = False
        self.warm_start_objective: Optional[float] = None

        self._build_master()
        self.subproblems = {
            truck: TruckFeasibilitySubproblem(context, truck)
            for truck in context.trucks
        }
        self._apply_truck_only_warm_start()

    def _truck_in_expr(self, node: int, truck: int, var: gp.tupledict) -> gp.LinExpr:
        return gp.quicksum(var[i, j, truck] for i, j in self.context.truck_in_arcs[node])

    def _truck_out_expr(self, node: int, truck: int, var: gp.tupledict) -> gp.LinExpr:
        return gp.quicksum(var[i, j, truck] for i, j in self.context.truck_out_arcs[node])

    def _robot_in_expr(self, node: int, truck: int) -> gp.LinExpr:
        return gp.quicksum(self.Z[i, j, truck] for i, j in self.context.robot_in_arcs[node])

    def _robot_out_expr(self, node: int, truck: int) -> gp.LinExpr:
        return gp.quicksum(self.Z[i, j, truck] for i, j in self.context.robot_out_arcs[node])

    def _departure_expr(self, truck: int) -> gp.LinExpr:
        return gp.quicksum(
            self.X[i, j, truck]
            for i, j in self.context.truck_out_arcs[self.context.depot]
            if i == self.context.depot
        )

    def _truck_arc_sort_key(self, arc: Arc) -> Tuple[float, float, float, int]:
        i, j = arc
        return (
            self.context.veh_cost_matrix[i][j],
            self.context.veh_time_matrix[i][j],
            self.context.veh_dist_matrix[i][j],
            j,
        )

    def _robot_arc_sort_key(self, arc: Arc) -> Tuple[float, float, int]:
        i, j = arc
        return (
            self.context.rob_cost_matrix[i][j],
            self.context.rob_power_matrix[i][j],
            j,
        )

    def _build_reduced_neighborhood(
        self,
        truck_neighbor_cap: Optional[int] = None,
        depot_neighbor_cap: Optional[int] = None,
        robot_neighbor_cap: Optional[int] = None,
        anchor_robot_cap: Optional[int] = None,
    ) -> Tuple[Set[Arc], Set[Arc]]:
        context = self.context
        truck_neighbor_cap = min(
            len(context.nodes) - 1,
            truck_neighbor_cap if truck_neighbor_cap is not None else max(6, len(context.customers) // 4 + len(context.trucks)),
        )
        depot_neighbor_cap = min(
            len(context.cp_nodes),
            depot_neighbor_cap if depot_neighbor_cap is not None else max(2 * len(context.trucks) + 4, truck_neighbor_cap),
        )
        robot_neighbor_cap = min(
            max(1, len(context.cp_nodes) - 1),
            robot_neighbor_cap if robot_neighbor_cap is not None else max(4, len(context.customers) // 6 + 2),
        )
        anchor_robot_cap = max(1, anchor_robot_cap if anchor_robot_cap is not None else max(2, robot_neighbor_cap))

        allowed_truck: Set[Arc] = set()
        for node in context.nodes:
            outgoing = sorted(context.truck_out_arcs[node], key=self._truck_arc_sort_key)
            incoming = sorted(context.truck_in_arcs[node], key=self._truck_arc_sort_key)
            out_cap = depot_neighbor_cap if node == context.depot else truck_neighbor_cap
            allowed_truck.update(outgoing[:out_cap])
            allowed_truck.update(incoming[:truck_neighbor_cap])

        for node in context.nodes:
            if node != context.depot and (node, context.depot) in self.truck_arc_set:
                allowed_truck.add((node, context.depot))

        allowed_robot: Set[Arc] = set()
        if context.a_r:
            for node in context.cp_nodes:
                outgoing = sorted(context.robot_out_arcs[node], key=self._robot_arc_sort_key)
                incoming = sorted(context.robot_in_arcs[node], key=self._robot_arc_sort_key)
                cap = robot_neighbor_cap + (1 if node in context.parking else 0)
                allowed_robot.update(outgoing[:cap])
                allowed_robot.update(incoming[:cap])

            for parking in context.parking:
                outgoing = sorted(context.robot_out_arcs[parking], key=self._robot_arc_sort_key)
                incoming = sorted(context.robot_in_arcs[parking], key=self._robot_arc_sort_key)
                allowed_robot.update(outgoing[:anchor_robot_cap])
                allowed_robot.update(incoming[:anchor_robot_cap])

        return allowed_truck, allowed_robot

    def _apply_branch_priorities(self) -> None:
        departure_arcs = {
            (i, j)
            for i, j in self.context.truck_out_arcs[self.context.depot]
            if i == self.context.depot
        }
        for truck in self.context.trucks:
            for i, j in self.context.a_v:
                self.X[i, j, truck].BranchPriority = 80 if (i, j) in departure_arcs else 60
                self.Y[i, j, truck].BranchPriority = 45
            for node in self.context.nodes:
                self.Xi[node, truck].BranchPriority = 25
                self.Delta[node, truck].BranchPriority = 25
            for i, j in self.context.a_r:
                self.Z[i, j, truck].BranchPriority = 10
        for node in self.context.parking:
            self.Phi[node].BranchPriority = 1
            self.Psi[node].BranchPriority = 1

    def _route_metrics(self, route: Sequence[int]) -> Optional[Tuple[float, float, float, float]]:
        if not route:
            return 0.0, 0.0, 0.0, 0.0

        load = 0.0
        prefix_time = 0.0
        travel_distance = 0.0
        travel_cost = self.context.fixed_cost
        previous = self.context.depot

        for node in route:
            if (previous, node) not in self.truck_arc_set:
                return None
            load += self.context.q[node]
            prefix_time += self.context.veh_time_matrix[previous][node]
            travel_distance += self.context.veh_dist_matrix[previous][node]
            travel_cost += self.context.veh_cost_matrix[previous][node]
            previous = node

        if (previous, self.context.depot) not in self.truck_arc_set:
            return None

        travel_distance += self.context.veh_dist_matrix[previous][self.context.depot]
        travel_cost += self.context.veh_cost_matrix[previous][self.context.depot]
        return load, prefix_time, travel_distance, travel_cost

    def _route_is_feasible(self, route: Sequence[int]) -> Optional[Tuple[float, float, float, float]]:
        metrics = self._route_metrics(route)
        if metrics is None:
            return None

        load, prefix_time, travel_distance, travel_cost = metrics
        if load > self.context.uv + 1e-9:
            return None
        if prefix_time > self.context.max_travel_time + 1e-9:
            return None
        if travel_distance > self.context.veh_dist_ub + 1e-9:
            return None
        return load, prefix_time, travel_distance, travel_cost

    def _build_truck_only_solution(self, truck: int, route: Sequence[int]) -> TruckMasterSolution:
        x = {arc: 0 for arc in self.context.a_v}
        y = {arc: 0 for arc in self.context.a_v}
        z = {arc: 0 for arc in self.context.a_r}
        xi = {node: 0 for node in self.context.nodes}
        delta = {node: 0 for node in self.context.nodes}
        selected_truck_arcs: List[Arc] = []

        if route:
            previous = self.context.depot
            for node in route:
                arc = (previous, node)
                x[arc] = 1
                selected_truck_arcs.append(arc)
                previous = node
            arc = (previous, self.context.depot)
            x[arc] = 1
            selected_truck_arcs.append(arc)

        truck_in_count = {node: 0 for node in self.context.nodes}
        for _, node in selected_truck_arcs:
            truck_in_count[node] += 1

        return TruckMasterSolution(
            truck=truck,
            active=bool(route),
            x=x,
            y=y,
            z=z,
            xi=xi,
            delta=delta,
            selected_truck_arcs=selected_truck_arcs,
            selected_robot_arcs=[],
            truck_in_count=truck_in_count,
            robot_in_count={node: 0 for node in self.context.cp_nodes},
            signature=tuple(),
        )

    def _try_customer_order_warm_start(
        self,
        ordering: Sequence[int],
    ) -> Optional[Tuple[Dict[int, List[int]], float]]:
        routes = {truck: [] for truck in self.context.trucks}
        route_costs = {truck: 0.0 for truck in self.context.trucks}

        for customer in ordering:
            best_choice: Optional[Tuple[Tuple[float, int, int], int, List[int], float]] = None
            for truck in self.context.trucks:
                base_route = routes[truck]
                base_cost = route_costs[truck]
                for pos in range(len(base_route) + 1):
                    candidate_route = base_route[:pos] + [customer] + base_route[pos:]
                    metrics = self._route_is_feasible(candidate_route)
                    if metrics is None:
                        continue
                    _, _, _, candidate_cost = metrics
                    delta_cost = candidate_cost - base_cost
                    score = (delta_cost, len(candidate_route), truck)
                    if best_choice is None or score < best_choice[0]:
                        best_choice = (score, truck, candidate_route, candidate_cost)

            if best_choice is None:
                return None

            _, truck, candidate_route, candidate_cost = best_choice
            routes[truck] = candidate_route
            route_costs[truck] = candidate_cost

        for truck, route in routes.items():
            if not route:
                continue
            solution = self._build_truck_only_solution(truck, route)
            if not self.subproblems[truck].check_feasibility(solution).feasible:
                return None

        return routes, sum(route_costs.values())

    def _apply_truck_only_warm_start(self) -> None:
        customers = list(self.context.customers)
        if not customers:
            self.warm_start_used = True
            self.warm_start_objective = 0.0
            return

        round_trip_cost = {
            customer: self.context.veh_cost_matrix[self.context.depot][customer]
            + self.context.veh_cost_matrix[customer][self.context.depot]
            for customer in customers
        }
        candidate_orders = [
            tuple(sorted(customers, key=lambda node: (round_trip_cost[node], self.context.q[node]), reverse=True)),
            tuple(sorted(customers, key=lambda node: (self.context.q[node], round_trip_cost[node]), reverse=True)),
            tuple(sorted(customers, key=lambda node: (round_trip_cost[node], self.context.q[node]))),
        ]

        best_routes: Optional[Dict[int, List[int]]] = None
        best_objective: Optional[float] = None
        tried_orders = set()
        for ordering in candidate_orders:
            if ordering in tried_orders:
                continue
            tried_orders.add(ordering)
            candidate = self._try_customer_order_warm_start(ordering)
            if candidate is None:
                continue
            routes, objective = candidate
            if best_objective is None or objective < best_objective:
                best_routes = routes
                best_objective = objective

        if best_routes is None:
            return

        for var in self.X.values():
            var.Start = 0.0
        for var in self.Y.values():
            var.Start = 0.0
        for var in self.Z.values():
            var.Start = 0.0
        for var in self.Xi.values():
            var.Start = 0.0
        for var in self.Delta.values():
            var.Start = 0.0
        for node in self.context.parking:
            self.Phi[node].Start = 1.0
            self.Psi[node].Start = 0.0

        for truck, route in best_routes.items():
            if not route:
                continue
            previous = self.context.depot
            for node in route:
                self.X[previous, node, truck].Start = 1.0
                previous = node
            self.X[previous, self.context.depot, truck].Start = 1.0

        self.warm_start_used = True
        self.warm_start_objective = best_objective

    def apply_external_start(self, depot_result: DepotSolveResult) -> None:
        for var in self.X.values():
            var.Start = 0.0
            var.VarHintVal = 0.0
        for var in self.Y.values():
            var.Start = 0.0
            var.VarHintVal = 0.0
        for var in self.Z.values():
            var.Start = 0.0
            var.VarHintVal = 0.0
        for var in self.Xi.values():
            var.Start = 0.0
            var.VarHintVal = 0.0
        for var in self.Delta.values():
            var.Start = 0.0
            var.VarHintVal = 0.0
        for node in self.context.parking:
            self.Phi[node].Start = 1.0
            self.Phi[node].VarHintVal = 1.0
            self.Psi[node].Start = 1.0
            self.Psi[node].VarHintVal = 1.0

        for key, value in depot_result.x.items():
            if value > INT_TOL and key in self.X:
                self.X[key].Start = 1.0
                self.X[key].VarHintVal = 1.0
        for key, value in depot_result.y.items():
            if value > INT_TOL and key in self.Y:
                self.Y[key].Start = 1.0
                self.Y[key].VarHintVal = 1.0
        for key, value in depot_result.z.items():
            if value > INT_TOL and key in self.Z:
                self.Z[key].Start = 1.0
                self.Z[key].VarHintVal = 1.0
        for key, value in depot_result.xi.items():
            if value > INT_TOL and key in self.Xi:
                self.Xi[key].Start = 1.0
                self.Xi[key].VarHintVal = 1.0
        for key, value in depot_result.delta.items():
            if value > INT_TOL and key in self.Delta:
                self.Delta[key].Start = 1.0
                self.Delta[key].VarHintVal = 1.0

        self.warm_start_used = depot_result.objective is not None
        self.warm_start_objective = depot_result.objective

    def _run_warm_optimize(self, warm_time_limit: float, heuristics: float, no_rel_heur_time: float) -> None:
        if warm_time_limit <= 0.0:
            return

        original_params = {
            'TimeLimit': self.master.Params.TimeLimit,
            'SolutionLimit': self.master.Params.SolutionLimit,
            'MIPFocus': self.master.Params.MIPFocus,
            'Heuristics': self.master.Params.Heuristics,
            'NoRelHeurTime': self.master.Params.NoRelHeurTime,
        }

        self.master.setParam('TimeLimit', warm_time_limit)
        self.master.setParam('SolutionLimit', 1)
        self.master.setParam('MIPFocus', 1)
        self.master.setParam('Heuristics', heuristics)
        self.master.setParam('NoRelHeurTime', min(no_rel_heur_time, warm_time_limit))
        self.master.optimize(self._callback)

        if self.master.SolCount > 0:
            for var in self.master.getVars():
                var.Start = var.X
            self.warm_start_used = True
            self.warm_start_objective = self.master.ObjVal

        self.master.setParam('TimeLimit', original_params['TimeLimit'])
        self.master.setParam('SolutionLimit', original_params['SolutionLimit'])
        self.master.setParam('MIPFocus', original_params['MIPFocus'])
        self.master.setParam('Heuristics', original_params['Heuristics'])
        self.master.setParam('NoRelHeurTime', original_params['NoRelHeurTime'])

    def _run_kernel_stage(
        self,
        allowed_truck: Set[Arc],
        allowed_robot: Set[Arc],
        *,
        allow_y: bool,
        allow_robot: bool,
        allow_anchor: bool,
        time_limit: float,
        heuristics: float,
        no_rel_heur_time: float,
    ) -> None:
        if self.warm_start_used or not allowed_truck:
            return

        bound_changes: List[Tuple[gp.Var, float]] = []
        for truck in self.context.trucks:
            for i, j in self.context.a_v:
                if (i, j) not in allowed_truck:
                    for var in (self.X[i, j, truck], self.Y[i, j, truck]):
                        if var.UB > 0.5:
                            bound_changes.append((var, var.UB))
                            var.UB = 0.0
                elif not allow_y and self.Y[i, j, truck].UB > 0.5:
                    bound_changes.append((self.Y[i, j, truck], self.Y[i, j, truck].UB))
                    self.Y[i, j, truck].UB = 0.0

            for i, j in self.context.a_r:
                var = self.Z[i, j, truck]
                if (not allow_robot or (i, j) not in allowed_robot) and var.UB > 0.5:
                    bound_changes.append((var, var.UB))
                    var.UB = 0.0

            if not allow_anchor:
                for node in self.context.nodes:
                    for var in (self.Xi[node, truck], self.Delta[node, truck]):
                        if var.UB > 0.5:
                            bound_changes.append((var, var.UB))
                            var.UB = 0.0

        if not bound_changes:
            return

        self.master.update()
        try:
            self._run_warm_optimize(time_limit, heuristics=heuristics, no_rel_heur_time=no_rel_heur_time)
        finally:
            for var, ub in bound_changes:
                var.UB = ub
            self.master.update()

    def _run_truck_only_reduced_warm_phase(self) -> None:
        if self.warm_start_used:
            return
        if len(self.context.customers) < 12 and len(self.context.trucks) < 2:
            return

        stages = [
            self._build_reduced_neighborhood(
                truck_neighbor_cap=max(3, len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 1, 4),
                robot_neighbor_cap=1,
                anchor_robot_cap=1,
            ),
            self._build_reduced_neighborhood(
                truck_neighbor_cap=max(4, len(self.context.customers) // 8 + len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 2, 5),
                robot_neighbor_cap=1,
                anchor_robot_cap=1,
            ),
            self._build_reduced_neighborhood(
                truck_neighbor_cap=max(5, len(self.context.customers) // 6 + len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 3, 6),
                robot_neighbor_cap=1,
                anchor_robot_cap=1,
            ),
        ]

        for idx, (allowed_truck, _) in enumerate(stages):
            self._run_kernel_stage(
                allowed_truck,
                set(),
                allow_y=False,
                allow_robot=False,
                allow_anchor=False,
                time_limit=min(4.5, 2.5 + idx),
                heuristics=0.99,
                no_rel_heur_time=1.5,
            )
            if self.warm_start_used:
                return

    def _run_reduced_neighborhood_warm_phase(self) -> None:
        if self.warm_start_used:
            return
        if len(self.context.customers) < 12 and len(self.context.trucks) < 2:
            return

        stages = [
            self._build_reduced_neighborhood(
                truck_neighbor_cap=max(6, len(self.context.customers) // 5 + len(self.context.trucks)),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 4, 7),
                robot_neighbor_cap=max(2, len(self.context.customers) // 12 + 1),
                anchor_robot_cap=2,
            ),
            self._build_reduced_neighborhood(
                truck_neighbor_cap=max(7, len(self.context.customers) // 4 + len(self.context.trucks)),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 5, 8),
                robot_neighbor_cap=max(3, len(self.context.customers) // 10 + 1),
                anchor_robot_cap=3,
            ),
        ]

        for idx, (allowed_truck, allowed_robot) in enumerate(stages):
            self._run_kernel_stage(
                allowed_truck,
                allowed_robot,
                allow_y=True,
                allow_robot=True,
                allow_anchor=True,
                time_limit=min(7.0, 4.0 + 2.0 * idx),
                heuristics=0.96,
                no_rel_heur_time=2.0,
            )
            if self.warm_start_used:
                return

    def _run_feasibility_warm_phase(self) -> None:
        if self.warm_start_used:
            return
        if len(self.context.customers) < 12 and len(self.context.trucks) < 2:
            return

        warm_time_limit = min(2.0, float(self.master.Params.TimeLimit))
        self._run_warm_optimize(warm_time_limit, heuristics=0.8, no_rel_heur_time=1.0)

    def _run_large_instance_heuristic_phase(self) -> None:
        if self.warm_start_used:
            return
        if len(self.context.customers) < 12 and len(self.context.trucks) < 2:
            return

        stage_specs = [
            dict(
                truck_neighbor_cap=max(4, len(self.context.trucks) + 2),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 2, 6),
                robot_neighbor_cap=1,
                anchor_robot_cap=1,
                allow_y=True,
                allow_robot=False,
                allow_anchor=False,
                time_limit=8.0,
                heuristics=0.99,
                no_rel_heur_time=2.0,
            ),
            dict(
                truck_neighbor_cap=max(5, len(self.context.customers) // 7 + len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 3, 7),
                robot_neighbor_cap=1,
                anchor_robot_cap=1,
                allow_y=True,
                allow_robot=False,
                allow_anchor=True,
                time_limit=10.0,
                heuristics=0.99,
                no_rel_heur_time=2.5,
            ),
            dict(
                truck_neighbor_cap=max(6, len(self.context.customers) // 6 + len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 4, 8),
                robot_neighbor_cap=max(2, len(self.context.customers) // 12 + 1),
                anchor_robot_cap=2,
                allow_y=True,
                allow_robot=True,
                allow_anchor=True,
                time_limit=12.0,
                heuristics=0.98,
                no_rel_heur_time=3.0,
            ),
            dict(
                truck_neighbor_cap=max(7, len(self.context.customers) // 5 + len(self.context.trucks) + 1),
                depot_neighbor_cap=max(2 * len(self.context.trucks) + 5, 9),
                robot_neighbor_cap=max(3, len(self.context.customers) // 10 + 1),
                anchor_robot_cap=3,
                allow_y=True,
                allow_robot=True,
                allow_anchor=True,
                time_limit=15.0,
                heuristics=0.97,
                no_rel_heur_time=4.0,
            ),
        ]

        for spec in stage_specs:
            allowed_truck, allowed_robot = self._build_reduced_neighborhood(
                truck_neighbor_cap=spec['truck_neighbor_cap'],
                depot_neighbor_cap=spec['depot_neighbor_cap'],
                robot_neighbor_cap=spec['robot_neighbor_cap'],
                anchor_robot_cap=spec['anchor_robot_cap'],
            )
            self._run_kernel_stage(
                allowed_truck,
                allowed_robot,
                allow_y=spec['allow_y'],
                allow_robot=spec['allow_robot'],
                allow_anchor=spec['allow_anchor'],
                time_limit=spec['time_limit'],
                heuristics=spec['heuristics'],
                no_rel_heur_time=spec['no_rel_heur_time'],
            )
            if self.warm_start_used:
                return

    def _run_first_incumbent_phase(self) -> None:
        original_params = {
            'TimeLimit': self.master.Params.TimeLimit,
            'SolutionLimit': self.master.Params.SolutionLimit,
            'MIPFocus': self.master.Params.MIPFocus,
            'Heuristics': self.master.Params.Heuristics,
            'NoRelHeurTime': self.master.Params.NoRelHeurTime,
            'RINS': self.master.Params.RINS,
            'PumpPasses': self.master.Params.PumpPasses,
        }

        search_limit = original_params['TimeLimit']
        if self.heuristic_time_limit is not None:
            search_limit = min(search_limit, float(self.heuristic_time_limit))
        else:
            search_limit = min(search_limit, 600.0)

        self.master.setParam('TimeLimit', search_limit)
        self.master.setParam('SolutionLimit', 1)
        self.master.setParam('MIPFocus', 1)
        self.master.setParam('Heuristics', max(0.95, original_params['Heuristics']))
        self.master.setParam('NoRelHeurTime', min(30.0, search_limit))
        self.master.setParam('RINS', 20)
        self.master.setParam('PumpPasses', 10)
        self.master.optimize(self._callback)

        if self.master.SolCount > 0:
            for var in self.master.getVars():
                var.Start = var.X
            self.warm_start_used = True
            self.warm_start_objective = self.master.ObjVal

        self.master.setParam('TimeLimit', original_params['TimeLimit'])
        self.master.setParam('SolutionLimit', original_params['SolutionLimit'])
        self.master.setParam('MIPFocus', original_params['MIPFocus'])
        self.master.setParam('Heuristics', original_params['Heuristics'])
        self.master.setParam('NoRelHeurTime', original_params['NoRelHeurTime'])
        self.master.setParam('RINS', original_params['RINS'])
        self.master.setParam('PumpPasses', original_params['PumpPasses'])

    def _build_master(self) -> None:
        context = self.context
        robot_arc_set = set(context.a_r)
        ordered_trucks = sorted(context.trucks)
        departure_targets = sorted(
            j for i, j in context.truck_out_arcs[context.depot]
            if i == context.depot
        )
        departure_rank = {node: rank for rank, node in enumerate(departure_targets, start=1)}

        truck_keys = [(i, j, truck) for truck in context.trucks for i, j in context.a_v]
        robot_keys = [(i, j, truck) for truck in context.trucks for i, j in context.a_r]
        local_node_keys = [(node, truck) for truck in context.trucks for node in context.nodes]

        self.X = self.master.addVars(truck_keys, vtype=GRB.BINARY, name="x")
        self.Y = self.master.addVars(truck_keys, vtype=GRB.BINARY, name="y")
        self.Z = self.master.addVars(robot_keys, vtype=GRB.BINARY, name="z")
        self.Xi = self.master.addVars(local_node_keys, vtype=GRB.BINARY, name="xi")
        self.Delta = self.master.addVars(local_node_keys, vtype=GRB.BINARY, name="delta")
        self.Phi = self.master.addVars(context.parking, vtype=GRB.BINARY, name="phi")
        self.Psi = self.master.addVars(context.parking, vtype=GRB.BINARY, name="psi")
        self._apply_branch_priorities()

        self.x_vars_by_truck: Dict[int, List[gp.Var]] = {}
        self.y_vars_by_truck: Dict[int, List[gp.Var]] = {}
        self.z_vars_by_truck: Dict[int, List[gp.Var]] = {}
        self.xi_vars_by_truck: Dict[int, List[gp.Var]] = {}
        self.delta_vars_by_truck: Dict[int, List[gp.Var]] = {}

        for truck in context.trucks:
            self.x_vars_by_truck[truck] = [self.X[i, j, truck] for i, j in context.a_v]
            self.y_vars_by_truck[truck] = [self.Y[i, j, truck] for i, j in context.a_v]
            self.z_vars_by_truck[truck] = [self.Z[i, j, truck] for i, j in context.a_r]
            self.xi_vars_by_truck[truck] = [self.Xi[node, truck] for node in context.nodes]
            self.delta_vars_by_truck[truck] = [self.Delta[node, truck] for node in context.nodes]

        obj1 = gp.quicksum(
            context.veh_cost_matrix[i][j] * (self.X[i, j, truck] + self.Y[i, j, truck])
            for truck in context.trucks
            for i, j in context.a_v
        )
        obj2 = gp.quicksum(
            context.rob_cost_matrix[i][j] * self.Z[i, j, truck]
            for truck in context.trucks
            for i, j in context.a_r
        )
        obj3 = gp.quicksum(
            context.fixed_cost * self.X[context.depot, j, truck]
            for truck in context.trucks
            for i, j in context.truck_out_arcs[context.depot]
            if i == context.depot
        )
        self.master.setObjective(obj1 + obj2 + obj3, GRB.MINIMIZE)

        self.master.addConstrs(
            (self._departure_expr(truck) <= 1 for truck in context.trucks),
            name="eq:c2",
        )
        if len(ordered_trucks) >= 2:
            self.master.addConstrs(
                (
                    self._departure_expr(truck_1) >= self._departure_expr(truck_2)
                    for truck_1, truck_2 in zip(ordered_trucks, ordered_trucks[1:])
                ),
                name="sym:use_order",
            )
            if departure_targets:
                rank_ub = max(departure_rank.values())
                self.master.addConstrs(
                    (
                        gp.quicksum(
                            departure_rank[node] * self.X[context.depot, node, truck_1]
                            for node in departure_targets
                        )
                        <= gp.quicksum(
                            departure_rank[node] * self.X[context.depot, node, truck_2]
                            for node in departure_targets
                        ) + rank_ub * (1 - self._departure_expr(truck_2))
                        for truck_1, truck_2 in zip(ordered_trucks, ordered_trucks[1:])
                    ),
                    name="sym:first_departure_order",
                )
        self.master.addConstrs(
            (
                self._truck_in_expr(node, truck, self.X) + self._truck_in_expr(node, truck, self.Y) <= 1
                for truck in context.trucks
                for node in context.nodes
            ),
            name="eq:c3",
        )
        self.master.addConstrs(
            (
                self.X[i, j, truck] + self.Y[i, j, truck] <= self._departure_expr(truck)
                for truck in context.trucks
                for i, j in context.a_v
            ),
            name="eq:c4a",
        )
        self.master.addConstrs(
            (
                self.Z[i, j, truck] <= self._departure_expr(truck)
                for truck in context.trucks
                for i, j in context.a_r
            ),
            name="eq:c4b",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    self._truck_in_expr(node, truck, self.X)
                    + self._truck_in_expr(node, truck, self.Y)
                    + self._robot_in_expr(node, truck)
                    for truck in context.trucks
                )
                == 1 + gp.quicksum(self.Delta[node, truck] for truck in context.trucks)
                for node in context.customers
            ),
            name="eq:c5",
        )
        self.master.addConstrs(
            (
                2 * gp.quicksum(self.Delta[node, truck] for truck in context.trucks)
                - gp.quicksum(
                    self._truck_in_expr(node, truck, self.X)
                    + self._truck_in_expr(node, truck, self.Y)
                    + self._robot_in_expr(node, truck)
                    for truck in context.trucks
                )
                <= 0
                for node in context.parking
            ),
            name="eq:c50",
        )
        self.master.addConstrs(
            (
                gp.quicksum(self.Xi[node, truck] for truck in context.trucks)
                - gp.quicksum(
                    self._truck_in_expr(node, truck, self.X)
                    + self._truck_in_expr(node, truck, self.Y)
                    + self._robot_in_expr(node, truck)
                    for truck in context.trucks
                )
                <= 0
                for node in context.parking
            ),
            name="eq:c51",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    self._truck_in_expr(node, truck, self.X)
                    + self._truck_in_expr(node, truck, self.Y)
                    + self._robot_in_expr(node, truck)
                    for truck in context.trucks
                )
                - 2 * gp.quicksum(self.Delta[node, truck] for truck in context.trucks)
                <= 2 * (1 - self.Phi[node])
                for node in context.parking
            ),
            name="eq:c52",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    self._truck_in_expr(node, truck, self.X)
                    + self._truck_in_expr(node, truck, self.Y)
                    + self._robot_in_expr(node, truck)
                    for truck in context.trucks
                )
                - gp.quicksum(self.Xi[node, truck] for truck in context.trucks)
                <= 2 * (1 - self.Psi[node])
                for node in context.parking
            ),
            name="eq:c53",
        )
        self.master.addConstrs((self.Phi[node] + self.Psi[node] >= 1 for node in context.parking), name="eq:c54")

        self.master.addConstrs(
            (
                self._truck_out_expr(node, truck, self.X)
                == self._truck_in_expr(node, truck, self.X) - self.Delta[node, truck] + self.Xi[node, truck]
                for truck in context.trucks
                for node in context.nodes
            ),
            name="eq:c7",
        )
        self.master.addConstrs(
            (
                self._truck_out_expr(node, truck, self.Y)
                == self._truck_in_expr(node, truck, self.Y) + self.Delta[node, truck] - self.Xi[node, truck]
                for truck in context.trucks
                for node in context.nodes
            ),
            name="eq:c8",
        )
        self.master.addConstrs(
            (
                self._robot_out_expr(node, truck)
                == self._robot_in_expr(node, truck) + self.Delta[node, truck] - self.Xi[node, truck]
                for truck in context.trucks
                for node in context.cp_nodes
            ),
            name="eq:c9a",
        )
        self.master.addConstrs(
            (
                self._robot_out_expr(node, truck) <= 1
                for truck in context.trucks
                for node in context.cp_nodes
            ),
            name="eq:c9b",
        )
        self.master.addConstrs(
            (
                2 * self.Delta[node, truck]
                <= self._truck_in_expr(node, truck, self.X)
                + self._truck_in_expr(node, truck, self.Y)
                + self._robot_in_expr(node, truck)
                for truck in context.trucks
                for node in context.cp_nodes
            ),
            name="eq:c13",
        )
        self.master.addConstrs(
            (
                self.Xi[node, truck] <= self._truck_in_expr(node, truck, self.X)
                for truck in context.trucks
                for node in context.parking
            ),
            name="eq:c14",
        )
        self.master.addConstrs(
            (
                gp.quicksum(self.Delta[node, truck] for truck in context.trucks) <= 1
                for node in context.customers
            ),
            name="eq:c15",
        )
        self.master.addConstrs(
            (
                gp.quicksum(self.Xi[node, truck] + self.Delta[node, truck] for truck in context.trucks) <= 2
                for node in context.parking
            ),
            name="eq:c16",
        )
        self.master.addConstrs(
            (
                gp.quicksum(self.Xi[node, truck] for truck in context.trucks) <= 1
                for node in context.cp_nodes
            ),
            name="eq:c17a",
        )
        self.master.addConstrs(
            (
                gp.quicksum(self.Delta[node, truck] for truck in context.trucks) <= 1
                for node in context.cp_nodes
            ),
            name="eq:c17b",
        )
        self.master.addConstrs(
            (
                self.Xi[node, truck_1] + self.Delta[node, truck_2] <= 1
                for node in context.parking
                for truck_1 in context.trucks
                for truck_2 in context.trucks
                if truck_1 != truck_2
            ),
            name="eq:c18",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    self.Z[i, j, truck]
                    for i in context.parking
                    for j in context.parking
                    if i != j and (i, j) in robot_arc_set
                )
                == 0
                for truck in context.trucks
            ),
            name="eq:c21",
        )
        self.master.addConstrs(
            (
                self.Xi[node, truck] == 0
                for truck in context.trucks
                for node in [context.depot] + context.customers
            ),
            name="eq:c25",
        )
        self.master.addConstrs(
            (
                self.Delta[node, truck] == 0
                for truck in context.trucks
                for node in [context.depot] + context.robot_only_customers
            ),
            name="eq:c26",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    context.veh_dist_matrix[i][j] * (self.X[i, j, truck] + self.Y[i, j, truck])
                    for i, j in context.a_v
                )
                <= context.veh_dist_ub
                for truck in context.trucks
            ),
            name="eq:c43",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    context.veh_time_matrix[i][j] * (self.X[i, j, truck] + self.Y[i, j, truck])
                    for i, j in context.a_v
                    if j in context.cp_nodes
                )
                <= context.max_travel_time * self._departure_expr(truck)
                for truck in context.trucks
            ),
            name="proxy:truck_prefix_time",
        )
        self.master.addConstrs(
            (
                gp.quicksum(
                    context.rob_power_matrix[i][j] * self.Z[i, j, truck]
                    for i, j in context.a_r
                )
                <= context.robot_ub * gp.quicksum(self.Delta[node, truck] for node in context.cp_nodes)
                for truck in context.trucks
            ),
            name="proxy:robot_energy_budget",
        )

        self.master.update()

    def _extract_truck_solution(self, model: gp.Model, truck: int) -> TruckMasterSolution:
        context = self.context
        x_vals = model.cbGetSolution(self.x_vars_by_truck[truck])
        y_vals = model.cbGetSolution(self.y_vars_by_truck[truck])
        z_vals = model.cbGetSolution(self.z_vars_by_truck[truck])
        xi_vals = model.cbGetSolution(self.xi_vars_by_truck[truck])
        delta_vals = model.cbGetSolution(self.delta_vars_by_truck[truck])

        x = {arc: int(value > INT_TOL) for arc, value in zip(context.a_v, x_vals)}
        y = {arc: int(value > INT_TOL) for arc, value in zip(context.a_v, y_vals)}
        z = {arc: int(value > INT_TOL) for arc, value in zip(context.a_r, z_vals)}
        xi = {node: int(value > INT_TOL) for node, value in zip(context.nodes, xi_vals)}
        delta = {node: int(value > INT_TOL) for node, value in zip(context.nodes, delta_vals)}

        selected_truck_arcs = [arc for arc in context.a_v if x[arc] or y[arc]]
        selected_robot_arcs = [arc for arc in context.a_r if z[arc]]

        truck_in_count = {node: 0 for node in context.nodes}
        robot_in_count = {node: 0 for node in context.cp_nodes}
        for _, j in selected_truck_arcs:
            truck_in_count[j] += 1
        for _, j in selected_robot_arcs:
            if j in robot_in_count:
                robot_in_count[j] += 1

        active = any(
            x.get((context.depot, j), 0) == 1
            for i, j in context.truck_out_arcs[context.depot]
            if i == context.depot
        )

        signature_bits: List[int] = []
        signature_bits.extend(int(value > INT_TOL) for value in x_vals)
        signature_bits.extend(int(value > INT_TOL) for value in y_vals)
        signature_bits.extend(int(value > INT_TOL) for value in z_vals)
        signature_bits.extend(xi[node] for node in self.free_xi_nodes)
        signature_bits.extend(delta[node] for node in self.free_delta_nodes)

        return TruckMasterSolution(
            truck=truck,
            active=active,
            x=x,
            y=y,
            z=z,
            xi=xi,
            delta=delta,
            selected_truck_arcs=selected_truck_arcs,
            selected_robot_arcs=selected_robot_arcs,
            truck_in_count=truck_in_count,
            robot_in_count=robot_in_count,
            signature=tuple(signature_bits),
        )

    def _add_fractional_root_truck_cuts(self, model: gp.Model, truck: int) -> bool:
        x_vals = model.cbGetNodeRel(self.x_vars_by_truck[truck])
        y_vals = model.cbGetNodeRel(self.y_vars_by_truck[truck])
        arc_weights = {
            arc: x_value + y_value
            for arc, x_value, y_value in zip(self.context.a_v, x_vals, y_vals)
        }
        support_arcs = [arc for arc, weight in arc_weights.items() if weight > FRAC_SUPPORT_TOL]
        if not support_arcs:
            return False

        cuts_added = False
        for component in _strongly_connected_components(self.context.nodes, support_arcs):
            if self.context.depot in component or len(component) <= 1:
                continue

            component_key = tuple(sorted(component))
            if component_key in self.root_truck_cut_cache[truck]:
                continue

            component_set = set(component)
            internal_value = sum(
                arc_weights[arc]
                for arc in self.context.a_v
                if arc[0] in component_set and arc[1] in component_set
            )
            if internal_value <= len(component) - 1 + FRAC_CUT_TOL:
                continue

            expr = gp.quicksum(
                self.X[i, j, truck] + self.Y[i, j, truck]
                for i, j in self.context.a_v
                if i in component_set and j in component_set
            )
            model.cbCut(expr <= len(component) - 1)
            self.root_truck_cut_cache[truck].add(component_key)
            self.root_truck_cuts += 1
            cuts_added = True

        return cuts_added

    def _find_truck_time_conflict_path(self, solution: TruckMasterSolution) -> Optional[List[Arc]]:
        active_arcs = [(i, j) for i, j in solution.selected_truck_arcs if j in self.context.cp_nodes]
        if not active_arcs:
            return None

        active_nodes = set()
        indegree: Dict[int, int] = {}
        adjacency: Dict[int, List[Tuple[int, float]]] = {}
        for i, j in active_arcs:
            active_nodes.add(i)
            active_nodes.add(j)
            indegree.setdefault(i, 0)
            indegree[j] = indegree.get(j, 0) + 1
            adjacency.setdefault(i, []).append((j, self.context.veh_time_matrix[i][j]))

        queue = deque(node for node in active_nodes if indegree.get(node, 0) == 0)
        if not queue:
            return active_arcs

        longest_path = {node: 0.0 for node in active_nodes}
        predecessor: Dict[int, int] = {}
        processed = 0
        while queue:
            node = queue.popleft()
            processed += 1
            for nxt, travel_time in adjacency.get(node, []):
                candidate = longest_path[node] + travel_time
                if candidate > longest_path.get(nxt, float('-inf')) + 1e-9:
                    longest_path[nxt] = candidate
                    predecessor[nxt] = node
                    if candidate > self.context.max_travel_time + 1e-9:
                        path_nodes = [nxt]
                        current = nxt
                        while current in predecessor:
                            current = predecessor[current]
                            path_nodes.append(current)
                        path_nodes.reverse()
                        return list(zip(path_nodes, path_nodes[1:]))
                indegree[nxt] -= 1
                if indegree[nxt] == 0:
                    queue.append(nxt)

        if processed != len(active_nodes):
            return active_arcs
        return None

    def _add_truck_time_conflict_cut(self, model: gp.Model, truck: int, path: Sequence[Arc]) -> None:
        expr = gp.quicksum(self.X[i, j, truck] + self.Y[i, j, truck] for i, j in path)
        model.cbLazy(expr <= len(path) - 1)

    def _robot_load_reset_vars(self, truck: int, node: int) -> List[gp.Var]:
        if self.feasibility_cut_mode != "exact":
            return []

        reset_vars: List[gp.Var] = []
        if node in self.context.parking:
            reset_vars.append(self.Xi[node, truck])
        if node in self.free_delta_nodes:
            reset_vars.append(self.Delta[node, truck])
        return reset_vars

    def _robot_energy_reset_vars(self, truck: int, nodes: Sequence[int]) -> List[gp.Var]:
        if self.feasibility_cut_mode != "exact":
            return []
        return [self.Delta[node, truck] for node in nodes if node in self.free_delta_nodes]

    def _find_robot_resource_conflict(
        self,
        solution: TruckMasterSolution,
    ) -> Optional[Tuple[List[Arc], List[gp.Var]]]:
        if not solution.selected_robot_arcs:
            return None

        successor: Dict[int, int] = {}
        indegree: Dict[int, int] = {}
        for i, j in solution.selected_robot_arcs:
            if i in successor and successor[i] != j:
                return ([ (i, j) ], [])
            successor[i] = j
            indegree[j] = indegree.get(j, 0) + 1
            if indegree[j] > 1:
                return ([ (i, j) ], [])
            indegree.setdefault(i, 0)

        remaining_arcs = set(solution.selected_robot_arcs)
        customer_set = set(self.context.customers)

        while remaining_arcs:
            start = next((node for node, _ in remaining_arcs if solution.delta.get(node, 0)), None)
            if start is None:
                start = next(iter(remaining_arcs))[0]

            current = start
            energy_since_delta = 0.0
            load_since_reset = 0.0
            energy_segment_arcs: List[Arc] = []
            load_segment_arcs: List[Arc] = []
            energy_reset_nodes: List[int] = []
            load_reset_vars: List[gp.Var] = []
            load_reset_var_names = set()

            while current in successor:
                nxt = successor[current]
                arc = (current, nxt)
                if arc not in remaining_arcs:
                    break

                remaining_arcs.remove(arc)

                energy_segment_arcs.append(arc)
                energy_since_delta += self.context.rob_power_matrix[current][nxt]
                if energy_since_delta > self.context.robot_ub + 1e-9:
                    return (energy_segment_arcs.copy(), self._robot_energy_reset_vars(solution.truck, energy_reset_nodes))

                load_segment_arcs.append(arc)
                current_load_reset_vars = self._robot_load_reset_vars(solution.truck, nxt)
                if nxt in customer_set and not solution.xi.get(nxt, 0) and not solution.delta.get(nxt, 0):
                    load_since_reset += self.context.q[nxt]
                    if load_since_reset > self.context.ur + 1e-9:
                        reset_vars = load_reset_vars.copy()
                        for var in current_load_reset_vars:
                            if var.VarName not in load_reset_var_names:
                                reset_vars.append(var)
                        return (load_segment_arcs.copy(), reset_vars)

                if solution.delta.get(nxt, 0):
                    energy_since_delta = 0.0
                    load_since_reset = 0.0
                    energy_segment_arcs = []
                    load_segment_arcs = []
                    energy_reset_nodes = []
                    load_reset_vars = []
                    load_reset_var_names = set()
                elif solution.xi.get(nxt, 0):
                    load_since_reset = 0.0
                    load_segment_arcs = []
                    load_reset_vars = []
                    load_reset_var_names = set()
                    if nxt in self.free_delta_nodes:
                        energy_reset_nodes.append(nxt)
                else:
                    if nxt in self.free_delta_nodes:
                        energy_reset_nodes.append(nxt)
                    for var in current_load_reset_vars:
                        if var.VarName not in load_reset_var_names:
                            load_reset_vars.append(var)
                            load_reset_var_names.add(var.VarName)

                current = nxt

        return None

    def _add_robot_resource_conflict_cut(
        self,
        model: gp.Model,
        truck: int,
        arcs: Sequence[Arc],
        reset_vars: Sequence[gp.Var],
    ) -> None:
        expr = gp.quicksum(self.Z[i, j, truck] for i, j in arcs) - gp.quicksum(reset_vars)
        model.cbLazy(expr <= len(arcs) - 1)


    def _add_truck_subtour_cuts(self, model: gp.Model, solution: TruckMasterSolution) -> bool:
        cuts_added = False
        for component in _cycle_components(self.context.nodes, solution.selected_truck_arcs):
            if self.context.depot in component:
                continue
            component_set = set(component)
            expr = gp.quicksum(
                self.X[i, j, solution.truck] + self.Y[i, j, solution.truck]
                for i, j in self.context.a_v
                if i in component_set and j in component_set
            )
            model.cbLazy(expr <= len(component) - 1)
            self.truck_subtour_cuts += 1
            cuts_added = True
        return cuts_added

    def _add_robot_subtour_cuts(self, model: gp.Model, solution: TruckMasterSolution) -> bool:
        cuts_added = False
        for component in _cycle_components(self.context.cp_nodes, solution.selected_robot_arcs):
            if any(solution.xi.get(node, 0) or solution.delta.get(node, 0) for node in component):
                continue
            component_set = set(component)
            expr = gp.quicksum(
                self.Z[i, j, solution.truck]
                for i, j in self.context.a_r
                if i in component_set and j in component_set
            )
            model.cbLazy(expr <= len(component) - 1)
            self.robot_subtour_cuts += 1
            cuts_added = True
        return cuts_added

    def _add_farkas_feasibility_cut(
        self,
        model: gp.Model,
        truck: int,
        farkas_cut: FarkasFeasibilityCut,
    ) -> None:
        expr = gp.LinExpr(farkas_cut.constant_term)
        expr += gp.quicksum(
            coeff * self.X[i, j, truck]
            for (i, j), coeff in farkas_cut.x_coeffs.items()
        )
        expr += gp.quicksum(
            coeff * self.Y[i, j, truck]
            for (i, j), coeff in farkas_cut.y_coeffs.items()
        )
        expr += gp.quicksum(
            coeff * self.Z[i, j, truck]
            for (i, j), coeff in farkas_cut.z_coeffs.items()
        )
        expr += gp.quicksum(
            coeff * self.Xi[node, truck]
            for node, coeff in farkas_cut.xi_coeffs.items()
        )
        expr += gp.quicksum(
            coeff * self.Delta[node, truck]
            for node, coeff in farkas_cut.delta_coeffs.items()
        )
        model.cbLazy(expr >= farkas_cut.min_activity)
        self.farkas_cuts += 1

    def _add_no_good_cut(self, model: gp.Model, solution: TruckMasterSolution) -> None:
        active_vars: List[gp.Var] = []
        inactive_vars: List[gp.Var] = []

        for i, j in self.context.a_v:
            x_var = self.X[i, j, solution.truck]
            y_var = self.Y[i, j, solution.truck]
            if solution.x.get((i, j), 0):
                active_vars.append(x_var)
            else:
                inactive_vars.append(x_var)
            if solution.y.get((i, j), 0):
                active_vars.append(y_var)
            else:
                inactive_vars.append(y_var)

        for i, j in self.context.a_r:
            z_var = self.Z[i, j, solution.truck]
            if solution.z.get((i, j), 0):
                active_vars.append(z_var)
            else:
                inactive_vars.append(z_var)

        if self.feasibility_cut_mode == "exact":
            for node in self.context.parking:
                x_out = sum(solution.x.get((node, j), 0) for _, j in self.context.truck_out_arcs[node])
                x_in = sum(solution.x.get((i, node), 0) for i, _ in self.context.truck_in_arcs[node])
                if x_out != x_in:
                    continue
                xi_var = self.Xi[node, solution.truck]
                if solution.xi.get(node, 0):
                    active_vars.append(xi_var)
                else:
                    inactive_vars.append(xi_var)

        expr = gp.quicksum(active_vars) - gp.quicksum(inactive_vars)
        model.cbLazy(expr <= len(active_vars) - 1)
        self.no_good_cuts += 1

    def _callback(self, model: gp.Model, where: int) -> None:
        if where == GRB.Callback.MIPNODE:
            node_status = model.cbGet(GRB.Callback.MIPNODE_STATUS)
            node_count = model.cbGet(GRB.Callback.MIPNODE_NODCNT)
            if node_status == GRB.OPTIMAL and node_count < 0.5:
                for truck in self.context.trucks:
                    self._add_fractional_root_truck_cuts(model, truck)
            return

        if where != GRB.Callback.MIPSOL:
            return

        for truck in self.context.trucks:
            solution = self._extract_truck_solution(model, truck)
            cuts_added = False
            cuts_added = self._add_truck_subtour_cuts(model, solution) or cuts_added
            cuts_added = self._add_robot_subtour_cuts(model, solution) or cuts_added
            if cuts_added or not solution.active:
                continue

            cached = self.feasibility_cache[truck].get(solution.signature)
            if cached is None:
                time_conflict_path = self._find_truck_time_conflict_path(solution)
                if time_conflict_path is not None:
                    self.screening_rejections += 1
                    self._add_truck_time_conflict_cut(model, truck, time_conflict_path)
                    self.feasibility_cache[truck][solution.signature] = False
                    continue

                robot_conflict = self._find_robot_resource_conflict(solution)
                if robot_conflict is not None:
                    self.screening_rejections += 1
                    conflict_arcs, reset_vars = robot_conflict
                    self._add_robot_resource_conflict_cut(model, truck, conflict_arcs, reset_vars)
                    self.feasibility_cache[truck][solution.signature] = False
                    continue

                self.subproblem_calls += 1
                check_result = self.subproblems[truck].check_feasibility(solution)
                cached = check_result.feasible
                self.feasibility_cache[truck][solution.signature] = cached
                if (
                    not cached
                    and self.feasibility_cut_mode == "exact"
                    and check_result.farkas_cut is not None
                ):
                    self._add_farkas_feasibility_cut(model, truck, check_result.farkas_cut)
                    continue

            if not cached:
                self._add_no_good_cut(model, solution)

    def _extract_final_solution(self) -> Tuple[
        Dict[TruckArcKey, float],
        Dict[TruckArcKey, float],
        Dict[TruckArcKey, float],
        Dict[NodeKey, float],
        Dict[NodeKey, float],
    ]:
        x_sol = {key: var.X for key, var in self.X.items() if var.X > INT_TOL}
        y_sol = {key: var.X for key, var in self.Y.items() if var.X > INT_TOL}
        z_sol = {key: var.X for key, var in self.Z.items() if var.X > INT_TOL}
        xi_sol = {key: var.X for key, var in self.Xi.items() if var.X > INT_TOL}
        delta_sol = {key: var.X for key, var in self.Delta.items() if var.X > INT_TOL}
        return x_sol, y_sol, z_sol, xi_sol, delta_sol

    def solve(self) -> DepotSolveResult:
        try:
            self._run_truck_only_reduced_warm_phase()
            self._run_reduced_neighborhood_warm_phase()
            if self.solve_mode == "heuristic":
                self._run_large_instance_heuristic_phase()
            self._run_feasibility_warm_phase()
            if self.solve_mode == "heuristic":
                self._run_first_incumbent_phase()
            else:
                self.master.optimize(self._callback)

            if self.master.Status == GRB.INFEASIBLE:
                self.master.computeIIS()
                self.master.write(f"model_depot_{self.context.depot}_LBBD.ilp")

            objective = self.master.ObjVal if self.master.SolCount > 0 else None
            best_bound = self.master.ObjBound if self.master.SolCount > 0 else None
            mip_gap = self.master.MIPGap if self.master.SolCount > 0 else None

            if self.master.SolCount > 0:
                x_sol, y_sol, z_sol, xi_sol, delta_sol = self._extract_final_solution()
            else:
                x_sol, y_sol, z_sol, xi_sol, delta_sol = {}, {}, {}, {}, {}

            return DepotSolveResult(
                depot=self.context.depot,
                status=self.master.Status,
                status_name=_status_name(self.master.Status),
                objective=objective,
                best_bound=best_bound,
                mip_gap=mip_gap,
                runtime=self.master.Runtime,
                x=x_sol,
                y=y_sol,
                z=z_sol,
                xi=xi_sol,
                delta=delta_sol,
                subproblem_calls=self.subproblem_calls,
                warm_start_used=self.warm_start_used,
                warm_start_objective=self.warm_start_objective,
                root_truck_cuts=self.root_truck_cuts,
                truck_subtour_cuts=self.truck_subtour_cuts,
                robot_subtour_cuts=self.robot_subtour_cuts,
                no_good_cuts=self.no_good_cuts,
                farkas_cuts=self.farkas_cuts,
                screening_rejections=self.screening_rejections,
            )
        finally:
            self.dispose()

    def dispose(self) -> None:
        for subproblem in self.subproblems.values():
            subproblem.dispose()
        if hasattr(self.master, "dispose"):
            self.master.dispose()
        if hasattr(self.env, "dispose"):
            self.env.dispose()


def _print_solution_summary(results: Sequence[DepotSolveResult]) -> None:
    objective_values = [result.objective for result in results]
    has_total_objective = all(value is not None for value in objective_values)
    total_objective = sum(value for value in objective_values if value is not None)

    print("Delta:")
    for result in results:
        for (node, truck), value in sorted(result.delta.items()):
            print(f"{(node, truck)}: {value}", end=" ")
    print("")

    print("Xi:")
    for result in results:
        for (node, truck), value in sorted(result.xi.items()):
            print(f"{(node, truck)}: {value}", end=" ")
    print("")

    for result in results:
        print(
            f"Depot {result.depot}: status={result.status_name}, "
            f"obj={_format_float(result.objective)}, time={result.runtime:.2f}s, "
            f"warm_start={_format_float(result.warm_start_objective) if result.warm_start_used else 'None'}, "
            f"subproblems={result.subproblem_calls}, "
            f"root_truck_cuts={result.root_truck_cuts}, "
            f"truck_cuts={result.truck_subtour_cuts}, "
            f"robot_cuts={result.robot_subtour_cuts}, "
            f"screened={result.screening_rejections}, "
            f"farkas_cuts={result.farkas_cuts}, "
            f"nogood_cuts={result.no_good_cuts}"
        )

    if has_total_objective:
        print(f"Total Objective: {_format_float(total_objective)}")
    else:
        print(f"Total Objective (available depots only): {_format_float(total_objective)}")


def _plot_solution(my_instance, results: Sequence[DepotSolveResult]) -> None:
    plt.figure(figsize=(8, 8))
    point_x = [point[0] for point in my_instance.V_coordinates]
    point_y = [point[1] for point in my_instance.V_coordinates]
    colors = ["blue", "red", "green", "black", "orange", "brown", "cyan", "magenta"]
    vp_set = set(my_instance.VP)
    vc_set = set(my_instance.VC)

    for depot in my_instance.VD:
        for node in my_instance.depot_with_node[depot]:
            if node in my_instance.VD:
                plt.scatter(point_x[node], point_y[node], color=colors[depot % len(colors)], marker="o")
            elif node in vc_set:
                plt.scatter(point_x[node], point_y[node], color=colors[depot % len(colors)], marker="^")
            elif node in vp_set:
                plt.scatter(point_x[node], point_y[node], color=colors[depot % len(colors)], marker="p")

    for index, pos in enumerate(my_instance.V_coordinates):
        plt.annotate(index, pos, textcoords="offset points", xytext=(0, 10), ha="center")

    for result in results:
        for (i, j, truck), value in result.x.items():
            if value > INT_TOL:
                plt.plot(
                    [my_instance.V_coordinates[i][0], my_instance.V_coordinates[j][0]],
                    [my_instance.V_coordinates[i][1], my_instance.V_coordinates[j][1]],
                    linestyle="-",
                    color=colors[truck % len(colors)],
                )
        for (i, j, truck), value in result.y.items():
            if value > INT_TOL:
                plt.plot(
                    [my_instance.V_coordinates[i][0], my_instance.V_coordinates[j][0]],
                    [my_instance.V_coordinates[i][1], my_instance.V_coordinates[j][1]],
                    linestyle="--",
                    color=colors[truck % len(colors)],
                )
        for (i, j, truck), value in result.z.items():
            if value > INT_TOL:
                plt.plot(
                    [my_instance.V_coordinates[i][0], my_instance.V_coordinates[j][0]],
                    [my_instance.V_coordinates[i][1], my_instance.V_coordinates[j][1]],
                    linestyle="-.",
                    color=colors[truck % len(colors)],
                )

    plt.savefig(f"figure/{my_instance.name}_LBBD.png")
    plt.close()


def gurobiSolver(
    myInstance,
    parallel_depots: bool = True,
    max_parallel_depots: Optional[int] = None,
    feasibility_cut_mode: str = "exact",
    solve_mode: str = "exact",
    heuristic_time_limit: Optional[float] = None,
    plot_solution: bool = True,
):
    with open("log.file", "a", encoding="utf-8") as log_file:
        log_file.write("---------New LBBD model----------\n")
        log_file.write("Model Name: " + myInstance.name + "\n")

    contexts = _build_depot_contexts(myInstance)
    results = _solve_depot_contexts(
        contexts,
        parallel_depots,
        max_parallel_depots,
        feasibility_cut_mode,
        solve_mode,
        heuristic_time_limit,
    )

    _print_solution_summary(results)

    has_incumbent_for_all = all(result.objective is not None for result in results)
    if has_incumbent_for_all and plot_solution:
        _plot_solution(myInstance, results)

    total_objective = sum(result.objective for result in results if result.objective is not None)
    overall_status = (
        GRB.INFEASIBLE
        if any(result.status == GRB.INFEASIBLE for result in results)
        else GRB.OPTIMAL
        if all(result.status == GRB.OPTIMAL for result in results)
        else GRB.SUBOPTIMAL
        if all(result.objective is not None for result in results)
        else GRB.TIME_LIMIT
    )

    gp.disposeDefaultEnv()
    return {
        "status": overall_status,
        "status_name": _status_name(overall_status),
        "objective": total_objective if has_incumbent_for_all else None,
        "depot_results": results,
    }









