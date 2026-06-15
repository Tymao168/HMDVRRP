from __future__ import annotations

import os
import random
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

from gurobipy import GRB

from LBBD.nh_lbbd_solver import (
    Arc,
    DepotContext,
    DepotSolveResult,
    INT_TOL,
    NodeKey,
    TruckArcKey,
    TruckFeasibilitySubproblem,
    TruckMasterSolution,
    SingleDepotLBBD,
    _basic_infeasibility_reason,
    _build_depot_contexts,
    _format_float,
    _plot_solution,
    _screened_depot_result,
    _status_name,
)


@dataclass
class RoutePlan:
    routes: Dict[int, Tuple[int, ...]]
    route_cost: float
    stop_to_truck: Dict[int, int]


@dataclass
class RepairMove:
    objective: float
    direct_customers: Tuple[int, ...]
    parking_groups: Dict[int, Tuple[int, ...]]
    inserted_customers: Tuple[int, ...]
    switched_customers: Tuple[int, ...]
    description: str


@dataclass
class RouteRepairMove:
    objective: float
    score: float
    routes: Dict[int, Tuple[int, ...]]
    parking_groups: Dict[int, Tuple[int, ...]]
    inserted_customers: Tuple[int, ...]
    description: str


@dataclass
class LNSTerminationConfig:
    warmup_iterations: int
    warmup_time: float
    best_stall_iterations: int
    best_stall_time: float
    accept_stall_iterations: int
    accept_stall_time: float
    progress_window: int
    min_progress_in_window: int


@dataclass
class HeuristicPattern:
    direct_customers: Tuple[int, ...]
    parking_groups: Dict[int, Tuple[int, ...]]
    truck_routes: Dict[int, Tuple[int, ...]]
    stop_to_truck: Dict[int, int]
    result: DepotSolveResult


class SingleDepotConstructionHeuristic:
    def __init__(self, context: DepotContext):
        self.context = context
        self.truck_arc_set = set(context.a_v)
        self.robot_arc_set = set(context.a_r)
        self.subproblems = {
            truck: TruckFeasibilitySubproblem(context, truck)
            for truck in context.trucks
        }
        self.validation_calls = 0
        self.robot_candidates: Dict[int, List[int]] = {
            customer: self._feasible_parking_candidates(customer)
            for customer in context.customers
        }

    def dispose(self) -> None:
        for subproblem in self.subproblems.values():
            subproblem.dispose()

    def _truck_roundtrip_cost(self, node: int) -> float:
        return (
            self.context.veh_cost_matrix[self.context.depot][node]
            + self.context.veh_cost_matrix[node][self.context.depot]
        )

    def _parking_open_proxy(self, parking: int) -> float:
        incoming = [
            self.context.veh_cost_matrix[i][parking]
            for i, _ in self.context.truck_in_arcs[parking]
            if i != parking
        ]
        outgoing = [
            self.context.veh_cost_matrix[parking][j]
            for _, j in self.context.truck_out_arcs[parking]
            if j != parking
        ]
        if not incoming or not outgoing:
            return float("inf")
        return min(incoming) + min(outgoing)

    def _stop_service_proxy(self, stop: int) -> float:
        if stop in self.context.parking:
            return self._parking_open_proxy(stop)
        return self._truck_roundtrip_cost(stop)

    def _basic_infeasibility_status(self) -> Optional[int]:
        return GRB.INFEASIBLE if _basic_infeasibility_reason(self.context) is not None else None

    def _pattern_demand_by_stop(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Dict[int, float]:
        demand_by_stop = {customer: self.context.q[customer] for customer in direct_customers}
        for parking, sequence in parking_groups.items():
            if sequence:
                demand_by_stop[parking] = sum(self.context.q[customer] for customer in sequence)
        return demand_by_stop

    def _build_route_plan(
        self,
        routes: Dict[int, Sequence[int]],
        total_cost: float,
    ) -> RoutePlan:
        normalized_routes: Dict[int, Tuple[int, ...]] = {}
        stop_to_truck: Dict[int, int] = {}
        for truck in self.context.trucks:
            route_tuple = tuple(routes.get(truck, ()))
            normalized_routes[truck] = route_tuple
            for stop in route_tuple:
                stop_to_truck[stop] = truck
        return RoutePlan(routes=normalized_routes, route_cost=total_cost, stop_to_truck=stop_to_truck)

    def _customer_to_parking(self, parking_groups: Dict[int, Sequence[int]]) -> Dict[int, int]:
        mapping: Dict[int, int] = {}
        for parking, sequence in parking_groups.items():
            for customer in sequence:
                mapping[customer] = parking
        return mapping

    def _remove_customers_from_pattern(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, Sequence[int]],
        removed_customers: Set[int],
    ) -> Tuple[Set[int], Dict[int, List[int]]]:
        next_direct = set(direct_customers)
        next_direct.difference_update(removed_customers)
        next_groups: Dict[int, List[int]] = {}
        for parking, sequence in parking_groups.items():
            filtered = [customer for customer in sequence if customer not in removed_customers]
            if filtered:
                next_groups[parking] = filtered
        return next_direct, next_groups

    def _remove_customer_from_groups(
        self,
        parking_groups: Dict[int, Sequence[int]],
        customer: int,
    ) -> Tuple[Dict[int, List[int]], Optional[int]]:
        updated: Dict[int, List[int]] = {}
        current_parking: Optional[int] = None
        for parking, sequence in parking_groups.items():
            filtered = [node for node in sequence if node != customer]
            if len(filtered) != len(sequence):
                current_parking = parking
            if filtered:
                updated[parking] = filtered
        return updated, current_parking

    def _group_proxy_value(self, parking: int, sequence: Sequence[int]) -> float:
        metrics = self._robot_path_metrics(parking, sequence)
        if metrics is None:
            return float("-inf")
        direct_proxy = sum(
            self._truck_roundtrip_cost(customer)
            for customer in sequence
            if customer in self.context.regular_customers
        )
        return direct_proxy - (self._parking_open_proxy(parking) + metrics[2])

    def _customers_from_stop(
        self,
        stop: int,
        direct_customers: Set[int],
        parking_groups: Dict[int, Sequence[int]],
    ) -> List[int]:
        if stop in parking_groups:
            return list(parking_groups[stop])
        if stop in direct_customers:
            return [stop]
        return []

    def _customer_relatedness(self, seed: int, candidate: int) -> Tuple[int, float, float, int]:
        shared_parkings = len(set(self.robot_candidates[seed]).intersection(self.robot_candidates[candidate]))
        return (
            0 if shared_parkings else 1,
            self.context.veh_cost_matrix[seed][candidate] + self.context.veh_cost_matrix[candidate][seed],
            abs(self.context.q[seed] - self.context.q[candidate]),
            -shared_parkings,
        )

    def _select_related_customers(
        self,
        seed: int,
        customer_pool: Sequence[int],
        limit: int,
    ) -> List[int]:
        if limit <= 0:
            return []
        ranked = sorted(
            (customer for customer in customer_pool if customer != seed),
            key=lambda customer: self._customer_relatedness(seed, customer),
        )
        return ranked[:limit]

    def _route_metrics(
        self,
        route: Sequence[int],
        demand_by_stop: Dict[int, float],
    ) -> Optional[Tuple[float, float, float, float]]:
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
            load += demand_by_stop[node]
            prefix_time += self.context.veh_time_matrix[previous][node]
            travel_distance += self.context.veh_dist_matrix[previous][node]
            travel_cost += self.context.veh_cost_matrix[previous][node]
            previous = node

        if (previous, self.context.depot) not in self.truck_arc_set:
            return None

        travel_distance += self.context.veh_dist_matrix[previous][self.context.depot]
        travel_cost += self.context.veh_cost_matrix[previous][self.context.depot]
        return load, prefix_time, travel_distance, travel_cost

    def _route_is_feasible(
        self,
        route: Sequence[int],
        demand_by_stop: Dict[int, float],
    ) -> Optional[Tuple[float, float, float, float]]:
        metrics = self._route_metrics(route, demand_by_stop)
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

    def _improve_route(
        self,
        route: Sequence[int],
        demand_by_stop: Dict[int, float],
    ) -> Tuple[List[int], float]:
        if len(route) <= 2:
            metrics = self._route_is_feasible(route, demand_by_stop)
            return list(route), 0.0 if metrics is None else metrics[3]

        best_route = list(route)
        best_metrics = self._route_is_feasible(best_route, demand_by_stop)
        if best_metrics is None:
            return list(route), float("inf")
        best_cost = best_metrics[3]

        improved = True
        while improved:
            improved = False

            for idx, node in enumerate(best_route):
                reduced = best_route[:idx] + best_route[idx + 1 :]
                for pos in range(len(reduced) + 1):
                    candidate = reduced[:pos] + [node] + reduced[pos:]
                    if candidate == best_route:
                        continue
                    metrics = self._route_is_feasible(candidate, demand_by_stop)
                    if metrics is None:
                        continue
                    if metrics[3] + 1e-9 < best_cost:
                        best_route = candidate
                        best_cost = metrics[3]
                        improved = True
                        break
                if improved:
                    break
            if improved:
                continue

            for i in range(len(best_route) - 1):
                for j in range(i + 1, len(best_route)):
                    candidate = list(best_route)
                    candidate[i], candidate[j] = candidate[j], candidate[i]
                    metrics = self._route_is_feasible(candidate, demand_by_stop)
                    if metrics is None:
                        continue
                    if metrics[3] + 1e-9 < best_cost:
                        best_route = candidate
                        best_cost = metrics[3]
                        improved = True
                        break
                if improved:
                    break
            if improved:
                continue

            for i in range(len(best_route) - 1):
                for j in range(i + 1, len(best_route)):
                    candidate = best_route[:i] + list(reversed(best_route[i : j + 1])) + best_route[j + 1 :]
                    metrics = self._route_is_feasible(candidate, demand_by_stop)
                    if metrics is None:
                        continue
                    if metrics[3] + 1e-9 < best_cost:
                        best_route = candidate
                        best_cost = metrics[3]
                        improved = True
                        break
                if improved:
                    break

        return best_route, best_cost

    def _robot_path_metrics(
        self,
        parking: int,
        sequence: Sequence[int],
    ) -> Optional[Tuple[float, float, float]]:
        total_demand = sum(self.context.q[node] for node in sequence)
        if total_demand > self.context.ur + 1e-9:
            return None

        if not sequence:
            return 0.0, 0.0, 0.0

        total_power = 0.0
        total_cost = 0.0
        previous = parking
        for node in sequence:
            if (previous, node) not in self.robot_arc_set:
                return None
            total_power += self.context.rob_power_matrix[previous][node]
            total_cost += self.context.rob_cost_matrix[previous][node]
            previous = node

        if (previous, parking) not in self.robot_arc_set:
            return None

        total_power += self.context.rob_power_matrix[previous][parking]
        total_cost += self.context.rob_cost_matrix[previous][parking]
        if total_power > self.context.robot_ub + 1e-9:
            return None

        return total_demand, total_power, total_cost

    def _best_robot_insertion(
        self,
        parking: int,
        base_sequence: Sequence[int],
        customer: int,
    ) -> Optional[Tuple[List[int], float, float, float]]:
        best_candidate: Optional[Tuple[List[int], float, float, float]] = None
        base_metrics = self._robot_path_metrics(parking, base_sequence)
        base_cost = 0.0 if base_metrics is None else base_metrics[2]

        for pos in range(len(base_sequence) + 1):
            candidate = list(base_sequence[:pos]) + [customer] + list(base_sequence[pos:])
            metrics = self._robot_path_metrics(parking, candidate)
            if metrics is None:
                continue
            _, total_power, total_cost = metrics
            delta_cost = total_cost - base_cost
            score = (delta_cost, total_power, total_cost)
            if best_candidate is None or score < best_candidate[1:]:
                best_candidate = (candidate, delta_cost, total_power, total_cost)

        return best_candidate

    def _feasible_parking_candidates(self, customer: int) -> List[int]:
        candidates = []
        for parking in self.context.parking:
            metrics = self._robot_path_metrics(parking, [customer])
            if metrics is not None:
                candidates.append(parking)
        candidates.sort(key=lambda node: self._parking_open_proxy(node))
        return candidates

    def _apply_robot_assignment(
        self,
        groups: Dict[int, List[int]],
        customer_to_parking: Dict[int, int],
        customer: int,
        parking: int,
        sequence: List[int],
    ) -> None:
        groups[parking] = sequence
        customer_to_parking[customer] = parking

    def _build_parking_groups(
        self,
        optional_limit: int,
        threshold_factor: float,
    ) -> Optional[Tuple[Dict[int, List[int]], Set[int]]]:
        groups: Dict[int, List[int]] = {parking: [] for parking in self.context.parking}
        customer_to_parking: Dict[int, int] = {}

        mandatory_customers = sorted(
            self.context.robot_only_customers,
            key=lambda node: (len(self.robot_candidates[node]), -self._truck_roundtrip_cost(node)),
        )
        for customer in mandatory_customers:
            best_choice = None
            for parking in self.robot_candidates[customer]:
                candidate = self._best_robot_insertion(parking, groups[parking], customer)
                if candidate is None:
                    continue
                sequence, delta_cost, _, total_cost = candidate
                adjusted_cost = delta_cost + (self._parking_open_proxy(parking) if not groups[parking] else 0.0)
                score = (adjusted_cost, total_cost, parking)
                if best_choice is None or score < best_choice[0]:
                    best_choice = (score, parking, sequence)

            if best_choice is None:
                return None

            _, parking, sequence = best_choice
            self._apply_robot_assignment(groups, customer_to_parking, customer, parking, sequence)

        optional_assigned = 0
        regular_customers = sorted(
            self.context.regular_customers,
            key=lambda node: (self._truck_roundtrip_cost(node), self.context.q[node]),
            reverse=True,
        )
        for customer in regular_customers:
            if optional_assigned >= optional_limit:
                break

            best_choice = None
            for parking in self.robot_candidates[customer]:
                candidate = self._best_robot_insertion(parking, groups[parking], customer)
                if candidate is None:
                    continue
                sequence, delta_cost, _, total_cost = candidate
                adjusted_cost = delta_cost + (self._parking_open_proxy(parking) if not groups[parking] else 0.0)
                score = (adjusted_cost, total_cost, parking)
                if best_choice is None or score < best_choice[0]:
                    best_choice = (score, parking, sequence)

            if best_choice is None:
                continue

            direct_cost = self._truck_roundtrip_cost(customer)
            adjusted_cost, _, _ = best_choice[0]
            if adjusted_cost > threshold_factor * direct_cost + 1e-9:
                continue

            _, parking, sequence = best_choice
            self._apply_robot_assignment(groups, customer_to_parking, customer, parking, sequence)
            optional_assigned += 1

        used_parking = {parking for parking, sequence in groups.items() if sequence}
        return ({parking: groups[parking] for parking in used_parking}, used_parking)

    def _build_randomized_parking_groups(
        self,
        optional_limit: int,
        threshold_factor: float,
        rng: random.Random,
    ) -> Optional[Tuple[Dict[int, List[int]], Set[int]]]:
        groups: Dict[int, List[int]] = {parking: [] for parking in self.context.parking}
        customer_to_parking: Dict[int, int] = {}

        mandatory_customers = list(self.context.robot_only_customers)
        mandatory_customers.sort(
            key=lambda node: (
                len(self.robot_candidates[node]),
                -self._truck_roundtrip_cost(node),
                rng.random(),
            )
        )
        for customer in mandatory_customers:
            choices = []
            for parking in self.robot_candidates[customer]:
                candidate = self._best_robot_insertion(parking, groups[parking], customer)
                if candidate is None:
                    continue
                sequence, delta_cost, total_power, total_cost = candidate
                adjusted_cost = delta_cost + (self._parking_open_proxy(parking) if not groups[parking] else 0.0)
                score = (
                    adjusted_cost * (1.0 + 0.06 * rng.random()),
                    total_power,
                    total_cost,
                    parking,
                )
                choices.append((score, parking, sequence))

            if not choices:
                return None

            choices.sort(key=lambda item: item[0])
            if len(choices) == 1 or rng.random() < 0.72:
                _, parking, sequence = choices[0]
            else:
                _, parking, sequence = choices[rng.randrange(min(3, len(choices)))]
            self._apply_robot_assignment(groups, customer_to_parking, customer, parking, sequence)

        optional_assigned = 0
        target_optional = min(optional_limit, len(self.context.regular_customers))
        regular_customers = list(self.context.regular_customers)
        regular_customers.sort(
            key=lambda node: (
                self._truck_roundtrip_cost(node) * (1.0 + 0.35 * rng.random()),
                self.context.q[node] * (1.0 + 0.2 * rng.random()),
                -len(self.robot_candidates[node]),
            ),
            reverse=True,
        )
        for customer in regular_customers:
            if optional_assigned >= target_optional:
                break

            choices = []
            for parking in self.robot_candidates[customer]:
                candidate = self._best_robot_insertion(parking, groups[parking], customer)
                if candidate is None:
                    continue
                sequence, delta_cost, _, total_cost = candidate
                adjusted_cost = delta_cost + (self._parking_open_proxy(parking) if not groups[parking] else 0.0)
                score = (
                    adjusted_cost * (1.0 + 0.08 * rng.random()),
                    total_cost,
                    -len(groups[parking]),
                    parking,
                )
                choices.append((score, adjusted_cost, parking, sequence))

            if not choices:
                continue

            choices.sort(key=lambda item: item[0])
            direct_cost = self._truck_roundtrip_cost(customer)
            relaxed_threshold = threshold_factor * (1.0 + 0.1 * rng.random())
            if choices[0][1] > relaxed_threshold * direct_cost + 1e-9 and rng.random() < 0.85:
                continue

            if len(choices) == 1 or rng.random() < 0.67:
                _, _, parking, sequence = choices[0]
            else:
                _, _, parking, sequence = choices[rng.randrange(min(3, len(choices)))]
            self._apply_robot_assignment(groups, customer_to_parking, customer, parking, sequence)
            optional_assigned += 1

        used_parking = {parking for parking, sequence in groups.items() if sequence}
        return ({parking: groups[parking] for parking in used_parking}, used_parking)

    def _generate_randomized_seed_patterns(
        self,
        solve_start_time: float,
        max_trials: int,
    ) -> List[HeuristicPattern]:
        if max_trials <= 0:
            return []

        rng = random.Random(15485863 + 7919 * self.context.depot + 104729 * len(self.context.customers))
        optional_caps = sorted(
            {
                0,
                min(len(self.context.regular_customers), max(1, len(self.context.regular_customers) // 6)),
                min(len(self.context.regular_customers), max(1, len(self.context.regular_customers) // 4)),
                min(len(self.context.regular_customers), max(1, len(self.context.regular_customers) // 3)),
                min(len(self.context.regular_customers), max(2, len(self.context.regular_customers) // 2)),
                len(self.context.regular_customers),
            }
        )
        threshold_factors = [0.95, 1.05, 1.2, 1.4, 1.7, 2.1]
        seed_patterns: List[HeuristicPattern] = []
        seen_signatures: Set[Tuple[Tuple[int, ...], Tuple[Tuple[int, Tuple[int, ...]], ...]]] = set()

        for _ in range(max_trials):
            optional_limit = rng.choice(optional_caps) if optional_caps else 0
            threshold_factor = rng.choice(threshold_factors)
            groups_data = self._build_randomized_parking_groups(optional_limit, threshold_factor, rng)
            if groups_data is None:
                continue

            parking_groups, _ = groups_data
            served_by_robot = {node for sequence in parking_groups.values() for node in sequence}
            direct_customers = [
                customer
                for customer in self.context.customers
                if customer not in self.context.robot_only_customers and customer not in served_by_robot
            ]

            if rng.random() < 0.8:
                intensified = self._intensify_pattern(
                    set(direct_customers),
                    {parking: list(sequence) for parking, sequence in parking_groups.items()},
                )
            else:
                intensified = None

            if intensified is not None:
                direct_candidate, parking_candidate = intensified
            else:
                direct_candidate = tuple(sorted(direct_customers))
                parking_candidate = {
                    parking: tuple(sequence)
                    for parking, sequence in parking_groups.items()
                    if sequence
                }

            signature = self._pattern_signature(direct_candidate, parking_candidate)
            if signature in seen_signatures:
                continue
            seen_signatures.add(signature)

            candidate = self._evaluate_pattern_candidate(
                direct_candidate,
                parking_candidate,
                time.perf_counter() - solve_start_time,
            )
            if candidate is not None:
                seed_patterns.append(candidate)

        return seed_patterns

    def _improve_route_set(
        self,
        routes: Dict[int, List[int]],
        demand_by_stop: Dict[int, float],
    ) -> Tuple[Dict[int, List[int]], float]:
        working = {truck: list(routes.get(truck, [])) for truck in self.context.trucks}
        route_costs: Dict[int, float] = {}
        total_cost = 0.0
        for truck in self.context.trucks:
            improved_route, improved_cost = self._improve_route(working[truck], demand_by_stop)
            working[truck] = improved_route
            route_costs[truck] = improved_cost
            total_cost += improved_cost

        trucks = list(self.context.trucks)
        improved = True
        while improved:
            improved = False
            best_move = None
            best_total = total_cost

            for source in trucks:
                source_route = working[source]
                if not source_route:
                    continue
                for idx, node in enumerate(source_route):
                    reduced_source = source_route[:idx] + source_route[idx + 1 :]
                    source_metrics = self._route_is_feasible(reduced_source, demand_by_stop)
                    if source_metrics is None:
                        continue
                    for target in trucks:
                        if target == source:
                            continue
                        target_route = working[target]
                        for pos in range(len(target_route) + 1):
                            candidate_target = target_route[:pos] + [node] + target_route[pos:]
                            target_metrics = self._route_is_feasible(candidate_target, demand_by_stop)
                            if target_metrics is None:
                                continue
                            candidate_total = (
                                total_cost
                                - route_costs[source]
                                - route_costs[target]
                                + source_metrics[3]
                                + target_metrics[3]
                            )
                            if candidate_total + 1e-9 < best_total:
                                best_total = candidate_total
                                best_move = ("relocate", source, target, reduced_source, candidate_target)

            for idx_a, truck_a in enumerate(trucks):
                route_a = working[truck_a]
                if not route_a:
                    continue
                for truck_b in trucks[idx_a + 1 :]:
                    route_b = working[truck_b]
                    if not route_b:
                        continue
                    for pos_a, node_a in enumerate(route_a):
                        base_a = route_a[:pos_a] + route_a[pos_a + 1 :]
                        for pos_b, node_b in enumerate(route_b):
                            base_b = route_b[:pos_b] + route_b[pos_b + 1 :]
                            for insert_a in range(len(base_a) + 1):
                                candidate_a = base_a[:insert_a] + [node_b] + base_a[insert_a:]
                                metrics_a = self._route_is_feasible(candidate_a, demand_by_stop)
                                if metrics_a is None:
                                    continue
                                for insert_b in range(len(base_b) + 1):
                                    candidate_b = base_b[:insert_b] + [node_a] + base_b[insert_b:]
                                    metrics_b = self._route_is_feasible(candidate_b, demand_by_stop)
                                    if metrics_b is None:
                                        continue
                                    candidate_total = (
                                        total_cost
                                        - route_costs[truck_a]
                                        - route_costs[truck_b]
                                        + metrics_a[3]
                                        + metrics_b[3]
                                    )
                                    if candidate_total + 1e-9 < best_total:
                                        best_total = candidate_total
                                        best_move = ("swap", truck_a, truck_b, candidate_a, candidate_b)

            if best_move is None:
                break

            improved = True
            if best_move[0] == "relocate":
                _, source, target, candidate_source, candidate_target = best_move
                working[source], cost_source = self._improve_route(candidate_source, demand_by_stop)
                working[target], cost_target = self._improve_route(candidate_target, demand_by_stop)
                total_cost = total_cost - route_costs[source] - route_costs[target] + cost_source + cost_target
                route_costs[source] = cost_source
                route_costs[target] = cost_target
            else:
                _, truck_a, truck_b, candidate_a, candidate_b = best_move
                working[truck_a], cost_a = self._improve_route(candidate_a, demand_by_stop)
                working[truck_b], cost_b = self._improve_route(candidate_b, demand_by_stop)
                total_cost = total_cost - route_costs[truck_a] - route_costs[truck_b] + cost_a + cost_b
                route_costs[truck_a] = cost_a
                route_costs[truck_b] = cost_b

        return working, total_cost

    def _construct_routes(
        self,
        stops: Sequence[int],
        demand_by_stop: Dict[int, float],
        intensify: bool = True,
    ) -> Optional[RoutePlan]:
        if not stops:
            return self._build_route_plan({truck: [] for truck in self.context.trucks}, 0.0)

        stop_proxy = {stop: self._stop_service_proxy(stop) for stop in stops}
        seed = (
            7919 * self.context.depot
            + 104729 * len(stops)
            + sum((idx + 1) * stop for idx, stop in enumerate(sorted(stops)))
        )
        rng = random.Random(seed)

        candidate_orders = [
            tuple(sorted(stops, key=lambda node: (stop_proxy[node], demand_by_stop[node], node), reverse=True)),
            tuple(sorted(stops, key=lambda node: (demand_by_stop[node], stop_proxy[node], node), reverse=True)),
            tuple(sorted(stops, key=lambda node: (self.context.veh_time_matrix[self.context.depot][node], demand_by_stop[node], node), reverse=True)),
            tuple(
                sorted(
                    stops,
                    key=lambda node: (
                        stop_proxy[node] / max(demand_by_stop[node], 1.0),
                        stop_proxy[node],
                        node,
                    ),
                    reverse=True,
                )
            ),
        ]
        random_order_count = min(6 if intensify else 2, max(1 if not intensify else 2, len(stops) // 2))
        for _ in range(random_order_count):
            candidate_orders.append(
                tuple(
                    sorted(
                        stops,
                        key=lambda node: (
                            stop_proxy[node] * (1.0 + 0.25 * rng.random())
                            + demand_by_stop[node] * (1.0 + 0.5 * rng.random()),
                            rng.random(),
                        ),
                        reverse=True,
                    )
                )
            )

        best_plan: Optional[RoutePlan] = None
        seen_orders = set()
        for ordering in candidate_orders:
            if ordering in seen_orders:
                continue
            seen_orders.add(ordering)
            routes = {truck: [] for truck in self.context.trucks}
            route_costs = {truck: 0.0 for truck in self.context.trucks}
            feasible = True

            for stop in ordering:
                best_choice = None
                for truck in self.context.trucks:
                    base_route = routes[truck]
                    base_cost = route_costs[truck]
                    for pos in range(len(base_route) + 1):
                        candidate_route = base_route[:pos] + [stop] + base_route[pos:]
                        metrics = self._route_is_feasible(candidate_route, demand_by_stop)
                        if metrics is None:
                            continue
                        candidate_cost = metrics[3]
                        delta_cost = candidate_cost - base_cost
                        score = (
                            delta_cost,
                            metrics[0] / max(self.context.uv, 1.0),
                            metrics[1] / max(self.context.max_travel_time, 1.0),
                            len(candidate_route),
                            truck,
                            pos,
                        )
                        if best_choice is None or score < best_choice[0]:
                            best_choice = (score, truck, candidate_route, candidate_cost)

                if best_choice is None:
                    feasible = False
                    break

                _, truck, candidate_route, candidate_cost = best_choice
                routes[truck] = candidate_route
                route_costs[truck] = candidate_cost

            if not feasible:
                continue

            if intensify:
                improved_routes, total_cost = self._improve_route_set(routes, demand_by_stop)
            else:
                total_cost = 0.0
                improved_routes = {}
                for truck, route in routes.items():
                    improved_route, improved_cost = self._improve_route(route, demand_by_stop)
                    improved_routes[truck] = improved_route
                    total_cost += improved_cost
            candidate_plan = self._build_route_plan(improved_routes, total_cost)
            if best_plan is None or candidate_plan.route_cost + 1e-9 < best_plan.route_cost:
                best_plan = candidate_plan

        return best_plan

    def _build_truck_solution(
        self,
        truck: int,
        route: Sequence[int],
        parking_groups: Dict[int, List[int]],
    ) -> TruckMasterSolution:
        x: Dict[Arc, int] = {}
        y: Dict[Arc, int] = {}
        z: Dict[Arc, int] = {}
        xi: Dict[int, int] = {}
        delta: Dict[int, int] = {}
        selected_truck_arcs: List[Arc] = []
        selected_robot_arcs: List[Arc] = []

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

        for node in route:
            if node in parking_groups:
                xi[node] = 1
                delta[node] = 1
                sequence = parking_groups[node]
                previous = node
                for customer in sequence:
                    arc = (previous, customer)
                    z[arc] = 1
                    selected_robot_arcs.append(arc)
                    previous = customer
                arc = (previous, node)
                z[arc] = 1
                selected_robot_arcs.append(arc)

        truck_in_count = {node: 0 for node in self.context.nodes}
        for _, node in selected_truck_arcs:
            truck_in_count[node] += 1

        robot_in_count = {node: 0 for node in self.context.cp_nodes}
        for _, node in selected_robot_arcs:
            robot_in_count[node] += 1

        return TruckMasterSolution(
            truck=truck,
            active=bool(route),
            x=x,
            y=y,
            z=z,
            xi=xi,
            delta=delta,
            selected_truck_arcs=selected_truck_arcs,
            selected_robot_arcs=selected_robot_arcs,
            truck_in_count=truck_in_count,
            robot_in_count=robot_in_count,
            signature=tuple(),
        )

    def _compose_result(
        self,
        status: int,
        runtime: float,
        truck_solutions: Optional[Dict[int, TruckMasterSolution]] = None,
    ) -> DepotSolveResult:
        if not truck_solutions:
            return DepotSolveResult(
                depot=self.context.depot,
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
                subproblem_calls=self.validation_calls,
                warm_start_used=False,
                warm_start_objective=None,
                root_truck_cuts=0,
                truck_subtour_cuts=0,
                robot_subtour_cuts=0,
                no_good_cuts=0,
                farkas_cuts=0,
                screening_rejections=0,
            )

        x_sol: Dict[TruckArcKey, float] = {}
        y_sol: Dict[TruckArcKey, float] = {}
        z_sol: Dict[TruckArcKey, float] = {}
        xi_sol: Dict[NodeKey, float] = {}
        delta_sol: Dict[NodeKey, float] = {}
        objective = 0.0

        for truck, solution in truck_solutions.items():
            if solution.active:
                objective += self.context.fixed_cost
            for i, j in solution.selected_truck_arcs:
                x_sol[i, j, truck] = 1.0
                objective += self.context.veh_cost_matrix[i][j]
            for i, j in solution.selected_robot_arcs:
                z_sol[i, j, truck] = 1.0
                objective += self.context.rob_cost_matrix[i][j]
            for node, value in solution.xi.items():
                if value > 0:
                    xi_sol[node, truck] = float(value)
            for node, value in solution.delta.items():
                if value > 0:
                    delta_sol[node, truck] = float(value)
            for i, j in solution.y:
                y_sol[i, j, truck] = 1.0

        return DepotSolveResult(
            depot=self.context.depot,
            status=status,
            status_name=_status_name(status),
            objective=objective,
            best_bound=None,
            mip_gap=None,
            runtime=runtime,
            x=x_sol,
            y=y_sol,
            z=z_sol,
            xi=xi_sol,
            delta=delta_sol,
            subproblem_calls=self.validation_calls,
            warm_start_used=True,
            warm_start_objective=objective,
            root_truck_cuts=0,
            truck_subtour_cuts=0,
            robot_subtour_cuts=0,
            no_good_cuts=0,
            farkas_cuts=0,
            screening_rejections=0,
        )

    def _pattern_proxy_objective(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Optional[Tuple[float, RoutePlan]]:
        normalized_groups = {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence}
        demand_by_stop = self._pattern_demand_by_stop(direct_customers, normalized_groups)
        route_plan = self._construct_routes(list(direct_customers) + list(normalized_groups), demand_by_stop, intensify=False)
        if route_plan is None:
            return None

        robot_cost = 0.0
        for parking, sequence in normalized_groups.items():
            metrics = self._robot_path_metrics(parking, sequence)
            if metrics is None:
                return None
            robot_cost += metrics[2]

        return route_plan.route_cost + robot_cost, route_plan

    def _score_repair_move(
        self,
        direct_customers: Set[int],
        parking_groups: Dict[int, List[int]],
        inserted_customers: Set[int],
        switched_customers: Set[int],
        description: str,
    ) -> Optional[RepairMove]:
        normalized_groups = {
            parking: tuple(sequence)
            for parking, sequence in parking_groups.items()
            if sequence
        }
        proxy = self._pattern_proxy_objective(tuple(sorted(direct_customers)), normalized_groups)
        if proxy is None:
            return None
        objective, _ = proxy
        return RepairMove(
            objective=objective,
            direct_customers=tuple(sorted(direct_customers)),
            parking_groups=normalized_groups,
            inserted_customers=tuple(sorted(inserted_customers)),
            switched_customers=tuple(sorted(switched_customers)),
            description=description,
        )

    def _augment_parking_group(
        self,
        parking: int,
        base_sequence: Sequence[int],
        direct_customers: Set[int],
        remaining_customers: Set[int],
        favor_robot: float,
    ) -> Tuple[List[int], Set[int], Set[int]]:
        sequence = list(base_sequence)
        inserted_customers: Set[int] = set()
        switched_customers: Set[int] = set()
        remaining_pool = [
            customer
            for customer in remaining_customers
            if customer not in sequence
            and parking in self.robot_candidates[customer]
        ]
        direct_pool = [
            customer
            for customer in direct_customers
            if customer in self.context.regular_customers
            and customer not in sequence
            and parking in self.robot_candidates[customer]
        ]

        extra_limit = 3 if not base_sequence else 2
        while extra_limit > 0:
            best_choice = None
            for origin, pool in (("removed", remaining_pool), ("direct", direct_pool)):
                for customer in list(pool):
                    candidate = self._best_robot_insertion(parking, sequence, customer)
                    if candidate is None:
                        continue
                    direct_proxy = max(self._truck_roundtrip_cost(customer), 1.0)
                    if customer in self.context.regular_customers and candidate[1] > favor_robot * direct_proxy + 1e-9:
                        continue
                    score = (
                        0 if customer in self.context.robot_only_customers else 1,
                        candidate[1] / direct_proxy,
                        candidate[2],
                        candidate[3],
                        customer,
                    )
                    if best_choice is None or score < best_choice[0]:
                        best_choice = (score, origin, customer, candidate[0])

            if best_choice is None:
                break

            _, origin, customer, new_sequence = best_choice
            sequence = list(new_sequence)
            if origin == "removed":
                inserted_customers.add(customer)
                remaining_pool.remove(customer)
            else:
                switched_customers.add(customer)
                direct_pool.remove(customer)
            extra_limit -= 1

        return sequence, inserted_customers, switched_customers

    def _generate_route_bootstrap_seeds(
        self,
        solve_start_time: float,
        max_attempts: int,
    ) -> List[HeuristicPattern]:
        if max_attempts <= 0:
            return []

        repair_modes = ("robot_bias", "regret", "greedy", "direct_bias")
        beam_width = max(4, min(10, max_attempts))
        empty_routes = {truck: tuple() for truck in self.context.trucks}
        frontier = [(dict(empty_routes), {}, frozenset(self.context.customers), 0)]
        seeds: List[HeuristicPattern] = []
        seen_seed_signatures = set()

        for _ in range(len(self.context.customers) + 1):
            candidate_map = {}
            next_frontier = []

            for routes, parking_groups, remaining, mode_offset in frontier:
                if not remaining:
                    candidate = self._evaluate_route_pattern_candidate(
                        routes,
                        parking_groups,
                        time.perf_counter() - solve_start_time,
                        intensify_routes=True,
                    )
                    if candidate is None:
                        continue
                    signature = self._route_pattern_signature(candidate.truck_routes, candidate.parking_groups)
                    if signature in seen_seed_signatures:
                        continue
                    seen_seed_signatures.add(signature)
                    seeds.append(candidate)
                    continue

                base_state = self._route_state_objective(routes, parking_groups, intensify=False)
                if base_state is None:
                    continue
                base_objective = base_state[0]
                remaining_set = set(remaining)

                for mode_idx, repair_mode in enumerate(repair_modes):
                    for customer in remaining:
                        customer_moves = self._generate_route_repair_moves(
                            customer,
                            routes,
                            parking_groups,
                            remaining_set,
                            repair_mode,
                            base_objective,
                        )
                        for move in customer_moves[:2]:
                            next_remaining = frozenset(remaining_set.difference(move.inserted_customers))
                            open_penalty = 0 if not move.description.startswith("robot-open") else 1
                            priority = (
                                len(next_remaining),
                                len(move.parking_groups),
                                open_penalty,
                                move.objective,
                                move.score,
                                -len(move.inserted_customers),
                            )
                            signature = (
                                len(next_remaining),
                                self._route_pattern_signature(move.routes, move.parking_groups),
                            )
                            state = (
                                priority,
                                dict(move.routes),
                                {parking: tuple(sequence) for parking, sequence in move.parking_groups.items()},
                                next_remaining,
                                (mode_offset + mode_idx + 1) % len(repair_modes),
                            )
                            incumbent = candidate_map.get(signature)
                            if incumbent is None or priority < incumbent[0]:
                                candidate_map[signature] = state

            if not candidate_map:
                break

            next_frontier = sorted(candidate_map.values(), key=lambda item: item[0])[:beam_width]
            frontier = [
                (routes, parking_groups, remaining, mode_offset)
                for _, routes, parking_groups, remaining, mode_offset in next_frontier
            ]

        seeds.sort(key=lambda pattern: pattern.result.objective)
        return seeds[: max_attempts]

    def _generate_repair_moves(
        self,
        customer: int,
        direct_customers: Set[int],
        parking_groups: Dict[int, List[int]],
        remaining_customers: Set[int],
        favor_robot: float,
    ) -> List[RepairMove]:
        moves: List[RepairMove] = []
        seen_signatures = set()

        if customer in self.context.regular_customers:
            direct_move = self._score_repair_move(
                set(direct_customers) | {customer},
                {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence},
                {customer},
                set(),
                f"direct:{customer}",
            )
            if direct_move is not None:
                signature = self._pattern_signature(direct_move.direct_customers, direct_move.parking_groups)
                seen_signatures.add(signature)
                moves.append(direct_move)

        for parking in self.robot_candidates[customer]:
            base_sequence = parking_groups.get(parking, [])
            candidate = self._best_robot_insertion(parking, base_sequence, customer)
            if candidate is None:
                continue

            sequence = list(candidate[0])
            inserted_customers = {customer}
            switched_customers: Set[int] = set()
            augmented_sequence, extra_inserted, extra_switched = self._augment_parking_group(
                parking,
                sequence,
                direct_customers,
                set(remaining_customers) - {customer},
                favor_robot,
            )
            inserted_customers.update(extra_inserted)
            switched_customers.update(extra_switched)

            groups_copy = {parking_node: list(seq) for parking_node, seq in parking_groups.items() if seq}
            groups_copy[parking] = augmented_sequence
            direct_copy = set(direct_customers)
            direct_copy.difference_update(inserted_customers)
            direct_copy.difference_update(switched_customers)
            robot_move = self._score_repair_move(
                direct_copy,
                groups_copy,
                inserted_customers,
                switched_customers,
                f"robot:{parking}",
            )
            if robot_move is None:
                continue
            signature = self._pattern_signature(robot_move.direct_customers, robot_move.parking_groups)
            if signature in seen_signatures:
                continue
            seen_signatures.add(signature)
            moves.append(robot_move)

        moves.sort(key=lambda move: move.objective)
        return moves

    def _intensify_pattern(
        self,
        direct_customers: Set[int],
        parking_groups: Dict[int, List[int]],
    ) -> Optional[Tuple[Tuple[int, ...], Dict[int, Tuple[int, ...]]]]:
        working_direct = set(direct_customers)
        working_groups = {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence}
        proxy = self._pattern_proxy_objective(working_direct, working_groups)
        if proxy is None:
            return None
        current_objective, _ = proxy

        for _ in range(2):
            best_state = None
            customer_to_parking = self._customer_to_parking(working_groups)

            direct_candidates = sorted(
                (
                    customer
                    for customer in working_direct
                    if customer in self.context.regular_customers and self.robot_candidates[customer]
                ),
                key=lambda customer: (
                    self._truck_roundtrip_cost(customer),
                    len(self.robot_candidates[customer]),
                    self.context.q[customer],
                ),
                reverse=True,
            )
            direct_candidates = direct_candidates[: min(len(direct_candidates), 6)]
            for customer in direct_candidates:
                for parking in self.robot_candidates[customer]:
                    candidate = self._best_robot_insertion(parking, working_groups.get(parking, []), customer)
                    if candidate is None:
                        continue
                    groups_copy = {parking_node: list(seq) for parking_node, seq in working_groups.items() if seq}
                    groups_copy[parking] = list(candidate[0])
                    direct_copy = set(working_direct)
                    direct_copy.remove(customer)
                    proxy_move = self._pattern_proxy_objective(direct_copy, groups_copy)
                    if proxy_move is None:
                        continue
                    objective, _ = proxy_move
                    if objective + 1e-9 < current_objective and (
                        best_state is None or objective + 1e-9 < best_state[0]
                    ):
                        best_state = (objective, direct_copy, groups_copy)

            robot_regular = sorted(
                (
                    customer
                    for customer in customer_to_parking
                    if customer in self.context.regular_customers
                ),
                key=lambda customer: (
                    len(self.robot_candidates[customer]),
                    self._truck_roundtrip_cost(customer),
                    self.context.q[customer],
                ),
                reverse=True,
            )[:6]
            for customer in robot_regular:
                groups_without, current_parking = self._remove_customer_from_groups(working_groups, customer)
                if current_parking is None:
                    continue

                direct_copy = set(working_direct)
                direct_copy.add(customer)
                proxy_move = self._pattern_proxy_objective(direct_copy, groups_without)
                if proxy_move is not None:
                    objective, _ = proxy_move
                    if objective + 1e-9 < current_objective and (
                        best_state is None or objective + 1e-9 < best_state[0]
                    ):
                        best_state = (objective, direct_copy, groups_without)

                for parking in self.robot_candidates[customer]:
                    if parking == current_parking:
                        continue
                    candidate = self._best_robot_insertion(parking, groups_without.get(parking, []), customer)
                    if candidate is None:
                        continue
                    groups_copy = {parking_node: list(seq) for parking_node, seq in groups_without.items() if seq}
                    groups_copy[parking] = list(candidate[0])
                    proxy_move = self._pattern_proxy_objective(working_direct, groups_copy)
                    if proxy_move is None:
                        continue
                    objective, _ = proxy_move
                    if objective + 1e-9 < current_objective and (
                        best_state is None or objective + 1e-9 < best_state[0]
                    ):
                        best_state = (objective, set(working_direct), groups_copy)

            if best_state is None:
                break

            current_objective, working_direct, working_groups = best_state

        cleaned_groups = {parking: tuple(sequence) for parking, sequence in working_groups.items() if sequence}
        return tuple(sorted(working_direct)), cleaned_groups

    def _evaluate_pattern(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, List[int]],
        runtime_so_far: float,
    ) -> Optional[Tuple[DepotSolveResult, RoutePlan]]:
        normalized_groups = {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence}
        demand_by_stop = self._pattern_demand_by_stop(direct_customers, normalized_groups)
        route_plan = self._construct_routes(list(direct_customers) + list(normalized_groups), demand_by_stop)
        if route_plan is None:
            return None

        truck_solutions: Dict[int, TruckMasterSolution] = {}
        for truck in self.context.trucks:
            solution = self._build_truck_solution(truck, route_plan.routes[truck], normalized_groups)
            truck_solutions[truck] = solution
            if solution.active:
                self.validation_calls += 1
                check = self.subproblems[truck].check_feasibility(solution)
                if not check.feasible:
                    return None

        return self._compose_result(GRB.SUBOPTIMAL, runtime_so_far, truck_solutions), route_plan

    def _pattern_signature(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Tuple[Tuple[int, ...], Tuple[Tuple[int, Tuple[int, ...]], ...]]:
        group_signature = tuple(
            (parking, tuple(parking_groups[parking]))
            for parking in sorted(parking_groups)
            if parking_groups[parking]
        )
        return tuple(sorted(direct_customers)), group_signature

    def _evaluate_pattern_candidate(
        self,
        direct_customers: Sequence[int],
        parking_groups: Dict[int, Sequence[int]],
        runtime_so_far: float,
    ) -> Optional[HeuristicPattern]:
        normalized_groups = {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence}
        evaluation = self._evaluate_pattern(direct_customers, normalized_groups, runtime_so_far)
        if evaluation is None:
            return None
        candidate, route_plan = evaluation
        if candidate.objective is None:
            return None
        return HeuristicPattern(
            direct_customers=tuple(sorted(direct_customers)),
            parking_groups={parking: tuple(normalized_groups[parking]) for parking in sorted(normalized_groups)},
            truck_routes=dict(route_plan.routes),
            stop_to_truck=dict(route_plan.stop_to_truck),
            result=candidate,
        )

    def _copy_routes(
        self,
        routes: Dict[int, Sequence[int]],
    ) -> Dict[int, List[int]]:
        return {truck: list(routes.get(truck, ())) for truck in self.context.trucks}

    def _copy_groups(
        self,
        parking_groups: Dict[int, Sequence[int]],
    ) -> Dict[int, List[int]]:
        return {
            parking: list(sequence)
            for parking, sequence in parking_groups.items()
            if sequence
        }

    def _route_demand_by_stop(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Dict[int, float]:
        groups = {parking: tuple(sequence) for parking, sequence in parking_groups.items() if sequence}
        demand_by_stop: Dict[int, float] = {}
        for truck in self.context.trucks:
            for stop in routes.get(truck, ()):
                if stop in groups:
                    demand_by_stop[stop] = sum(self.context.q[customer] for customer in groups[stop])
                else:
                    demand_by_stop[stop] = self.context.q[stop]
        return demand_by_stop

    def _route_pattern_signature(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Tuple[
        Tuple[Tuple[int, Tuple[int, ...]], ...],
        Tuple[Tuple[int, Tuple[int, ...]], ...],
    ]:
        route_signature = tuple(
            (truck, tuple(routes.get(truck, ())))
            for truck in self.context.trucks
        )
        group_signature = tuple(
            (parking, tuple(parking_groups[parking]))
            for parking in sorted(parking_groups)
            if parking_groups[parking]
        )
        return route_signature, group_signature

    def _route_plan_from_existing_routes(
        self,
        routes: Dict[int, Sequence[int]],
        demand_by_stop: Dict[int, float],
        intensify: bool = False,
    ) -> Optional[RoutePlan]:
        working_routes = self._copy_routes(routes)
        seen_stops: Set[int] = set()
        total_cost = 0.0
        for truck in self.context.trucks:
            route = working_routes[truck]
            for stop in route:
                if stop in seen_stops:
                    return None
                seen_stops.add(stop)
            metrics = self._route_is_feasible(route, demand_by_stop)
            if metrics is None:
                return None
            total_cost += metrics[3]

        if intensify:
            improved_routes, total_cost = self._improve_route_set(working_routes, demand_by_stop)
            return self._build_route_plan(improved_routes, total_cost)
        return self._build_route_plan(working_routes, total_cost)

    def _route_state_objective(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        intensify: bool = False,
    ) -> Optional[
        Tuple[
            float,
            RoutePlan,
            Dict[int, Tuple[int, ...]],
            Tuple[int, ...],
            Set[int],
        ]
    ]:
        working_routes = self._copy_routes(routes)
        normalized_groups = {
            parking: tuple(sequence)
            for parking, sequence in parking_groups.items()
            if sequence
        }
        parking_stops = set(normalized_groups)
        route_parking_stops: Set[int] = set()
        seen_stops: Set[int] = set()
        served_customers: Set[int] = set()
        direct_customers: List[int] = []
        demand_by_stop: Dict[int, float] = {}
        robot_cost = 0.0

        for parking, sequence in normalized_groups.items():
            if parking not in self.context.parking:
                return None
            metrics = self._robot_path_metrics(parking, sequence)
            if metrics is None:
                return None
            demand_by_stop[parking] = metrics[0]
            robot_cost += metrics[2]

        for truck in self.context.trucks:
            for stop in working_routes[truck]:
                if stop in seen_stops:
                    return None
                seen_stops.add(stop)
                if stop in parking_stops:
                    route_parking_stops.add(stop)
                    for customer in normalized_groups[stop]:
                        if customer not in self.context.customers or customer in served_customers:
                            return None
                        served_customers.add(customer)
                    continue
                if stop not in self.context.regular_customers or stop in served_customers:
                    return None
                served_customers.add(stop)
                direct_customers.append(stop)
                demand_by_stop[stop] = self.context.q[stop]

        if route_parking_stops != parking_stops:
            return None

        route_plan = self._route_plan_from_existing_routes(working_routes, demand_by_stop, intensify=intensify)
        if route_plan is None:
            return None

        return (
            route_plan.route_cost + robot_cost,
            route_plan,
            normalized_groups,
            tuple(sorted(direct_customers)),
            served_customers,
        )

    def _evaluate_route_pattern_candidate(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        runtime_so_far: float,
        intensify_routes: bool = True,
    ) -> Optional[HeuristicPattern]:
        evaluation = self._route_state_objective(routes, parking_groups, intensify=intensify_routes)
        if evaluation is None:
            return None
        _, route_plan, normalized_groups, direct_customers, served_customers = evaluation
        if served_customers != set(self.context.customers):
            return None

        truck_solutions: Dict[int, TruckMasterSolution] = {}
        list_groups = {parking: list(sequence) for parking, sequence in normalized_groups.items()}
        for truck in self.context.trucks:
            solution = self._build_truck_solution(truck, route_plan.routes[truck], list_groups)
            truck_solutions[truck] = solution
            if solution.active:
                self.validation_calls += 1
                check = self.subproblems[truck].check_feasibility(solution)
                if not check.feasible:
                    return None

        result = self._compose_result(GRB.SUBOPTIMAL, runtime_so_far, truck_solutions)
        if result.objective is None:
            return None
        return HeuristicPattern(
            direct_customers=direct_customers,
            parking_groups={parking: tuple(normalized_groups[parking]) for parking in sorted(normalized_groups)},
            truck_routes=dict(route_plan.routes),
            stop_to_truck=dict(route_plan.stop_to_truck),
            result=result,
        )

    def _remove_stop_from_routes(
        self,
        routes: Dict[int, Sequence[int]],
        stop: int,
    ) -> Tuple[Dict[int, List[int]], bool]:
        updated_routes = self._copy_routes(routes)
        for truck in self.context.trucks:
            route = updated_routes[truck]
            if stop in route:
                route.remove(stop)
                return updated_routes, True
        return updated_routes, False

    def _remove_route_component(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        stop: int,
    ) -> Tuple[Dict[int, List[int]], Dict[int, List[int]], List[int]]:
        updated_routes, removed = self._remove_stop_from_routes(routes, stop)
        updated_groups = self._copy_groups(parking_groups)
        if not removed:
            return updated_routes, updated_groups, []
        if stop in updated_groups:
            removed_customers = list(updated_groups.pop(stop))
        elif stop in self.context.regular_customers:
            removed_customers = [stop]
        else:
            removed_customers = []
        return updated_routes, updated_groups, removed_customers

    def _stop_relatedness(
        self,
        seed_stop: int,
        candidate_stop: int,
        parking_groups: Dict[int, Sequence[int]],
        stop_to_truck: Dict[int, int],
        demand_by_stop: Dict[int, float],
    ) -> Tuple[int, int, float, float, int]:
        return (
            0 if stop_to_truck.get(seed_stop) == stop_to_truck.get(candidate_stop) else 1,
            0 if ((seed_stop in parking_groups) == (candidate_stop in parking_groups)) else 1,
            self.context.veh_cost_matrix[seed_stop][candidate_stop]
            + self.context.veh_cost_matrix[candidate_stop][seed_stop],
            abs(demand_by_stop.get(seed_stop, 0.0) - demand_by_stop.get(candidate_stop, 0.0)),
            candidate_stop,
        )

    def _route_stop_marginal_cost(
        self,
        stop: int,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
    ) -> float:
        current_state = self._route_state_objective(routes, parking_groups, intensify=False)
        if current_state is None:
            return 0.0
        reduced_routes, reduced_groups, _ = self._remove_route_component(routes, parking_groups, stop)
        reduced_state = self._route_state_objective(reduced_routes, reduced_groups, intensify=False)
        if reduced_state is None:
            return 0.0
        return current_state[0] - reduced_state[0]

    def _destroy_route_state(
        self,
        pattern: HeuristicPattern,
        iteration: int,
        rng: random.Random,
        operator: str,
        destroy_scale: float = 1.0,
    ) -> Tuple[Dict[int, List[int]], Dict[int, List[int]], Set[int]]:
        working_routes = self._copy_routes(pattern.truck_routes)
        working_groups = self._copy_groups(pattern.parking_groups)
        removed_customers: Set[int] = set()
        removed_stops: Set[int] = set()
        all_stops = [
            stop
            for truck in self.context.trucks
            for stop in pattern.truck_routes.get(truck, ())
        ]
        if not all_stops:
            return working_routes, working_groups, removed_customers

        demand_by_stop = self._route_demand_by_stop(pattern.truck_routes, pattern.parking_groups)
        stop_to_truck = dict(pattern.stop_to_truck)
        base_target = min(
            len(self.context.customers),
            max(2, len(self.context.customers) // 5),
        )
        scaled_target = max(base_target, int(round(base_target * max(1.0, destroy_scale))))
        customer_target = min(len(self.context.customers), scaled_target)
        customer_target = min(
            len(self.context.customers),
            max(customer_target, 2 + iteration % 3),
        )
        if operator == "large_escape":
            customer_target = min(
                len(self.context.customers),
                max(customer_target + 1, len(self.context.customers) // 3),
            )
        elif iteration % 9 == 4:
            customer_target = min(
                len(self.context.customers),
                max(customer_target + 1, len(self.context.customers) // 3),
            )

        def apply_stop(stop: int) -> None:
            nonlocal working_routes, working_groups
            if stop in removed_stops:
                return
            next_routes, next_groups, removed_now = self._remove_route_component(working_routes, working_groups, stop)
            if not removed_now:
                return
            working_routes = next_routes
            working_groups = next_groups
            removed_stops.add(stop)
            removed_customers.update(removed_now)

        if operator == "segment":
            non_empty_trucks = [truck for truck in self.context.trucks if pattern.truck_routes.get(truck)]
            if non_empty_trucks:
                target_truck = max(
                    non_empty_trucks,
                    key=lambda truck: (
                        len(pattern.truck_routes[truck]),
                        sum(demand_by_stop.get(stop, 0.0) for stop in pattern.truck_routes[truck]),
                        -truck,
                    ),
                )
                route = list(pattern.truck_routes[target_truck])
                segment_len = min(len(route), max(1, min(3, customer_target)))
                start = 0 if len(route) == segment_len else rng.randrange(len(route) - segment_len + 1)
                for stop in route[start : start + segment_len]:
                    apply_stop(stop)
        elif operator == "truck_cluster":
            truck_scores = []
            for truck in self.context.trucks:
                route = tuple(pattern.truck_routes.get(truck, ()))
                if not route:
                    continue
                metrics = self._route_is_feasible(route, demand_by_stop)
                route_cost = float("inf") if metrics is None else metrics[3] / max(1, len(route))
                load = sum(demand_by_stop.get(stop, 0.0) for stop in route)
                truck_scores.append((route_cost, load, len(route), truck))
            if truck_scores:
                target_truck = max(truck_scores)[-1]
                route = list(pattern.truck_routes[target_truck])
                seed_stop = max(
                    route,
                    key=lambda stop: (
                        self._route_stop_marginal_cost(stop, pattern.truck_routes, pattern.parking_groups),
                        demand_by_stop.get(stop, 0.0),
                        stop,
                    ),
                )
                apply_stop(seed_stop)
                idx = route.index(seed_stop)
                neighborhood = [
                    route[pos]
                    for pos in range(max(0, idx - 2), min(len(route), idx + 3))
                    if route[pos] != seed_stop
                ]
                neighborhood.sort(
                    key=lambda stop: self._stop_relatedness(
                        seed_stop,
                        stop,
                        pattern.parking_groups,
                        stop_to_truck,
                        demand_by_stop,
                    )
                )
                for stop in neighborhood:
                    if len(removed_customers) >= customer_target:
                        break
                    apply_stop(stop)
                ranked = sorted(
                    (stop for stop in route if stop != seed_stop),
                    key=lambda stop: (
                        self._route_stop_marginal_cost(stop, pattern.truck_routes, pattern.parking_groups),
                        demand_by_stop.get(stop, 0.0),
                        stop,
                    ),
                    reverse=True,
                )
                for stop in ranked:
                    if len(removed_customers) >= customer_target:
                        break
                    apply_stop(stop)
        elif operator == "related":
            seed_stop = rng.choice(all_stops)
            apply_stop(seed_stop)
            ranked = sorted(
                (stop for stop in all_stops if stop != seed_stop),
                key=lambda stop: self._stop_relatedness(
                    seed_stop,
                    stop,
                    pattern.parking_groups,
                    stop_to_truck,
                    demand_by_stop,
                ),
            )
            for stop in ranked:
                if len(removed_customers) >= customer_target:
                    break
                apply_stop(stop)
        elif operator == "parking" and pattern.parking_groups:
            ranked_parkings = sorted(
                pattern.parking_groups,
                key=lambda parking: (
                    self._group_proxy_value(parking, pattern.parking_groups[parking]),
                    len(pattern.parking_groups[parking]),
                    self._parking_open_proxy(parking),
                ),
            )
            seed_stop = rng.choice(ranked_parkings[: min(3, len(ranked_parkings))])
            apply_stop(seed_stop)
            truck = stop_to_truck.get(seed_stop)
            if truck is not None:
                route = list(pattern.truck_routes.get(truck, ()))
                if seed_stop in route:
                    idx = route.index(seed_stop)
                    neighborhood = [
                        route[pos]
                        for pos in range(max(0, idx - 2), min(len(route), idx + 3))
                        if route[pos] != seed_stop
                    ]
                    neighborhood.sort(
                        key=lambda stop: self._stop_relatedness(
                            seed_stop,
                            stop,
                            pattern.parking_groups,
                            stop_to_truck,
                            demand_by_stop,
                        )
                    )
                    for stop in neighborhood:
                        if len(removed_customers) >= customer_target:
                            break
                        apply_stop(stop)
        elif operator == "costly":
            ranked = sorted(
                all_stops,
                key=lambda stop: (
                    self._route_stop_marginal_cost(stop, pattern.truck_routes, pattern.parking_groups),
                    demand_by_stop.get(stop, 0.0),
                    stop,
                ),
                reverse=True,
            )
            for stop in ranked:
                if len(removed_customers) >= customer_target:
                    break
                apply_stop(stop)
        elif operator == "large_escape":
            ranked_seeds = sorted(
                all_stops,
                key=lambda stop: (
                    self._route_stop_marginal_cost(stop, pattern.truck_routes, pattern.parking_groups),
                    stop in pattern.parking_groups,
                    demand_by_stop.get(stop, 0.0),
                    stop,
                ),
                reverse=True,
            )
            seed_stops = ranked_seeds[: min(2, len(ranked_seeds))]
            for seed_stop in seed_stops:
                apply_stop(seed_stop)
                related = sorted(
                    (stop for stop in all_stops if stop != seed_stop and stop not in removed_stops),
                    key=lambda stop: self._stop_relatedness(
                        seed_stop,
                        stop,
                        pattern.parking_groups,
                        stop_to_truck,
                        demand_by_stop,
                    ),
                )
                for stop in related:
                    if len(removed_customers) >= customer_target:
                        break
                    apply_stop(stop)
                if len(removed_customers) >= customer_target:
                    break
        else:
            shuffled_stops = list(all_stops)
            rng.shuffle(shuffled_stops)
            for stop in shuffled_stops:
                if len(removed_customers) >= customer_target:
                    break
                apply_stop(stop)

        if not removed_customers:
            apply_stop(rng.choice(all_stops))

        if len(removed_customers) < customer_target:
            remaining_stops = [stop for stop in all_stops if stop not in removed_stops]
            if removed_stops:
                seed_stop = next(iter(removed_stops))
                remaining_stops.sort(
                    key=lambda stop: self._stop_relatedness(
                        seed_stop,
                        stop,
                        pattern.parking_groups,
                        stop_to_truck,
                        demand_by_stop,
                    )
                )
            else:
                rng.shuffle(remaining_stops)
            for stop in remaining_stops:
                if len(removed_customers) >= customer_target:
                    break
                apply_stop(stop)

        return working_routes, working_groups, removed_customers

    def _route_move_score(
        self,
        objective: float,
        base_objective: float,
        inserted_customers: Sequence[int],
        description: str,
        repair_mode: str,
    ) -> float:
        delta = objective - base_objective
        inserted_count = max(1, len(inserted_customers))
        score = delta / inserted_count
        if repair_mode == "robot_bias":
            if description.startswith("robot-existing"):
                score *= 0.92
            elif description.startswith("robot-open"):
                score *= 0.96
            else:
                score *= 1.06
        elif repair_mode == "direct_bias":
            if description.startswith("direct"):
                score *= 0.93
            else:
                score *= 1.05
        elif repair_mode == "regret" and inserted_count > 1:
            score *= max(0.82, 1.0 - 0.08 * (inserted_count - 1))
        if description.startswith("robot-open") and repair_mode != "robot_bias":
            score *= 1.02
        return score

    def _score_route_repair_move(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        inserted_customers: Set[int],
        description: str,
        base_objective: float,
        repair_mode: str,
    ) -> Optional[RouteRepairMove]:
        state = self._route_state_objective(routes, parking_groups, intensify=False)
        if state is None:
            return None
        objective, route_plan, normalized_groups, _, _ = state
        return RouteRepairMove(
            objective=objective,
            score=self._route_move_score(
                objective,
                base_objective,
                tuple(sorted(inserted_customers)),
                description,
                repair_mode,
            ),
            routes=dict(route_plan.routes),
            parking_groups=normalized_groups,
            inserted_customers=tuple(sorted(inserted_customers)),
            description=description,
        )

    def _rebuild_robot_group_sequence(
        self,
        parking: int,
        customers: Sequence[int],
        reference_order: Optional[Sequence[int]] = None,
    ) -> Optional[List[int]]:
        unique_customers = tuple(dict.fromkeys(customers))
        if not unique_customers:
            return []

        seed_order = sorted(
            unique_customers,
            key=lambda customer: (
                self._truck_roundtrip_cost(customer),
                self.context.q[customer],
                -len(self.robot_candidates[customer]),
                customer,
            ),
            reverse=True,
        )
        fallback_order = sorted(
            unique_customers,
            key=lambda customer: (
                len(self.robot_candidates[customer]),
                -self._truck_roundtrip_cost(customer),
                -self.context.q[customer],
                customer,
            ),
        )
        orderings = []
        if reference_order is not None:
            base = [customer for customer in reference_order if customer in unique_customers]
            missing = [customer for customer in unique_customers if customer not in base]
            orderings.append(tuple(base + missing))
            if len(base) > 1:
                orderings.append(tuple(reversed(base)) + tuple(missing))
        orderings.append(tuple(seed_order))
        orderings.append(tuple(fallback_order))
        orderings.append(tuple(sorted(unique_customers)))

        best_sequence: Optional[List[int]] = None
        best_score = None
        seen_orders = set()
        for ordering in orderings:
            if ordering in seen_orders:
                continue
            seen_orders.add(ordering)
            sequence: List[int] = []
            feasible = True
            for customer in ordering:
                candidate = self._best_robot_insertion(parking, sequence, customer)
                if candidate is None:
                    feasible = False
                    break
                sequence = list(candidate[0])
            if not feasible:
                continue
            metrics = self._robot_path_metrics(parking, sequence)
            if metrics is None:
                continue
            score = (metrics[2], metrics[1], len(sequence))
            if best_score is None or score < best_score:
                best_score = score
                best_sequence = sequence

        return best_sequence

    def _best_route_positions_for_stop(
        self,
        stop: int,
        stop_demand: float,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        top_k: int = 6,
    ) -> List[Tuple[float, int, int, int, List[int]]]:
        working_routes = self._copy_routes(routes)
        working_groups = self._copy_groups(parking_groups)
        current_demand = self._route_demand_by_stop(working_routes, working_groups)
        base_metrics_by_truck: Dict[int, Tuple[float, float, float, float]] = {}
        for truck in self.context.trucks:
            metrics = self._route_is_feasible(working_routes[truck], current_demand)
            if metrics is None:
                return []
            base_metrics_by_truck[truck] = metrics

        candidate_demand = dict(current_demand)
        candidate_demand[stop] = stop_demand
        positions: List[Tuple[float, int, int, int, List[int]]] = []
        for truck in self.context.trucks:
            route = working_routes[truck]
            base_cost = base_metrics_by_truck[truck][3]
            for pos in range(len(route) + 1):
                candidate_route = route[:pos] + [stop] + route[pos:]
                metrics = self._route_is_feasible(candidate_route, candidate_demand)
                if metrics is None:
                    continue
                positions.append((metrics[3] - base_cost, len(route), truck, pos, candidate_route))

        positions.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
        return positions[:top_k]

    def _intensify_route_state(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
    ) -> Optional[Tuple[Dict[int, Tuple[int, ...]], Dict[int, Tuple[int, ...]], float]]:
        working_routes = self._copy_routes(routes)
        working_groups = self._copy_groups(parking_groups)
        state = self._route_state_objective(working_routes, working_groups, intensify=False)
        if state is None or state[4] != set(self.context.customers):
            return None
        current_objective = state[0]

        def consider_candidate(
            candidate_routes: Dict[int, Sequence[int]],
            candidate_groups: Dict[int, Sequence[int]],
            incumbent_best: Optional[Tuple[float, Dict[int, List[int]], Dict[int, List[int]]]],
        ) -> Optional[Tuple[float, Dict[int, List[int]], Dict[int, List[int]]]]:
            candidate_state = self._route_state_objective(candidate_routes, candidate_groups, intensify=False)
            if candidate_state is None or candidate_state[4] != set(self.context.customers):
                return incumbent_best
            if candidate_state[0] + 1e-9 >= current_objective:
                return incumbent_best
            candidate_routes_norm = {
                truck: list(candidate_state[1].routes[truck])
                for truck in self.context.trucks
            }
            candidate_groups_norm = {
                parking: list(sequence)
                for parking, sequence in candidate_state[2].items()
            }
            if incumbent_best is None or candidate_state[0] + 1e-9 < incumbent_best[0]:
                return (candidate_state[0], candidate_routes_norm, candidate_groups_norm)
            return incumbent_best

        for _ in range(2):
            best_state: Optional[Tuple[float, Dict[int, List[int]], Dict[int, List[int]]]] = None
            current_direct = sorted(
                (
                    customer
                    for customer in state[3]
                    if customer in self.context.regular_customers and self.robot_candidates[customer]
                ),
                key=lambda customer: (
                    self._truck_roundtrip_cost(customer),
                    self.context.q[customer],
                    -len(self.robot_candidates[customer]),
                    customer,
                ),
                reverse=True,
            )[:6]
            for customer in current_direct:
                reduced_routes, reduced_groups, removed_now = self._remove_route_component(working_routes, working_groups, customer)
                if not removed_now:
                    continue
                for parking in self.robot_candidates[customer][:4]:
                    if parking in reduced_groups:
                        candidate = self._best_robot_insertion(parking, reduced_groups[parking], customer)
                        if candidate is None:
                            continue
                        groups_copy = self._copy_groups(reduced_groups)
                        groups_copy[parking] = list(candidate[0])
                        best_state = consider_candidate(reduced_routes, groups_copy, best_state)
                        continue
                    positions = self._best_route_positions_for_stop(
                        parking,
                        self.context.q[customer],
                        reduced_routes,
                        reduced_groups,
                        top_k=4,
                    )
                    for _, _, truck, _, candidate_route in positions:
                        routes_copy = self._copy_routes(reduced_routes)
                        routes_copy[truck] = candidate_route
                        groups_copy = self._copy_groups(reduced_groups)
                        groups_copy[parking] = [customer]
                        best_state = consider_candidate(routes_copy, groups_copy, best_state)

            robot_regular = sorted(
                (
                    (0 if len(sequence) == 1 else 1, len(self.robot_candidates[customer]), -self._truck_roundtrip_cost(customer), -self.context.q[customer], parking, customer)
                    for parking, sequence in working_groups.items()
                    for customer in sequence
                    if customer in self.context.regular_customers
                ),
                key=lambda item: item,
            )[:8]
            for _, _, _, _, parking, customer in robot_regular:
                routes_copy = self._copy_routes(working_routes)
                groups_copy = self._copy_groups(working_groups)
                remaining_customers = [node for node in groups_copy[parking] if node != customer]
                if remaining_customers:
                    rebuilt_sequence = self._rebuild_robot_group_sequence(parking, remaining_customers, groups_copy[parking])
                    if rebuilt_sequence is None:
                        continue
                    groups_copy[parking] = rebuilt_sequence
                else:
                    groups_copy.pop(parking, None)
                    routes_copy, removed = self._remove_stop_from_routes(routes_copy, parking)
                    if not removed:
                        continue
                positions = self._best_route_positions_for_stop(
                    customer,
                    self.context.q[customer],
                    routes_copy,
                    groups_copy,
                    top_k=4,
                )
                for _, _, truck, _, candidate_route in positions:
                    direct_routes = self._copy_routes(routes_copy)
                    direct_routes[truck] = candidate_route
                    best_state = consider_candidate(direct_routes, groups_copy, best_state)

            parking_candidates = sorted(
                (
                    parking
                    for parking, sequence in working_groups.items()
                    if sequence and any(
                        len(self.robot_candidates[customer]) >= 2
                        for customer in sequence
                    )
                ),
                key=lambda parking: (
                    self._group_proxy_value(parking, working_groups[parking]),
                    len(working_groups[parking]),
                    self._parking_open_proxy(parking),
                ),
            )[:4]
            for parking in parking_candidates:
                reduced_routes, removed = self._remove_stop_from_routes(working_routes, parking)
                if not removed:
                    continue
                reduced_groups = self._copy_groups(working_groups)
                sequence = list(reduced_groups.pop(parking))
                feasible_parkings = [
                    candidate_parking
                    for candidate_parking in self.context.parking
                    if candidate_parking != parking
                    and candidate_parking not in reduced_groups
                    and all(candidate_parking in self.robot_candidates[customer] for customer in sequence)
                ]
                feasible_parkings.sort(key=lambda node: self._parking_open_proxy(node))
                for candidate_parking in feasible_parkings[:4]:
                    rebuilt_sequence = self._rebuild_robot_group_sequence(candidate_parking, sequence, sequence)
                    if rebuilt_sequence is None:
                        continue
                    positions = self._best_route_positions_for_stop(
                        candidate_parking,
                        sum(self.context.q[node] for node in rebuilt_sequence),
                        reduced_routes,
                        reduced_groups,
                        top_k=4,
                    )
                    for _, _, truck, _, candidate_route in positions:
                        routes_copy = self._copy_routes(reduced_routes)
                        routes_copy[truck] = candidate_route
                        groups_copy = self._copy_groups(reduced_groups)
                        groups_copy[candidate_parking] = rebuilt_sequence
                        best_state = consider_candidate(routes_copy, groups_copy, best_state)

            if best_state is None:
                break
            current_objective, working_routes, working_groups = best_state
            state = self._route_state_objective(working_routes, working_groups, intensify=False)
            if state is None:
                return None

        final_state = self._route_state_objective(working_routes, working_groups, intensify=False)
        if final_state is None or final_state[4] != set(self.context.customers):
            return None
        return dict(final_state[1].routes), final_state[2], final_state[0]

    def _generate_route_repair_moves(
        self,
        customer: int,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        remaining_customers: Set[int],
        repair_mode: str,
        base_objective: float,
    ) -> List[RouteRepairMove]:
        moves: List[RouteRepairMove] = []
        seen_signatures = set()
        working_routes = self._copy_routes(routes)
        working_groups = self._copy_groups(parking_groups)
        current_demand = self._route_demand_by_stop(working_routes, working_groups)
        base_metrics_by_truck: Dict[int, Tuple[float, float, float, float]] = {}
        for truck in self.context.trucks:
            metrics = self._route_is_feasible(working_routes[truck], current_demand)
            if metrics is None:
                return []
            base_metrics_by_truck[truck] = metrics

        favor_robot = {
            "greedy": 1.0,
            "regret": 1.08,
            "robot_bias": 1.18,
            "direct_bias": 0.96,
        }.get(repair_mode, 1.02)

        def push_move(move: Optional[RouteRepairMove]) -> None:
            if move is None:
                return
            signature = self._route_pattern_signature(move.routes, move.parking_groups)
            if signature in seen_signatures:
                return
            seen_signatures.add(signature)
            moves.append(move)

        if customer in self.context.regular_customers:
            direct_positions = []
            candidate_demand = dict(current_demand)
            candidate_demand[customer] = self.context.q[customer]
            for truck in self.context.trucks:
                route = working_routes[truck]
                base_cost = base_metrics_by_truck[truck][3]
                for pos in range(len(route) + 1):
                    candidate_route = route[:pos] + [customer] + route[pos:]
                    metrics = self._route_is_feasible(candidate_route, candidate_demand)
                    if metrics is None:
                        continue
                    direct_positions.append((metrics[3] - base_cost, len(route), truck, pos, candidate_route))
            direct_positions.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
            for _, _, truck, pos, candidate_route in direct_positions[:6]:
                routes_copy = self._copy_routes(working_routes)
                routes_copy[truck] = candidate_route
                push_move(
                    self._score_route_repair_move(
                        routes_copy,
                        working_groups,
                        {customer},
                        f"direct:{truck}:{pos}",
                        base_objective,
                        repair_mode,
                    )
                )

        for parking in self.robot_candidates[customer]:
            if parking not in working_groups:
                continue
            candidate = self._best_robot_insertion(parking, working_groups[parking], customer)
            if candidate is None:
                continue
            augmented_sequence, extra_inserted, _ = self._augment_parking_group(
                parking,
                candidate[0],
                set(),
                set(remaining_customers) - {customer},
                favor_robot,
            )
            groups_copy = self._copy_groups(working_groups)
            groups_copy[parking] = augmented_sequence
            push_move(
                self._score_route_repair_move(
                    working_routes,
                    groups_copy,
                    {customer} | extra_inserted,
                    f"robot-existing:{parking}",
                    base_objective,
                    repair_mode,
                )
            )

        open_positions = []
        for parking in self.robot_candidates[customer]:
            if parking in working_groups:
                continue
            augmented_sequence, extra_inserted, _ = self._augment_parking_group(
                parking,
                [customer],
                set(),
                set(remaining_customers) - {customer},
                favor_robot,
            )
            candidate_demand = dict(current_demand)
            candidate_demand[parking] = sum(self.context.q[node] for node in augmented_sequence)
            parking_penalty = 0.5 * self._parking_open_proxy(parking)
            inserted_customers = tuple(sorted({customer} | extra_inserted))
            for truck in self.context.trucks:
                route = working_routes[truck]
                base_cost = base_metrics_by_truck[truck][3]
                for pos in range(len(route) + 1):
                    candidate_route = route[:pos] + [parking] + route[pos:]
                    metrics = self._route_is_feasible(candidate_route, candidate_demand)
                    if metrics is None:
                        continue
                    open_positions.append(
                        (
                            metrics[3] - base_cost + parking_penalty,
                            truck,
                            pos,
                            parking,
                            tuple(augmented_sequence),
                            inserted_customers,
                            candidate_route,
                        )
                    )
        open_positions.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
        for _, truck, pos, parking, sequence, inserted_customers, candidate_route in open_positions[:6]:
            routes_copy = self._copy_routes(working_routes)
            routes_copy[truck] = candidate_route
            groups_copy = self._copy_groups(working_groups)
            groups_copy[parking] = list(sequence)
            push_move(
                self._score_route_repair_move(
                    routes_copy,
                    groups_copy,
                    set(inserted_customers),
                    f"robot-open:{parking}:{truck}:{pos}",
                    base_objective,
                    repair_mode,
                )
            )

        moves.sort(key=lambda move: (move.score, move.objective, -len(move.inserted_customers), move.description))
        return moves[:10]

    def _repair_route_state(
        self,
        routes: Dict[int, Sequence[int]],
        parking_groups: Dict[int, Sequence[int]],
        removed_customers: Set[int],
        repair_mode: str,
        rng: random.Random,
    ) -> Optional[Tuple[Dict[int, Tuple[int, ...]], Dict[int, Tuple[int, ...]], float]]:
        working_routes = self._copy_routes(routes)
        working_groups = self._copy_groups(parking_groups)
        remaining_customers = set(removed_customers)

        while remaining_customers:
            base_state = self._route_state_objective(working_routes, working_groups, intensify=False)
            if base_state is None:
                return None
            base_objective = base_state[0]
            choice_bundle = []
            ordered_customers = list(remaining_customers)
            rng.shuffle(ordered_customers)
            ordered_customers.sort(
                key=lambda customer: (
                    0 if customer in self.context.robot_only_customers else 1,
                    len(self.robot_candidates[customer]),
                    -self._truck_roundtrip_cost(customer),
                    -self.context.q[customer],
                )
            )

            for customer in ordered_customers:
                customer_moves = self._generate_route_repair_moves(
                    customer,
                    working_routes,
                    working_groups,
                    remaining_customers,
                    repair_mode,
                    base_objective,
                )
                if not customer_moves:
                    return None
                best_move = customer_moves[0]
                second_score = (
                    customer_moves[1].score
                    if len(customer_moves) >= 2
                    else best_move.score + max(1.0, 0.05 * abs(best_move.score) + 1.0)
                )
                regret = second_score - best_move.score
                if repair_mode == "greedy":
                    priority = (
                        0 if customer in self.context.robot_only_customers else 1,
                        best_move.score,
                        best_move.objective,
                        len(customer_moves),
                        -len(best_move.inserted_customers),
                        customer,
                    )
                else:
                    priority = (
                        0 if customer in self.context.robot_only_customers else 1,
                        len(customer_moves),
                        -regret,
                        best_move.score,
                        best_move.objective,
                        -len(best_move.inserted_customers),
                        customer,
                    )
                choice_bundle.append((priority, best_move))

            if not choice_bundle:
                return None

            _, selected_move = min(choice_bundle, key=lambda item: item[0])
            working_routes = self._copy_routes(selected_move.routes)
            working_groups = self._copy_groups(selected_move.parking_groups)
            remaining_customers.difference_update(selected_move.inserted_customers)

        return self._intensify_route_state(working_routes, working_groups)

    def _select_weighted_operator(
        self,
        weights: Dict[str, float],
        rng: random.Random,
    ) -> str:
        total = sum(max(0.05, weight) for weight in weights.values())
        threshold = rng.random() * total
        cumulative = 0.0
        for name, weight in weights.items():
            cumulative += max(0.05, weight)
            if cumulative >= threshold:
                return name
        return next(iter(weights))

    def _update_operator_weight(
        self,
        weights: Dict[str, float],
        operator: str,
        reward: float,
        reaction: float = 0.18,
    ) -> None:
        current = weights.get(operator, 1.0)
        weights[operator] = max(0.05, (1.0 - reaction) * current + reaction * reward)

    def _destroy_pattern(
        self,
        pattern: HeuristicPattern,
        iteration: int,
        rng: random.Random,
    ) -> Tuple[Set[int], Dict[int, List[int]], Set[int]]:
        direct_customers = set(pattern.direct_customers)
        parking_groups = {parking: list(sequence) for parking, sequence in pattern.parking_groups.items()}
        removed: Set[int] = set()
        all_customers = list(self.context.customers)
        destroy_size = min(len(all_customers), max(3, len(all_customers) // 5))
        customer_to_parking = self._customer_to_parking(parking_groups)
        demand_by_stop = self._pattern_demand_by_stop(direct_customers, parking_groups)
        mode = iteration % 6

        if mode == 0 and all_customers:
            seed = rng.choice(all_customers)
            removed.add(seed)
            removed.update(self._select_related_customers(seed, all_customers, destroy_size - 1))
        elif mode == 1 and parking_groups:
            ranked_parkings = sorted(
                parking_groups,
                key=lambda parking: (
                    self._group_proxy_value(parking, parking_groups[parking]),
                    len(parking_groups[parking]),
                    self._parking_open_proxy(parking),
                ),
            )
            target_parking = rng.choice(ranked_parkings[: min(2, len(ranked_parkings))])
            removed.update(parking_groups[target_parking])
            related_direct = sorted(
                (
                    customer
                    for customer in self.context.regular_customers
                    if customer not in removed and target_parking in self.robot_candidates[customer]
                ),
                key=lambda customer: (
                    len(self.robot_candidates[customer]),
                    self._truck_roundtrip_cost(customer),
                    self.context.q[customer],
                ),
                reverse=True,
            )
            removed.update(related_direct[: max(0, destroy_size - len(removed))])
        elif mode == 2:
            mode_candidates = [
                customer
                for customer in self.context.regular_customers
                if customer in direct_customers or customer in customer_to_parking
            ]
            ranked = sorted(
                mode_candidates,
                key=lambda customer: (
                    customer in direct_customers,
                    len(self.robot_candidates[customer]),
                    self.context.q[customer],
                    self._truck_roundtrip_cost(customer),
                ),
                reverse=True,
            )
            removed.update(ranked[:destroy_size])
        elif mode == 3 and pattern.truck_routes:
            truck_metrics = []
            for truck, route in pattern.truck_routes.items():
                load = sum(demand_by_stop.get(stop, 0.0) for stop in route)
                metrics = self._route_is_feasible(route, demand_by_stop)
                route_cost = float("inf") if metrics is None else metrics[3]
                truck_metrics.append((route_cost, load, len(route), truck))
            if truck_metrics:
                target_truck = max(truck_metrics)[-1]
                ranked_stops = sorted(
                    pattern.truck_routes.get(target_truck, ()),
                    key=lambda stop: (
                        demand_by_stop.get(stop, 0.0),
                        self._stop_service_proxy(stop),
                        stop,
                    ),
                    reverse=True,
                )
                for stop in ranked_stops:
                    removed.update(self._customers_from_stop(stop, direct_customers, parking_groups))
                    if len(removed) >= destroy_size:
                        break
        elif mode == 4:
            multi_candidate = [
                customer
                for customer in self.context.customers
                if len(self.robot_candidates[customer]) >= 2
            ]
            if multi_candidate:
                seed = rng.choice(multi_candidate)
                removed.add(seed)
                removed.update(self._select_related_customers(seed, multi_candidate, destroy_size - 1))
        else:
            random_size = min(len(all_customers), max(destroy_size + 1, len(all_customers) // 4))
            removed.update(rng.sample(all_customers, random_size))

        if not removed and all_customers:
            removed.add(rng.choice(all_customers))
        if len(removed) == 1 and len(all_customers) > 1:
            seed = next(iter(removed))
            removed.update(self._select_related_customers(seed, all_customers, 1))

        direct_seed, parking_seed = self._remove_customers_from_pattern(direct_customers, parking_groups, removed)
        return direct_seed, parking_seed, removed

    def _repair_pattern(
        self,
        direct_customers: Set[int],
        parking_groups: Dict[int, List[int]],
        removed_customers: Set[int],
        favor_robot: float,
        rng: random.Random,
    ) -> Optional[Tuple[Tuple[int, ...], Dict[int, Tuple[int, ...]]]]:
        remaining_customers = set(removed_customers)
        working_direct = set(direct_customers)
        working_groups = {parking: list(sequence) for parking, sequence in parking_groups.items() if sequence}

        while remaining_customers:
            choice_bundle = []
            ordered_customers = list(remaining_customers)
            rng.shuffle(ordered_customers)
            ordered_customers.sort(
                key=lambda customer: (
                    0 if customer in self.context.robot_only_customers else 1,
                    len(self.robot_candidates[customer]),
                    -self._truck_roundtrip_cost(customer),
                    -self.context.q[customer],
                )
            )

            for customer in ordered_customers:
                customer_moves = self._generate_repair_moves(
                    customer,
                    working_direct,
                    working_groups,
                    remaining_customers,
                    favor_robot,
                )
                if not customer_moves:
                    return None
                best_move = customer_moves[0]
                second_objective = (
                    customer_moves[1].objective
                    if len(customer_moves) >= 2
                    else best_move.objective + max(10.0, 0.01 * best_move.objective)
                )
                regret = second_objective - best_move.objective
                priority = (
                    0 if customer in self.context.robot_only_customers else 1,
                    len(customer_moves),
                    -regret,
                    best_move.objective,
                    -len(best_move.inserted_customers),
                    customer,
                )
                choice_bundle.append((priority, best_move))

            if not choice_bundle:
                return None

            _, selected_move = min(choice_bundle, key=lambda item: item[0])
            working_direct = set(selected_move.direct_customers)
            working_groups = {
                parking: list(sequence)
                for parking, sequence in selected_move.parking_groups.items()
                if sequence
            }
            remaining_customers.difference_update(selected_move.inserted_customers)

        intensified = self._intensify_pattern(working_direct, working_groups)
        if intensified is not None:
            return intensified

        cleaned_groups = {parking: tuple(sequence) for parking, sequence in working_groups.items() if sequence}
        return tuple(sorted(working_direct)), cleaned_groups

    def _build_lns_termination_config(self, lns_time_limit: float) -> LNSTerminationConfig:
        customer_count = max(1, len(self.context.customers))
        progress_window = max(30, 4 * customer_count)
        best_stall_time = max(60.0, min(300.0, 0.25 * lns_time_limit))
        accept_stall_time = max(25.0, min(150.0, 0.12 * lns_time_limit))
        warmup_time = min(best_stall_time * 0.5, max(15.0, 0.05 * lns_time_limit))
        return LNSTerminationConfig(
            warmup_iterations=max(20, 3 * customer_count),
            warmup_time=warmup_time,
            best_stall_iterations=max(progress_window, 6 * customer_count),
            best_stall_time=best_stall_time,
            accept_stall_iterations=max(progress_window // 2, 3 * customer_count),
            accept_stall_time=accept_stall_time,
            progress_window=progress_window,
            min_progress_in_window=max(1, progress_window // 20),
        )

    def _should_stop_lns(
        self,
        config: LNSTerminationConfig,
        iteration: int,
        phase_elapsed: float,
        iterations_since_best: int,
        iterations_since_accept: int,
        time_since_best: float,
        time_since_accept: float,
        recent_progress: Sequence[int],
    ) -> bool:
        if iteration < config.warmup_iterations or phase_elapsed < config.warmup_time:
            return False
        if len(recent_progress) < config.progress_window:
            return False
        return (
            iterations_since_best >= config.best_stall_iterations
            and time_since_best >= config.best_stall_time
            and iterations_since_accept >= config.accept_stall_iterations
            and time_since_accept >= config.accept_stall_time
            and sum(recent_progress) <= config.min_progress_in_window
        )

    def _select_seed_patterns(
        self,
        patterns: Sequence[HeuristicPattern],
        max_count: int,
    ) -> List[HeuristicPattern]:
        unique_patterns: Dict[
            Tuple[Tuple[Tuple[int, Tuple[int, ...]], ...], Tuple[Tuple[int, Tuple[int, ...]], ...]],
            HeuristicPattern,
        ] = {}
        for pattern in patterns:
            signature = self._route_pattern_signature(pattern.truck_routes, pattern.parking_groups)
            incumbent = unique_patterns.get(signature)
            if incumbent is None or pattern.result.objective + 1e-9 < incumbent.result.objective:
                unique_patterns[signature] = pattern

        ranked = sorted(unique_patterns.values(), key=lambda pattern: pattern.result.objective)
        if len(ranked) <= max_count:
            return ranked

        selected = [ranked[0]]
        remaining = ranked[1:]

        def diversity_score(candidate: HeuristicPattern, chosen: Sequence[HeuristicPattern]) -> int:
            candidate_direct = set(candidate.direct_customers)
            candidate_parking = set(candidate.parking_groups)
            return min(
                len(candidate_direct.symmetric_difference(set(pattern.direct_customers)))
                + 2 * len(candidate_parking.symmetric_difference(set(pattern.parking_groups)))
                for pattern in chosen
            )

        while remaining and len(selected) < max_count:
            best_idx = max(
                range(len(remaining)),
                key=lambda idx: (
                    diversity_score(remaining[idx], selected),
                    -remaining[idx].result.objective,
                ),
            )
            selected.append(remaining.pop(best_idx))

        selected.sort(key=lambda pattern: pattern.result.objective)
        return selected

    def _run_lns(
        self,
        initial_pattern: HeuristicPattern,
        solve_start_time: float,
        phase_time_limit: float,
        incumbent_pattern: Optional[HeuristicPattern] = None,
        random_seed: Optional[int] = None,
    ) -> HeuristicPattern:
        best_pattern = initial_pattern
        if incumbent_pattern is not None and incumbent_pattern.result.objective + 1e-9 < best_pattern.result.objective:
            best_pattern = incumbent_pattern
        if phase_time_limit <= 0.0:
            return best_pattern

        rng = random.Random((1009 + self.context.depot) if random_seed is None else random_seed)
        termination_config = self._build_lns_termination_config(max(phase_time_limit, 1.0))
        lns_start_time = time.perf_counter()
        last_best_time = lns_start_time
        last_accept_time = lns_start_time
        iterations_since_best = 0
        iterations_since_accept = 0
        recent_progress = deque(maxlen=termination_config.progress_window)
        seen_patterns = {
            self._route_pattern_signature(initial_pattern.truck_routes, initial_pattern.parking_groups),
            self._route_pattern_signature(best_pattern.truck_routes, best_pattern.parking_groups),
        }
        current_pattern = initial_pattern
        destroy_weights = {
            "random_stop": 1.0,
            "segment": 1.0,
            "truck_cluster": 1.0,
            "related": 1.0,
            "parking": 1.0,
            "costly": 1.0,
            "large_escape": 1.0,
        }
        repair_weights = {
            "regret": 1.0,
            "greedy": 1.0,
            "robot_bias": 1.0,
            "direct_bias": 1.0,
        }
        iteration = 0

        def record_iteration(progress: bool, improved: bool, event_time: float) -> None:
            nonlocal last_best_time, last_accept_time, iterations_since_best, iterations_since_accept
            recent_progress.append(1 if progress else 0)
            if improved:
                iterations_since_best = 0
                last_best_time = event_time
            else:
                iterations_since_best += 1
            if progress:
                iterations_since_accept = 0
                last_accept_time = event_time
            else:
                iterations_since_accept += 1

        while time.perf_counter() - lns_start_time < phase_time_limit:
            if iteration > 0 and iteration % 10 == 0 and (iterations_since_best <= 8 or rng.random() < 0.55):
                current_pattern = best_pattern

            destroy_scale = 1.0 + min(1.5, 0.08 * max(0, iterations_since_best - 4))
            if iteration % 11 == 5:
                destroy_scale += 0.25
            destroy_scale = min(2.5, destroy_scale)
            destroy_operator = self._select_weighted_operator(destroy_weights, rng)
            if iterations_since_best >= 10 and rng.random() < 0.35:
                destroy_operator = "large_escape"
                destroy_scale = max(destroy_scale, 1.8)
            repair_operator = self._select_weighted_operator(repair_weights, rng)
            routes_seed, parking_seed, removed_customers = self._destroy_route_state(
                current_pattern,
                iteration,
                rng,
                destroy_operator,
                destroy_scale,
            )
            repaired = self._repair_route_state(
                routes_seed,
                parking_seed,
                removed_customers,
                repair_operator,
                rng,
            )
            iteration += 1

            if repaired is None:
                self._update_operator_weight(destroy_weights, destroy_operator, 0.05)
                self._update_operator_weight(repair_weights, repair_operator, 0.05)
                event_time = time.perf_counter()
                record_iteration(False, False, event_time)
                if self._should_stop_lns(
                    termination_config,
                    iteration,
                    event_time - lns_start_time,
                    iterations_since_best,
                    iterations_since_accept,
                    event_time - last_best_time,
                    event_time - last_accept_time,
                    recent_progress,
                ):
                    break
                continue

            routes_candidate, parking_candidate, proxy_objective = repaired
            signature = self._route_pattern_signature(routes_candidate, parking_candidate)
            if signature in seen_patterns:
                self._update_operator_weight(destroy_weights, destroy_operator, 0.12)
                self._update_operator_weight(repair_weights, repair_operator, 0.12)
                event_time = time.perf_counter()
                record_iteration(False, False, event_time)
                if self._should_stop_lns(
                    termination_config,
                    iteration,
                    event_time - lns_start_time,
                    iterations_since_best,
                    iterations_since_accept,
                    event_time - last_best_time,
                    event_time - last_accept_time,
                    recent_progress,
                ):
                    break
                continue
            seen_patterns.add(signature)

            proxy_relax = min(0.06, 0.0025 * max(0, iterations_since_best - 6))
            proxy_limit = current_pattern.result.objective * ((1.04 if iteration <= 12 else 1.025) + proxy_relax)
            if proxy_objective > proxy_limit + 1e-9 and rng.random() > 0.05:
                self._update_operator_weight(destroy_weights, destroy_operator, 0.2)
                self._update_operator_weight(repair_weights, repair_operator, 0.2)
                event_time = time.perf_counter()
                record_iteration(False, False, event_time)
                if self._should_stop_lns(
                    termination_config,
                    iteration,
                    event_time - lns_start_time,
                    iterations_since_best,
                    iterations_since_accept,
                    event_time - last_best_time,
                    event_time - last_accept_time,
                    recent_progress,
                ):
                    break
                continue

            candidate = self._evaluate_route_pattern_candidate(
                routes_candidate,
                parking_candidate,
                time.perf_counter() - solve_start_time,
                intensify_routes=False,
            )
            if candidate is None:
                self._update_operator_weight(destroy_weights, destroy_operator, 0.08)
                self._update_operator_weight(repair_weights, repair_operator, 0.08)
                event_time = time.perf_counter()
                record_iteration(False, False, event_time)
                if self._should_stop_lns(
                    termination_config,
                    iteration,
                    event_time - lns_start_time,
                    iterations_since_best,
                    iterations_since_accept,
                    event_time - last_best_time,
                    event_time - last_accept_time,
                    recent_progress,
                ):
                    break
                continue

            current_objective = current_pattern.result.objective
            should_intensify = (
                candidate.result.objective <= best_pattern.result.objective * 1.01 + 1e-9
                or candidate.result.objective <= current_objective * 1.003 + 1e-9
                or iteration % 7 == 0
            )
            if should_intensify:
                intensified = self._evaluate_route_pattern_candidate(
                    candidate.truck_routes,
                    candidate.parking_groups,
                    time.perf_counter() - solve_start_time,
                    intensify_routes=True,
                )
                if intensified is not None and intensified.result.objective + 1e-9 < candidate.result.objective:
                    candidate = intensified
                    seen_patterns.add(
                        self._route_pattern_signature(candidate.truck_routes, candidate.parking_groups)
                    )

            if candidate.result.objective + 1e-9 < best_pattern.result.objective:
                best_pattern = candidate
                current_pattern = candidate
                self._update_operator_weight(destroy_weights, destroy_operator, 6.0)
                self._update_operator_weight(repair_weights, repair_operator, 6.0)
                record_iteration(True, True, time.perf_counter())
                continue

            accepted = False
            reward = 0.35
            accept_relax = 0.01 + min(0.035, 0.002 * max(0, iterations_since_best - 5))
            accept_prob = min(0.18, 0.04 + 0.01 * max(0, iterations_since_best - 8))
            if candidate.result.objective + 1e-9 < current_objective:
                current_pattern = candidate
                accepted = True
                reward = 3.0
            elif candidate.result.objective <= current_objective * (1.0 + accept_relax) + 1e-9 or rng.random() < accept_prob:
                current_pattern = candidate
                accepted = True
                reward = 0.9 if candidate.result.objective > current_objective + 1e-9 else 1.1

            self._update_operator_weight(destroy_weights, destroy_operator, reward)
            self._update_operator_weight(repair_weights, repair_operator, reward)
            event_time = time.perf_counter()
            record_iteration(accepted, False, event_time)
            if self._should_stop_lns(
                termination_config,
                iteration,
                event_time - lns_start_time,
                iterations_since_best,
                iterations_since_accept,
                event_time - last_best_time,
                event_time - last_accept_time,
                recent_progress,
            ):
                break

        return best_pattern

    def solve(self, lns_time_limit: Optional[float] = None) -> DepotSolveResult:
        start = time.perf_counter()
        try:
            modes = [
                (0, 1.0),
                (max(1, len(self.context.regular_customers) // 5), 1.1),
                (max(2, len(self.context.regular_customers) // 3), 1.3),
                (max(3, len(self.context.regular_customers) // 2), 1.6),
                (len(self.context.regular_customers), 1.0),
            ]
            best_pattern: Optional[HeuristicPattern] = None
            seed_patterns: List[HeuristicPattern] = []

            for optional_limit, threshold_factor in modes:
                groups_data = self._build_parking_groups(optional_limit, threshold_factor)
                if groups_data is None:
                    continue
                parking_groups, _ = groups_data
                served_by_robot = {node for sequence in parking_groups.values() for node in sequence}
                direct_customers = [
                    customer
                    for customer in self.context.customers
                    if customer not in self.context.robot_only_customers and customer not in served_by_robot
                ]
                intensified = self._intensify_pattern(
                    set(direct_customers),
                    {parking: list(sequence) for parking, sequence in parking_groups.items()},
                )
                if intensified is not None:
                    direct_candidate, parking_candidate = intensified
                else:
                    direct_candidate = tuple(sorted(direct_customers))
                    parking_candidate = {
                        parking: tuple(sequence)
                        for parking, sequence in parking_groups.items()
                        if sequence
                    }
                candidate = self._evaluate_pattern_candidate(
                    direct_candidate,
                    parking_candidate,
                    time.perf_counter() - start,
                )
                if candidate is None:
                    continue
                seed_patterns.append(candidate)
                if best_pattern is None or candidate.result.objective < best_pattern.result.objective:
                    best_pattern = candidate

            all_direct_seed = self._intensify_pattern(set(self.context.customers), {})
            if all_direct_seed is not None:
                all_direct_candidate = self._evaluate_pattern_candidate(
                    all_direct_seed[0],
                    all_direct_seed[1],
                    time.perf_counter() - start,
                )
            else:
                all_direct_candidate = self._evaluate_pattern_candidate(
                    list(self.context.customers),
                    {},
                    time.perf_counter() - start,
                )
            if all_direct_candidate is not None:
                seed_patterns.append(all_direct_candidate)
                if best_pattern is None or all_direct_candidate.result.objective < best_pattern.result.objective:
                    best_pattern = all_direct_candidate

            if lns_time_limit is not None and lns_time_limit > 0.0 and len(self.context.customers) > 10:
                extra_seed_trials = 24 if best_pattern is None else 12
                for extra_candidate in self._generate_randomized_seed_patterns(start, extra_seed_trials):
                    seed_patterns.append(extra_candidate)
                    if best_pattern is None or extra_candidate.result.objective < best_pattern.result.objective:
                        best_pattern = extra_candidate

                bootstrap_trials = 18 if best_pattern is None else 8
                for bootstrap_candidate in self._generate_route_bootstrap_seeds(start, bootstrap_trials):
                    seed_patterns.append(bootstrap_candidate)
                    if best_pattern is None or bootstrap_candidate.result.objective < best_pattern.result.objective:
                        best_pattern = bootstrap_candidate

            runtime = time.perf_counter() - start
            if best_pattern is None:
                status = self._basic_infeasibility_status()
                if status is None:
                    status = GRB.TIME_LIMIT
                return self._compose_result(status, runtime, None)

            if lns_time_limit is not None and lns_time_limit > 0.0:
                available_lns_time = max(0.0, lns_time_limit - (time.perf_counter() - start))
                if available_lns_time > 1e-9:
                    if available_lns_time >= 45.0:
                        seed_count = 3
                        seed_weights = [1.0, 0.75, 0.5]
                    elif available_lns_time >= 20.0:
                        seed_count = 2
                        seed_weights = [1.0, 0.6]
                    else:
                        seed_count = 1
                        seed_weights = [1.0]
                    selected_seeds = self._select_seed_patterns(seed_patterns or [best_pattern], seed_count)
                    if not selected_seeds:
                        selected_seeds = [best_pattern]
                    phase_patterns = list(selected_seeds)
                    if available_lns_time >= 45.0:
                        while len(phase_patterns) < 3:
                            phase_patterns.append(best_pattern)
                    elif available_lns_time >= 20.0 and len(phase_patterns) < 2:
                        phase_patterns.append(best_pattern)
                    base_weights = [1.0, 0.75, 0.5, 0.35]
                    seed_weights = base_weights[: len(phase_patterns)]
                    lns_phase_start = time.perf_counter()
                    incumbent = best_pattern
                    for idx, seed in enumerate(phase_patterns):
                        elapsed_lns = time.perf_counter() - lns_phase_start
                        remaining = available_lns_time - elapsed_lns
                        if remaining <= 1e-9:
                            break
                        remaining_weight = sum(seed_weights[idx:])
                        if idx == len(phase_patterns) - 1 or remaining_weight <= 0.0:
                            phase_budget = remaining
                        else:
                            phase_budget = max(6.0, remaining * seed_weights[idx] / remaining_weight)
                            phase_budget = min(remaining, phase_budget)
                        incumbent = self._run_lns(
                            seed,
                            start,
                            phase_budget,
                            incumbent_pattern=incumbent,
                            random_seed=1009 + self.context.depot + 997 * idx,
                        )
                    best_pattern = incumbent

            best_pattern.result.runtime = time.perf_counter() - start
            return best_pattern.result
        finally:
            self.dispose()


def _solve_single_depot_heuristic(
    context: DepotContext,
    lns_time_limit: Optional[float],
) -> DepotSolveResult:
    solver = SingleDepotConstructionHeuristic(context)
    return solver.solve(lns_time_limit=lns_time_limit)


def _solve_depot_contexts_heuristic(
    contexts: Sequence[DepotContext],
    parallel_depots: bool,
    max_parallel_depots: Optional[int],
    lns_time_limit: Optional[float],
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
        return [_solve_single_depot_heuristic(context, lns_time_limit) for context in contexts]

    worker_cap = max_parallel_depots if max_parallel_depots is not None else (os.cpu_count() or 1)
    max_workers = max(1, min(len(contexts), worker_cap))
    if max_workers <= 1:
        return [_solve_single_depot_heuristic(context, lns_time_limit) for context in contexts]

    results: List[DepotSolveResult] = []
    with ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="HeuristicDepot") as executor:
        futures = {
            executor.submit(_solve_single_depot_heuristic, context, lns_time_limit): context.depot
            for context in contexts
        }
        for future in as_completed(futures):
            results.append(future.result())

    return sorted(results, key=lambda result: result.depot)


def _polish_single_depot_result(
    context: DepotContext,
    incumbent: DepotSolveResult,
    polish_time_limit: float,
    feasibility_cut_mode: str,
) -> DepotSolveResult:
    if incumbent.objective is None or polish_time_limit <= 0.0:
        return incumbent

    solver = SingleDepotLBBD(context, feasibility_cut_mode=feasibility_cut_mode, solve_mode="exact")
    solver.master.setParam("TimeLimit", polish_time_limit)
    solver.master.setParam("MIPFocus", 1)
    solver.master.setParam("Heuristics", 0.6)
    solver.master.setParam("RINS", 25)
    solver.master.setParam("NoRelHeurTime", min(20.0, polish_time_limit))
    solver.apply_external_start(incumbent)
    polished = solver.solve()

    total_runtime = incumbent.runtime + polished.runtime
    if polished.objective is not None and polished.objective + 1e-9 < incumbent.objective:
        polished.runtime = total_runtime
        return polished

    incumbent.runtime = total_runtime
    return incumbent


def _polish_results(
    contexts: Sequence[DepotContext],
    base_results: Sequence[DepotSolveResult],
    parallel_depots: bool,
    max_parallel_depots: Optional[int],
    polish_time_limit: float,
    feasibility_cut_mode: str,
) -> List[DepotSolveResult]:
    context_by_depot = {context.depot: context for context in contexts}
    incumbents = sorted(base_results, key=lambda result: result.depot)

    if not parallel_depots or len(incumbents) <= 1:
        return [
            _polish_single_depot_result(
                context_by_depot[result.depot],
                result,
                polish_time_limit,
                feasibility_cut_mode,
            )
            for result in incumbents
        ]

    worker_cap = max_parallel_depots if max_parallel_depots is not None else (os.cpu_count() or 1)
    max_workers = max(1, min(len(incumbents), worker_cap))
    if max_workers <= 1:
        return [
            _polish_single_depot_result(
                context_by_depot[result.depot],
                result,
                polish_time_limit,
                feasibility_cut_mode,
            )
            for result in incumbents
        ]

    polished_results: List[DepotSolveResult] = []
    with ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="PolishDepot") as executor:
        futures = {
            executor.submit(
                _polish_single_depot_result,
                context_by_depot[result.depot],
                result,
                polish_time_limit,
                feasibility_cut_mode,
            ): result.depot
            for result in incumbents
        }
        for future in as_completed(futures):
            polished_results.append(future.result())

    return sorted(polished_results, key=lambda result: result.depot)


def _print_heuristic_summary(results: Sequence[DepotSolveResult]) -> None:
    objective_values = [result.objective for result in results]
    has_total_objective = all(value is not None for value in objective_values)
    total_objective = sum(value for value in objective_values if value is not None)

    for result in results:
        print(
            f"Depot {result.depot}: status={result.status_name}, "
            f"obj={_format_float(result.objective)}, time={result.runtime:.2f}s, "
            f"validation_calls={result.subproblem_calls}"
        )

    if has_total_objective:
        print(f"Total Objective: {_format_float(total_objective)}")
    else:
        print(f"Total Objective (available depots only): {_format_float(total_objective)}")


def heuristicSolver(
    myInstance,
    parallel_depots: bool = True,
    max_parallel_depots: Optional[int] = None,
    lns_time_limit: Optional[float] = None,
    polish_time_limit: Optional[float] = None,
    polish_feasibility_cut_mode: str = "exact",
    plot_solution: bool = True,
):
    contexts = _build_depot_contexts(myInstance)
    results = _solve_depot_contexts_heuristic(contexts, parallel_depots, max_parallel_depots, lns_time_limit)
    if polish_time_limit is not None and polish_time_limit > 0.0:
        results = _polish_results(
            contexts,
            results,
            parallel_depots,
            max_parallel_depots,
            polish_time_limit,
            polish_feasibility_cut_mode,
        )
    _print_heuristic_summary(results)

    has_incumbent_for_all = all(result.objective is not None for result in results)
    if has_incumbent_for_all and plot_solution:
        _plot_solution(myInstance, results)

    total_objective = sum(result.objective for result in results if result.objective is not None)
    overall_status = (
        GRB.INFEASIBLE
        if any(result.status == GRB.INFEASIBLE for result in results)
        else GRB.SUBOPTIMAL
        if has_incumbent_for_all
        else GRB.TIME_LIMIT
    )

    return {
        "status": overall_status,
        "status_name": _status_name(overall_status),
        "objective": total_objective if has_incumbent_for_all else None,
        "depot_results": results,
    }
