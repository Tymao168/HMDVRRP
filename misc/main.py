import argparse
import json
import os
import re
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Dict, List, Optional

venv_path = r"E:\HMDVRRPpython\.venv\Lib\site-packages"
if venv_path in sys.path:
    sys.path.remove(venv_path)
sys.path.insert(0, venv_path)

from instance_generation import generate_instance
from LBBD.nh_lbbd_solver import gurobiSolver
from Heuristic.nh_construction_solver import heuristicSolver
from utils import (
    dict_to_data_instance,
    instance_save,
    is_file_in_folder,
    parse_string,
    read_json_from_file,
)


BASELINE_PATH = os.path.join("reference", "ReferenceValue0.3.json")
DEFAULT_OUTPUT_PATH = os.path.join("reference", "MainMethodResults.json")
DEFAULT_FOLDER_PATH = "instances"
DEFAULT_HEURISTIC_TIME_LIMIT = 3600.0
SMALL_INSTANCE_CUSTOMER_THRESHOLD = 15


def load_or_generate_instance(instance_name: str, folder_path: str = DEFAULT_FOLDER_PATH):
    if is_file_in_folder(instance_name, folder_path):
        data = read_json_from_file(instance_name, folder_path)
        return dict_to_data_instance(data)

    info_dict = parse_string(instance_name)
    my_instance = generate_instance(info_dict)
    instance_save(folder_path, my_instance)
    return my_instance


def instance_filename_to_label(instance_name: str) -> str:
    solver_info = parse_string(instance_name)
    depot_num = solver_info.get("depot_num")
    customer_num = solver_info.get("customer_number_each_depot")
    vehicle_num = solver_info.get("vehicle_number_each_depot")
    if depot_num is None or customer_num is None or vehicle_num is None:
        raise ValueError(f"Cannot map instance filename to baseline label: {instance_name}")
    return f"E{depot_num}-{customer_num}-{vehicle_num}"


def label_to_instance_filename(label: str) -> str:
    match = re.fullmatch(r"E(\d+)-(\d+)-(\d+)", label)
    if match is None:
        raise ValueError(f"Unsupported baseline label: {label}")
    depot_num, customer_num, vehicle_num = match.groups()
    return f"M-d{depot_num}-n{customer_num}-k{vehicle_num}-p2.json"


def should_use_heuristic(instance_name: str) -> bool:
    solver_info = parse_string(instance_name)
    return solver_info.get("customer_number_each_depot", 0) > SMALL_INSTANCE_CUSTOMER_THRESHOLD


def format_optional_float(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.2f}"


def compute_relative_change_pct(current_obj: Optional[float], baseline_obj: Optional[float]) -> str:
    if current_obj is None or baseline_obj in (None, 0):
        return "-"
    return f"{((current_obj - baseline_obj) / baseline_obj) * 100:+.2f}%"


def build_output_record(
    instance_label: str,
    instance_name: str,
    baseline_obj: Optional[float],
    solve_result: Dict,
) -> Dict:
    status_name = solve_result["status_name"]
    objective = solve_result["objective"]
    runtime = solve_result["runtime_s"]

    return {
        "Instance": instance_label,
        "Instance_file": instance_name,
        "Method": solve_result["method"],
        "Status": status_name,
        "Baseline_obj": round(baseline_obj, 2) if baseline_obj is not None else "-",
        "Total_obj": round(objective, 2) if objective is not None else "-",
        "Obj_change_pct": compute_relative_change_pct(objective, baseline_obj),
        "Time_s": round(runtime, 2),
    }


def solve_instance(
    instance_name: str,
    folder_path: str = DEFAULT_FOLDER_PATH,
    heuristic_time_limit: float = DEFAULT_HEURISTIC_TIME_LIMIT,
    max_parallel_depots: Optional[int] = None,
    plot_solution: bool = False,
) -> Dict:
    start_time = time.perf_counter()
    method = "Heuristic" if should_use_heuristic(instance_name) else "LBBD"

    try:
        my_instance = load_or_generate_instance(instance_name, folder_path)

        if method == "LBBD":
            solver_result = gurobiSolver(
                my_instance,
                parallel_depots=False,
                feasibility_cut_mode="exact",
                plot_solution=plot_solution,
            )
        else:
            parallel_depots = max_parallel_depots is not None and max_parallel_depots > 1
            solver_result = heuristicSolver(
                my_instance,
                parallel_depots=parallel_depots,
                max_parallel_depots=max_parallel_depots,
                lns_time_limit=heuristic_time_limit,
                plot_solution=plot_solution,
            )

        return {
            "instance_name": instance_name,
            "method": method,
            "status_name": solver_result.get("status_name", "UNKNOWN"),
            "objective": solver_result.get("objective"),
            "runtime_s": time.perf_counter() - start_time,
            "error": None,
        }
    except Exception as exc:
        return {
            "instance_name": instance_name,
            "method": method,
            "status_name": "ERROR",
            "objective": None,
            "runtime_s": time.perf_counter() - start_time,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }


def solve_large_instance_worker(task: Dict) -> Dict:
    return solve_instance(
        instance_name=task["instance_name"],
        folder_path=task["folder_path"],
        heuristic_time_limit=task["heuristic_time_limit"],
        max_parallel_depots=task["max_parallel_depots"],
        plot_solution=task["plot_solution"],
    )


def load_baseline_entries(baseline_path: str) -> List[Dict]:
    with open(baseline_path, "r", encoding="utf-8") as baseline_file:
        return json.load(baseline_file)


def choose_large_worker_count(large_instance_count: int, requested_workers: Optional[int]) -> int:
    if large_instance_count <= 0:
        return 0
    if requested_workers is not None:
        return max(1, min(large_instance_count, requested_workers))

    cpu_count = os.cpu_count() or 1
    auto_workers = max(1, cpu_count // 2)
    return min(large_instance_count, auto_workers)


def solve_from_baseline(
    baseline_path: str = BASELINE_PATH,
    output_path: str = DEFAULT_OUTPUT_PATH,
    folder_path: str = DEFAULT_FOLDER_PATH,
    heuristic_time_limit: float = DEFAULT_HEURISTIC_TIME_LIMIT,
    large_workers: Optional[int] = None,
    plot_solution: bool = False,
    limit: Optional[int] = None,
) -> List[Dict]:
    baseline_entries = load_baseline_entries(baseline_path)
    if limit is not None:
        baseline_entries = baseline_entries[:limit]

    jobs = []
    for entry in baseline_entries:
        instance_label = entry["Instance"]
        baseline_obj = entry.get("Best_obj")
        instance_name = label_to_instance_filename(instance_label)
        jobs.append(
            {
                "Instance": instance_label,
                "Baseline_obj": baseline_obj,
                "Instance_file": instance_name,
                "Is_large": should_use_heuristic(instance_name),
            }
        )

    small_jobs = [job for job in jobs if not job["Is_large"]]
    large_jobs = [job for job in jobs if job["Is_large"]]

    results_by_label: Dict[str, Dict] = {}

    print(f"Small instances (sequential): {len(small_jobs)}")
    for job in small_jobs:
        print(f"Solving {job['Instance']} with LBBD sequentially...")
        solve_result = solve_instance(
            instance_name=job["Instance_file"],
            folder_path=folder_path,
            heuristic_time_limit=heuristic_time_limit,
            max_parallel_depots=None,
            plot_solution=plot_solution,
        )
        if solve_result.get("error"):
            print(f"  ERROR: {solve_result['error']}")
        results_by_label[job["Instance"]] = build_output_record(
            instance_label=job["Instance"],
            instance_name=job["Instance_file"],
            baseline_obj=job["Baseline_obj"],
            solve_result=solve_result,
        )

    large_worker_count = choose_large_worker_count(len(large_jobs), large_workers)
    if large_jobs:
        cpu_count = os.cpu_count() or 1
        per_instance_parallel_depots = max(1, cpu_count // max(1, large_worker_count))
        print(
            f"Large instances (parallel): {len(large_jobs)}, "
            f"workers={large_worker_count}, max_parallel_depots={per_instance_parallel_depots}"
        )
        with ProcessPoolExecutor(max_workers=large_worker_count) as executor:
            future_to_job = {
                executor.submit(
                    solve_large_instance_worker,
                    {
                        "instance_name": job["Instance_file"],
                        "folder_path": folder_path,
                        "heuristic_time_limit": heuristic_time_limit,
                        "max_parallel_depots": per_instance_parallel_depots,
                        "plot_solution": plot_solution,
                    },
                ): job
                for job in large_jobs
            }
            for future in as_completed(future_to_job):
                job = future_to_job[future]
                solve_result = future.result()
                if solve_result.get("error"):
                    print(f"  ERROR in {job['Instance']}: {solve_result['error']}")
                else:
                    print(
                        f"Completed {job['Instance']}: "
                        f"status={solve_result['status_name']}, "
                        f"obj={format_optional_float(solve_result['objective'])}, "
                        f"time={solve_result['runtime_s']:.2f}s"
                    )
                results_by_label[job["Instance"]] = build_output_record(
                    instance_label=job["Instance"],
                    instance_name=job["Instance_file"],
                    baseline_obj=job["Baseline_obj"],
                    solve_result=solve_result,
                )

    ordered_results = [results_by_label[entry["Instance"]] for entry in baseline_entries]

    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(ordered_results, output_file, indent=2, ensure_ascii=False)

    print(f"Saved results to {output_path}")
    return ordered_results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch-run HMDVRRP instances against the baseline list.")
    parser.add_argument("--instance", help="Solve a single instance file, for example M-d2-n3-k1-p2.json.")
    parser.add_argument("--baseline", default=BASELINE_PATH, help="Baseline JSON path.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH, help="Output JSON path.")
    parser.add_argument("--folder", default=DEFAULT_FOLDER_PATH, help="Instance folder path.")
    parser.add_argument(
        "--heuristic-time-limit",
        type=float,
        default=DEFAULT_HEURISTIC_TIME_LIMIT,
        help="Heuristic LNS time limit in seconds for large instances.",
    )
    parser.add_argument(
        "--large-workers",
        type=int,
        default=None,
        help="Number of parallel workers for large instances. Defaults to half of logical CPUs.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Only solve the first N baseline instances. Useful for quick checks.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Save figure outputs for solved instances.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.instance:
        solve_result = solve_instance(
            instance_name=args.instance,
            folder_path=args.folder,
            heuristic_time_limit=args.heuristic_time_limit,
            max_parallel_depots=None if not should_use_heuristic(args.instance) else (os.cpu_count() or 1),
            plot_solution=args.plot,
        )
        print(json.dumps(solve_result, indent=2, ensure_ascii=False))
        return

    solve_from_baseline(
        baseline_path=args.baseline,
        output_path=args.output,
        folder_path=args.folder,
        heuristic_time_limit=args.heuristic_time_limit,
        large_workers=args.large_workers,
        plot_solution=args.plot,
        limit=args.limit,
    )


if __name__ == "__main__":
    main()
