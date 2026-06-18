"""Instance filename parsing and JSON serialization helpers."""

import json
import os

from .instance import instance


def instance_to_dict(data_instance):
    return {
        "name": data_instance.name,
        "V_coordinates": data_instance.V_coordinates,
        "V": data_instance.V,
        "VD": data_instance.VD,
        "VC": data_instance.VC,
        "VP": data_instance.VP,
        "VC1": data_instance.VC1,
        "VC2": data_instance.VC2,
        "demand": data_instance.demand,
        "service_time": data_instance.service_time,
        "K": data_instance.K,
        "veh_dist_matrix": data_instance.veh_dist_matrix,
        "rob_power_matrix": data_instance.rob_power_matrix,
        "vehicle_cost_matrix": data_instance.vehicle_cost_matrix,
        "robot_cost_matrix": data_instance.robot_cost_matrix,
        "vehicle_time_matrix": data_instance.vehicle_time_matrix,
        "robot_time_matrix": data_instance.robot_time_matrix,
        "cus_with_depot": data_instance.cus_with_depot,
        "veh_with_depot": data_instance.veh_with_depot,
        "veh_depot_dict": data_instance.veh_depot_dict,
        "depot_with_node": data_instance.depot_with_node,
        "VC2_with_VP": data_instance.VC2_with_VP,
        "max_travel_time": data_instance.max_travel_time,
        "cf": data_instance.cf,
        "vehicle_speed": data_instance.vehicle_speed,
        "robot_speed": data_instance.robot_speed,
        "vehicle_cost": data_instance.vehicle_cost,
        "robot_cost": data_instance.robot_cost,
        "very_big": data_instance.very_big,
        "veh_cap": data_instance.veh_cap,
        "rob_cap": data_instance.rob_cap,
        "rob_weight": data_instance.rob_weight,
        "operation_time": data_instance.operation_time,
        "veh_dist_ub": data_instance.veh_dist_ub,
        "robot_ub": data_instance.robot_ub,
        "robot_unit_consumption": data_instance.robot_unit_consumption,
    }


def instance_save(folder_path, data_instance):
    os.makedirs(folder_path, exist_ok=True)
    filename = os.path.join(folder_path, data_instance.name + ".json")
    with open(filename, "w", encoding="utf-8") as file:
        json.dump(instance_to_dict(data_instance), file, indent=2)


def is_file_in_folder(filename, folder_path):
    return os.path.exists(os.path.join(folder_path, filename))


def read_json_from_file(filename, folder_path):
    file_path = os.path.join(folder_path, filename)
    with open(file_path, "r", encoding="utf-8") as file:
        return json.load(file)


def dict_to_data_instance(data_dict):
    data_instance = instance()
    data_instance.name = data_dict.get("name")
    data_instance.V_coordinates = data_dict.get("V_coordinates")
    data_instance.V = data_dict.get("V")
    data_instance.VD = data_dict.get("VD")
    data_instance.VC = data_dict.get("VC")
    data_instance.VP = data_dict.get("VP")
    data_instance.VC1 = data_dict.get("VC1")
    data_instance.VC2 = data_dict.get("VC2")
    data_instance.demand = data_dict.get("demand")
    data_instance.service_time = data_dict.get("service_time")
    data_instance.K = data_dict.get("K")
    data_instance.veh_dist_matrix = data_dict.get("veh_dist_matrix")
    data_instance.rob_power_matrix = data_dict.get("rob_power_matrix")
    data_instance.vehicle_cost_matrix = data_dict.get("vehicle_cost_matrix")
    data_instance.robot_cost_matrix = data_dict.get("robot_cost_matrix")
    data_instance.vehicle_time_matrix = data_dict.get("vehicle_time_matrix")
    data_instance.robot_time_matrix = data_dict.get("robot_time_matrix")
    data_instance.cus_with_depot = _dict_modified(data_dict.get("cus_with_depot"))
    data_instance.veh_with_depot = _dict_modified(data_dict.get("veh_with_depot"))
    data_instance.veh_depot_dict = _dict_modified(data_dict.get("veh_depot_dict"))
    data_instance.depot_with_node = _dict_modified(data_dict.get("depot_with_node"))
    data_instance.VC2_with_VP = _dict_modified(data_dict.get("VC2_with_VP"))
    data_instance.max_travel_time = data_dict.get("max_travel_time")
    data_instance.cf = data_dict.get("cf")
    data_instance.vehicle_speed = data_dict.get("vehicle_speed")
    data_instance.robot_speed = data_dict.get("robot_speed")
    data_instance.vehicle_cost = data_dict.get("vehicle_cost")
    data_instance.robot_cost = data_dict.get("robot_cost")
    data_instance.very_big = data_dict.get("very_big")
    data_instance.veh_cap = data_dict.get("veh_cap")
    data_instance.rob_cap = data_dict.get("rob_cap")
    data_instance.rob_weight = data_dict.get("rob_weight")
    data_instance.operation_time = data_dict.get("operation_time")
    data_instance.veh_dist_ub = data_dict.get("veh_dist_ub")
    data_instance.robot_ub = data_dict.get("robot_ub")
    data_instance.robot_unit_consumption = data_dict.get("robot_unit_consumption")
    return data_instance


def parse_string(input_string):
    info_dict = {}
    parts = input_string.split("-")

    if not input_string.endswith(".json"):
        return info_dict

    info_dict["M"] = parts[0]
    for part in parts[1:]:
        if part.startswith("d"):
            info_dict["depot_num"] = int(part[1:])
        elif part.startswith("n"):
            info_dict["customer_number_each_depot"] = int(part[1:])
        elif part.startswith("k"):
            info_dict["vehicle_number_each_depot"] = int(part[1:])
        elif part.startswith("p"):
            info_dict["parking_point_num"] = int(part[1 : len(part) - 5])

    return info_dict


def _dict_modified(a_dict):
    if a_dict is None:
        return {}
    return {int(key): value for key, value in a_dict.items()}
