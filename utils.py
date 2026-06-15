import json
import os
from instance_generation.instance import instance
import matplotlib.pyplot as plt


def transpose_dict(dictionary):
    transposed_dict = {}
    for key, value in dictionary.items():
        if isinstance(value, list):
            for item in value:
                if item not in transposed_dict:
                    transposed_dict[item] = key
                else:
                    transposed_dict[item].append(key)
        else:
            if value not in transposed_dict:
                transposed_dict[value] = key
            else:
                transposed_dict[value].append(key)
    return transposed_dict


def Remove(A: list, B):
    # from A remove B
    if isinstance(B, list):
        return set(A) - set(B)
    else:
        return set(A) - {B}


def doubleRemove(A: list, B: list, C: int):
    return set(A) - set(B) - {C}


def Intersection(A, B):
    C = []
    for i in A:
        if i in B:
            C.append(i)
    return C


def instance_save(folder_path, data_instance):
    # Convert the class instance to a dictionary
    data_dict = {
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
        "robot_unit_consumption": data_instance.robot_unit_consumption
    }

    os.makedirs(folder_path, exist_ok=True)
    filename = os.path.join(folder_path, data_dict["name"] + ".json")
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(json.dumps(data_dict, indent=2))


def is_file_in_folder(filename, folder_path):
    """
    判断文件是否存在于指定文件夹中。

    参数:
        filename (str): 要检查的文件名。
        folder_path (str): 指定文件夹的路径。

    返回:
        bool: 如果文件存在于指定文件夹中，则返回True，否则返回False。
    """
    # 使用os.path.join()构建完整路径
    file_path = os.path.join(folder_path, filename)

    # 使用os.path.exists()检查文件是否存在
    return os.path.exists(file_path)


def read_json_from_file(filename, folder_path):
    """
    从指定文件夹中读取JSON数据。

    参数:
        filename (str): 要读取的文件名。
        folder_path (str): 指定文件夹的路径。

    返回:
        dict: 如果成功读取JSON数据，则返回解析后的字典对象，否则返回None。
    """
    # 使用os.path.join()构建完整路径
    file_path = os.path.join(folder_path, filename)

    with open(file_path, 'r') as file:
        # 使用json.load()读取JSON数据
        data = json.load(file)
    return data


def dict_to_data_instance(data_dict):
    """
    将字典转换为数据实例。

    参数:
        data_dict (dict): 包含属性值的字典。

    返回:
        DataInstance: 数据实例对象。
    """
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
    """
    解析字符串并提取有用的信息。

    参数:
        input_string (str): 输入的字符串。

    返回:
        dict: 包含提取的信息的字典。
    """
    info_dict = {}
    parts = input_string.split('-')

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
            info_dict["parking_point_num"] = int(part[1:len(part) - 5])

    return info_dict


def _dict_modified(aDict):
    new_dict = {}
    for key, value in aDict.items():
        new_dict[int(key)] = value
    return new_dict


def instancePlot(myInstance):
    V = myInstance.V
    V_coordinates = myInstance.V_coordinates
    depot_with_node = myInstance.depot_with_node
    VD = myInstance.VD
    VC = myInstance.VC
    VP = myInstance.VP
    point_x = [point[0] for point in V_coordinates]
    point_y = [point[1] for point in V_coordinates]
    colors = ['blue', 'red', 'green', 'black', 'purple', 'orange']
    for d in VD:
        nodes = depot_with_node[d]
        for i in nodes:
            if i in VD:
                plt.scatter(point_x[i],
                            point_y[i],
                            color=colors[d],
                            label='Depot Points',
                            marker='o')
            if i in VC:
                plt.scatter(point_x[i],
                            point_y[i],
                            color=colors[d],
                            label='Customer Points',
                            marker='^')
            if i in VP:
                plt.scatter(point_x[i],
                            point_y[i],
                            color=colors[d],
                            label='Parking Points',
                            marker='p')
    for index, pos in enumerate(V_coordinates):
        plt.annotate(index,
                     pos,
                     textcoords="offset points",
                     xytext=(0, 10),
                     ha='center')
    plt.show()
