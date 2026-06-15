import math
import random as rn
import matplotlib.pyplot as plt
from utils import transpose_dict
from .instance import instance


def __generate_depot_points(num_points, point_range):
    # 生成指定数量的仓库节点
    points = []
    mid_point = (point_range[0] + point_range[1]) / 2
    min_distance = int(mid_point * 0.4)
    max_distance = int(mid_point * 0.8)
    angle = 0

    for _ in range(num_points):
        while True:
            dist = rn.uniform(min_distance, max_distance)
            x = round(dist * math.cos(angle) + mid_point)
            y = round(dist * math.sin(angle) + mid_point)
            point = (x, y)

            if all(distance(point, p) >= min_distance for p in points):
                points.append(point)
                angle += (2 * math.pi) / num_points
                break

    return points


def __generate_customer_points(depot_num, a, point_range, depot_points):
    # 生成客户节点
    points = []
    points = set()
    while len(points) < depot_num * a:
        x = rn.randint(0 + 5, point_range[1] - 1)
        y = rn.randint(0 + 5, point_range[1] - 1)
        if (x, y) not in points and (x, y) not in depot_points:
            points.add((x, y))
    return list(points)


def __select_robot_only_points(first_group, second_group, num_points, range):
    # 从所有的客户节点中随机选取一定数量的仅能有robot访问的车辆
    selected_indices = []
    min_distance = range[1] * 0.15  # 不要距离仓库太近的停车节点
    while len(selected_indices) < num_points:
        candidate_index = rn.randint(0, len(second_group) - 1)
        candidate_point = second_group[candidate_index]
        if all(
                distance(candidate_point, point) >= min_distance for point in
                first_group) and candidate_index not in selected_indices:
            selected_indices.append(candidate_index)
    return selected_indices


def __generate_parking_points(robot_only_point_indices, customer_points,
                              parking_point_num, point_range,
                              parking_point_range,
                              parking_point_distance_range, existed_points):
    # 为robot_only节点生成停车节点

    def is_valid_spacing(points):
        # 检查一个robot_only点周围停车节点的分布情况
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dis = distance(points[i], points[j])
                if dis < parking_point_distance_range[
                    0] or dis > parking_point_distance_range[1]:
                    return False
        return True

    parking_points = []
    for index in robot_only_point_indices:
        point = customer_points[index]
        vaild_points = False
        while not vaild_points:
            tem_parking_points = []
            for _ in range(parking_point_num):
                valid_point = False
                while not valid_point:
                    angle = rn.uniform(0, 2 * math.pi)
                    dist = rn.uniform(parking_point_range[0],
                                      parking_point_range[1])
                    x_offset = dist * math.cos(angle)
                    y_offset = dist * math.sin(angle)
                    new_x = round(point[0] + x_offset)
                    new_y = round(point[1] + y_offset)
                    if point_range[0] < new_x < point_range[1] and point_range[
                        0] < new_y < point_range[1] and (
                            new_x, new_y) not in existed_points:
                        new_point = (new_x, new_y)
                        tem_parking_points.append(new_point)
                        valid_point = True
            # 检查这组停车节点是不是符合空间分布要求
            if is_valid_spacing(tem_parking_points):
                parking_points.extend(tem_parking_points)
                vaild_points = True
    return parking_points


def distance(p1, p2):
    return round(((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5, 2)


def __add_value_to_list(lst, value):
    result = []
    for num in lst:
        result.append(num + value)
    return result


def __determain_normal_customer(listA, listB):
    new_list = []
    for item in listA:
        if item not in listB:
            new_list.append(item)
    return new_list


def __generate_instance_demand_or_service_time(num1: int, num2: int, num3: int,
                                               my_range: list) -> list:
    # 用于生成所有节点上的需求或服务时间
    res = [0] * num1
    for _ in range(num2):
        res.append(rn.randint(my_range[0], my_range[1]))
    res.extend([0] * num3)
    return res


def __calculate_matrix(point_cooradinates: list, depot_list: list,
                       normal_customer_list: list, robot_customer_list: list,
                       park_robot: list, vehicle_speed: float,
                       vehicle_cost: float, robot_speed: float,
                       robot_cost: float, robot_unit_consumption: float):
    num = len(point_cooradinates)
    veh_dist_mat = [[0] * num for _ in range(num)]
    rob_power_mat = [[0] * num for _ in range(num)]
    veh_cost_mat = [[0] * num for _ in range(num)]
    rob_cost_mat = [[0] * num for _ in range(num)]
    veh_time_mat = [[0] * num for _ in range(num)]
    rob_time_mat = [[0] * num for _ in range(num)]
    for i in range(num):
        for j in range(i, num):
            if i == j:
                veh_dist = 1e5
                rob_dist = 1e5
                veh_cost = 1e5
                rob_cost = 1e5
                veh_time = 1e5
                rob_time = 1e5
            elif (i in depot_list):
                if j not in robot_customer_list:
                    veh_dist = distance(point_cooradinates[i], point_cooradinates[j])
                    veh_cost = round(veh_dist * vehicle_cost, 2)
                    veh_time = round(veh_dist / vehicle_speed, 2)
                else:
                    veh_dist = 1e5
                    veh_cost = 1e5
                    veh_time = 1e5
                rob_dist = 1e5
                rob_cost = 1e5
                rob_time = 1e5
            elif (i in robot_customer_list):
                veh_dist = 1e5
                veh_cost = 1e5
                veh_time = 1e5
                if j in depot_list:
                    rob_dist = 1e5
                    rob_cost = 1e5
                    rob_time = 1e5
                else:
                    rob_dist = distance(point_cooradinates[i], point_cooradinates[j])
                    rob_cost = round(rob_dist * robot_cost, 2)
                    rob_time = round(rob_dist / robot_speed, 2)
            else:
                if j not in robot_customer_list:
                    veh_dist = distance(point_cooradinates[i], point_cooradinates[j])
                    veh_cost = round(veh_dist * vehicle_cost, 2)
                    veh_time = round(veh_dist / vehicle_speed, 2)
                else:
                    veh_dist = 1e5
                    veh_cost = 1e5
                    veh_time = 1e5
                rob_dist = distance(point_cooradinates[i], point_cooradinates[j])
                rob_cost = round(rob_dist * robot_cost, 2)
                rob_time = round(rob_dist / robot_speed, 2)

            veh_dist_mat[i][j] = veh_dist
            veh_dist_mat[j][i] = veh_dist
            rob_power_mat[i][j] = rob_dist * robot_unit_consumption
            rob_power_mat[j][i] = rob_dist * robot_unit_consumption
            veh_cost_mat[i][j] = veh_cost
            veh_cost_mat[j][i] = veh_cost
            rob_cost_mat[i][j] = rob_cost
            rob_cost_mat[j][i] = rob_cost
            veh_time_mat[i][j] = veh_time
            veh_time_mat[j][i] = veh_time
            rob_time_mat[i][j] = rob_time
            rob_time_mat[j][i] = rob_time
    return veh_dist_mat, rob_power_mat, veh_cost_mat, rob_cost_mat, veh_time_mat, rob_time_mat


def __vehicle_customer_with_depot(customer_indices: list,
                                  vehicle_indices: list, depot_num: int,
                                  cus_num_each: int, veh_num_each: int):
    # 建立仓库与客户/车辆间的对应关系
    cus_dict = {}
    veh_dict = {}
    for i in range(depot_num):
        cus = customer_indices[i * cus_num_each:(i + 1) * cus_num_each]
        veh = vehicle_indices[i * veh_num_each:(i + 1) * veh_num_each]
        cus_dict[i] = cus
        veh_dict[i] = veh
    return cus_dict, veh_dict


def __parking_point_with_robot_customer(VC2, VP):
    res = []
    num = int(len(VP) / len(VC2))
    for index, i in enumerate(VC2):
        res.append([i] + VP[index * num:(index + 1) * num])
    return res


def __max_travel_time(time_mat) -> float:
    n = len(time_mat)
    max_time = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            t = time_mat[i][j]
            if t < 1e5:
                max_time += t
    return round(max_time, 2)


def __depot_with_node(cus_dict, park):
    point_dict = {}
    for k in cus_dict.keys():
        point_dict[k] = []
        for i in cus_dict[k]:
            point_dict[k].append(i)
        for sub_park in park:
            if sub_park[0] in cus_dict[k]:
                point_dict[k] += sub_park[1:]
        point_dict[k] += [k]
    return point_dict


def generate_instance(info):
    depot_num = info["depot_num"]
    customer_number_each_depot = info["customer_number_each_depot"]
    vehicle_number_each_depot = info["vehicle_number_each_depot"]
    parking_point_num = info["parking_point_num"]

    robot_only_points_number = math.floor(depot_num *
                                          customer_number_each_depot / 3)
    point_range = [0, 100]
    parking_point_range = [1, 5]
    parking_point_distance_range = [4, 10]
    demand_range = [10, 30]
    service_time_range = [10, 20]
    rn.seed(1)

    # Generate depot_points
    depot_points = __generate_depot_points(depot_num, point_range)

    # Generate customer points
    customer_points = __generate_customer_points(depot_num,
                                                 customer_number_each_depot,
                                                 point_range, depot_points)
    # Select robot only points from all customer points
    robot_only_points_indices = __select_robot_only_points(
        depot_points, customer_points, robot_only_points_number, point_range)

    # Generate parking points for each robot only point
    existed_points = depot_points + customer_points  # 不会更新其中任意一个list
    parking_points = __generate_parking_points(robot_only_points_indices,
                                               customer_points,
                                               parking_point_num, point_range,
                                               parking_point_range,
                                               parking_point_distance_range,
                                               existed_points)

    # Merge all points (depot, customer, pariking) and indices
    customer_num = depot_num * customer_number_each_depot
    myInstance = instance
    myInstance.name = "M-d" + str(depot_num) + "-n" + str(
        customer_number_each_depot) + "-k" + str(
        vehicle_number_each_depot) + "-p" + str(parking_point_num)
    myInstance.V_coordinates = existed_points + parking_points
    myInstance.V = [i for i in range(len(myInstance.V_coordinates))]
    myInstance.VD = [i for i in range(depot_num)]
    myInstance.VC = [i for i in range(depot_num, depot_num + customer_num)]
    myInstance.VP = [
        i for i in range(
            depot_num + customer_num, depot_num + customer_num +
            robot_only_points_number * parking_point_num)
    ]
    myInstance.VC2 = __add_value_to_list(robot_only_points_indices, depot_num)
    myInstance.VC1 = __determain_normal_customer(myInstance.VC, myInstance.VC2)
    myInstance.K = [i for i in range(depot_num * vehicle_number_each_depot)]
    myInstance.demand = __generate_instance_demand_or_service_time(
        depot_num, customer_num, robot_only_points_number * parking_point_num,
        demand_range)
    myInstance.service_time = __generate_instance_demand_or_service_time(
        depot_num, customer_num, robot_only_points_number * parking_point_num,
        service_time_range)
    park_robot = __parking_point_with_robot_customer(myInstance.VC2,
                                                     myInstance.VP)
    (myInstance.veh_dist_matrix, myInstance.rob_power_matrix, myInstance.vehicle_cost_matrix,
     myInstance.robot_cost_matrix, myInstance.vehicle_time_matrix, myInstance.robot_time_matrix) = __calculate_matrix(
        myInstance.V_coordinates, myInstance.VD, myInstance.VC1,
        myInstance.VC2, park_robot, myInstance.vehicle_speed,
        myInstance.vehicle_cost, myInstance.robot_speed, myInstance.robot_cost, myInstance.robot_unit_consumption)
    myInstance.cus_with_depot, myInstance.veh_with_depot = __vehicle_customer_with_depot(
        myInstance.VC, myInstance.K, depot_num, customer_number_each_depot,
        vehicle_number_each_depot)
    myInstance.veh_depot_dict = transpose_dict(myInstance.veh_with_depot)
    myInstance.depot_with_node = __depot_with_node(myInstance.cus_with_depot, park_robot)
    myInstance.max_travel_time = __max_travel_time(
        myInstance.vehicle_time_matrix)
    myInstance.VC2_with_VP = {item[0]: item[1:] for item in park_robot}
    if (len(myInstance.V) != myInstance.VP[-1] + 1):
        print("存在问题")
    return myInstance


def plot_points(depot_points, customer_points, robot_only_points_indices,
                parking_points):
    # Extract x and y coordinates of depot_points
    depot_x_values = [point[0] for point in depot_points]
    depot_y_values = [point[1] for point in depot_points]

    # Extract x and y coordinates of points
    customer_x_values = [point[0] for point in customer_points]
    customer_y_values = [point[1] for point in customer_points]

    # Extract x and y coordinates of points
    robot_only_x_values = [
        customer_points[index][0] for index in robot_only_points_indices
    ]
    robot_only_y_values = [
        customer_points[index][1] for index in robot_only_points_indices
    ]

    parking_x_values = [point[0] for point in parking_points]
    parking_y_values = [point[1] for point in parking_points]

    # Plot depot_points and points
    plt.scatter(depot_x_values,
                depot_y_values,
                color='blue',
                label='Depot Points')
    plt.scatter(customer_x_values,
                customer_y_values,
                color='red',
                label='customer Points')
    plt.scatter(robot_only_x_values,
                robot_only_y_values,
                color='green',
                label='robot-only Points')
    plt.scatter(parking_x_values,
                parking_y_values,
                color='black',
                label='Parking Points')
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Depot Points and customer Points')
    plt.legend()
    plt.grid(True)
    plt.show()
