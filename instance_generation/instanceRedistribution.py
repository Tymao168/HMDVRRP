import os
import json
import re
import random
import copy

def process_json_file(file_path):
    try:
        match = re.search(r'n(\d+)', file_path)
        if match:
            n_value = int(match.group(1))

            # 检查条件，仅处理 n < 7 的文件
            if n_value <= 10 and not os.path.basename(file_path).startswith('R'):
                # 打开 JSON 文件并解析数据
                with open(file_path, 'r') as json_file:
                    data = json.load(json_file)

                    cus_with_depot = random_assignment(data["cus_with_depot"])
                    depot_with_node = basedAssignment(cus_with_depot,data["VC2_with_VP"])
                    data["cus_with_depot"] = cus_with_depot
                    data["depot_with_node"] = depot_with_node
                    data["name"] = "R"+data["name"]
                    # 构造新文件名，在 "M" 前面添加 "R"
                    new_file_name = 'R' + os.path.basename(file_path)[0:]

                    # 构造新文件的完整路径
                    new_file_path = os.path.join(os.path.dirname(file_path), new_file_name)

                    # 将处理后的数据写入新文件
                    with open(new_file_path, 'w') as new_json_file:
                        json.dump(data, new_json_file, indent=2)

    except json.JSONDecodeError as e:
        print(f'Error decoding JSON file {file_path}: {e}')


def process_json_files_in_folder(folder_path):
    # 获取文件夹内所有文件的列表
    file_list = os.listdir(folder_path)

    # 遍历文件列表
    for file_name in file_list:
        # 构造文件的完整路径
        file_path = os.path.join(folder_path, file_name)

        # 检查文件是否为 JSON 文件
        if file_name.endswith('.json'):
            # 调用处理函数
            process_json_file(file_path)


def random_assignment(input_dict):
    random.seed(42)
    result_dict = {key: [] for key in input_dict}

    # 将所有整数收集到一个列表中
    all_integers = [value for values in input_dict.values() for value in values]

    # 先为每个键添加一个整数
    for key in result_dict:
        # 至少保留一个整数
        if all_integers:
            result_dict[key].append(all_integers.pop(random.randint(0, len(all_integers) - 1)))

    # 随机分配其他整数
    while all_integers:
        value = all_integers.pop(random.randint(0, len(all_integers) - 1))
        key = random.choice(list(result_dict.keys()))
        result_dict[key].append(value)
    return result_dict


def basedAssignment(input_dictA, input_dictB):
    result_dict = copy.deepcopy(input_dictA)  # 创建原字典的副本，防止修改原字典

    for keyB in input_dictB:
        for keyA in input_dictA:
            if int(keyB) in input_dictA[keyA]:
                result_dict[keyA].extend(input_dictB[keyB])
    for keyA in input_dictA:
        result_dict[keyA].append(int(keyA))
    return result_dict


if __name__ == "__main__":
    process_json_files_in_folder("instances")
