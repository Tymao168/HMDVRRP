from instance_generation import generate_instance
# from gurobiSolveC import gurobiSolver
from solvers.legacy.gurobiSolveCSECv1 import gurobiSolver
from utils import instance_save, is_file_in_folder, read_json_from_file, dict_to_data_instance, parse_string
from utils import instancePlot


def load_or_generate_instance(instance_name, folder_path="instances"):
    """
    加载已存在的实例或生成新实例

    Args:
        instance_name (str): 实例文件名
        folder_path (str): 文件夹路径，默认为"instances"

    Returns:
        object: 实例对象
    """
    if is_file_in_folder(instance_name, folder_path):
        data = read_json_from_file(instance_name, folder_path)
        myInstance = dict_to_data_instance(data)
    else:
        info_dict = parse_string(instance_name)
        myInstance = generate_instance(info_dict)
        instance_save(folder_path, myInstance)

    return myInstance


def solve_instance(instance_name, folder_path="instances", plot=False):
    """
    求解指定的实例

    Args:
        instance_name (str): 实例文件名
        folder_path (str): 文件夹路径，默认为"instances"
        plot (bool): 是否绘制实例图，默认为False

    Returns:
        求解结果
    """
    # 加载或生成实例
    myInstance = load_or_generate_instance(instance_name, folder_path)

    # 可选择绘制实例图
    if plot:
        instancePlot(myInstance)

    # 求解实例
    result = gurobiSolver(myInstance)
    return result


def main(instance_name="M-d3-n6-k1-p2.json", folder_path="instances", plot=False):
    """
    主函数，执行完整的实例处理和求解流程

    Args:
        instance_name (str): 实例文件名，默认为"M-d3-n6-k1-p2.json"
        folder_path (str): 文件夹路径，默认为"instances"
        plot (bool): 是否绘制实例图，默认为False

    Returns:
        求解结果
    """
    return solve_instance(instance_name, folder_path, plot)


# 使用示例
if __name__ == "__main__":
    # 方式1：使用默认参数
    result = main()

    # 方式2：指定参数
    # result = main("M-d3-n6-k1-p2.json", "instances", plot=True)

    # 方式3：只求解特定实例
    # result = solve_instance("M-d3-n6-k1-p2.json")

    # 方式4：只加载/生成实例
    # instance = load_or_generate_instance("M-d3-n6-k1-p2.json")
