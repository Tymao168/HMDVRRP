import csv
import matplotlib.pyplot as plt

def readCsv(csv_file_path):
    # 用于存储迭代次数和每次迭代前的 bestSolution 的 totalCost
    iterations = []
    total_costs = []

    # 读取 CSV 文件
    try:
        with open(csv_file_path, mode='r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader)  # 跳过标题行
            for row in reader:
                iterations.append(int(row[0]))
                total_costs.append(float(row[1]))
    except FileNotFoundError:
        print("File not found.")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
    return iterations, total_costs


def plotLineChart(iterations, total_costs):
    # 画折线图
    plt.plot(iterations, total_costs, linestyle='-')


def endPLot():
    # 添加标题和标签
    plt.title('Total Cost vs Iteration')
    plt.xlabel('Iteration')
    plt.ylabel('Total Cost')

    # 显示图例
    plt.legend(['all','no','only'])

    # 显示图形
    plt.show()

def main():
    n = 16000 # 跳过尾部迭代次数
    iterations1, total_cost1 = readCsv("n5d20k2/allIn.csv")
    iterations2, total_cost2 = readCsv("n5d20k2/noChain.csv")
    iterations3, total_cost3 = readCsv("n5d20k2/onlyChain.csv")
    plotLineChart(iterations1[:-n],total_cost1[:-n])
    plotLineChart(iterations2,total_cost2)
    plotLineChart(iterations3[:-n],total_cost3[:-n])
    endPLot()


if __name__ == "__main__":
    main()
