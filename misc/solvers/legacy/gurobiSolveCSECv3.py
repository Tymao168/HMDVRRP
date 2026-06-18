import gurobipy as gp
from gurobipy import GRB
from utils import Remove, doubleRemove
import matplotlib.pyplot as plt


def gurobiSolver(myInstance):
    # --------------------------
    # data prepare

    V = myInstance.V
    V_coordinates = myInstance.V_coordinates
    VD = myInstance.VD
    VC = myInstance.VC
    VP = myInstance.VP
    VC1 = myInstance.VC1  # VC'
    VC2 = myInstance.VC2  # VC''
    K = myInstance.K
    cf = myInstance.cf
    q = myInstance.demand
    qjmax=max(q)
    service_time = myInstance.service_time
    veh_dist_mat = myInstance.veh_dist_matrix
    rob_power_mat = myInstance.rob_power_matrix
    veh_cost_mat = myInstance.vehicle_cost_matrix
    rob_cost_mat = myInstance.robot_cost_matrix
    veh_time_mat = myInstance.vehicle_time_matrix
    rob_time_mat = myInstance.robot_time_matrix
    cus_with_depot = myInstance.cus_with_depot
    veh_with_depot = myInstance.veh_with_depot
    # veh_depot_dict = myInstance.veh_depot_dict  # transpose of veh_with_depot
    Uv = myInstance.veh_cap
    Ur = myInstance.rob_cap
    # qr = myInstance.rob_weight
    operation_time = myInstance.operation_time
    max_travel_time = myInstance.max_travel_time
    depot_with_node = myInstance.depot_with_node
    robot_ub = myInstance.robot_ub
    veh_dist_ub = myInstance.veh_dist_ub
    # tail indices of partial arc set
    # V5 = VC1 + VP  # A5
    # V_V2 = VD + VC1 + VP  # A\A2, V\V2
    # V_V3 = VC + VP  # A\A3, V\VD
    M = myInstance.very_big
    # 在日志文件中记录模型名称
    with open("log.file", "a") as log_file:
        log_file.write(f"---------New model----------")
        log_file.write(f"Model Name: " + myInstance.name)
    # --------------------------
    # build model
    md = gp.Model(myInstance.name)

    # model parameters set
    # md.setParam('TimeLimit', 10800)  # 将求解时间限制设置为3600秒
    md.setParam('TimeLimit', 10800)  # 将求解时间限制设置为3600秒
    md.setParam('Threads', 1)  # 设置最大求解线程数为4
    md.setParam('OutputFlag', True)  # 禁用求解过程中的输出信息
    md.setParam('LogFile', 'log.file')
    md.setParam('Method', 1)
    # md.setParam('Persolve', 0)
    # 设置MIP求解方法为:-1=automatic,0=primal simplex,
    # 1=dual simplex,2=barrier,3=concurrent,
    # 4=deterministic concurrent,5=deterministic concurrent simplex.
    # md.setParam('MIPGap', 0.25)  # 设置MIP求解终止准则为0.01
    md.setParam('MIPGap', 0.0001)  # 设置MIP求解终止准则为0.01
    # md.setParam('IntFeasTol', 1e-9)
    md.setParam('IntegralityFocus', 1)
    md.Params.LazyConstraints = 1
    #
    md._vertices_num = len(V)

    # decision variables
    # arc
    X = md.addVars(V, V, K, vtype=GRB.BINARY, name='x[i,j,k]')
    Y = md.addVars(V, V, K, vtype=GRB.BINARY, name='Y[i,j,k]')
    Z = md.addVars(V, V, K, vtype=GRB.BINARY, name='Z[i,j,k]')
    Zeta = md.addVars(V, K, vtype=GRB.BINARY, name='Zeta[i,k]')
    Xi = md.addVars(V, K, vtype=GRB.BINARY, name='Xi[i,k]')
    Delta = md.addVars(V, K, vtype=GRB.BINARY, name='Delta[i,k]')
    Psi = md.addVars(V, vtype=GRB.BINARY, name='Psi[i]')
    Phi = md.addVars(V, vtype=GRB.BINARY, name='Phi[i]')
    # load
    QV = md.addVars(V,
                    VD,
                    K,
                    lb=0.0,
                    ub=Uv,
                    vtype=GRB.CONTINUOUS,
                    name='QV[i,d,k]')
    QR = md.addVars(V,
                    VD,
                    K,
                    lb=0.0,
                    ub=Ur,
                    vtype=GRB.CONTINUOUS,
                    name='QR[i,d,k]')
    qT = md.addVars(V, lb=0.0, ub=Uv, vtype=GRB.CONTINUOUS, name='qT[i]')
    # tour size
    E = md.addVars(V,
                   K,
                   lb=0.0,
                   ub=robot_ub,
                   vtype=GRB.CONTINUOUS,
                   name='E[i,k]')
    # Mu = md.addVars(V, K, lb=0, vtype=GRB.INTEGER, name='Mu[i,k]')
    TV = md.addVars(V, K, lb=0.0, ub=max_travel_time, vtype=GRB.CONTINUOUS, name='TV[i,k]')

    md.update()

    # object function
    obj1 = gp.quicksum(veh_cost_mat[i][j] * X[i, j, k] + veh_cost_mat[i][j] * Y[i, j, k] +
                       rob_cost_mat[i][j] * Z[i, j, k] for i in V for j in V for k in K)
    obj2 = gp.quicksum(cf * X[d, i, k] for d in VD for i in V for k in veh_with_depot[d])

    md.setObjective(obj1 + obj2, GRB.MINIMIZE)

    # constraints in arc
    md.addConstrs((gp.quicksum(X[d, j, k] for j in V if d != j) <= 1 for d in VD for k in veh_with_depot[d]),
                  name='c2')
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] for i in V if i != j) <= 1
                   for j in V for k in K),
                  name='c3')
    md.addConstrs((X[i, j, k] + Y[i, j, k] + Z[i, j, k] <= gp.quicksum(X[d, l, k] for l in V if d != l)
                   for d in VD for k in veh_with_depot[d] for j in V for i in V if i != j),
                  name='c4')
    md.addConstrs(
        (gp.quicksum(
            X[i, j, k] + Y[i, j, k] + Z[i, j, k] for i in V if i != j for k in veh_with_depot[d]) <= 1 + gp.quicksum(
            Delta[j, k] for k in veh_with_depot[d]) for d in VD for j in depot_with_node[d]),
        name='c10')
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k]
                               for i in V if i != j
                               for k in K) == 1 + gp.quicksum(Zeta[j, k] for k in Remove(K, veh_with_depot[d])) +
                   gp.quicksum(Delta[j, k] for k in K)
                   for d in VD for j in cus_with_depot[d]),
                  name='c5')  # z
    md.addConstrs((2 * gp.quicksum(Delta[j, k] for k in K) - gp.quicksum(
        X[i, j, k] + Y[i, j, k] + Z[i, j, k] for i in V if i != j for k in K) <= 0 for j in VP),
                  name='c6a')  # z
    md.addConstrs((gp.quicksum(Xi[j, k] for k in K) - gp.quicksum(
        X[i, j, k] + Y[i, j, k] + Z[i, j, k] for i in V if i != j for k in K) <= 0 for j in VP),
                  name='c6b')
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k] for i in V if i != j for k in K) - 2 *
                   gp.quicksum(Delta[j, k] for k in K) <= M * (1 - Psi[j]) for j in VP), name='c6c')  # z
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k] for i in V if i != j for k in K) - gp.quicksum(
        Xi[j, k] for k in K) <= M * (1 - Phi[j]) for j in VP), name='c6d')
    md.addConstrs((Psi[i] + Phi[i] >= 1 for i in VP), name='c6e')

    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k]
                               for k in veh_with_depot[d]
                               for i in V if i != j) >= gp.quicksum(Zeta[j, k] for k in Remove(K, veh_with_depot[d]))
                   for d in VD for j in cus_with_depot[d]),
                  name='c11')  # z
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k]
                               for i in V if i != j) >= Zeta[j, k]
                   for d in VD for j in cus_with_depot[d] for k in Remove(K, veh_with_depot[d])),
                  name='c12a')  # z
    md.addConstrs((gp.quicksum(X[i, d, k] + Y[i, d, k] + Z[i, d, k]
                               for i in V if i != d) >= Zeta[d, k]
                   for d in VD for k in Remove(K, veh_with_depot[d])),
                  name='c12b')  # z
    md.addConstrs((gp.quicksum(X[i, j, k] + Y[i, j, k] + Z[i, j, k]
                               for i in V if i != j) >= 2 * Delta[j, k]
                   for j in VC + VP for k in K),
                  name='c13')
    md.addConstrs((gp.quicksum(X[i, j, k]
                               for i in V if i != j) >= Xi[j, k]
                   for j in VP for k in K),
                  name='c14')
    md.addConstrs((gp.quicksum(X[i, j, k] for i in V if i != j) -
                   gp.quicksum(X[j, i, k]
                               for i in V if i != j) + Delta[j, k] - Xi[j, k] == 0
                   for j in V for k in K),
                  name='c7')
    md.addConstrs((gp.quicksum(Y[i, j, k] for i in V if i != j) -
                   gp.quicksum(Y[j, i, k]
                               for i in V if i != j) - Delta[j, k] + Xi[j, k] == 0
                   for j in V for k in K),
                  name='c8')
    md.addConstrs((gp.quicksum(Z[i, j, k] for i in V if i != j) -
                   gp.quicksum(Z[j, i, k]
                               for i in V if i != j) - Delta[j, k] + Xi[j, k] == 0
                   for j in V for k in K),
                  name='c9')
    md.addConstrs((gp.quicksum(Z[i, j, k] for i in V if i != j)<= 1
                   for j in V for k in K),
                  name='c9b')
    md.addConstrs((gp.quicksum(Xi[i, k] + Delta[i, k] for k in K) + gp.quicksum(
        Zeta[i, k] for k in Remove(K, veh_with_depot[d])) <= 1
                   for d in VD for i in cus_with_depot[d]),
                  name='c15')  # z
    md.addConstrs((gp.quicksum((Xi[i, k] + Delta[i, k]) for k in K) <= 2 for d in VD for i in
                   doubleRemove(depot_with_node[d], cus_with_depot[d], d)),
                  name='c16')  # z
    md.addConstrs((Xi[i, k1] + Delta[i, k2] <= 1 for i in VP for k1 in K
                   for k2 in K if k1 != k2),
                  name='c18')
    md.addConstrs((gp.quicksum(Zeta[i, k] for k in Remove(K, veh_with_depot[d])) <= 1 for d in VD for i in
                   cus_with_depot[d]), name='c19')  # z
    md.addConstrs((Zeta[d, k] == gp.quicksum(X[d, j, k] for j in V) for d in VD for k in Remove(K, veh_with_depot[d])),
                  name='c20')  # z
    md.addConstrs((gp.quicksum(Xi[i, k] for k in K) <= 1 for i in V), name='c17a')
    md.addConstrs((gp.quicksum(Delta[i, k] for k in K) <= 1 for i in V), name='c17b')
    md.addConstrs((Xi[i, k] == 0 for i in VC + VD for k in K), name='c29')
    md.addConstrs((Delta[i, k] == 0 for i in VC2 + VD for k in K), name='c30')
    md.addConstrs((Zeta[i, k] == 0 for k in K for i in VC2 + VP), name='c32')
    md.addConstrs((Zeta[d, k] == 0 for d in VD for k in veh_with_depot[d]), name='aaaaaaa')

    #
    md.addConstrs((Z[i,j,k]==0 for i in VP for j in VP for k in K),name="dadadada")
    # subtour elimination constraint
    # md.addConstrs(
    #     (Mu[i, k] - Mu[j, k] + len(V) * (X[i, j, k] + Y[i, j, k]) <= len(V) - 1 for d in VD for k in veh_with_depot[d]
    #      for i in Remove(V, d) for j in Remove(V, d) if i != j), name='c24')
    # md.addConstrs(
    #     (Mu[i, k] - Mu[j, k] <= M * (1 + Delta[j, k] - Z[i, j, k]) for i in V for j in V if i != j for k in K),
    #     name='c25a')
    # md.addConstrs(
    #     (Mu[j, k] - Mu[i, k] <= M * (1 + Delta[j, k] - Z[i, j, k]) for i in V for j in V if i != j for k in K),
    #     name='c25b')
    # md.addConstrs(
    #     (Mu[i, k] - Mu[j, k] <= M * (2 - Delta[j, k] - Z[i, j, k]) for i in V for j in V if i != j for k in K),
    #     name='c26')
    def subtour_elimination_callback(model, where):
        """
        子回路消除回调函数，适用于Gurobi 10.0.3
        """
        if where == GRB.Callback.MIPSOL:
            # 获取当前解的值
            x_vals = model.cbGetSolution(model._X)
            y_vals = model.cbGetSolution(model._Y)

            # 为每个车辆k检查子回路
            for k in model._K:
                # 构建当前车辆k的图
                edges = []
                for i in model._V:
                    for j in model._V:
                        if i != j:
                            if (x_vals[i, j, k] > 0.5 or
                                    y_vals[i, j, k] > 0.5):
                                edges.append((i, j))

                if not edges:
                    continue

                # 找到所有连通分量
                subtours = find_subtours_from_edges(edges, model._V)

                # 为每个不包含depot的子回路添加消除约束
                for subtour in subtours:
                    if len(subtour) >= 2:
                        has_depot = any(node in model._VD for node in subtour)
                        if not has_depot:
                            # 添加子回路消除的lazy constraint
                            model.cbLazy(
                                gp.quicksum(
                                    model._X[i, j, k] +
                                    model._Y[i, j, k]

                                    for i in subtour
                                    for j in subtour
                                    if i != j
                                ) <= len(subtour) - 1
                            )
                            print(f"添加子回路消除约束: 车辆{k}, 子回路{subtour}")

    def find_subtours_from_edges(edges, V):
        """
        从边列表中找到所有子回路
        """
        if not edges:
            return []

        # 构建邻接表
        graph = {v: [] for v in V}
        for i, j in edges:
            graph[i].append(j)

        visited = set()
        subtours = []

        def dfs(node, path, start):
            if node in path:
                # 找到环
                cycle_start = path.index(node)
                cycle = path[cycle_start:]
                if len(cycle) >= 2:
                    return cycle
                return None

            if node in visited:
                return None

            path.append(node)

            for neighbor in graph[node]:
                if neighbor == start and len(path) >= 2:
                    # 回到起始点，形成完整回路
                    return path[:]
                elif neighbor not in visited:
                    result = dfs(neighbor, path[:], start)
                    if result:
                        return result

            return None

        for start_node in V:
            if start_node not in visited:
                subtour = dfs(start_node, [], start_node)
                if subtour:
                    subtours.append(subtour)
                    visited.update(subtour)

        return subtours

    # md.addConstrs((Z[i, j, k] <= 0 for i in VP for j in VP if i != j for k in K), name='57')
    # md.addConstrs(
    #     (Delta[i, k] + Delta[j, k] - 1 <= M * (1 - Z[i, j, k]) for i in VC + VP for j in VC + VP if i != j for k in K),
    #     name='58')
    # md.addConstrs((Zeta[i, k] <= 0 for d in VD for i in depot_with_node[d] for k in veh_with_depot[d]), name='59')
    # Constraints in time
    md.addConstrs(
        (TV[i, k] + veh_time_mat[i][j] - TV[j, k] <= M * (1 - X[i, j, k] - Y[i, j, k]) for d in VD for k in
         veh_with_depot[d] for i
         in V for j in VC1 + VP), name='t1')
    md.addConstrs(
        (TV[i, k] + veh_time_mat[i][j] - TV[j, k] <= M * (1 - X[i, j, k] - Y[i, j, k]) for d in VD for k in
         veh_with_depot[d] for i
         in V for j in Remove(VD, d)), name='t1')
    md.addConstrs((TV[i, k1] - TV[i, k2] <= M * (1 - Zeta[i, k2]) for d in VD for k1 in veh_with_depot[d] for k2 in
                   Remove(K, veh_with_depot[d]) for i in cus_with_depot[d]), name='t7')
    # # Constraints in load
    md.addConstrs((QV[j, d, k] + q[j] + qT[j] - QV[i, d, k] <= (2*Uv+qjmax) *
                   (1 + Xi[j, k] + Delta[j, k] - X[i, j, k] - Y[i, j, k]) for d in VD
                   for k in veh_with_depot[d] for i in V
                   for j in Remove(depot_with_node[d], d) if i != j),
                  name='c33')
    md.addConstrs((QV[j, d, k] + (1 - Zeta[j, k]) * q[j] - qT[j] - QV[i, d, k] <= (Uv+qjmax) *
                   (1 + Xi[j, k] + Delta[j, k] - X[i, j, k] - Y[i, j, k])
                   for d in VD for k in Remove(K, veh_with_depot[d]) for i in V
                   for j in depot_with_node[d] if i != j),
                  name='c34')  # z
    md.addConstrs((QV[j, d, k] - QV[i, d, k] <= Uv *
                   (1 + Xi[j, k] + Delta[j, k] - X[i, j, k] - Y[i, j, k]) for d in VD
                   for k in K for i in V
                   for j in Remove(V, depot_with_node[d]) if i != j),
                  name='c35')
    md.addConstrs(
        (QR[j, d, k] <= Ur * (1 + Xi[j, k] - gp.quicksum(X[i, j, k] + Y[i, j, k] for i in V if i != j)) for
         d in VD for k in K for j in V), name='c36')

    md.addConstrs((QR[j, d, k] + q[j] - QR[i, d, k] <= (Ur+qjmax) *
                   (1 + Delta[j, k] + Xi[j, k] - Z[i, j, k]) for d in VD
                   for k in K for i in V
                   for j in Remove(depot_with_node[d], d) if i != j),
                  name='c37')
    md.addConstrs((QR[j, d, k] - QR[i, d, k] <= Ur *
                   (1 + Xi[j, k] + Delta[j, k] - Z[i, j, k])
                   for d in VD for k in K for i in V
                   for j in Remove(V, depot_with_node[d]) if i != j),
                  name='c39')
    md.addConstrs(
        (QV[j, d, k] <= Uv * (1 + Delta[j, k] - gp.quicksum(Z[i, j, k] for i in V if i != j)) for d in VD for
         k in K for
         j in V), name='c40_aaaaaa')

    md.addConstrs((QV[j, d, k] + QR[j, d, k] - QV[i, d, k] <= (Uv+Ur) * (2 + Delta[j, k] - X[i, j, k] - Xi[j, k])
                   for d in VD for k in K for i in V
                   for j in VP if i != j),
                  name='c41')
    md.addConstrs((QV[j, d, k] + q[j] - QV[i, d, k] - QR[l, d, k] <= (Uv+qjmax) *
                   (2 - Y[i, j, k] - Z[l, j, k])
                   for d in VD for k in K for i in V for l in V for j in cus_with_depot[d]
                   if i != j if i != l if j != l),
                  name='c42a')

    md.addConstrs((QV[j, d, k] - QV[i, d, k] - QR[l, d, k] <= Uv *
                   (2 - Y[i, j, k] - Z[l, j, k])
                   for d in VD for k in K for i in V for l in V for j in Remove(VC, cus_with_depot[d])
                   if i != j if i != l if j != l),
                  name='c42b')
    md.addConstrs((QV[j, d, k] - QV[i, d, k] - QR[l, d, k] <= Uv *
                   (2 - Y[i, j, k] - Z[l, j, k])
                   for d in VD for k in K for i in V for j in VP for l in V
                   if i != j if i != l if j != l),
                  name='c42c')

    md.addConstrs((QV[j, d, k] + QR[j, d, k] - QV[i, d, k] - QR[l, d, k] <= (Uv+Ur) *
                   (2 - X[i, j, k] - Z[l, j, k])
                   for d in VD for k in K for i in V for j in VP for l in V
                   if i != j if i != l if j != l),
                  name='c43')

    md.addConstrs((gp.quicksum(QV[i, d, k] + QR[i, d, k] for d in Remove(VD, i)) <= 0 for i in VD
                   for k in veh_with_depot[i]),
                  name='c44')
    md.addConstrs((QR[d, d, k] <= 0 for d in VD for k in veh_with_depot[d]),
                  name='c45')

    md.addConstrs((gp.quicksum(QV[i, d, k] for d in VD) <= Uv
                   for k in K for i in V),
                  name='c46')
    md.addConstrs((gp.quicksum(QR[i, d, k] for d in VD) <= Ur
                   for k in K for i in V),
                  name='c47')
    md.addConstrs(
        (qT[i] <= Uv * Zeta[i, k] for d in VD for i in cus_with_depot[d] for k in Remove(K, veh_with_depot[d])),
        name='c48')

    # Constraints in tour size
    md.addConstrs((gp.quicksum(
        veh_dist_mat[i][j] * X[i, j, k] + veh_dist_mat[i][j] * Y[i, j, k]
        for i in V for j in V) <= veh_dist_ub for k in K),
                  name='c21')
    md.addConstrs(
        (E[j, k] + rob_power_mat[i][j] - E[i, k] <= M * (1 + Delta[j, k] - Z[i, j, k])
         for k in K for i in V for j in VC + VP if i != j),
        name='c22')
    md.addConstrs(
        (rob_power_mat[i][j] - E[i, k] <= M * (2 - Delta[j, k] - Z[i, j, k])
         for k in K for i in V for j in VC + VP if i != j),
        name='c23')

    md.update()
    md._V = V
    md._VD = VD
    md._K = K
    md._X = X
    md._Y = Y
    md._Z = Z

    md.optimize()
    if md.status == GRB.OPTIMAL or md.Runtime >= 4000:
        # 节点状态
        print("Zeta: ")
        for i in V:
            for k in K:
                if Zeta[i, k].x > 0.5:
                    print(str((i, k)) + ": " + str(Zeta[i, k].x), end=' ')
        print("")
        print("Delta: ")
        for i in V:
            for k in K:
                if Delta[i, k].x > 0.5:
                    print(str((i, k)) + ": " + str(Delta[i, k].x), end=' ')
        print("")
        print("Xi: ")
        for i in V:
            for k in K:
                if Xi[i, k].x > 0.5:
                    print(str((i, k)) + ": " + str(Xi[i, k].x), end=' ')
        print("")

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
        # 简单的路线
        for k in K:
            print("k= " + str(k))
            line_color = colors[k]
            for i in V:
                for j in V:
                    if X[i, j, k].x > 0.5:
                        print("X", end=' ')
                        print(i, j, end=' ')
                        for d in VD:
                            print("QV[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QV[i, d, k].x), end=' ')
                        for d in VD:
                            print("QR[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QR[i, d, k].x), end=' ')
                        print("qT[" + str(j) + "]=" + str(qT[j].x))
                        plt.plot([V_coordinates[i][0], V_coordinates[j][0]],
                                 [V_coordinates[i][1], V_coordinates[j][1]],
                                 linestyle='-',
                                 color=line_color)
                    if Y[i, j, k].x > 0.5:
                        print("Y", end=' ')
                        print(i, j, end=' ')
                        for d in VD:
                            print("QV[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QV[i, d, k].x), end=' ')
                        for d in VD:
                            print("QR[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QR[i, d, k].x), end=' ')
                        print("qT[" + str(j) + "]=" + str(qT[j].x))
                        plt.plot([V_coordinates[i][0], V_coordinates[j][0]],
                                 [V_coordinates[i][1], V_coordinates[j][1]],
                                 linestyle='--',
                                 color=line_color)
                    if Z[i, j, k].x > 0.5:
                        print("Z", end=' ')
                        print(i, j, end=' ')
                        for d in VD:
                            print("QV[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QV[i, d, k].x), end=' ')
                        for d in VD:
                            print("QR[" + str(i) + "," + str(d) + "," + str(k) + ']=' + str(QR[i, d, k].x), end=' ')
                        print("qT[" + str(j) + "]=" + str(qT[j].x))
                        plt.plot([V_coordinates[i][0], V_coordinates[j][0]],
                                 [V_coordinates[i][1], V_coordinates[j][1]],
                                 linestyle='-.',
                                 color=line_color)
        # plt.show()
        plt.savefig('figure/'+myInstance.name+'.png')  # 保存为PNG格式
        plt.close()  # 关闭图形，释放内存

    elif md.status == GRB.INFEASIBLE:
        md.computeIIS()
        md.write('model.ilp')
    gp.disposeDefaultEnv()
