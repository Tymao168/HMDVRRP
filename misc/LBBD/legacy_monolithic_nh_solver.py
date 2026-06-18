import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt


def gurobiSolver(myInstance):
    V = myInstance.V
    V_coordinates = myInstance.V_coordinates
    VD = myInstance.VD
    VC = myInstance.VC
    VP = myInstance.VP
    VC1 = myInstance.VC1
    VC2 = myInstance.VC2
    K = myInstance.K
    cf = myInstance.cf
    q = myInstance.demand
    qjmax = max(q)
    veh_dist_mat = myInstance.veh_dist_matrix
    rob_power_mat = myInstance.rob_power_matrix
    veh_cost_mat = myInstance.vehicle_cost_matrix
    rob_cost_mat = myInstance.robot_cost_matrix
    veh_time_mat = myInstance.vehicle_time_matrix
    cus_with_depot = myInstance.cus_with_depot
    veh_with_depot = myInstance.veh_with_depot
    Uv = myInstance.veh_cap
    Ur = myInstance.rob_cap
    max_travel_time = myInstance.max_travel_time
    depot_with_node = myInstance.depot_with_node
    robot_ub = myInstance.robot_ub
    veh_dist_ub = myInstance.veh_dist_ub
    M = myInstance.very_big

    with open("log.file", "a") as log_file:
        log_file.write("---------New model----------")
        log_file.write("Model Name: " + myInstance.name)

    md = gp.Model(myInstance.name)
    md.setParam("TimeLimit", 10800)
    md.setParam("Threads", 1)
    md.setParam("OutputFlag", True)
    md.setParam("LogFile", "log.file")
    md.setParam("Method", 1)
    md.setParam("MIPGap", 0.0001)
    md.setParam("IntegralityFocus", 1)

    VP_set = set(VP)
    VC_set = set(VC)
    VC1_set = set(VC1)
    arc_block_value = 1e5

    Vd_nh = {d: list(depot_with_node[d]) for d in VD}
    Vc_d = {d: list(cus_with_depot[d]) for d in VD}
    Vp_d = {d: [i for i in Vd_nh[d] if i in VP_set] for d in VD}
    Vcp_d = {d: Vc_d[d] + Vp_d[d] for d in VD}
    Vc1_d = {d: [i for i in Vc_d[d] if i in VC1_set] for d in VD}
    Kd = {d: list(veh_with_depot[d]) for d in VD}
    A_v = {
        d: [
            (i, j)
            for i in Vd_nh[d]
            for j in Vd_nh[d]
            if i != j and veh_cost_mat[i][j] < arc_block_value and veh_time_mat[i][j] < arc_block_value
        ]
        for d in VD
    }
    A_r = {
        d: [
            (i, j)
            for i in Vcp_d[d]
            for j in Vcp_d[d]
            if i != j and rob_cost_mat[i][j] < arc_block_value
        ]
        for d in VD
    }
    feasible_robot_energies = [rob_power_mat[i][j] for d in VD for (i, j) in A_r[d]]
    e_max = max(feasible_robot_energies) if feasible_robot_energies else 0.0

    truck_keys = [(i, j, k) for d in VD for k in Kd[d] for (i, j) in A_v[d]]
    robot_keys = [(i, j, k) for d in VD for k in Kd[d] for (i, j) in A_r[d]]
    local_node_keys = [(i, k) for d in VD for k in Kd[d] for i in Vd_nh[d]]
    robot_node_keys = [(i, k) for d in VD for k in Kd[d] for i in Vcp_d[d]]
    parking_node_keys = [(i, k) for d in VD for k in Kd[d] for i in Vp_d[d]]

    X = md.addVars(truck_keys, vtype=GRB.BINARY, name="x[i,j,k]")
    Y = md.addVars(truck_keys, vtype=GRB.BINARY, name="y[i,j,k]")
    Z = md.addVars(robot_keys, vtype=GRB.BINARY, name="z[i,j,k]")
    Xi = md.addVars(V, K, vtype=GRB.BINARY, name="Xi[i,k]")
    Delta = md.addVars(V, K, vtype=GRB.BINARY, name="Delta[i,k]")
    Phi = md.addVars(VP, vtype=GRB.BINARY, name="Phi[i]")
    Psi = md.addVars(VP, vtype=GRB.BINARY, name="Psi[i]")
    QV = md.addVars(local_node_keys, lb=0.0, ub=Uv, vtype=GRB.CONTINUOUS, name="QV[i,k]")
    QR = md.addVars(local_node_keys, lb=0.0, ub=Ur, vtype=GRB.CONTINUOUS, name="QR[i,k]")
    E = md.addVars(robot_node_keys, lb=0.0, ub=robot_ub, vtype=GRB.CONTINUOUS, name="E[i,k]")
    TV = md.addVars(local_node_keys, lb=0.0, ub=max_travel_time, vtype=GRB.CONTINUOUS, name="TV[i,k]")

    def truck_in(d, j, k, var):
        return gp.quicksum(var.get((i, j, k), 0.0) for i in Vd_nh[d] if i != j)

    def truck_out(d, j, k, var):
        return gp.quicksum(var.get((j, i, k), 0.0) for i in Vd_nh[d] if i != j)

    def robot_in(d, j, k):
        return gp.quicksum(Z.get((i, j, k), 0.0) for i in Vcp_d[d] if i != j)

    def robot_out(d, j, k):
        return gp.quicksum(Z.get((j, i, k), 0.0) for i in Vcp_d[d] if i != j)

    obj1 = gp.quicksum(
        veh_cost_mat[i][j] * (X[i, j, k] + Y[i, j, k])
        for d in VD for k in Kd[d] for (i, j) in A_v[d]
    )
    obj2 = gp.quicksum(
        rob_cost_mat[i][j] * Z[i, j, k]
        for d in VD for k in Kd[d] for (i, j) in A_r[d]
    )
    obj3 = gp.quicksum(
        cf * X.get((d, j, k), 0.0)
        for d in VD for k in Kd[d] for j in Vd_nh[d] if j != d
    )
    md.setObjective(obj1 + obj2 + obj3, GRB.MINIMIZE)

    md.addConstrs(
        (gp.quicksum(X.get((d, j, k), 0.0) for j in Vd_nh[d] if j != d) <= 1
         for d in VD for k in Kd[d]),
        name="eq:c2"
    )
    md.addConstrs(
        (truck_in(d, j, k, X) + truck_in(d, j, k, Y) <= 1
         for d in VD for k in Kd[d] for j in Vd_nh[d]),
        name="eq:c3"
    )
    md.addConstrs(
        (X[i, j, k] + Y[i, j, k] <= gp.quicksum(X.get((d, l, k), 0.0) for l in Vd_nh[d] if l != d)
         for d in VD for k in Kd[d] for (i, j) in A_v[d]),
        name="eq:c4a"
    )
    md.addConstrs(
        (Z[i, j, k] <= gp.quicksum(X.get((d, l, k), 0.0) for l in Vd_nh[d] if l != d)
         for d in VD for k in Kd[d] for (i, j) in A_r[d]),
        name="eq:c4b"
    )
    md.addConstrs(
        (
            gp.quicksum(truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k) for k in Kd[d])
            == 1 + gp.quicksum(Delta[j, k] for k in Kd[d])
            for d in VD for j in Vc_d[d]
        ),
        name="eq:c5"
    )
    md.addConstrs(
        (
            2 * gp.quicksum(Delta[j, k] for k in Kd[d])
            - gp.quicksum(truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k) for k in Kd[d])
            <= 0
            for d in VD for j in Vp_d[d]
        ),
        name="eq:c50"
    )
    md.addConstrs(
        (
            gp.quicksum(Xi[j, k] for k in Kd[d])
            - gp.quicksum(truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k) for k in Kd[d])
            <= 0
            for d in VD for j in Vp_d[d]
        ),
        name="eq:c51"
    )
    md.addConstrs(
        (
            gp.quicksum(truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k) for k in Kd[d])
            - 2 * gp.quicksum(Delta[j, k] for k in Kd[d])
            <= 2 * (1 - Phi[j])
            for d in VD for j in Vp_d[d]
        ),
        name="eq:c52"
    )
    md.addConstrs(
        (
            gp.quicksum(truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k) for k in Kd[d])
            - gp.quicksum(Xi[j, k] for k in Kd[d])
            <= 2 * (1 - Psi[j])
            for d in VD for j in Vp_d[d]
        ),
        name="eq:c53"
    )
    md.addConstrs((Phi[j] + Psi[j] >= 1 for j in VP), name="eq:c54")

    md.addConstrs(
        (
            truck_out(d, j, k, X) == truck_in(d, j, k, X) - Delta[j, k] + Xi[j, k]
            for d in VD for k in Kd[d] for j in Vd_nh[d]
        ),
        name="eq:c7"
    )
    md.addConstrs(
        (
            truck_out(d, j, k, Y) == truck_in(d, j, k, Y) + Delta[j, k] - Xi[j, k]
            for d in VD for k in Kd[d] for j in Vd_nh[d]
        ),
        name="eq:c8"
    )
    md.addConstrs(
        (
            robot_out(d, j, k) == robot_in(d, j, k) + Delta[j, k] - Xi[j, k]
            for d in VD for k in Kd[d] for j in Vcp_d[d]
        ),
        name="eq:c9a"
    )
    md.addConstrs(
        (robot_out(d, j, k) <= 1 for d in VD for k in Kd[d] for j in Vcp_d[d]),
        name="eq:c9b"
    )
    md.addConstrs(
        (
            2 * Delta[j, k] <= truck_in(d, j, k, X) + truck_in(d, j, k, Y) + robot_in(d, j, k)
            for d in VD for k in Kd[d] for j in Vcp_d[d]
        ),
        name="eq:c13"
    )
    md.addConstrs(
        (Xi[j, k] <= truck_in(d, j, k, X) for d in VD for k in Kd[d] for j in Vp_d[d]),
        name="eq:c14"
    )
    md.addConstrs(
        (gp.quicksum(Delta[i, k] for k in Kd[d]) <= 1 for d in VD for i in Vc_d[d]),
        name="eq:c15"
    )
    md.addConstrs(
        (gp.quicksum(Xi[i, k] + Delta[i, k] for k in Kd[d]) <= 2 for d in VD for i in Vp_d[d]),
        name="eq:c16"
    )
    md.addConstrs(
        (gp.quicksum(Xi[i, k] for k in Kd[d]) <= 1 for d in VD for i in Vcp_d[d]),
        name="eq:c17a"
    )
    md.addConstrs(
        (gp.quicksum(Delta[i, k] for k in Kd[d]) <= 1 for d in VD for i in Vcp_d[d]),
        name="eq:c17b"
    )
    md.addConstrs(
        (
            Xi[i, k1] + Delta[i, k2] <= 1
            for d in VD for i in Vp_d[d] for k1 in Kd[d] for k2 in Kd[d] if k1 != k2
        ),
        name="eq:c18"
    )
    md.addConstrs(
        (
            gp.quicksum(Z.get((i, j, k), 0.0) for i in Vp_d[d] for j in Vp_d[d] if i != j) == 0
            for d in VD for k in Kd[d]
        ),
        name="eq:c21"
    )
    md.addConstrs((Xi[i, k] == 0 for i in VD + VC for k in K), name="eq:c25")
    md.addConstrs((Delta[i, k] == 0 for i in VC2 + VD for k in K), name="eq:c26")

    md.addConstrs(
        (
            QV[j, k] + q[j] <= QV[i, k] + (Uv + qjmax) * (
                1 - X.get((i, j, k), 0.0) - Y.get((i, j, k), 0.0) + Xi[j, k] + Delta[j, k]
            )
            for d in VD for k in Kd[d] for i in Vd_nh[d] for j in Vcp_d[d] if i != j
        ),
        name="eq:c27"
    )
    md.addConstrs(
        (
            QR[j, k] <= Ur * (1 - truck_in(d, j, k, X) - truck_in(d, j, k, Y) + Xi[j, k])
            for d in VD for k in Kd[d] for j in Vcp_d[d]
        ),
        name="eq:c30"
    )
    md.addConstrs(
        (
            QR[j, k] + q[j] <= QR[i, k] + (Ur + qjmax) * (1 - Z[i, j, k] + Xi[j, k] + Delta[j, k])
            for d in VD for k in Kd[d] for (i, j) in A_r[d]
        ),
        name="eq:c31"
    )
    md.addConstrs(
        (
            QV[j, k] <= Uv * (1 - robot_in(d, j, k) + Delta[j, k])
            for d in VD for k in Kd[d] for j in Vcp_d[d]
        ),
        name="eq:c33"
    )
    md.addConstrs(
        (
            QV[j, k] + QR[j, k] <= QV[i, k] + (Uv + Ur) * (
                2 - X.get((i, j, k), 0.0) - Xi[j, k] + Delta[j, k]
            )
            for d in VD for k in Kd[d] for i in Vd_nh[d] for j in Vp_d[d] if i != j
        ),
        name="eq:c34"
    )
    md.addConstrs(
        (
            QV[j, k] + q[j] <= QV[i, k] + QR[l, k] + (Uv + qjmax) * (
                2 - Y.get((i, j, k), 0.0) - Z.get((l, j, k), 0.0)
            )
            for d in VD for k in Kd[d] for i in Vd_nh[d] for l in Vcp_d[d] for j in Vc1_d[d]
            if i != j and l != j
        ),
        name="eq:c35"
    )
    md.addConstrs(
        (
            QV[j, k] + QR[j, k] <= QV[i, k] + QR[l, k] + (Uv + Ur) * (
                2 - X.get((i, j, k), 0.0) - Z.get((l, j, k), 0.0)
            )
            for d in VD for k in Kd[d] for i in Vd_nh[d] for l in Vcp_d[d] for j in Vp_d[d]
            if i != j and l != j
        ),
        name="eq:c36"
    )
    md.addConstrs((QR[d, k] <= 0 for d in VD for k in Kd[d]), name="eq:c38")
    md.addConstrs((QV[i, k] <= Uv for d in VD for k in Kd[d] for i in Vd_nh[d]), name="eq:c39")
    md.addConstrs((QR[i, k] <= Ur for d in VD for k in Kd[d] for i in Vd_nh[d]), name="eq:c40")

    md.addConstrs(
        (
            gp.quicksum(veh_dist_mat[i][j] * (X[i, j, k] + Y[i, j, k]) for (i, j) in A_v[d]) <= veh_dist_ub
            for d in VD for k in Kd[d]
        ),
        name="eq:c43"
    )
    md.addConstrs(
        (
            E[j, k] + rob_power_mat[i][j] <= E[i, k] + (robot_ub + e_max) * (1 - Z[i, j, k] + Delta[j, k])
            for d in VD for k in Kd[d] for (i, j) in A_r[d]
        ),
        name="eq:c44"
    )
    md.addConstrs(
        (
            rob_power_mat[i][j] <= E[i, k] + e_max * (2 - Z[i, j, k] - Delta[j, k])
            for d in VD for k in Kd[d] for (i, j) in A_r[d]
        ),
        name="eq:c45"
    )

    md.addConstrs(
        (
            TV[i, k] + veh_time_mat[i][j]
            <= TV[j, k] + M * (1 - X[i, j, k] - Y[i, j, k])
            for d in VD
            for k in Kd[d]
            for (i, j) in A_v[d]
            if j in Vcp_d[d]
        ),
        name="eq:c47"
    )

    md.update()
    md._V = V
    md._VD = VD
    md._K = K
    md._X = X
    md._Y = Y
    md._Z = Z

    md.optimize()

    if md.status == GRB.OPTIMAL or md.Runtime >= 4000:
        print("Delta:")
        for d in VD:
            for k in Kd[d]:
                for i in Vd_nh[d]:
                    if Delta[i, k].X > 0.5:
                        print(str((i, k)) + ": " + str(Delta[i, k].X), end=" ")
        print("")

        print("Xi:")
        for d in VD:
            for k in Kd[d]:
                for i in Vd_nh[d]:
                    if Xi[i, k].X > 0.5:
                        print(str((i, k)) + ": " + str(Xi[i, k].X), end=" ")
        print("")

        point_x = [point[0] for point in V_coordinates]
        point_y = [point[1] for point in V_coordinates]
        colors = ["blue", "red", "green", "black", "orange", "brown"]
        for d in VD:
            nodes = depot_with_node[d]
            for i in nodes:
                if i in VD:
                    plt.scatter(point_x[i], point_y[i], color=colors[d % len(colors)], label="Depot Points", marker="o")
                if i in VC_set:
                    plt.scatter(point_x[i], point_y[i], color=colors[d % len(colors)], label="Customer Points", marker="^")
                if i in VP_set:
                    plt.scatter(point_x[i], point_y[i], color=colors[d % len(colors)], label="Parking Points", marker="p")

        for index, pos in enumerate(V_coordinates):
            plt.annotate(index, pos, textcoords="offset points", xytext=(0, 10), ha="center")

        for d in VD:
            for k in Kd[d]:
                line_color = colors[k % len(colors)]
                print("k= " + str(k))
                for (i, j) in A_v[d]:
                    if X[i, j, k].X > 0.5:
                        print("X", i, j, "QV[" + str(i) + "," + str(k) + "]=" + str(QV[i, k].X),
                              "QR[" + str(i) + "," + str(k) + "]=" + str(QR[i, k].X))
                        plt.plot(
                            [V_coordinates[i][0], V_coordinates[j][0]],
                            [V_coordinates[i][1], V_coordinates[j][1]],
                            linestyle="-",
                            color=line_color
                        )
                    if Y[i, j, k].X > 0.5:
                        print("Y", i, j, "QV[" + str(i) + "," + str(k) + "]=" + str(QV[i, k].X),
                              "QR[" + str(i) + "," + str(k) + "]=" + str(QR[i, k].X))
                        plt.plot(
                            [V_coordinates[i][0], V_coordinates[j][0]],
                            [V_coordinates[i][1], V_coordinates[j][1]],
                            linestyle="--",
                            color=line_color
                        )
                for (i, j) in A_r[d]:
                    if Z[i, j, k].X > 0.5:
                        print("Z", i, j, "QV[" + str(i) + "," + str(k) + "]=" + str(QV[i, k].X),
                              "QR[" + str(i) + "," + str(k) + "]=" + str(QR[i, k].X))
                        plt.plot(
                            [V_coordinates[i][0], V_coordinates[j][0]],
                            [V_coordinates[i][1], V_coordinates[j][1]],
                            linestyle="-.",
                            color=line_color
                        )

        plt.savefig("figure/" + myInstance.name + ".png")
        plt.close()

    elif md.status == GRB.INFEASIBLE:
        md.computeIIS()
        md.write("model.ilp")

    gp.disposeDefaultEnv()
