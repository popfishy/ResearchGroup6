import numpy as np
import networkx as nx
import rospy
import sys

np.set_printoptions(threshold=np.inf)
from scipy import integrate
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
import math

tra_frame = 0

class GVF_ode:
    # 初始化输入无人机id列表，无人机数量，初始坐标
    def __init__(self, plane_id, uav_num, x_coords, y_coords):
        self.id = plane_id
        self.x_coords = x_coords
        self.y_coords = y_coords

        # TODO：uav_type和id号存在映射关系
        self.uav_type = "plane"

        self.uav_num = int(uav_num)

        self.cmd = String()
        self.formation_dict = {}

        # TODO：313表示点的数量
        self.multi_local_pose = [PoseStamped() for i in range(313)]

        self.global_paths = []

        self.multi_pose_pub = [None] * int(uav_num)

        # params
        self.n = 3
        self.alpha, self.beta, self.k1, self.k2, self.k3, self.kc = 1, 1, 1, 1, 1, 1
        self.tmp = np.linspace(0, 2 * np.pi, int(uav_num) + 1)

        # these delta make a square
        self.delta1 = np.array([x_coords])
        self.delta2 = np.array([y_coords])
        self.manual_v = np.array([[0, 0, 0, 1, 1]])

        # graph
        self.A = np.roll(np.eye(int(uav_num), int(uav_num)), 1, axis=1)
        self.A = self.A + self.A.T
        self.L = 2 * np.eye(int(uav_num), int(uav_num)) - self.A
        self.G = nx.from_numpy_array(self.A)

        # ODE settomgs
        self.area = 10
        self.seed = 1
        np.random.seed(self.seed)
        self.tspan = np.array([0, 40])
        self.x1_init = np.random.rand(1, int(uav_num)) * (2 * int(uav_num)) - self.area
        self.x1_init = self.x1_init.reshape((1, int(uav_num)))
        self.x2_init = np.random.rand(1, int(uav_num)) * (2 * self.area) - self.area
        self.x2_init = self.x2_init.reshape((1, int(uav_num)))
        self.x3_init = np.random.rand(1, int(uav_num)) * (2 * self.area) - self.area
        self.x3_init = self.x3_init.reshape((1, int(uav_num)))
        self.w1_init = self.delta1  # if initial w values have the desired distances, then the performanc is better.
        self.w2_init = self.delta2  # if initial w values have the desired distances, then the performanc is better.

        # p_init
        self.p_init = []
        for i in range(int(uav_num)):
            self.p_init.append(self.x1_init[0][i])
            self.p_init.append(self.x2_init[0][i])
            self.p_init.append(self.x3_init[0][i])
            self.p_init.append(self.w1_init[0][i])
            self.p_init.append(self.w2_init[0][i])

        ##### trajectory(x y z t)
        self.trajectory_list = []
        self.p_final = None

    #
    def update_waypoint(self, path):
        # self.trajectory_list = [[0, 0, 0, 0], [1000, 1000, 50, 50], [1000, 2000, 100, 100], [0, 3000, 50, 50]]
        self.trajectory_list = path

    ### system equations
    def cal_covf(self, pos_all, n, uav_num, A, L, delta1, delta2):
        pos_all_array = np.array(pos_all)
        tmp = pos_all_array.reshape(uav_num, -1)
        tmp = np.transpose(tmp)

        w1 = tmp[n, :]
        w2 = tmp[n + 1, :]

        hatw1 = (w1 - delta1).T
        hatw2 = (w2 - delta2).T

        vec1 = np.zeros((n + 2, 1))
        vec1[n] = 1
        vec1[n + 1] = 0

        vec2 = np.zeros((n + 2, 1))
        vec2[n] = 0
        vec2[n + 1] = 1

        covf1 = np.kron(-np.dot(L, hatw1), vec1)
        covf2 = np.kron(-np.dot(L, hatw2), vec2)

        covf_all = covf1 + covf2
        return covf_all

    def cal_pfvf(self, pos_all, n, uav_num, manual_v):
        len_pos = len(pos_all)
        m_v1 = manual_v[0][0]
        m_v2 = manual_v[0][1]
        m_v3 = manual_v[0][2]
        # m_v3 = self.trajectory_list[tra_frame + 1][2]
        # m_v4 = manual_v[0][3]
        m_v4 = (self.trajectory_list[tra_frame + 1][1] - self.trajectory_list[tra_frame][1]) / self.trajectory_list[tra_frame + 1][3]
        # m_v5 = manual_v[0][4]
        m_v5 = (-(self.trajectory_list[tra_frame + 1][0] - self.trajectory_list[tra_frame][0]) / self.trajectory_list[tra_frame + 1][3])

        if uav_num != len_pos // (n + 2):
            print("Error! N is not correct!")
            return None

        pfvf_all = np.zeros((len_pos, 1))
        e_all = np.zeros((1, n * uav_num))

        for i in range(uav_num):
            j = i * (n + 2)
            l = i * n
            x1 = pos_all[j]
            x2 = pos_all[j + 1]
            x3 = pos_all[j + 2]
            w1 = pos_all[j + 3]
            w2 = pos_all[j + 4]
            scaled_w1 = self.beta * w1
            scaled_w2 = self.beta * w2

            # only need to change this part
            f1w = scaled_w1
            f2w = scaled_w2
            f3w = self.trajectory_list[tra_frame + 1][2]

            # only need to change this part
            phi1 = self.alpha * (x1 - f1w)
            phi2 = self.alpha * (x2 - f2w)
            phi3 = self.alpha * (x3 - f3w)
            sign = (-1) ** n
            v = np.array([sign * m_v5 - self.k1 * phi1,
                    sign * (- m_v4) - self.k2 * phi2,
                    - self.k3 * phi3 ,
                    sign * m_v5 + self.k1 * phi1,
                    sign * - m_v4 + self.k2 * phi2])
            
            pfvf_all[j : j + n + 2, 0] = v
            phi = np.array([phi1, phi2, phi3])
            e_all[0, l : l + n] = phi

        return pfvf_all, e_all

    def multipf(self, t, p, n, uav_num, A, L, delta1, delta2, manual_v):
        [pfvf, e_all] = self.cal_pfvf(p, n, int(uav_num), manual_v)
        copf = self.cal_covf(p, n, int(uav_num), A, L, delta1, delta2)
        vf = pfvf + self.kc * copf
        dxidt = vf
        return dxidt, e_all

    def calculate_path(self, timestep=None):
        global tra_frame
        for tra_frame in range(len(self.trajectory_list) - 1):
            ### ode45
            t0, tf = 0, self.trajectory_list[tra_frame + 1][3]
            numpoints = 313
            self.drawnum = 1
            t = np.linspace(t0, tf, numpoints)
            p_init = self.p_init
            if tra_frame > 0:
                p_init = self.p_final.T
                p_init = p_init.squeeze()
            p = np.zeros((len(t), len(p_init)))
            p[0, :] = p_init

            r = integrate.ode(self.multipf).set_integrator("dopri5")
            r.set_initial_value(p_init, t0)
            r.set_f_params(self.n, int(self.uav_num), self.A, self.L, self.delta1, self.delta2, self.manual_v)
            for i in range(1, len(t)):
                p[i, :] = r.integrate(t[i])
                if not r.successful():
                    raise RuntimeError("Could not integrate")

            self.p_final = p[numpoints - 1 :]
            self.global_paths.append(p)


# if __name__ == "__main__":

#     plane_type = sys.argv[1]
#     plane_id = int(sys.argv[2])
#     plane_num = int(sys.argv[3])
#     # rospy.init_node('GVF_ode' + str(self.id), anonymous=True)
#     rospy.init_node("GVF_ode" + str(plane_id))
#     rate = rospy.Rate(4)
#     pub = rospy.Publisher(plane_type + "_" + str(plane_id) + "/mavros/GVF_ode/pose", PoseStamped, queue_size=1)

#     gvf_ode = GVF_ode(sys.argv[1], sys.argv[2], sys.argv[3])
#     tra_frame = 0
#     tra_num = 3
#     while tra_frame < tra_num:
#         print(f"执行第 {tra_frame + 1} 次轨迹")
#         tra_frame = gvf_ode.main(tra_frame)

#     print("所有轨迹执行完毕")
