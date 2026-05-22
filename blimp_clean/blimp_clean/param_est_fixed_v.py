import numpy as np
from rclpy.node import Node
from blimp_msgs.msg import MotorMsg, OptiTrackPose
from std_msgs.msg import Bool, Float32MultiArray

from scipy.linalg import solve_discrete_are
from scipy.signal import place_poles
import os
import threading

import tinympc
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.stats import chi2

'''
Things done:
- Create a simulation to ensure that the logic was functioning
- Made an inital parameter estimator using real-world estimates
- Implemented an MPC controller via estimates to see performance
- Measurement gate via test statistic
- Used NIS to tune Q and R based on the consistency
    - Decreased the variance of R when NIS was consistently outside the bounds
    -
'''



RHO_AIR = 1.225 #kg/m^3
RHO_HE = 0.165 # kg/m^3
M_chassis = 51/1000 #kg
M_AZ = 0.0311 #kg
g = 9.81

# 50 cm tall
# 91.44 cm across

# 'Kv_Cd' — estimate Kv + Cd  (V fixed)
# 'Kv_V'  — estimate Kv + V   (Cd fixed)
# 'Cd'    — estimate Cd only  (V and Kv fixed)
ESTIMATE_MODE = 'Cd'

#OG VALUES
# V = 0.0508
# Kv = 0.15
# Cd = 0.01

# Kv_Cd mode: V is fixed, Kv and Cd are estimated
V = 0.046
# V = 0.0508
Kv_guess = 0.1307
# Kv_guess = 0.15
Cd_guess = 0.0300

# Kv_V mode: Cd is fixed, Kv and V are estimated
Cd_fixed = 0.0300
V_guess  = 0.046
# V_guess = 0.0508

# Cd mode: V and Kv are both fixed
Kv_fixed = 0.1307
# Kv_fixed = 0.15


MAX_VOLTAGE = 0.8

# Module-level defaults — imported by plot_test_statistic.py
N_TS = 10
MEASURE_V = False


class Model(object):
    def __init__(self, d0=0.0):
        '''
        Initialize model update class

        '''
        self.d0 = d0
        v_init = V_guess if ESTIMATE_MODE == 'Kv_V' else V
        self.m = M_chassis + RHO_HE*v_init + M_AZ
        self.m_RB = self.m - M_AZ

    def get_FG(self, X, d, dt, d0):

        u = d

        if ESTIMATE_MODE == 'Cd':
            # 3-state: [z, zdot, Cd]
            z, zdot, Cd = X

            F = np.array([
                [1, dt, 0],
                [0, 1 - Cd/self.m*dt, -zdot/self.m*dt],
                [0, 0, 1],
            ])
            G = np.array([
                [0],
                [2/self.m*Kv_fixed*dt],
                [0],
            ])
            return F, G

        # 4-state modes: [z, zdot, Kv, p_b]
        z, zdot, Kv, p_b = X

        if ESTIMATE_MODE == 'Kv_V':
            # p_b is V; update mass from current volume estimate
            cur_V = p_b
            self.m = M_chassis + RHO_HE*cur_V + M_AZ
            self.m_RB = self.m - M_AZ

            N = (RHO_AIR - RHO_HE)*g*cur_V - M_chassis*g + 2*Kv*u - Cd_fixed*zdot
            dN_dV = (RHO_AIR - RHO_HE)*g
            dm_dV = RHO_HE

            # Quotient rule, numerator is N denominator is m so (N'm - m'N) / m^2
            dV_entry = (dN_dV*self.m - N*dm_dV) / self.m**2

            # State order: [z, zdot, Kv, V]
            F = np.array([
                [1, dt, 0, 0],
                [0, 1 - Cd_fixed/self.m*dt, 2/self.m*u*dt, dV_entry*dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        else:
            # p_b is Cd; mass is fixed constant V
            Cd = p_b

            # State order: [z, zdot, Kv, Cd]
            # Dynamics: m*v_dot = (RHO_AIR-RHO_HE)*g*V - M_chassis*g + 2*Kv*u - Cd*zdot
            F = np.array([
                [1, dt, 0, 0],
                [0, 1 - 1/self.m * Cd * dt, 2/self.m*u*dt, -zdot/self.m*dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])

        # G maps thrust-voltage process noise into velocity row via 2*Kv*u.
        G = np.array([
            [0],
            [2/self.m*Kv*dt],
            [0],
            [0],
        ])

        return F, G

class ParamEstimation(Node):
    def __init__(self, ns, com_port):
        super().__init__('param_estimation',namespace=ns)
        self.ns = ns
        self.com = com_port

        # ROS Setup
        self.create_subscription(OptiTrackPose,f'optitrack_node/pose',self.ekf,5)
        self.create_subscription(Bool, f'start_calibration', self.start, 2)
        self.motor_pub = self.create_publisher(MotorMsg,f'motor_cmd',5) #Goal publisher to control testing mode
        self.covar_pub = self.create_publisher(Float32MultiArray, f'covariance',5) # These update the graphs in the GUI
        self.pred_pub = self.create_publisher(Float32MultiArray, f'state_est',5)

        # Initializers
        self.run_calibration = False
        self.finished_calibration = False
        self.measure_v = MEASURE_V



        self.d0 = 0.0
        self.N_avg = 1
        self.N_ts = N_TS
        self.N_update_goal = 1

        self.count_measurements = 0

        if ESTIMATE_MODE == 'Kv_V':
            self.X = [0.0, 0.0, Kv_guess, V_guess]   # [z, zdot, Kv, V]
        elif ESTIMATE_MODE == 'Cd':
            self.X = [0.0, 0.0, Cd_guess]             # [z, zdot, Cd]
        else:
            self.X = [0.0, 0.0, Kv_guess, Cd_guess]  # [z, zdot, Kv, Cd]
        self.model = Model()

        #Initialize controller
        # self.controller = LQR()
        self.controller = MPC(5,self)
        self.alt_goal = 1.5
        self.alt_goal_target = 1.6

        # Tracking
        self.last_time = None
        self.last_z = None
        self.update_lock = threading.Lock()
        self.saved = False

        self.t0 = None
        self.dt = None

        # Plotting
        self.state_estimates = []
        self.covariance_estimates = []
        self.test_statistics = []
        self.avg_test_statistics = []
        self.innovations = []

        self.last_P = None

        self.zs = []
        self.goal_zs = []

        self.init_params()

    def init_params(self):
        if ESTIMATE_MODE == 'Cd':
            # 3-state: [z, zdot, Cd]
            if self.measure_v:
                self.H = np.array([[1, 0, 0],
                                    [0, 1, 0]])
                self.R = np.diag([0.003**2, 0.1**2])
            else:
                self.H = np.array([[1, 0, 0]])
                self.R = np.array([[0.003**2]])
            self.Q = np.eye(1)*2.75**2
            self.P = np.diag([0.003**2, 0.1**2, 0.045**2])
        else:
            # 4-state modes
            if self.measure_v:
                self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
                self.R = np.diag([0.003**2, 0.1**2])
                if ESTIMATE_MODE == 'Kv_V':
                    self.Q = np.eye(1)*0.03**2
                    self.P = np.diag([0.003**2, 0.01**2, 0.09**2, 0.0075**2])
                else:
                    self.Q = np.eye(1)*2.75**2
                    self.P = np.diag([0.003**2, 0.1**2, 0.19**2, 0.05**2])
            else:
                self.H = np.array([[1, 0, 0, 0]])
                self.R = np.array([[0.003**2]])
                if ESTIMATE_MODE == 'Kv_V':
                    self.Q = np.eye(1)*2.95**2
                    self.P = np.diag([0.003**2, 0.3**2, 0.09**2, 0.07**2])
                else:
                    self.Q = np.eye(1)*2.75**2
                    self.P = np.diag([0.003**2, 0.1**2, 0.09**2, 0.075**2])



    def _save_plots(self, out_dir: str) -> None:
        state_arr = np.array(self.state_estimates)   # (N, 4)
        covar_arr = np.array(self.covariance_estimates)  # (N, 4, 4)
        avg_ts = np.array(self.avg_test_statistics)
        t = np.arange(len(state_arr))

        # --- NIS plot ---
        if avg_ts.size > 0:
            dof = 2 if self.measure_v else 1
            df_sum = self.N_ts * dof
            lo = chi2.ppf(0.025, df_sum) / self.N_ts
            hi = chi2.ppf(0.975, df_sum) / self.N_ts
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.axhspan(lo, hi, color='tab:green', alpha=0.2, label='95% band')
            ax.axhline(dof, color='tab:gray', linestyle=':', label=f'E[χ²]={dof}')
            ax.plot(avg_ts, color='tab:blue', label='rolling NIS')
            ax.set_xlabel('update index'); ax.set_ylabel('avg χ² statistic')
            measure_str = 'P' if self.measure_v else 'R'
            ax.set_title(f'NIS — N_ts={self.N_ts}, dof={dof} ({"z,zdot" if self.measure_v else "z only"}), last {measure_str} shown')
            ax.legend(); ax.grid(alpha=0.3)
            fig.savefig(os.path.join(out_dir, 'nis.png'), dpi=150)
            plt.close(fig)

        # --- Parameter estimates ---
        if state_arr.ndim == 2 and state_arr.shape[1] == 3:
            # Cd-only mode: single parameter
            fig, ax = plt.subplots(figsize=(9, 3))
            ax.plot(t, state_arr[:, 2], color='tab:red')
            ax.set_ylabel('Cd'); ax.set_xlabel('update index'); ax.grid(alpha=0.3)
            fig.suptitle('Parameter estimate (Cd)')
            fig.savefig(os.path.join(out_dir, 'param_estimates.png'), dpi=150)
            plt.close(fig)

            if covar_arr.ndim == 3:
                fig, ax = plt.subplots(figsize=(9, 3))
                ax.plot(t, np.sqrt(covar_arr[:, 2, 2]), color='tab:red')
                ax.set_ylabel('σ(Cd)'); ax.set_xlabel('update index'); ax.grid(alpha=0.3)
                fig.suptitle('Parameter std dev from P')
                fig.savefig(os.path.join(out_dir, 'param_stddev.png'), dpi=150)
                plt.close(fig)

        elif state_arr.ndim == 2 and state_arr.shape[1] == 4:
            p_b_label = 'V' if ESTIMATE_MODE == 'Kv_V' else 'Cd'
            fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
            axes[0].plot(t, state_arr[:, 2], color='tab:orange')
            axes[0].set_ylabel('Kv'); axes[0].grid(alpha=0.3)
            axes[1].plot(t, state_arr[:, 3], color='tab:red')
            axes[1].set_ylabel(p_b_label); axes[1].set_xlabel('update index'); axes[1].grid(alpha=0.3)
            fig.suptitle(f'Parameter estimates (Kv, {p_b_label})')
            fig.savefig(os.path.join(out_dir, 'param_estimates.png'), dpi=150)
            plt.close(fig)

            # --- Parameter std devs from P diagonal ---
            if covar_arr.ndim == 3:
                fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
                axes[0].plot(t, np.sqrt(covar_arr[:, 2, 2]), color='tab:orange')
                axes[0].set_ylabel('σ(Kv)'); axes[0].grid(alpha=0.3)
                axes[1].plot(t, np.sqrt(covar_arr[:, 3, 3]), color='tab:red')
                axes[1].set_ylabel(f'σ({p_b_label})'); axes[1].set_xlabel('update index'); axes[1].grid(alpha=0.3)
                fig.suptitle('Parameter std dev from P')
                fig.savefig(os.path.join(out_dir, 'param_stddev.png'), dpi=150)
                plt.close(fig)

        # --- Altitude tracking ---
        if len(self.zs) > 0:
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot(self.zs, color='tab:blue', label='z')
            ax.plot(self.goal_zs, color='tab:orange', linestyle='--', label='goal z')
            ax.set_xlabel('measurement index'); ax.set_ylabel('altitude (m)')
            ax.set_title('Altitude tracking during calibration')
            ax.legend(); ax.grid(alpha=0.3)
            fig.savefig(os.path.join(out_dir, 'altitude.png'), dpi=150)
            plt.close(fig)

            # --- Altitude error ---
            zs_arr = np.array(self.zs)
            goal_arr = np.array(self.goal_zs)
            error = zs_arr - goal_arr
            fig, ax = plt.subplots(figsize=(9, 4))
            ax.plot(error, color='tab:purple', label='z - goal z')
            ax.axhline(0, color='tab:gray', linestyle=':', linewidth=1.0)
            ax.set_xlabel('measurement index'); ax.set_ylabel('altitude error (m)')
            ax.set_title('Altitude tracking error during calibration')
            ax.legend(); ax.grid(alpha=0.3)
            fig.savefig(os.path.join(out_dir, 'altitude_error.png'), dpi=150)
            plt.close(fig)

        self.get_logger().info(f'Plots saved to {out_dir}')

    def start(self,msg):
        self.run_calibration = msg.data

    def ekf(self, msg):
        if not self.finished_calibration:
            self.count_measurements += 1
            if self.last_z is not None:
                
                dt = msg.time - self.last_time

                v = (msg.z - self.last_z) / dt
                if abs(v) > 1.5:
                    return

                if self.measure_v:
                    measurement = np.array([msg.z,v])
                else:
                    measurement = msg.z
                

                # Updating initial position until calibration starts
                if not self.run_calibration:
                    self.X[0] = msg.z
                    self.X[1] = v

                


                if self.run_calibration:
                    self.get_logger().info(f'Current velocity: {round(self.X[1],3)}')
                    with self.update_lock:
                        t0 = time.time()
                        
                        if self.count_measurements % self.N_update_goal == 0:
                            # instead of jumping setpoint, ramp it
                            self.alt_goal = 1.5
                            # if self.count_measurements % self.N_update_goal == 0:
                            #     # Flip target when we get close enough (hysteresis on the target, not the ramp)
                            #     if self.alt_goal_target == 1.6 and msg.z > 1.55:
                            #         self.alt_goal_target = 1.2
                            #     elif self.alt_goal_target == 1.2 and msg.z < 1.25:
                            #         self.alt_goal_target = 1.6

                            # # Smoothly ramp alt_goal toward the target every callback (not just every 10)
                            # ramp_step = 0.01   # m per callback ≈ 0.5 m/s setpoint rate at 100 Hz
                            # if self.alt_goal < self.alt_goal_target:
                            #     self.alt_goal = min(self.alt_goal_target, self.alt_goal + ramp_step)
                            # elif self.alt_goal > self.alt_goal_target:
                            #     self.alt_goal = max(self.alt_goal_target, self.alt_goal - ramp_step)

                        # Get voltage output
                        u = self.controller.get_control([msg.z,self.X[1]], np.array([self.alt_goal, 0.0]))
                        run_ekf = True

                        cmd = MotorMsg()
                        cmd.id = msg.id
                        cmd.com = self.com
                        cmd.voltages = Float32MultiArray(data=list(np.array([0.0,0.0,u,-u,0.0,0.0])))
                        self.motor_pub.publish(cmd) 

                        if run_ekf:

                            Xk = np.array(self.X.copy())

                            # Predict — state size and acceleration formula depend on mode
                            Xnext = Xk.copy()
                            Xnext[0] = Xk[1]  # zdot
                            if ESTIMATE_MODE == 'Cd':
                                n = 3
                                Cd = Xk[2]
                                xdot = Xk[1]
                                Xnext[1] = 1/self.model.m * ((RHO_AIR-RHO_HE)*g*V - M_chassis*g + 2*Kv_fixed*u - Cd*xdot)
                                Xnext[2] = 0  # Cd is constant
                            else:
                                n = 4
                                x, xdot, Kv, p_b = Xk
                                if ESTIMATE_MODE == 'Kv_V':
                                    cur_V = p_b
                                    cur_m = M_chassis + RHO_HE*cur_V + M_AZ
                                    Xnext[1] = 1/cur_m * ((RHO_AIR-RHO_HE)*g*cur_V - M_chassis*g + 2*Kv*u - Cd_fixed*xdot)
                                else:
                                    Cd = p_b
                                    Xnext[1] = 1/self.model.m * ((RHO_AIR-RHO_HE)*g*V - M_chassis*g + 2*Kv*u - Cd*xdot)
                                Xnext[2] = 0  # parameters are constant
                                Xnext[3] = 0

                            Xp = Xk + dt*Xnext  # Predicted state
                            F, G = self.model.get_FG(Xp, u, dt, self.d0)
                            Pp = F@self.P@F.T + G@self.Q@G.T  # Predicted covariance

                            # Kalman gain
                            S = self.H@Pp@self.H.T + self.R
                            K = Pp@self.H.T@np.linalg.inv(S)

                            # Innovation
                            nu = measurement - self.H@Xp
                            test_statistic = nu.T @ np.linalg.inv(S) @ nu
                            # 95% chi2 threshold
                            if self.measure_v:
                                chi2_threshold = 5.991  # 2 DOF
                            else:
                                chi2_threshold = 3.841

                            chi2_threshold = float('inf')

                            self.get_logger().info(f'Test statistic: {test_statistic}')
                            if test_statistic < chi2_threshold:  # Measurement rejection
                                self.innovations.append(nu.squeeze())

                                #Update
                                self.X = Xp + K @ nu
                                self.P = (np.eye(n) - K@self.H)@Pp@(np.eye(n)-K@self.H).T + K@self.R@K.T

                                self.X[2] = max(1e-6, self.X[2])  # Cd (Cd mode) or Kv (4-state modes)
                                if n == 4:
                                    self.X[3] = max(1e-6, self.X[3])  # Cd or V must be positive

                                self.state_estimates.append(self.X)
                                self.covariance_estimates.append(self.P)
                                self.test_statistics.append(test_statistic)

                                if len(self.test_statistics) > self.N_ts:
                                    NIS = np.mean(self.test_statistics[-self.N_ts:])
                                    self.avg_test_statistics.append(NIS)

                                if len(self.innovations) > self.N_ts:
                                    window = self.innovations[-self.N_ts:]
                                    
                                    mean_inn = np.mean(window)
                                    autocorr = np.corrcoef(window[:-1], window[1:])[0,1]

                                    self.get_logger().info(f'Mean innovation: {mean_inn}, Autocorrelation: {autocorr}')

                                    if len(self.avg_test_statistics)>500: #abs(mean_inn) < 1e-3 and abs(autocorr) < 0.05:
                                        self.finished_calibration = True
                                        self.finished_time = msg.time
                                        if ESTIMATE_MODE == 'Kv_V':
                                            self.get_logger().info(f'Estimated Kv: {self.X[2]}, V: {self.X[3]}')
                                        elif ESTIMATE_MODE == 'Cd':
                                            self.get_logger().info(f'Estimated Cd: {self.X[2]}')
                                        else:
                                            self.get_logger().info(f'Estimated Kv: {self.X[2]}, Cd: {self.X[3]}')                        

                                        for i in range(10):
                                            cmd = MotorMsg()
                                            cmd.id = msg.id
                                            cmd.com = self.com
                                            cmd.voltages = Float32MultiArray(data=list(np.array([0.0,0.0,0.0,0.0,0.0,0.0])))
                                            self.motor_pub.publish(cmd) 
                                        return
                                
                                self.last_P = self.P

                                # Publish motor command and visualization messages
                                self.covar_pub.publish(msg=Float32MultiArray(data=list(self.P.flatten())))
                                self.pred_pub.publish(msg=Float32MultiArray(data=list(self.X.flatten())))
                            else:
                                # self.X[:2] = (Xp + K @ nu)[:2]
                                self.X[:2] = [msg.z, v]
                                # self.P[:2,:2] = ((np.eye(4) - K@self.H)@Pp@(np.eye(4)-K@self.H).T + K@self.R@K.T)[:2,:2]
                                pass

            self.zs.append(msg.z)
            self.goal_zs.append(self.alt_goal)
        
        else:

            if not self.saved:
                os.makedirs(self.ns, exist_ok=True)
                self.get_logger().info(f'Saving... {len(self.avg_test_statistics)}')
                np.save(self.ns + '/state_estimates.npy', self.state_estimates)
                np.save(self.ns + '/covariance_estimates.npy', self.covariance_estimates)
                np.save(self.ns + '/finished_time.npy', self.finished_time)
                np.save(self.ns + '/test_statistics.npy', self.test_statistics)
                np.save(self.ns + '/avg_test_statistics.npy', self.avg_test_statistics)
                np.save(self.ns + '/zs.npy', self.zs)
                np.save(self.ns + '/goal_zs.npy', self.goal_zs)
                self._save_plots(self.ns)
                self.saved = True

        self.last_time = msg.time
        self.last_z = msg.z



class MPC(object):
    '''
    MPC Control using tinyMPC
    Pros: Fast, Very effective
    Cons: Not realistic for decentralized system with cheap hardware
    '''
    def __init__(self,N,node):

        self.node = node
        #Discretize continuous A,B matrices
        dt = 1/(100/node.N_avg)
        # Ad, Bd, _, _, _ = \
        #             cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt, method='zoh')
        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = m - M_AZ

        A = np.array([
            [1, dt],
            [0, 1 - Cd_guess/m*dt]
        ])
        B = np.array([
            [0],
            [dt/m]
        ])

        Fb = (RHO_AIR-RHO_HE)*g*V
        self.node.get_logger().info(f'Estimated net buoyancy force: {Fb}')
        u0 = M_chassis*g - Fb
        self.node.get_logger().info(f'Diff between gravity and buoyancy: {u0}')

        Q_alt = np.diag([10.0,5.0])
        R_alt = np.array([[5.0]])

        self.Ad = A
        self.Bd = B
        self.Q = np.asarray(Q_alt, dtype=np.float64)
        self.R = np.asarray(R_alt, dtype=np.float64)
        self.N = N
        self.rho = 1.0

        self.solver = tinympc.TinyMPC()
        # Protects TinyMPC C++ state from concurrent solve/setup under MultiThreadedExecutor
        self.solver_lock = threading.Lock()
        # self.solver.set_x_ref([self.node.x_goal[0],self.node.pitch_goal[0],self.node.x_goal[1],self.pitch_goal[1]])
        self.u0 = [u0]
        self.solver.setup(self.Ad,self.Bd,self.Q,self.R,self.N,rho=self.rho,verbose=False)
        self.solver.set_u_ref(np.array(self.u0))

    def update_u0(self,new_u0):
        self.u0 = new_u0

    def get_control(self,pose,goal):

        state = np.array(pose[:2])
        local_goal = goal.copy() # dont change goal array

        with self.solver_lock:
            self.solver.set_x0(state)
            self.solver.set_u_ref(np.array(self.u0))
            self.solver.set_x_ref(local_goal)
            thrust = self.solver.solve()['controls'][0]

        u = thrust/(2*Kv_guess)

        return float(np.clip(u, -MAX_VOLTAGE, MAX_VOLTAGE))
