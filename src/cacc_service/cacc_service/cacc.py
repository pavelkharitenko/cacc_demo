import numpy as np
from matplotlib import pyplot as plt 

# for standalone execution of this script, see final lines below
import sys

class CACC():
    """
    Implements a CACC algorithm that utilizes preceding vehicle's state and input acceleration, to compute input acceleration of ego vehicle.
    Longitudinal PD control of spacing error between ego and preceding vehicle. 
    
    Based on https://arxiv.org/abs/2305.17443.
    """

    def __init__(self):

        # initialize controller paramters
        self.dt = 0.05 # time step delta
        self.tau = 0.3 # time engine constant (tau>0)
        self.h = 1  # time headway constant (h>0)
        self.desired_distance_offset = 20 # minimal distance between ego and preceding vehicle

        # control gains
        self.K_d = 45
        self.K_p = 1

        print("CACC started")
        
    

    def calc_u(self, x, u_previous, des_acc_ego_previous, data_preceding):
        """
        Computes next input acceleration u to minimize distance error between ego and preceding vehicle.
        Variables consist of following vectors:
        State x = [position, velocity, acceleration] of ego vehicle
        Input u =  acceleration for ego vehicle
        State & input in one vector data_preceding = [position, velocity, acceleration_preceding] of preceding vehicle
        """


        # Compute error to minimize:

        # distance
        distance = data_preceding[0] - x[0] # between vehicle in front and ego vehicle

        # spacing error and derivative, using old u_t-1 and old desired acceleration 
        spacing_error = distance - self.calc_desired_distance(x[1])
        spacing_error_derivative = data_preceding[1] - x[1] - self.h * x[2] # also called speed difference

        input_of_preceding = data_preceding[2] # desired_acceleration from predicing vehicle

        # from paper equation (4), compute hu with xi and then u
        xi = self.K_p * spacing_error + self.K_d * spacing_error_derivative + input_of_preceding
        h_du = -des_acc_ego_previous + xi
        du = (1/self.h) * h_du
        next_u = u_previous + self.dt * du

        return next_u
    

    def calc_desired_distance(self, v_ego):
        """
        Computes hv (velocity-dependent desired distance) and adds to desired distance, used in controller
        """
        return self.h * v_ego + self.desired_distance_offset


    def vehicle_dynamics(self, x, u):
        """
        Optional, included so standalone mode can work. Identical to same method in VehicleDynamics node.
        To simulate EDGARs dynamics (x_t+1)
        x contains [position, velocity, acceleration]
        u - input u from controller
        """
        dt = self.dt

        dpos = x[1]* dt + x[0]
        dvel = x[2] * dt + x[1]
        dacc = dt*(1/self.tau)*(-x[2] + u) + x[2]    # a_dot = -(1/tau)a + (1/tau)u
        dx = [dpos, dvel, dacc]

        return dx

    def run_simluation_and_plot_graph(self):
        """
        Optional, for standalone simulation without ros2 and distributed nodes. Can be run as a test for this class alone.
        """
        # 1. lists to record pos, vel or acc of EDGAR
        s_list = []
        v_list = []
        a_list = []
        u_list = []

        # 2. specify initial conditions (EDGAR is parked behind Fortiss vehicle)
        s_list.append(0)
        v_list.append(0)
        a_list.append(0)
        u_list.append(0)

        # Create OBU dummy data (later from OBU node)

        timeframe = np.arange(0,14,0.05)

        obu_data_list = self.create_dummy_obu_data(timeframe)

        #self.plot_dynamics(timeframe, [[tpl[0] for tpl in obu_data_list], [tpl[1] for tpl in obu_data_list], [tpl[2] for tpl in obu_data_list]], 
        #                   "Passed by OBU", ["position","velocity", "acceleration"])

        plt.ion()
        figure, ax = plt.subplots()
        line1, = ax.plot(timeframe, timeframe)

        plt.title("Ego vehicle dynamics", fontsize=20)
 
        # setting x-axis label and y-axis label
        plt.xlabel("time in s")
        plt.ylabel("distance in meter")


        # 3. run simulation loop
        steps = len(obu_data_list)
        for i in range(1, steps):

            
            s_old = s_list[i-1]
            v_old = v_list[i-1]
            a_old = a_list[i-1]
            x_old = [s_old, v_old, a_old]

            u_old = u_list[i-1]


            line1.set_xdata(timeframe[0:i])
            line1.set_ydata(s_list)
            figure.canvas.draw()
            figure.canvas.flush_events()

            


            obu_data_catched = obu_data_list[i-1]

            # compute control value
            u = self.calc_u(x_old, u_old ,0, obu_data_catched)

            # compute next state
            x = self.vehicle_dynamics(x_old, u)
            
            s_list.append(x[0])
            v_list.append(x[1])
            a_list.append(x[2])
            u_list.append(u)

            
        # plot simulation
        print("simulation ended")
        print("total time points computed: ", len(s_list))
        self.plot_dynamics(timeframe, [s_list, v_list, a_list], "Ego vehicle response", ["position","velocity", "acceleration"])



    def create_dummy_obu_data(self, x=np.arange(0,10,0.005)):
        """
        Optional, included so standalone mode can work. Identical to same method in OBUSensor node.
        Generates dummy data for standalone version of CACC. 
        """

        preceding_pos = 20
        arr = x
        s = lambda x : 0.01*(x-2)**3 + preceding_pos
        v = lambda x : 0.03*(x-2)**2
        a = lambda x : 0.06*(x-2)
        non_zero = []
        
        for sva in [(s(t), v(t), a(t)) for t in arr]:
            
            if (sva[0] or sva[1] or sva[2]) < preceding_pos:
                non_zero.append((preceding_pos,0,0))
            else:
                non_zero.append(sva)

        return non_zero

    def plot_dynamics(self, t, state_variables, plot_name, label_list):
        """
        Plots all xi in a state vector x as a graph for each timestep in t.
        """
        plt.title(plot_name) 
        plt.xlabel("time in s") 
        plt.ylabel("distance in meter") 

        for i, state_var in enumerate(state_variables):
            
            plt.plot(t, state_var, label=label_list[i]) 
        plt.legend(loc='best')
        plt.show()

        plt.savefig('cacc_standalone_results.png')


# for standalone execution of CACC algorithm. Simulates the OBU-CACC-VehicleDynamics loop as a for-loop above instead of using all three nodes.
if(len(sys.argv) == 2):
            if sys.argv[1] == "standalone":
                CACC().run_simluation_and_plot_graph()  