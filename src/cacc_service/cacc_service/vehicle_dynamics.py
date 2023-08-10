# ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cacc_interfaces.msg import ControlInput, DynamicUpdate

# imports for plotting
import numpy as np
from matplotlib import pyplot as plt 

class VehicleDynamics(Node):
    """
    Node simulating ego vehicle model. 

    Subscribes to 'acceleration_topic' where it receives control input data and vehicle state.
    Publishes to 'dynamic_model_topic' after computing the next state X_t+1.

    Creates a plot after simulation and saves at root of current ros2 workspace (in ./../src/cacc_service)
    """

    def __init__(self):
        super().__init__('vehicle_dynamics')

        # 1. create lists to record simulation: pos, vel, acc and input of Ego Vehicle
        self.s_list = []
        self.v_list = []
        self.a_list = []
        self.u_list = []

        # simulation related data
        self.dt = 0.05
        self.timeframe = np.arange(0,14,0.05) # simulation parameters, 14 seconds, step size dt
        self.tau = 0.3
        self.i = 1 # start at 1, as initial values at 0 passed from cacc_service
        

        # 2. specify initial conditions (Ego vehicle is parked behind preceding vehicle)
        self.s_list.append(0)
        self.v_list.append(0)
        self.a_list.append(0)
        self.u_list.append(0)


        # 3. create publisher and subscriber (to communicate with cacc_service node)
        self.publisher = self.create_publisher(DynamicUpdate, 'dynamic_model_topic', 10)
        self.subscription = self.create_subscription(ControlInput,'acceleration_topic', self.update_vehicle_model, 10)
        self.subscription  # prevent unused variable warning

        # 4. Log setup complete
        self.get_logger().info("vehicle model started and waiting...")


    def update_vehicle_model(self, update_msg):
        """
        Executed after receiving new control input data from cacc_service.
        """

        # Unpack input u and statevector from update_msg, run dynamic function.
        statevector = update_msg.statevector
        input_a = update_msg.inputacceleration
        next_x = self.vehicle_dynamics(statevector, input_a, self.dt)

        # Add result (next state X_t+1) to lists for plotting and next state back to cacc_service 
        self.s_list.append(next_x[0])
        self.v_list.append(next_x[1])
        self.a_list.append(next_x[2])
        self.u_list.append(input_a) 

        # publish new state to cacc_service
        new_state = DynamicUpdate()
        new_state.statevector = next_x
        self.publisher.publish(new_state)

        # Log state update
        self.get_logger().info('Received new vehicle data "%s"' % input_a)

        # Create final plot and store at root of current ros2 workspace.
        self.i += 1
        if self.i == len(self.timeframe)-1:
            self.plot_dynamics(self.timeframe[:-1], [self.s_list, self.v_list, self.a_list], "Simulation Result Ego Vehicle", ["position", "velocity", "acceleration"])


    def plot_dynamics(self, t, state_variables, plot_name, label_list):
        """
        Plots all state variables X_i of a list state vector X, at timeframe t.
        Length of timeframe t_0 to t_f and length of state variables X_0 to X_f needs to be equal.

        Usage example:

        y = create_dummy_obu_data(np.arange(0,7,0.005))

        plot_dynamics(np.arange(0,7,0.005), [[dp[0] for dp in y], [dp[1] for dp in y], [dp[2] for dp in y]])
        """

        # create plt plot
        plt.title(plot_name) 
        plt.xlabel("time in s") 
        plt.ylabel("distance in meter") 

        # create for each state variable a graph on plot, insert names to each state variable plot from lables list
        for i, state_var in enumerate(state_variables):
            plt.plot(t, state_var, label=label_list[i]) 
        
        plt.legend(loc='best')
        #plt.savefig('cacc-sim-result.png')
        plt.show()

    
    def vehicle_dynamics(self, x, u, dt):
        """
        To simulate ego vehicle longitudinal dynamics (x_t+1). Computes next state X_t+1.
        x contains [position, velocity, acceleration]
        u - input acceleration u from cacc controller
        """

        dpos = x[1]* dt + x[0] # new position is old position and dt times velocity
        dvel = x[2] * dt + x[1] # new velocity is old velocity and dt times acceleration
        dacc = dt*(1/self.tau)*(-x[2] + u) + x[2] # acceleration from paper, a_dot = -(1/tau)a + (1/tau)u
        dx = [dpos, dvel, dacc]

        return dx


def main(args=None):
    # Create and run node
    rclpy.init(args=args)
    vehicle_dynamics = VehicleDynamics()
    rclpy.spin(vehicle_dynamics)

    # Destroy the node explicitly
    vehicle_dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()