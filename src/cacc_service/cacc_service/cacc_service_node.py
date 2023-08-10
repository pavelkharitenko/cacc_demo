# ros2 related imports
import rclpy
from rclpy.node import Node

# import message format from 'cacc_interface' ros2 package
from cacc_interfaces.msg import OBUdata, ControlInput, DynamicUpdate

# use CACC class
from .cacc import CACC

class CACC_Service(Node):
    """
    Node for executing CACC algorithm. 
    Instantiates CACC class, subscribes to OBU sensor node. Computes and publishes input u to vehicle dynamics node.
    Subribes to vehicle_dynamics node to get next state x.
    """

    def __init__(self):
        super().__init__('cacc_service')

        # simulation related parameters
        self.i = 0 # for sim iterations (used below)
        self.last_vehicle_state = [0, 0, 0] # initial vehicle state is pos, velocity and acceleration all 0
        self.old_input = 0 # controller needs u_t-1, in the beginning, u_t-1 is 0
        self.desired_acceleration = 0
        
        # use cacc class to execute CACC algorithm
        self.cacc = CACC()

        # create publisher to send computed input acceleration u to vehicle model
        self.publisher_ = self.create_publisher(ControlInput, 'acceleration_topic', 10)

        # subscribe to data published by a obu sensor node
        self.subscription_obu = self.create_subscription(OBUdata,'obu_topic', self.process_obu_data, 10)
        self.subscription_obu  # prevent unused variable warning

        # subscribe to data published by vehicle model (next state x_t+1)
        self.subscription_dynamics = self.create_subscription(DynamicUpdate,'dynamic_model_topic', self.update_vehicle_state, 10)
        self.subscription_dynamics  # prevent unused variable warning


    def update_vehicle_state(self, update_msg):
        """
        Executed after receiving next state x_t+1 from vehicle model.
        Updating vehicle state to new.
        """
        self.last_vehicle_state = update_msg.statevector

        # log event (optional)
        self.get_logger().info('Vehicle state update from Dynamic Model node received: "%s"' % self.last_vehicle_state)


    def process_obu_data(self, input_msg):
        """
        Executed after receiving new obu sensor data. 
        Unpacks obu sensor data from preceding vehicle, and computes next acceleration u.
        Publishes new input acceleration to vehicle_dynamics node via topic.
        """
        
        # Unpack obu data [pos, vel, input_acceleration] of preceding vehicle
        obu_data = input_msg.statevector

        # Compute input acceleration u for ego vehicle
        next_u = self.cacc.calc_u(self.last_vehicle_state, self.old_input, self.desired_acceleration, obu_data)

        # update u_t
        self.old_input = next_u

        # Create message of format ControlInput, (X, U), and publish to 'acceleration_topic'
        vehicle_dynamics_update_msg = ControlInput()
        vehicle_dynamics_update_msg.statevector = [float(self.last_vehicle_state[0]), float(self.last_vehicle_state[1]), float(self.last_vehicle_state[2])]
        vehicle_dynamics_update_msg.inputacceleration = next_u
        
        self.publisher_.publish(vehicle_dynamics_update_msg)

        # log event (optional)
        self.get_logger().info('Received new data from OBU and sent to vehicle model: "%s"' % next_u)



def main(args=None):

    # Create and run node
    rclpy.init(args=args)
    cacc_service = CACC_Service()
    rclpy.spin(cacc_service)

    # Destroy the node explicitly
    cacc_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()