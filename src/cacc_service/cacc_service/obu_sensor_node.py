# ros2 related imports
import rclpy
from rclpy.node import Node
# import message formats from cacc_interfaces package
from cacc_interfaces.msg import OBUdata

from matplotlib import pyplot as plt 
# else 
import numpy as np

class OBUSensor(Node):
    """
    Simulates OBU data coming from preceding vehicle, as generated dummy data. OBU data messages consist of [poisiton, velocity, input_acceleration] of preceding vehicle.
    
    Sends dummy data as following: Generates discrete points of some acceleration function and publishes points every (timer_period)-seconds to 'obu_topic'.

    Timeframe of simulation needs to be same as in vehicle_dynamics node.
    """

    def __init__(self):
        super().__init__('obu_sensor')

        # simulation related parameters
        # change together with timeframe in vehicle_dynamics.py node
        self.timeframe = np.arange(0,14,0.05) # last entry is the step size dt

        # obu data generation
        self.obu_data_list = self.generate_obu_data(self.timeframe)
        self.i = 0

        # create publisher to send obu data every couple seconds to cacc_service
        self.publisher = self.create_publisher(OBUdata, 'obu_topic', 10)

        # timer simulates receiving rate from preceding vehicle
        timer_period = 0.05  # seconds, can be set to arbitrary value, howevery could represent simulation dt.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):
        """
        Executed when new obu data should be available. Timer executes this function every timer_period seconds.
        Creates OBU message format to send to other ros2 nodes. Publishes on 'obu_topic'
        """

        if(self.i < len(self.obu_data_list)): # publish as long as obu data is available

            # get current (i-th) data point of preceding vehicle (pos, vel, acc)
            s,v,a = self.obu_data_list[self.i]
            #print(s)  # log next positon of preceding vehicle (optional)

            # prepare OBUdata message format
            msg = OBUdata()
            msg.statevector = [float(s),float(v),float(a)]

            # publish OBUdata message to 'obu_topic'
            self.publisher.publish(msg)
            # log published data (optional)
            self.get_logger().info('Publishing data: "%s"' % msg.statevector)

            # proceed to next data point.
            self.i += 1


    def generate_obu_data(self, x=np.arange(0,10,0.005)):
        """
        Generates position, velocity and acceleation curves of preceding vehicle. 
        Currently position is cubic "f(x) = 0.01x^3 + s_0", where x is time t_i

        Generates discrete points from t_0 to t_f and every dt steps.
        Default timeframe is from 0 t0 10 seconds and dt = 0.005 unless x is passed.
        """
        
        # model movement functions of preceding vehicle
        preceding_pos = 20 # start position of preceding vehicle (20m)

        # functions for position, velocity and acceleration (v and afirst and second derivative)
        s = lambda x : 0.01*(x-2)**3 + preceding_pos
        v = lambda x : 0.03*(x-2)**2
        a = lambda x : 0.06*(x-2)

        # apply functions on every timestep t_i in timeframe, and add as tuple to list
        non_zero = []
        for sva in [(s(t), v(t), a(t)) for t in x]:
            # remove negative values if continous function as x^3
            if (sva[0] or sva[1] or sva[2]) < preceding_pos:
                # set value to start position if negative or less that start position
                non_zero.append((preceding_pos,0,0))
            else:
                non_zero.append(sva)

        # plot data sent by to other nodes (optional)
        #plt.plot(self.timeframe, [i[0] for i in non_zero])
        #plt.title("Sample plot of x_0 sent by OBU")
        #plt.show()

        return non_zero


def main(args=None):
    # Create and run node
    rclpy.init(args=args)
    obu_sensor = OBUSensor()
    rclpy.spin(obu_sensor)

    # Destroy the node explicitly
    obu_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()