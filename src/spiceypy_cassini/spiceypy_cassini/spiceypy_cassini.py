import rclpy
from rclpy.node import Node

from . import util_spiceypy_cassini

from std_msgs.msg import Float64MultiArray

class spiceypy_cassini(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/spiceypy_cassini/state', 10)

        self.get_logger().info(f"Starting spiceypy_cassini version = 0.0.1")

        #Defining inputs
        self.declare_parameter('start_t', 'Jun 20, 2004') # Time must be in format 'MMM DD, YYYY'
        self.declare_parameter('finish_t', 'Dec 1, 2005') 
        self.declare_parameter('num_steps', 1000) # Number of steps

        self.declare_parameter('publish_freq', 10.0)   
        
        self.start_t = self.get_parameter('start_t').get_parameter_value().string_value
        self.finish_t = self.get_parameter('finish_t').get_parameter_value().string_value
        self.num_steps = self.get_parameter('num_steps').get_parameter_value().integer_value

        # Defining encounter for publisher
        self.i = 0        

        #################################
        # Calling simulation function using parameters declared above
        self.res_orb =  util_spiceypy_cassini.run(self,
                                      start_t = self.start_t,
                                      finish_t = self.finish_t,
                                      num_steps = self.num_steps)
        
        self.state_msg = Float64MultiArray()
        timer_period = 1/self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        self.state_msg.data = [self.res_orb[self.i][0], self.res_orb[self.i][1], self.res_orb[self.i][2]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_orb):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = spiceypy_cassini()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()