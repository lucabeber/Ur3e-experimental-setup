import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time

class FTAnalysisNode(Node):
    def __init__(self):
        super().__init__('ft_analysis_node')
        self.subscription = self.create_subscription(
            WrenchStamped,
            'ft_sensor',
            self.ft_sensor_callback,
            10)
        self.collection_time = 30  # seconds to collect data
        self.data_buffer = []
        self.collection_start_time = None
        self.output_file = 'ft_statistics.txt'
        self.shutdown_timer = None

    def ft_sensor_callback(self, msg):
        if self.collection_start_time is None:
            self.collection_start_time = time.time()

        elapsed_time = time.time() - self.collection_start_time
        self.data_buffer.append(msg.wrench)
        
        self.get_logger().info(elapsed_time)

        if elapsed_time > self.collection_time:
            self.calculate_statistics()
    
    def shutdown_node(self):
        self.get_logger().info("Node shutting down.")
        self.destroy_node()
        rclpy.shutdown()

    def calculate_statistics(self):
        data_array = np.array([[
            wrench.force.x, wrench.force.y, wrench.force.z,
            wrench.torque.x, wrench.torque.y, wrench.torque.z
        ] for wrench in self.data_buffer])

        mean_values = np.mean(data_array, axis=0)
        std_dev_values = np.std(data_array, axis=0)

        self.get_logger().info("Mean Forces: {}".format(mean_values[0:3]))
        self.get_logger().info("Mean Torques: {}".format(mean_values[3:]))
        self.get_logger().info("Std Dev Forces: {}".format(std_dev_values[0:3]))
        self.get_logger().info("Std Dev Torques: {}".format(std_dev_values[3:]))

        self.shutdown_node()
	    
            



def main(args=None):
    rclpy.init(args=args)
    ft_analysis_node = FTAnalysisNode()
    rclpy.spin(ft_analysis_node)

if __name__ == '__main__':
    main()
