import rclpy
from car_control_pkg.ros_communicator import (
    ROSCommunicator,
)
from car_control_pkg.data_processor import DataProcessor
from car_control_pkg.nav_processing import Nav2Processing
from car_control_pkg.car_controller import CarController
import time


def main():
    rclpy.init()
    ros_communicator = ROSCommunicator()
    data_processor = DataProcessor(ros_communicator)
    nav_processing = Nav2Processing(ros_communicator, data_processor)
    car_controller = CarController(ros_communicator, nav_processing, data_processor)

    try:
        while rclpy.ok():
            rclpy.spin_once(
                ros_communicator,
            )  # 讓節點保持運行
            car_controller.car_control()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        ros_communicator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
