# Copyright 2022 iwatake2222
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import random
import time


class MinimalPublisher(Node):

    def __init__(self, toipc_name: str, interval_sec: float = 1.0):
        super().__init__('minimal_publisher_' + toipc_name)
        self.publisher_ = self.create_publisher(String, toipc_name, 10)
        timer_period = interval_sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        if random.randint(0, 50) == 0:
            self.get_logger().info('Delay occur!!')
            time.sleep(self.timer.timer_period_ns / 1e9 * random.random())

        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class MinimalSubscriber(Node):

    def __init__(self, toipc_name: str):
        super().__init__('minimal_subscriber_' + toipc_name)
        self.subscription = self.create_subscription(
            String,
            toipc_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    exec = MultiThreadedExecutor()

    node_list: list[Node] = []
    for i in [10, 50, 100, 500]:
        interval_sec = i / 1000
        minimal_publisher = MinimalPublisher(f'topic_{i}_ms', interval_sec)
        minimal_subscriber = MinimalSubscriber(f'topic_{i}_ms')
        exec.add_node(minimal_publisher)
        exec.add_node(minimal_subscriber)
        node_list.append(minimal_publisher)
        node_list.append(minimal_subscriber)

    exec.spin()

    exec.shutdown()
    for node in node_list:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
