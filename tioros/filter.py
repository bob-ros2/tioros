#
# Copyright 2023 Bob Ros
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
#

import os
import re
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from std_msgs.msg import String


class FilterNode(Node):
    """Basic topic filter node class."""

    def __init__(self):
        super().__init__('filter')

        desc_wth = 'String array with white list rules.'
        def_wth = os.getenv('TIOROS_WHITE_FILTER', '').split(',') \
            if os.getenv('TIOROS_WHITE_FILTER') else ['']
        self.declare_parameter('white_filter', def_wth, ParameterDescriptor(description=desc_wth))

        desc_blk = 'String array with blacklist rules.'
        def_blk = os.getenv('TIOROS_BLACK_FILTER', '').split(',') \
            if os.getenv('TIOROS_BLACK_FILTER') else ['']
        self.declare_parameter('black_filter', def_blk, ParameterDescriptor(description=desc_blk))

        desc_wl = "White list file. This overrides parameter white_filter.\n" \
                  "Yaml file with a list of strings containing regex rules."
        self.declare_parameter('white_list', os.getenv('TIOROS_WHITE_LIST', ""),
                               ParameterDescriptor(description=desc_wl))

        desc_bl = "Black list file. This overrides parameter black_filter.\n" \
                  "Yaml file with a list of strings containing regex rules."
        self.declare_parameter('black_list', os.getenv('TIOROS_BLACK_LIST', ""),
                               ParameterDescriptor(description=desc_bl))

        desc_sub = "Substitute regex for the string message. Expects array: ['pattern','replace']"
        def_sub = os.getenv('TIOROS_SUBSTITUTE', '').split(',') \
            if os.getenv('TIOROS_SUBSTITUTE') else ['']
        self.declare_parameter('substitute', def_sub, ParameterDescriptor(description=desc_sub))

        self.white_filter = self.get_parameter('white_filter').value
        self.black_filter = self.get_parameter('black_filter').value
        self.white_list = self.get_parameter('white_list').value
        self.black_list = self.get_parameter('black_list').value
        self.substitute = self.get_parameter('substitute').value

        if self.white_list:
            self.white_filter = self.load_yaml(self.white_list)
        if self.black_list:
            self.black_filter = self.load_yaml(self.black_list)

        self.sub = self.create_subscription(String, 'chat', self.chat_input_callback, 10)
        self.pub = self.create_publisher(String, 'chat_filtered', 10)
        self.pub_rejected = self.create_publisher(String, 'rejected', 10)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def transform(self, s):
        """Substitutes a string similar to python re.sub."""
        if len(self.substitute) == 2:
            return re.sub(self.substitute[0], self.substitute[1], s)
        return s

    def chat_input_callback(self, msg: String):
        """Filter incoming message based on white and black lists."""
        is_white = False
        for f in self.white_filter:
            if f and re.search(f, msg.data):
                is_white = True
                break

        if is_white or self.white_filter == [''] or len(self.white_filter) < 1:
            for f in self.black_filter:
                if f and re.search(f, msg.data):
                    self.get_logger().debug("Skipping %s" % msg.data)
                    self.pub_rejected.publish(msg)
                    return
            msg.data = self.transform(msg.data)
            self.pub.publish(msg)
            return
        self.pub_rejected.publish(msg)

    def load_yaml(self, filename):
        """Load YAML file and return content."""
        try:
            with open(filename, 'r') as file:
                content = yaml.load(file, Loader=yaml.FullLoader)
                self.get_logger().info("Loaded items: %d" % len(content))
            return content
        except Exception as e:
            self.get_logger().error("Loading %s: %s" % (filename, str(e)))
            return []

    def parameter_callback(self, params):
        """Parameter callback used by Dynamic Reconfigure."""
        for param in params:
            if param.name == "white_filter":
                self.white_filter = param.value
            elif param.name == "black_filter":
                self.black_filter = param.value
            elif param.name == "substitute":
                self.substitute = param.value
            elif param.name == "black_list":
                if self.black_list != param.value:
                    self.get_logger().info("Reload %s" % param.value)
                    self.black_filter = self.load_yaml(param.value)
                self.black_list = param.value
            elif param.name == "white_list":
                if self.white_list != param.value:
                    self.get_logger().info("Reload %s" % param.value)
                    self.white_filter = self.load_yaml(param.value)
                self.white_list = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    n = FilterNode()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
