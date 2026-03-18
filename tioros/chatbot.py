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
import json
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from twitchio.ext import commands, routines
from .auth import credentials_from_json_file, token_from_refresh_token


class Chatbot(commands.Bot):

    def __init__(self, node):
        self.node = node
        self._is_ready = False

        # 1. Parameter: Direct token (highest priority)
        self.node.declare_parameter('token', os.getenv('TIOROS_TOKEN', ''))

        # 2. Parameter: Credentials file path (fallback)
        home_creds = os.path.join(os.path.expanduser('~'), '.credentials')
        default_creds = os.getenv('TIOROS_CREDENTIALS', home_creds)
        self.node.declare_parameter('credentials', default_creds)

        token = self.node.get_parameter('token').get_parameter_value().string_value

        if not token:
            path = self.node.get_parameter('credentials').get_parameter_value().string_value
            if os.path.exists(path):
                # Try to load as JSON first
                data = credentials_from_json_file(path)
                if data:
                    # Case A: Combined credentials JSON
                    if 'access_token' in data:
                        token = str(data['access_token']).strip()
                    elif 'refresh_token' in data and 'client_id' in data:
                        # Case B: Fetch token using refresh_token if available
                        self.node.get_logger().info("Fetching token from refresh_token...")
                        tdata = token_from_refresh_token(
                            client_id=data['client_id'],
                            client_secret=data.get('client_secret'),
                            refresh_token=data['refresh_token'])
                        if tdata and 'access_token' in tdata:
                            token = tdata['access_token'].strip()
                else:
                    # Case C: It's not JSON, so treat it as plain text token file
                    with open(path, 'r') as f:
                        token = f.read().strip()

        if not token or "\n" in token or "\r" in token:
            self.node.get_logger().error("Invalid or missing Twitch token! Aborting.")
            raise ValueError("Twitch token contains forbidden characters or is empty.")

        # Basic config
        default_channel = os.getenv('TIOROS_CHANNEL', 'superbob_6110')
        self.node.declare_parameter('channel', default_channel)

        default_frame_id = os.getenv('TIOROS_FRAME_ID', self.node.get_parameter(
            'channel').get_parameter_value().string_value)
        self.node.declare_parameter('frame_id', default_frame_id)

        self.node.declare_parameter('prefix', os.getenv('TIOROS_PREFIX', '!'))

        self.frame_id = self.node.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.pub_chat = self.node.create_publisher(String, 'chat', 10)
        self.pub_json = self.node.create_publisher(String, 'json', 10)

        self.sub_chat_input = self.node.create_subscription(
            String, 'chat_input', self.chat_input, 10)

        super().__init__(
            token=token,
            prefix=self.node.get_parameter('prefix').get_parameter_value().string_value,
            initial_channels=[self.node.get_parameter(
                'channel').get_parameter_value().string_value])

        self.spin.start(self.node)
        self.healthcheck.start()

    @routines.routine(seconds=30.0)
    async def healthcheck(self):
        # Skip healthcheck until the bot has initialized at least once
        if not self._is_ready:
            return

        is_actually_alive = False
        if hasattr(self, '_connection') and self._connection:
            is_actually_alive = self._connection.is_alive

        self.node.get_logger().debug(f"[Twitch Healthcheck] Alive: {is_actually_alive}")

        if not is_actually_alive:
            self.node.get_logger().debug("[Twitch Healthcheck] Attempting to reconnect...")
            try:
                # connect() is safe if run() is already running but connection dropped
                await self.connect()
            except Exception as e:
                self.node.get_logger().debug(f"[Twitch Healthcheck] Reconnect failed: {e}")

    @routines.routine(seconds=0.5)
    async def spin(self, node: Node):
        rclpy.spin_once(node, timeout_sec=0.01)

    def chat_input(self, msg):
        channel_name = self.node.get_parameter('channel').get_parameter_value().string_value
        channel = self.get_channel(channel_name)
        if channel:
            # Using loop.create_task for thread safety if called from ROS callback
            asyncio.run_coroutine_threadsafe(channel.send(msg.data), self.loop)

    def jsonfy(self, msg, header):
        stamp = float("%d.%09d" % (header.stamp.sec, header.stamp.nanosec))
        msg.data = json.dumps({
            "metadata": [
                {"key": "stamp", "value": stamp},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "tags", "value": ["chat", header.frame_id]},
                {"key": "type", "value": msg.data.split(" ")[0]},
                {"key": "user_id", "value": msg.data.split(" ")[1]},
                {"key": "user_name", "value": msg.data.split(" ")[2]},
            ],
            "data": msg.data
        })
        return msg

    def publish(self, text):
        msg = String()
        new_header = Header()
        new_header.stamp = self.node.get_clock().now().to_msg()
        new_header.frame_id = self.frame_id
        msg.data = text
        self.pub_chat.publish(msg)
        self.pub_json.publish(self.jsonfy(msg, new_header))

    async def event_ready(self):
        if not self._is_ready:
            self.node.get_logger().info(f'Logged in as | {self.nick}')
            self.publish("event_ready %d %s" % (self.user_id, self.nick))
            self._is_ready = True

    async def event_message(self, message):
        if message.echo:
            return
        userdata = await self.fetch_users([message.author.name])
        log_msg = "event_message %d %s %s" % (
            userdata[0].id, message.author.name, message.content
        )
        self.publish(log_msg)
        await self.handle_commands(message)

    @commands.command()
    async def hello(self, ctx: commands.Context):
        await ctx.send(f'Hello {ctx.author.name}!')


class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot')
        self.bot = Chatbot(self)
        self.bot.run()


def main(args=None):
    rclpy.init(args=args)
    try:
        ChatbotNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
