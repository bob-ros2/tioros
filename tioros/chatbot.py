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

"""Module for the Tioros Twitch chatbot ROS 2 node."""

import asyncio
import json
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String

from twitchio.ext import commands, routines

from .auth import credentials_from_json_file, token_from_refresh_token


class Chatbot(commands.Bot):
    """Twitch chatbot implementation using twitchio and ROS 2."""

    def __init__(self, node):
        """Initialize the Chatbot with a ROS 2 node."""
        self.node = node
        self._is_ready = False
        self._client_id = None
        self._client_secret = None

        # 1. Parameter: Direct token (highest priority)
        self.node.declare_parameter('token', os.getenv('TIOROS_TOKEN', ''))

        # 2. Parameter: Credentials file path (fallback)
        home_creds = os.path.join(os.path.expanduser('~'), '.credentials')
        default_creds = os.getenv('TIOROS_CREDENTIALS', home_creds)
        self.node.declare_parameter('credentials', default_creds)

        token_param = self.node.get_parameter('token')
        token = token_param.get_parameter_value().string_value

        if not token:
            creds_param = self.node.get_parameter('credentials')
            path = creds_param.get_parameter_value().string_value
            if os.path.exists(path):
                data = credentials_from_json_file(path)
                if data:
                    self._client_id = data.get('client_id')
                    self._client_secret = data.get('client_secret')
                    if 'access_token' in data:
                        token = str(data['access_token']).strip()
                    elif 'refresh_token' in data and self._client_id:
                        self.node.get_logger().info(
                            'Fetching token from refresh_token...')
                        tdata = token_from_refresh_token(
                            client_id=self._client_id,
                            client_secret=self._client_secret,
                            refresh_token=data['refresh_token'])
                        if tdata and 'access_token' in tdata:
                            token = tdata['access_token'].strip()
                else:
                    with open(path, 'r') as f:
                        token = f.read().strip()

        if not token or '\n' in token or '\r' in token:
            self.node.get_logger().error(
                'Invalid or missing Twitch token! Aborting.')
            raise ValueError(
                'Twitch token contains forbidden characters or is empty.')

        # Basic config
        default_channel = os.getenv('TIOROS_CHANNEL', 'superbob_6110')
        self.node.declare_parameter('channel', default_channel)

        default_frame_id = os.getenv(
            'TIOROS_FRAME_ID',
            self.node.get_parameter(
                'channel').get_parameter_value().string_value)
        self.node.declare_parameter('frame_id', default_frame_id)

        self.node.declare_parameter(
            'prefix', os.getenv('TIOROS_PREFIX', '!'))

        self.frame_id = self.node.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.pub_chat = self.node.create_publisher(String, 'chat', 10)
        self.pub_json = self.node.create_publisher(String, 'json', 10)

        self.sub_chat_input = self.node.create_subscription(
            String, 'chat_input', self.chat_input, 10)

        super().__init__(
            token=token,
            client_id=self._client_id,
            client_secret=self._client_secret,
            prefix=self.node.get_parameter(
                'prefix').get_parameter_value().string_value,
            initial_channels=[self.node.get_parameter(
                'channel').get_parameter_value().string_value])

        self.spin.start(self.node)
        self.healthcheck.start()

    @routines.routine(seconds=30.0)
    async def healthcheck(self):
        """Perform a periodic healthcheck to ensure the connection is alive."""
        if not self._is_ready:
            return

        is_actually_alive = False
        if hasattr(self, '_connection') and self._connection:
            is_actually_alive = self._connection.is_alive

        if not is_actually_alive:
            self.node.get_logger().warn(
                '[Twitch Healthcheck] Connection lost. Reconnecting...')
            try:
                await self.connect()
            except Exception as e:
                self.node.get_logger().error(
                    f'[Twitch Healthcheck] Reconnect failed: {e}')

    @routines.routine(seconds=0.5)
    async def spin(self, node: Node):
        """Spin the ROS 2 node."""
        rclpy.spin_once(node, timeout_sec=0.01)

    def chat_input(self, msg):
        """Handle incoming chat messages from ROS to be sent to Twitch."""
        channel_name = self.node.get_parameter(
            'channel').get_parameter_value().string_value
        channel = self.get_channel(channel_name)
        if channel:
            asyncio.run_coroutine_threadsafe(channel.send(msg.data), self.loop)

    def jsonfy(self, msg, header):
        """Wrap the message data in a JSON structure with metadata."""
        stamp = float('%d.%09d' % (header.stamp.sec, header.stamp.nanosec))
        msg.data = json.dumps({
            'metadata': [
                {'key': 'stamp', 'value': stamp},
                {'key': 'frame_id', 'value': header.frame_id},
                {'key': 'tags', 'value': ['chat', header.frame_id]},
                {'key': 'type', 'value': msg.data.split(' ')[0]},
                {'key': 'user_id', 'value': msg.data.split(' ')[1]},
                {'key': 'user_name', 'value': msg.data.split(' ')[2]},
            ],
            'data': msg.data
        })
        return msg

    def publish(self, text):
        """Publish a message to the ROS chat and json topics."""
        msg = String()
        new_header = Header()
        new_header.stamp = self.node.get_clock().now().to_msg()
        new_header.frame_id = self.frame_id
        msg.data = text
        self.pub_chat.publish(msg)
        self.pub_json.publish(self.jsonfy(msg, new_header))

    async def event_ready(self):
        """Handle the event when the bot is logged in and ready."""
        if not self._is_ready:
            self.node.get_logger().info(f'Logged in as | {self.nick}')
            self.publish('event_ready %d %s' % (self.user_id, self.nick))
            self._is_ready = True

    async def event_join(self, user):
        """Handle user join events."""
        if user.name.lower() == self.nick.lower():
            return
        try:
            # Using ID 0 to avoid Twitch API Rate-Limits
            log_msg = 'event_join 0 %s join' % (user.name)
            self.publish(log_msg)
        except Exception as e:
            self.node.get_logger().error(f'Error handling event_join: {e}')

    async def event_message(self, message):
        """Handle incoming Twitch chat messages."""
        if message.echo:
            return

        try:
            userdata = await self.fetch_users([message.author.name])
            user_id = userdata[0].id if userdata else 0
            log_msg = 'event_message %d %s %s' % (
                user_id, message.author.name, message.content
            )
            self.publish(log_msg)
            await self.handle_commands(message)
        except Exception as e:
            self.node.get_logger().error(f'Error handling event_message: {e}')

    @commands.command()
    async def hello(self, ctx: commands.Context):
        """Handle the hello command."""
        await ctx.send(f'Hello {ctx.author.name}!')


class ChatbotNode(Node):
    """ROS 2 Node that wraps the Twitch chatbot."""

    def __init__(self):
        """Initialize the ChatbotNode."""
        super().__init__('chatbot')
        self.bot = Chatbot(self)
        self.bot.run()


def main(args=None):
    """Entry point for the chatbot node."""
    rclpy.init(args=args)
    try:
        ChatbotNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
