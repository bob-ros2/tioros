#
# Copyright 2023 BobRos
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
from std_msgs.msg import String
from std_msgs.msg import Header
from twitchio.ext import commands
from twitchio.ext import routines
# https://twitchio.dev/en/latest/reference.message_contenthtml


class Chatbot(commands.Bot):

    def __init__(self, node):
        self.node = node

        self.node.declare_parameter('secrets', 
            os.path.join(os.path.expanduser('~'), '.secrets'))
        with open(self.node.get_parameter(
            'secrets').get_parameter_value().string_value, 'r') as f:
            token = f.read()
 
        self.node.declare_parameter('channel', 'superbob_6110')
        self.node.declare_parameter('frame_id', self.node.get_parameter(
            'channel').get_parameter_value().string_value)

        self.frame_id = self.node.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.pub_chat = self.node.create_publisher(
            String, 'chat', 10)
        self.pub_json = self.node.create_publisher(
            String, 'json', 10)

        self.sub_chat_input = self.node.create_subscription(
            String, 'chat_input', self.chat_input, 10)

        super().__init__(token=token, prefix='!', 
            initial_channels=[self.node.get_parameter(
                'channel').get_parameter_value().string_value])

        self.spin.start(self.node)


    @routines.routine(seconds=0.5)
    async def spin(self, node: Node):
        rclpy.spin_once(node, timeout_sec=0.01)


    def chat_input(self, msg):
        self.node.get_logger().info('chat_input: %s' % msg.data)
        channel = self.get_channel(self.node.get_parameter('channel')
            .get_parameter_value().string_value)
        loop = asyncio.get_event_loop()
        loop.create_task(channel.send(msg.data))


    async def player_done(self):
        self.node.get_logger().info('Finished playing sound')


    def jsonfy(self, msg, header):
        msg.data = json.dumps({
            "metadata": [
                {"key": "stamp", "value": float("%d.%09d" 
                    % (header.stamp.sec, header.stamp.nanosec))},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "tags", "value": ["chat",header.frame_id]},
                {"key": "type", "value": msg.data.split(" ")[0]},
                {"key": "user_id", "value": msg.data.split(" ")[1]},
                {"key": "user_name", "value": msg.data.split(" ")[2]},
            ],
            "data": msg.data
        })
        return msg


    def publish(self, text):
        msg = String()
        header = Header() 
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg.data = text
        self.pub_chat.publish(msg)
        self.pub_json.publish(self.jsonfy(msg, header))
        self.node.get_logger().debug(text)


    async def event_ready(self):
        # We are logged in and ready to chat and use commands...
        self.node.get_logger().info(f'Logged in as | {self.nick}')
        self.node.get_logger().info(f'User id is | {self.user_id}')
        self.publish("event_ready %d %s" % (self.user_id, self.nick))


    async def event_message(self, message):
        # Messages with echo set to True are messages sent by the bot...
        # For now we just want to ignore them
        if message.echo:
            return
        # Output contents of our message
        userdata = await self.fetch_users([message.author.name])
        self.publish("event_message %d %s %s" 
            % (userdata[0].id, message.author.name, message.content))
        # Since we have commands and are overriding the default event_message
        # We must let the bot know we want to handle and invoke our commands...
        await self.handle_commands(message)


    async def event_join(self, channel, user):
        userdata = await self.fetch_users([user.name])
        self.publish("event_join %s %s" % (userdata[0].id, user.name))
        await channel.send("Welcome @%s!" % user.name)


    @commands.command()
    async def hello(self, ctx: commands.Context):
        userdata = await self.fetch_users([ctx.author.name])
        self.publish("command %d %s hello" % (userdata[0].id, ctx.author.name))
        await ctx.send(f'Hello {ctx.author.name}!')


    @commands.command()
    async def play(self, ctx: commands.Context, *, search: str) -> None:
        #track = await sounds.Sound.ytdl_search(search)
        #self.player.play(track)
        await ctx.send(f'Now playing')


class ChatbotNode(Node):
    def __init__(self):
        super().__init__('chatbot')
        self.bot = Chatbot(self)
        self.bot.run()


def main(args=None):
    rclpy.init(args=args)
    ChatbotNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

