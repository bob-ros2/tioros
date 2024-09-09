#!/usr/bin/env python3
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

import os, sys, asyncio, signal, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
import twitchio
from twitchio.ext import commands, eventsub, routines
from .auth import credentials_from_json_file, token_from_refresh_token

class Bot(commands.Bot):
    """
    TwitchIO EventSub chat bot
    """

    def __init__(self, node):

        # prepare ROS node
        self.node = node

        # load client credentials
        self.node.declare_parameter('credentials', 
            os.path.join(os.path.expanduser('~'), '.credentials'))
        credencials = credentials_from_json_file(self.node.get_parameter(
            'credentials').get_parameter_value().string_value)
        tdata = token_from_refresh_token(
            client_id     = credencials['client_id'],
            client_secret = credencials['client_secret'],
            refresh_token =  credencials['refresh_token'])
        if not tdata:
            self.node.get_logger().error(
                'token_from_refresh_token failed')
            sys.exit(1)

        # channel params
        self.node.declare_parameter('channel', 
            os.getenv('TW_CHANNEL','superbob_6110'))
        self.node.declare_parameter('broadcaster_id', 
            os.getenv('TW_BROADCASTER_ID','123456'))
        self.node.declare_parameter('moderator_id', 
            os.getenv('TW_MODERATOR_ID',
                self.node.get_parameter(
                    'broadcaster_id').get_parameter_value().string_value))
        self.node.declare_parameter('callback_port', 
            int(os.getenv('TW_CALLBACK_PORT','4000')))
        self.node.declare_parameter('events_only', False)
        
        # create ROS publisher
        self.node.declare_parameter('frame_id', self.node.get_parameter(
            'channel').get_parameter_value().string_value)
        self.pub_chat = self.node.create_publisher(
            String, 'eventsub', 10)
        self.pub_json = self.node.create_publisher(
            String, 'json', 10)

        # init EventSub extension

        self.esbot = commands.Bot.from_client_credentials(
            client_id = credencials['client_id'],
            client_secret = credencials['client_secret'])

        self.esclient = eventsub.EventSubClient(self.esbot,
            webhook_secret = credencials['webhook_secret'],
            callback_route = credencials['callback_route'])

        # init bot with scoped user token
        super().__init__(
            token = tdata['access_token'],
            prefix = '!',
            initial_channels=[self.node.get_parameter(
                'channel').get_parameter_value().string_value])

        # handle ROS spin
        self.spin.start(self.node)


    # twitch subscriber

    async def subscribe_channel_follows_v2(self):
        try:
            await self.esclient.subscribe_channel_follows_v2(
                broadcaster = self.node.get_parameter(
                    'broadcaster_id').get_parameter_value().string_value,
                moderator = self.node.get_parameter(
                    'moderator_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_follows_v2 done')
        except Exception as e:
            self.node.get_logger().error(
                'subscribe_channel_follows_v2 failed: ' + str(e))


    async def subscribe_channel_subscriptions(self):
        try:
            await self.esclient.subscribe_channel_subscriptions(
                self.node.get_parameter('broadcaster_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_subscriptions done')
        except Exception as e:
            self.node.get_logger().error(
                'subscribe_channel_subscriptions failed: ' + str(e))


    async def subscribe_channel_raid(self):
        try:
            await self.esclient.subscribe_channel_raid(
                to_broadcaster = self.node.get_parameter(
                    'broadcaster_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_raid done')
        except Exception as e:
            self.node.get_logger().error(
                'subscribe_channel_raid failed: ' + str(e))


    async def __ainit__(self):
        self.loop.create_task(self.esclient.listen(port = self.node.get_parameter(
            'callback_port').get_parameter_value().integer_value))

        # moderator:read:followers channel:read:subscriptions chat:edit chat:read
        await self.esclient.delete_all_active_subscriptions()

        await self.subscribe_channel_follows_v2()
        await self.subscribe_channel_subscriptions()
        await self.subscribe_channel_raid()


    def shutdown(self):
        self.node.get_logger().info('shutting down EventSub bot ...')
        self.esclient.delete_all_active_subscriptions()
        self.esclient.stop()
        self.esbot.close()


    async def event_ready(self) -> None:
        self.node.get_logger().info('Bot is ready')


    @routines.routine(seconds=0.5)
    async def spin(self, node: Node):
        rclpy.spin_once(node, timeout_sec=0.01)


    def publish(self, text):
        msg = String()
        header = Header() 
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = self.node.get_parameter(
            'frame_id').get_parameter_value().string_value
        msg.data = text
        self.pub_chat.publish(msg)
        self.pub_json.publish(self.jsonfy(msg, header))
        self.node.get_logger().debug(text)


    def jsonfy(self, msg, header):
        msg.data = json.dumps({
            "metadata": [
                {"key": "stamp", "value": float("%d.%09d" 
                    % (header.stamp.sec, header.stamp.nanosec))},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "tags", "value": ["eventsub",header.frame_id]},
                {"key": "type", "value": msg.data.split(" ")[0]},
                {"key": "user_id", "value": msg.data.split(" ")[1]},
                {"key": "user_name", "value": msg.data.split(" ")[2]},
            ],
            "data": msg.data
        })
        return msg


    async def channel_send(self, msg):
        if self.node.get_parameter(
            'events_only').get_parameter_value().bool_value: 
            return
        channel = node.get_parameter(
            'channel').get_parameter_value().string_value
        channel = node.bot.get_channel(channel)
        await channel.send(msg)


class EventSubNode(Node):
    def __init__(self):
        super().__init__('eventsub')
        self.bot = Bot(self)
        for s in (signal.SIGTERM, signal.SIGINT):
            self.bot.loop.add_signal_handler(
                s, lambda s=s: asyncio.create_task(self.bot.shutdown()))
        self.bot.loop.run_until_complete(self.bot.__ainit__())


# setup

rclpy.init(args=None)
node = EventSubNode()


# eventsub handler

@node.bot.esbot.event()
async def event_eventsub_notification_followV2(
    payload: eventsub.ChannelFollowData):
    node.get_logger().info('Received followV2 event')
    await node.bot.channel_send(
        f'@{payload.data.user.name} followed, WOW! <3 Thank you!!!')
    node.bot.publish(
        f'eventsub_channelfollow {payload.data.user.id} {payload.data.user.name} '
        + str(payload))


@node.bot.esbot.event()
async def eventsub_notification_subscription(
    payload: eventsub.ChannelSubscribeData):
    node.get_logger().info('Received subscription event')
    node.bot.channel_send(
        f'@{payload.data.user.name} subscribed, WOW! <3 Thank you!!!')
    node.bot.publish(
        f'eventsub_subscription {payload.data.user.id} {payload.data.user.name} '
        + str(payload))


@node.bot.esbot.event()
async def eventsub_notification_raid(
    event: eventsub.ChannelRaidData):
    node.get_logger().info('Received raid event')
    user = event.raider
    node.bot.channel_send(
        f'@{user.name} is raiding with {event.viewer_count} people, WOW! <3 Thank you!!!')
    node.bot.channel_send(
        f'/shoutout {user.name}')
    node.bot.publish(
        f'eventsub_raid {user.id} {user.name} '
        + str(event))


# finally run

node.bot.run()
rclpy.shutdown()
