#!/usr/bin/env python3
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
import sys
import asyncio
import signal
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from twitchio.ext import commands, eventsub, routines
from .auth import credentials_from_json_file, token_from_refresh_token


class Bot(commands.Bot):
    """TwitchIO EventSub chat bot."""

    def __init__(self, node):
        self.node = node

        # load client credentials
        cred_path = os.path.join(os.path.expanduser('~'), '.credentials')
        default_credentials = os.getenv('TIOROS_CREDENTIALS', cred_path)
        self.node.declare_parameter('credentials', default_credentials)
        credentials = credentials_from_json_file(self.node.get_parameter(
            'credentials').get_parameter_value().string_value)
        tdata = token_from_refresh_token(
            client_id=credentials['client_id'],
            client_secret=credentials['client_secret'],
            refresh_token=credentials['refresh_token'])
        if not tdata:
            self.node.get_logger().error('token_from_refresh_token failed')
            sys.exit(1)

        # channel params
        default_channel = os.getenv('TIOROS_CHANNEL', 'superbob_6110')
        self.node.declare_parameter('channel', default_channel)

        default_broadcaster_id = os.getenv('TIOROS_BROADCASTER_ID', '123456')
        self.node.declare_parameter('broadcaster_id', default_broadcaster_id)

        default_moderator_id = os.getenv('TIOROS_MODERATOR_ID', self.node.get_parameter(
            'broadcaster_id').get_parameter_value().string_value)
        self.node.declare_parameter('moderator_id', default_moderator_id)

        default_callback_port = int(os.getenv('TIOROS_CALLBACK_PORT', '4000'))
        self.node.declare_parameter('callback_port', default_callback_port)

        default_events_only = os.getenv('TIOROS_EVENTS_ONLY', 'False').lower() == 'true'
        self.node.declare_parameter('events_only', default_events_only)

        # create ROS publisher
        default_frame_id = os.getenv('TIOROS_FRAME_ID', self.node.get_parameter(
            'channel').get_parameter_value().string_value)
        self.node.declare_parameter('frame_id', default_frame_id)

        self.pub_chat = self.node.create_publisher(String, 'eventsub', 10)
        self.pub_json = self.node.create_publisher(String, 'json', 10)

        # init EventSub extension
        self.esbot = commands.Bot.from_client_credentials(
            client_id=credentials['client_id'],
            client_secret=credentials['client_secret'])

        self.esclient = eventsub.EventSubClient(
            self.esbot,
            webhook_secret=credentials['webhook_secret'],
            callback_route=credentials['callback_route'])

        # init bot with scoped user token
        super().__init__(
            token=tdata['access_token'],
            prefix='!',
            initial_channels=[self.node.get_parameter(
                'channel').get_parameter_value().string_value])

        # handle ROS spin
        self.spin.start(self.node)

    async def subscribe_channel_follows_v2(self):
        try:
            await self.esclient.subscribe_channel_follows_v2(
                broadcaster=self.node.get_parameter(
                    'broadcaster_id').get_parameter_value().string_value,
                moderator=self.node.get_parameter(
                    'moderator_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_follows_v2 done')
        except Exception as e:
            msg = 'subscribe_channel_follows_v2 failed: '
            self.node.get_logger().error(msg + str(e))

    async def subscribe_channel_subscriptions(self):
        try:
            await self.esclient.subscribe_channel_subscriptions(
                self.node.get_parameter('broadcaster_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_subscriptions done')
        except Exception as e:
            msg = 'subscribe_channel_subscriptions failed: '
            self.node.get_logger().error(msg + str(e))

    async def subscribe_channel_raid(self):
        try:
            await self.esclient.subscribe_channel_raid(
                to_broadcaster=self.node.get_parameter(
                    'broadcaster_id').get_parameter_value().string_value)
            self.node.get_logger().info('subscribe_channel_raid done')
        except Exception as e:
            self.node.get_logger().error('subscribe_channel_raid failed: ' + str(e))

    async def __ainit__(self):
        port = self.node.get_parameter('callback_port').get_parameter_value().integer_value
        self.loop.create_task(self.esclient.listen(port=port))

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
        new_header = Header()
        new_header.stamp = self.node.get_clock().now().to_msg()
        new_header.frame_id = self.node.get_parameter(
            'frame_id').get_parameter_value().string_value
        msg.data = text
        self.pub_chat.publish(msg)
        self.pub_json.publish(self.jsonfy(msg, new_header))
        self.node.get_logger().debug(text)

    def jsonfy(self, msg, header):
        stamp = float("%d.%09d" % (header.stamp.sec, header.stamp.nanosec))
        msg.data = json.dumps({
            "metadata": [
                {"key": "stamp", "value": stamp},
                {"key": "frame_id", "value": header.frame_id},
                {"key": "tags", "value": ["eventsub", header.frame_id]},
                {"key": "type", "value": msg.data.split(" ")[0]},
                {"key": "user_id", "value": msg.data.split(" ")[1]},
                {"key": "user_name", "value": msg.data.split(" ")[2]},
            ],
            "data": msg.data
        })
        return msg

    async def channel_send(self, msg):
        if self.node.get_parameter('events_only').get_parameter_value().bool_value:
            return
        channel_name = self.node.get_parameter('channel').get_parameter_value().string_value
        channel = self.bot.get_channel(channel_name)
        if channel:
            await channel.send(msg)


class EventSubNode(Node):
    def __init__(self):
        super().__init__('eventsub')
        self.bot = Bot(self)
        for s in (signal.SIGTERM, signal.SIGINT):
            self.bot.loop.add_signal_handler(
                s, lambda s=s: asyncio.create_task(self.bot.shutdown()))

        # Register EventSub handlers
        @self.bot.esbot.event()
        async def event_eventsub_notification_followV2(payload: eventsub.ChannelFollowData):
            self.get_logger().info('Received followV2 event')
            await self.bot.channel_send(f'@{payload.data.user.name} followed! Thank you!')
            text = f'eventsub_channelfollow {payload.data.user.id} {payload.data.user.name} ' \
                   + str(payload)
            self.bot.publish(text)

        @self.bot.esbot.event()
        async def eventsub_notification_subscription(payload: eventsub.ChannelSubscribeData):
            self.get_logger().info('Received subscription event')
            await self.bot.channel_send(f'@{payload.data.user.name} subscribed! Thank you!')
            text = f'eventsub_subscription {payload.data.user.id} {payload.data.user.name} ' \
                   + str(payload)
            self.bot.publish(text)

        @self.bot.esbot.event()
        async def eventsub_notification_raid(event: eventsub.ChannelRaidData):
            self.get_logger().info('Received raid event')
            user = event.raider
            msg = f'@{user.name} is raiding with {event.viewer_count} people! Thank you!'
            await self.bot.channel_send(msg)
            await self.bot.channel_send(f'/shoutout {user.name}')
            self.bot.publish(f'eventsub_raid {user.id} {user.name} ' + str(event))

        self.bot.loop.run_until_complete(self.bot.__ainit__())


def main(args=None):
    rclpy.init(args=args)
    node = EventSubNode()
    node.bot.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
