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
import time
from functools import partial

import aiohttp

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String

from twitchio.ext import commands, routines

from .auth import (
    credentials_from_json_file,
    save_credentials_to_json_file,
    token_from_refresh_token,
    validate_token,
)


class Chatbot(commands.Bot):
    """Twitch chatbot implementation using twitchio and ROS 2."""

    def __init__(self, node):
        """Initialize the Chatbot with a ROS 2 node."""
        self.node = node
        self._is_ready = False
        self._client_id = None
        self._client_secret = None
        self._creds_data = None
        self._creds_path = None
        self._token_expires_at = 0.0
        self._last_connect_attempt = 0.0

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
                    self._creds_data = data
                    self._creds_path = path
                    self._client_id = data.get('client_id')
                    self._client_secret = data.get('client_secret')
                    if 'access_token' in data:
                        token = str(data['access_token']).strip()
                    elif 'refresh_token' in data and self._client_id:
                        self.node.get_logger().info(
                            'Fetching initial token from refresh_token...')
                        tdata = token_from_refresh_token(
                            client_id=self._client_id,
                            client_secret=self._client_secret,
                            refresh_token=data['refresh_token'])
                        if tdata and 'access_token' in tdata:
                            token = tdata['access_token'].strip()
                            expires_in = tdata.get('expires_in', 14400)
                            self._token_expires_at = time.time() + expires_in
                            self._creds_data['access_token'] = token
                            if 'refresh_token' in tdata:
                                self._creds_data['refresh_token'] = (
                                    tdata['refresh_token'])
                            save_credentials_to_json_file(
                                self._creds_data, self._creds_path)
                else:
                    with open(path, 'r') as f:
                        token = f.read().strip()
                        self._creds_data = None
                        self._creds_path = path

        # Validate initial token if we loaded an existing access_token
        if token and not self._token_expires_at:
            valid, vdata = validate_token(token)
            if valid:
                expires_in = vdata.get('expires_in', 14400)
                self._token_expires_at = time.time() + expires_in
                self.node.get_logger().info(
                    f'Initial token valid. Expires in ~{expires_in}s.')
            elif self._creds_data and self._creds_data.get('refresh_token'):
                self.node.get_logger().warn(
                    f'Initial token invalid ({vdata}). Refreshing...')
                tdata = token_from_refresh_token(
                    client_id=self._client_id,
                    client_secret=self._client_secret,
                    refresh_token=self._creds_data['refresh_token'])
                if tdata and 'access_token' in tdata:
                    token = tdata['access_token'].strip()
                    expires_in = tdata.get('expires_in', 14400)
                    self._token_expires_at = time.time() + expires_in
                    self._creds_data['access_token'] = token
                    if 'refresh_token' in tdata:
                        self._creds_data['refresh_token'] = (
                            tdata['refresh_token'])
                    save_credentials_to_json_file(
                        self._creds_data, self._creds_path)

        clean_token = token.replace('oauth:', '').strip() if token else ''
        if not clean_token or '\n' in clean_token or '\r' in clean_token:
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

        channel_to_join = self.node.get_parameter(
            'channel').get_parameter_value().string_value

        super().__init__(
            token=clean_token,
            client_id=self._client_id,
            client_secret=self._client_secret,
            prefix=self.node.get_parameter(
                'prefix').get_parameter_value().string_value,
            initial_channels=[channel_to_join])

        # Hook TwitchIO WebSocket connection to ensure safe reconnects
        if hasattr(self, '_connection') and self._connection:
            self._orig_ws_connect = self._connection._connect
            self._connection._connect = self._safe_ws_connect
            self._connection._keep_alive = self._safe_keep_alive

        initial_ttl = max(int(self._token_expires_at - time.time()), 300) \
            if self._token_expires_at else 14400
        self._apply_new_token(clean_token, expires_in=initial_ttl)

        self.spin.start(self.node)
        self.healthcheck.start()

    def _apply_new_token(
        self,
        new_token: str,
        expires_in: int = 14400,
        new_refresh_token: str = None
    ):
        """Synchronize the new token across all internal components."""
        clean_token = new_token.replace('oauth:', '').strip()
        self._token = clean_token

        # Update TwitchIO HTTP client
        if hasattr(self, '_http') and self._http:
            self._http.token = clean_token
            self._http.app_token = clean_token

        # Update TwitchIO WebSocket / IRC connection
        if hasattr(self, '_connection') and self._connection:
            self._connection._token = clean_token

        self._token_expires_at = time.time() + max(int(expires_in), 300)

        if hasattr(self, '_creds_data') and self._creds_data:
            self._creds_data['access_token'] = clean_token
            if new_refresh_token:
                self._creds_data['refresh_token'] = new_refresh_token
            if hasattr(self, '_creds_path') and self._creds_path:
                save_credentials_to_json_file(
                    self._creds_data, self._creds_path)

    async def refresh_access_token(self):
        """Fetch a fresh access token using the stored refresh token."""
        if not hasattr(self, '_creds_data') or not self._creds_data:
            self.node.get_logger().error(
                '[Twitch Token] Cannot refresh: no credentials data loaded.')
            return None

        refresh_token = self._creds_data.get('refresh_token')
        if not refresh_token or not self._client_id:
            self.node.get_logger().error(
                '[Twitch Token] Cannot refresh: missing refresh_token.')
            return None

        self.node.get_logger().info(
            '[Twitch Token] Refreshing access token from Twitch...')
        try:
            tdata = await self.loop.run_in_executor(
                None,
                token_from_refresh_token,
                self._client_id,
                self._client_secret,
                refresh_token)

            if tdata and 'access_token' in tdata:
                new_token = tdata['access_token'].strip()
                expires_in = tdata.get('expires_in', 14400)
                new_refresh = tdata.get('refresh_token')
                self._apply_new_token(
                    new_token=new_token,
                    expires_in=expires_in,
                    new_refresh_token=new_refresh)
                self.node.get_logger().info(
                    f'[Twitch Token] Token refreshed! Expires in '
                    f'~{expires_in}s.')
                return new_token
            self.node.get_logger().error(
                f'[Twitch Token] Refresh failed with response: {tdata}')
            return None
        except Exception as e:
            self.node.get_logger().error(
                f'[Twitch Token] Exception during refresh: {e}')
            return None

    async def event_token_expired(self):
        """Handle event when OAuth token expires (TwitchIO HTTP hook)."""
        self.node.get_logger().warn(
            '[Twitch HTTP] Access token expired. Refreshing...')
        new_token = await self.refresh_access_token()
        return new_token

    async def event_raw_data(self, data):
        """Intercept raw data to detect authentication failure notices."""
        if not isinstance(data, str):
            return
        if 'NOTICE * :Login authentication failed' in data or \
           'NOTICE * :Login unsuccessful' in data:
            self.node.get_logger().error(
                '[Twitch IRC] Auth failure detected in IRC stream!')
            self._token_expires_at = 0.0
            await self.refresh_access_token()

    async def _safe_keep_alive(self):
        """Run robust keep-alive loop handling errors without crashing."""
        ws_conn = self._connection
        await ws_conn._ws_ready_event.wait()
        ws_conn._ws_ready_event.clear()

        if not ws_conn._last_ping:
            ws_conn._last_ping = time.time()

        while (ws_conn._websocket and not ws_conn._websocket.closed and
               not ws_conn._reconnect_requested):
            try:
                msg = await ws_conn._websocket.receive()
            except Exception as e:
                self.node.get_logger().warn(
                    f'[Twitch WS] Websocket receive exception: {e}')
                break

            if msg.type in (aiohttp.WSMsgType.CLOSED,
                            aiohttp.WSMsgType.ERROR,
                            aiohttp.WSMsgType.CLOSING):
                self.node.get_logger().warn(
                    f'[Twitch WS] Websocket closed/error (type={msg.type}): '
                    f'{msg.extra}')
                break

            if msg.type == aiohttp.WSMsgType.TEXT:
                data = msg.data
                if data and isinstance(data, str):
                    ws_conn.dispatch('raw_data', data)
                    events = data.split('\r\n')
                    for event in events:
                        if not event:
                            continue
                        task = asyncio.create_task(
                            ws_conn._process_data(event))
                        task.add_done_callback(
                            partial(ws_conn._task_callback, event))
                        ws_conn._background_tasks.append(task)

        self.node.get_logger().info(
            '[Twitch WS] Keep-alive loop exited. Scheduling reconnect...')
        ws_conn._background_tasks.append(
            asyncio.create_task(ws_conn._connect()))

    async def _safe_ws_connect(self):
        """Throttle and wrap TwitchIO _connect to refresh tokens if needed."""
        # 1. Throttle rapid reconnect attempts
        now = time.time()
        elapsed = now - self._last_connect_attempt
        if elapsed < 3.0:
            delay = 3.0 - elapsed
            self.node.get_logger().info(
                f'[Twitch Reconnect] Throttling reconnect, '
                f'waiting {delay:.1f}s')
            await asyncio.sleep(delay)
        self._last_connect_attempt = time.time()

        # 2. Check token freshness before connecting
        if hasattr(self, '_token_expires_at') and self._token_expires_at:
            if time.time() >= self._token_expires_at - 600:
                self.node.get_logger().info(
                    '[Twitch Reconnect] Token near expiry. '
                    'Refreshing first...')
                await self.refresh_access_token()

        # 3. Guard initial_channels against #TWITCHIOFAILURE removal
        target_channel = self.node.get_parameter(
            'channel').get_parameter_value().string_value
        if hasattr(self, '_connection') and self._connection:
            if not self._connection._initial_channels:
                self._connection._initial_channels = [target_channel]

        try:
            return await self._orig_ws_connect()
        except Exception as e:
            self.node.get_logger().error(
                f'[Twitch Reconnect] Connection error: {e}')
            # Schedule another attempt after backoff
            await asyncio.sleep(5.0)
            return asyncio.create_task(self._safe_ws_connect())

    @routines.routine(seconds=60.0)
    async def healthcheck(self):
        """Perform periodic healthcheck and proactive token renewal."""
        # 1. Proactive Token Refresh (10 minutes before expiration)
        if hasattr(self, '_token_expires_at') and self._token_expires_at:
            if time.time() >= self._token_expires_at - 600:
                self.node.get_logger().info(
                    '[Twitch Healthcheck] Proactive refresh '
                    'triggered (near expiry)...')
                await self.refresh_access_token()

        if not self._is_ready:
            return

        # 2. Connection State Check
        is_ready = False
        is_alive = False
        if hasattr(self, '_connection') and self._connection:
            is_ready = self._connection.is_ready.is_set()
            is_alive = self._connection.is_alive

        if not is_ready or not is_alive:
            self.node.get_logger().warn(
                f'[Twitch Healthcheck] Connection degraded '
                f'(alive={is_alive}, ready={is_ready}). Checking status...')

            valid, vdata = await self.loop.run_in_executor(
                None, validate_token, self._token)
            if not valid:
                self.node.get_logger().warn(
                    f'[Twitch Healthcheck] Token invalid ({vdata}). '
                    'Refreshing...')
                await self.refresh_access_token()
            elif isinstance(vdata, dict) and vdata.get('expires_in'):
                self._token_expires_at = time.time() + vdata['expires_in']

            target_channel = self.node.get_parameter(
                'channel').get_parameter_value().string_value
            if hasattr(self, '_connection') and self._connection:
                if not self._connection._initial_channels:
                    self._connection._initial_channels = [target_channel]

                # Actively recover if connection is dead
                if not is_alive:
                    self.node.get_logger().info(
                        '[Twitch Healthcheck] Actively triggering reconnect '
                        'because socket is dead...')
                    self._connection.is_ready.clear()
                    if (self._connection._keeper and
                            not self._connection._keeper.done()):
                        self._connection._keeper.cancel()
                    asyncio.create_task(self._connection._connect())

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
        self.node.get_logger().info(f'Logged in as | {self.nick}')
        self.publish('event_ready %d %s' % (self.user_id, self.nick))
        self._is_ready = True

    async def event_error(self, error, data=None):
        """Handle errors occurring in the bot."""
        self.node.get_logger().error(f'[Twitch] Error: {error}')
        if data:
            self.node.get_logger().error(f'[Twitch] Error data: {data}')
        err_str = str(error).lower()
        if 'unauthorized' in err_str or 'authentication' in err_str:
            self.node.get_logger().warn(
                '[Twitch] Auth error detected. Refreshing token...')
            await self.refresh_access_token()

    async def event_join(self, channel, user):
        """Handle user join events."""
        if user.name.lower() == self.nick.lower():
            return
        try:
            # Using ID 0 to avoid Twitch API Rate-Limits
            log_msg = 'event_join 0 %s' % (user.name)
            self.publish(log_msg)
        except Exception as e:
            self.node.get_logger().error(f'Error handling event_join: {e}')

    async def event_message(self, message):
        """Handle incoming Twitch chat messages."""
        if message.echo and not os.getenv(
            'TIOROS_ALLOW_SELF', '1'
        ).lower() in ('1', 'true', 'yes'):
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
