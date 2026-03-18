# ROS Package [tioros](https://github.com/bob-ros2/tioros)
[![ROS CI](https://github.com/bob-ros2/tioros/actions/workflows/ros-ci.yml/badge.svg)](https://github.com/bob-ros2/tioros/actions/workflows/ros-ci.yml)

[TwitchIO](https://twitchio.dev/en/stable) ROS Chatbot Nodes. 
A [ROS 2](https://ros.org) - Twitch Chat Bridge.

## Installation Prerequisites

Use the package manager [pip3](https://pip.pypa.io/en/stable/) 
to install the below dependencies.

```bash
pip3 install twitchio requests
```

## Setup Package

```bash
# run in your ros2_ws/src folder
git clone https://github.com/bob-ros2/tioros.git
cd ..
colcon build --packages-select tioros
. install/setup.bash
```

## Authentication Methods

There are two primary ways to authenticate with Twitch, depending on the node you are using. You can store both in a unified `~/.credentials` file:

1. **OAuth Flow (Traditional IRC Chatbot)**:
   * **What is it?**: A single long-living Access Token.
   * **Storage**: A plain text file containing just the token (e.g., `oauth:xxxxxxxxxxxxx`) **OR** a `access_token` field in a JSON file.
   * **Usage**: Standard for sending/receiving chat messages.
   * **Env Var**: `TIOROS_TOKEN` (direct token) or `TIOROS_CREDENTIALS` (path to file).

2. **App Credentials (EventSub Node)**:
   * **What is it?**: `client_id`, `client_secret` and a `refresh_token`.
   * **Storage**: A JSON file (recommended for automation with `auth.py`). 
   * **Example `.credentials` (JSON)**:
     ```json
     {
       "client_id": "YOUR_CLIENT_ID",
       "client_secret": "YOUR_CLIENT_SECRET",
       "refresh_token": "YOUR_REFRESH_TOKEN",
       "access_token": "YOUR_CURRENT_OAUTH_TOKEN",
       "webhook_secret": "YOUR_WEBHOOK_SECRET",
       "callback_route": "https://your.callback.url/path"
     }
     ```
   * **Usage**: Required for rich events (follows, raids).

# ROS Node: chatbot

## Usage

```bash
# Set token directly via environment variable
export TIOROS_TOKEN=oauth:xxxxxxxxxxxxxx
ros2 run tioros chatbot --ros-args -p channel:=myTwitchTv

# Or use a credentials file (Smart Load: supports plain text or JSON field)
export TIOROS_CREDENTIALS=~/.credentials
ros2 run tioros chatbot --ros-args -p channel:=myTwitchTv
```

## Supported TwitchIO Events
The following received Twitch events are published as `std_msgs/String` messages.

* **event_ready**: `event_ready %user_id %nick`
* **event_join**: `event_join %user_id %user_name`
* **event_message**: `event_message %user_id %author_name %message_content`

## Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | string | `superbob_6110` | Twitch channel to join. (Env: `TIOROS_CHANNEL`) |
| `token`   | string | `""` | Direct Twitch Access Token. (Env: `TIOROS_TOKEN`) |
| `credentials` | string | `~/.credentials` | Path to credentials file (plain or JSON). (Env: `TIOROS_CREDENTIALS`) |
| `frame_id`| string | `channel` | Frame ID for published messages. (Env: `TIOROS_FRAME_ID`) |
| `prefix`  | string | `!` | Command prefix for the bot. (Env: `TIOROS_PREFIX`) |

## Topics

* **Subscribed**: `chat_input` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Messages to send to Twitch.
* **Published**: `chat` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Raw chat events.
* **Published**: `json` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Structured event data.

# ROS Node: eventsub

This node uses the [Twitch EventSub extension](https://twitchio.dev/en/stable/exts/eventsub.html) for rich event tracking (follows, raids, etc.).

## Usage
```bash
export TIOROS_CREDENTIALS=~/.credentials
ros2 run tioros eventsub --ros-args \
  -p channel:=myChannelName \
  -p broadcaster_id:="'123456'"
```

## Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | string | `superbob_6110` | Twitch channel name. (Env: `TIOROS_CHANNEL`) |
| `broadcaster_id` | string | `123456` | The numeric Twitch ID of the broadcaster. (Env: `TIOROS_BROADCASTER_ID`) |
| `moderator_id` | string | `broadcaster_id` | The numeric Twitch ID of the moderator account. (Env: `TIOROS_MODERATOR_ID`) |
| `credentials` | string | `~/.credentials` | Path to JSON file with API credentials. (Env: `TIOROS_CREDENTIALS`) |
| `callback_port` | int | `4000` | Port for incoming webhook notifications. (Env: `TIOROS_CALLBACK_PORT`) |
| `events_only` | bool | `false` | If true, disables automated "Thank you" messages in chat. (Env: `TIOROS_EVENTS_ONLY`) |
| `frame_id` | string | `channel` | Frame ID for message headers. (Env: `TIOROS_FRAME_ID`) |

## Topics

* **Published**: `eventsub` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Human-readable event string.
* **Published**: `json` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Structured event data.

# ROS Node: filter

## Usage
```bash
# Use a whitelist and regex substitution
ros2 run tioros filter --ros-args \
  -r chat:=/chat_source \
  -p "substitute:=['^[^ ]+ [^ ]+ ([^ ]+) (.*)', '\\1: \\2']"
```

## Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `white_filter` | string array | `['']` | Comma-separated list for whitelist rules. (Env: `TIOROS_WHITE_FILTER`) |
| `black_filter` | string array | `['']` | Comma-separated list for blacklist rules. (Env: `TIOROS_BLACK_FILTER`) |
| `white_list` | string | `""` | Path to YAML file with whitelist regex. (Env: `TIOROS_WHITE_LIST`) |
| `black_list` | string | `""` | Path to YAML file with blacklist regex. (Env: `TIOROS_BLACK_LIST`) |
| `substitute` | string array | `['']` | Regex pairs: `['pattern1','replace1','pattern2','replace2',...]`. Must be an even number of items. (Env: `TIOROS_SUBSTITUTE`) |

## Topics

* **Subscribed**: `chat` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Incoming chat messages.
* **Published**: `chat_filtered` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Messages passing filters.
* **Published**: `rejected` ([std_msgs/String](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)) - Blocked messages.

# Tool: AUTH

Simple Twitch authentication and token management.

## Usage

```bash
# Help output
ros2 run tioros auth -h

# Generate token from refresh token
ros2 run tioros auth -f ~/.credentials -r YOUR_REFRESH_TOKEN
```

# Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.
Check the [CONTRIBUTING.md](CONTRIBUTING.md) for licensing details.
