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

# ROS Node: chatbot

## Usage

```bash
# Set credentials via environment variable
export TIOROS_SECRETS=~/.secrets
ros2 run tioros chatbot --ros-args -p channel:=myTwitchTv

# Or start the node with standard ROS parameters
ros2 run tioros chatbot --ros-args \
  -p channel:=myTwitchTv \
  -p secrets:=~/hidden/tokenfile
```

## Supported TwitchIO Events
The following received Twitch events are published as `std_msgs/String` messages.

* **event_ready**: `event_ready %user_id %nick`
* **event_join**: `event_join %user_id %user_name`
* **event_message**: `event_message %user_id %author_name %message_content`

## Node Parameters

| Parameter | Type | Default | Env Var | Description |
|-----------|------|---------|---------|-------------|
| `channel` | string | `superbob_6110` | `TIOROS_CHANNEL` | Twitch channel to join. |
| `secrets` | string | `~/.secrets` | `TIOROS_SECRETS` | Path to file containing the Twitch access token. |
| `frame_id`| string | `channel` | `TIOROS_FRAME_ID` | Frame ID for published messages. |
| `prefix`  | string | `!` | `TIOROS_PREFIX` | Command prefix for the bot. |

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
  -p broadcaster_id:="'12345'"
```

## Node Parameters

| Parameter | Type | Default | Env Var | Description |
|-----------|------|---------|---------|-------------|
| `channel` | string | `superbob_6110` | `TIOROS_CHANNEL` | Twitch channel name. |
| `broadcaster_id` | string | `123456` | `TIOROS_BROADCASTER_ID` | The numeric Twitch ID of the broadcaster. |
| `moderator_id` | string | `broadcaster_id` | `TIOROS_MODERATOR_ID` | The numeric Twitch ID of the moderator account. |
| `credentials` | string | `~/.credentials` | `TIOROS_CREDENTIALS` | Path to JSON file with API credentials. |
| `callback_port` | int | `4000` | `TIOROS_CALLBACK_PORT` | Port for incoming webhook notifications. |
| `events_only` | bool | `false` | `TIOROS_EVENTS_ONLY` | If true, disables automated "Thank you" messages in chat. |
| `frame_id` | string | `channel` | `TIOROS_FRAME_ID` | Frame ID for message headers. |

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

| Parameter | Type | Default | Env Var | Description |
|-----------|------|---------|---------|-------------|
| `white_filter` | string array | `['']` | `TIOROS_WHITE_FILTER` | Comma-separated list for whitelist rules. |
| `black_filter` | string array | `['']` | `TIOROS_BLACK_FILTER` | Comma-separated list for blacklist rules. |
| `white_list` | string | `""` | `TIOROS_WHITE_LIST` | Path to YAML file with whitelist regex. |
| `black_list` | string | `""` | `TIOROS_BLACK_LIST` | Path to YAML file with blacklist regex. |
| `substitute` | string array | `['']` | `TIOROS_SUBSTITUTE` | Pattern and replacement: `['pattern','replace']`. |

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
