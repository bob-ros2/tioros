# TIOROS

[TwitchIO](https://twitchio.dev/en/stable) ROS Chatbot Nodes. 
A [ROS](https://ros.org) - Twitch Chat Bridge.

## Index
<!-- https://ecotrust-canada.github.io/markdown-toc/ -->
- [TIOROS](#tioros)
  * [Index](#index)
  * [Installation Prerequisites](#installation-prerequisites)
  * [Setup Package](#setup-package)
- [ROS Node CHATBOT](#ros-node-chatbot)
  * [Usage](#usage)
  * [Supported TwitchIO Events](#supported-twitchio-events)
  * [Node Parameter](#node-parameter)
  * [Subscribed Topics](#subscribed-topics)
  * [Published Topics](#published-topics)
- [ROS Node FILTER](#ros-node-filter)
  * [Usage](#usage-1)
  * [Node Parameter](#node-parameter-1)
  * [Subscribed Topics](#subscribed-topics-1)
  * [Published Topics](#published-topics-1)
- [Contributing](#contributing)

## Installation Prerequisites

Use the package manager [pip3](https://pip.pypa.io/en/stable/) 
to install the below dependencies.

```bash
pip3 install twitchio
```

## Setup Package ##

```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/tioros.git
cd ..
colcon build
. install/setup.bash
```

# ROS Node CHATBOT

## Usage

```bash
# place twitch access token without trailing \n into ~/.secrets
# start the node with your twitch id
ros2 run tioros chatbot channel:=myTwitchTv

# start the node with custom secrets file
ros2 run tioros chatbot channel:=myTwitchTv secrets:=~/hidden/tokenfile
```
Related documentation:
- https://twitchio.dev/en/stable
- https://dev.twitch.tv
- https://twitchtokengenerator.com

## Supported TwitchIO Events
The following received Twitch events are published as std_msgs/String 
topic messages.

> ~event_ready\
Format: event_ready %user_id %nick

> ~event_join\
Format: event_join %user_id %user_name

> ~event_message\
Format: event_message %user_id %author_name %message_content

## Node Parameter

> ~channel (string, default: "")\
Twitch channel user of the chat.

> ~secrets (string, default: "\~/.secrets")\
Path to file containing the twitch access token without trailing \n

> ~frame_id (string, default: channel)\
Frame ID of the Node.

## Subscribed Topics

> ~chat_input (std_msgs/String)\
Chat message to send to twitch.

## Published Topics

> ~chat (std_msgs/String)\
Received chat events from twitch.

# Ros Node FILTER

## Usage
```bash
# Use a input whitelist, remap input and provide re.sub regex to manipulate output
ros2 run tioros filter --ros-args -r chat:=/chat_source -p white_list:="whitelist.yaml" -p "substitute:=['^[^ ]+ [^ ]+ ([^ ]+) (.*)', '\\1: \\2']" --log-level debug

# a way to throttle messages (rate)
# see also https://github.com/ros-tooling/topic_tools
# sudo apt install ros-humble-topic-tools
#ros2 run topic_tools throttle messages <intopic> <msgs_per_sec> [outtopic]
ros2 run topic_tools throttle messages /SuperBob/chat_filtered 0.1 /gpt_in
```

## Node Parameter

> ~black_filter\
  Type: string array\
  String array with blacklist rules.

> ~black_list\
  Type: string\
  Black list file. This overides parameter black_filter.\
  Format: Yaml file with a list of strings containing regex rules.

> ~white_filter\
  Type: string array\
  String array with white list rules.

> ~white_list\
  Type: string\
  White list file. This overides parameter white_filter.\
  Format: Yaml file with a list of string containing regex rules.

> ~substitute\
  Type: string array\
  Substitute regex for the string message similar to python re.sub.\
  Expects an array with two entries: ['pattern','replace']

## Subscribed Topics

> ~chat (std_msgs/String)\
Message input.

## Published Topics

> ~chat_filtered (std_msgs/String)\
Filtered message output.

> ~rejected (std_msgs/String)\
Rejected message output.

# Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.\
Please make sure to update tests as appropriate.
