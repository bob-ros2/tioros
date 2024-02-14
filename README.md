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
- [Ros Node EVENTSUB](#ros-node-eventsub)
  * [Pre conditions](#pre-conditions)
    + [Related links](#related-links)
  * [Usage](#usage-1)
  * [Node Parameter](#node-parameter-1)
  * [Supported EventSub Events](#supported-eventsub-events)
  * [Published Topics](#published-topics-1)
- [Ros Node FILTER](#ros-node-filter)
  * [Usage](#usage-2)
  * [Node Parameter](#node-parameter-2)
  * [Subscribed Topics](#subscribed-topics-1)
  * [Published Topics](#published-topics-2)
  * [Dynamic Reconfigure Parameter](#dynamic-reconfigure-parameter)
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

# Ros Node EVENTSUB

This Twitch ROS node is dedicated to the Twitch [EventSub extension](https://twitchio.dev/en/stable/exts/eventsub.html). With this extension more events can be retrieved than in a normal chat bot where just an app token will be used. To set it up some more things are needed.

## Pre conditions
This is just a rough overview what is needed:


Existing application in the [Twitch developer console](https://dev.twitch.tv/console/)
- Client credentials client_id and client_secret are known
- Redirect callback URL where to publish events
- Redirect to http://localhost:3000 to retrieve the user (yourself) scopes code from authorization_code grant flow to get in a next step the intial token and refresh_token

Web proxy dispatching the incoming https callback service via http to the eventsub bot listen port (default 4000)
- Externaly known hostname (may use dyn DNS service for private networks)
- Open port 443
- SSL Certificate (get a free one from [lets encrypt](https://letsencrypt.org/))

### Related links
Everything needed to understand EventSub can be found here:
- https://twitchio.dev/en/stable/exts/eventsub.html\
- https://dev.twitch.tv/docs/eventsub/\
- https://dev.twitch.tv/docs/authentication/\
- https://discuss.dev.twitch.com/

## Usage
```bash
# run with given parameter
ros2 run tioros eventsub --ros-args \
-p channel:=myChannelName \
-p broadcaster_id:="'12345'" \
-p credentials:=/path/to/secret.json

```

## Node Parameter

> ~broadcaster_id (string, default: '12345')\
The broadcaster_id ID

> ~moderator_id (string, default: broadcaster_id)\
The moderator_id ID (maybe from yourself), this is the numeric presentation of the account which has the required scopes. Currently neccesary:\
`moderator:read:followers channel:read:subscriptions chat:edit chat:read`
        

> ~callback_port: (int, default: 4000)\
The listen port for incoming event notifications from callback URL.

> ~channel (string, default: 'myChannel')\
The Twitch channel name.

> ~credentials (string, default: $HOME/.credentials)\
Path to a JSON file with a dict containing the client credentials client_id, client_secret and refresh_token.

> ~events_only (bool, default: false)\
If this parameter is set to true the bot does not respond with a thank you message in the channel chat for incoming events. Events are only published via the eventsub ROS topic.

> ~frame_id (string, default: channel)\
Used frame_id in the JSON data published by the json ROS topic. This data can be used e.g. to store the data in a DB.

## Supported EventSub Events
The following received Twitch EventSub events are published as std_msgs/String topic messages.

> ~eventsub_channelfollow\
Format: eventsub_channelfollow %user_id %user_name %raw

> ~eventsub_subscription\
Format: eventsub_subscription %user_id %user_name %raw

> ~eventsub_raid\
Format: eventsub_raid %user_id %user_name %raw

## Published Topics

> ~eventsub (std_msgs/String)\
Representation from received event in JSON form.

> ~json (std_msgs/String)\
Representation from received event in JSON form.

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

## Dynamic Reconfigure Parameter
If the file path changes the data will be reloaded.
> ~white_list\
  ~black_list

# Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.\
Please make sure to update tests as appropriate.
