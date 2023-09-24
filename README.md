# TIOROS

TwitchIO ROS Chatbot Node. A ROS - Twitch Chat Bridge.

## Installation Prerequisites

Use the package manager [pip3](https://pip.pypa.io/en/stable/) 
to install the below dependencies.

```bash
pip3 install twitchio
```

## ROS Node Chatbot

### Usage

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

### Supported TwitchIO events
The following received Twitch events are published as std_msgs/String 
topic messages.

> ~event_ready\
Format: event_ready %user_id %nick

> ~event_join\
Format: event_join %user_id %user_name

> ~event_message\
Format: event_message %user_id %author_name %message_content

### Node Parameter

> ~channel (string, default: "")\
Twitch channel user of the chat.

> ~secrets (string, default: "\~/.secrets")\
Path to file containing the twitch access token without trailing \n

> ~frame_id (string, default: channel)\
Frame ID of the Node.

### Published Topics

> ~chat (std_msgs/String)\
Received chat events from twitch.

### Subscribed Topics

> ~chat_input (std_msgs/String)\
Chat message to send to twitch.

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[Apache2.0](https://www.apache.org/licenses/LICENSE-2.0)