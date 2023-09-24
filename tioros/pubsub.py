import twitchio
import asyncio
from twitchio.ext import pubsub

# TODO not yet used

my_token = "..."
users_oauth_token = "..."
users_channel_id = 12345
client = twitchio.Client(token=my_token)
client.pubsub = pubsub.PubSubPool(client)

@client.event()
async def event_pubsub_bits(event: pubsub.PubSubBitsMessage):
    pass # do stuff on bit redemptions

@client.event()
async def event_pubsub_channel_points(event: pubsub.PubSubChannelPointsMessage):
    pass # do stuff on channel point redemptions

async def main():
    topics = [
        pubsub.channel_points(users_oauth_token)[users_channel_id],
        pubsub.bits(users_oauth_token)[users_channel_id]
    ]
    await client.pubsub.subscribe_topics(topics)
    await client.start()

client.loop.run_until_complete(main())