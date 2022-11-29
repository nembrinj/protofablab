import asyncio

import client

ws = client.Client()
asyncio.run(ws.start())
