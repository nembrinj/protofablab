# TODO: FOR REMOVAL

import asyncio
import json
import logging
import random

import cv2
import toml
import websockets

from shared import image_util


class Client:
    def __init__(self, config_path: str = None):
        if config_path is None:
            config_path = '../config.toml'

        conf = toml.load(config_path)

        self.host: str = conf['Server']['host']
        self.port: int = conf['Server']['port']
        self.target = f'ws://{self.host}:{self.port}'
        self.log_level: str = conf['Client']['log_level'].upper()
        self.log_file: str = conf['Client']['log_file']

        handler = logging.FileHandler(self.log_file)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s: [%(levelname)s] - %(message)s'))
        self.logger = logging.getLogger('Client')
        self.logger.setLevel(self.log_level)
        self.logger.addHandler(handler)

    def start(self):
        async def main():
            async with websockets.connect(self.target, logger=self.logger) as ws:
                await self.handler(ws)

        asyncio.run(main())

    async def handler(self, ws):
        text = random.random()
        img = image_util.to_json_value(cv2.imread('../resources/xkcd.png'))

        msg = {
            'text': text,
            'image': img,
        }
        msg = json.dumps(msg)
        self.logger.info(f'Sending to {ws.remote_address}: {msg}')
        await ws.send(msg)

        recv = await ws.recv()
        self.logger.info(f'Received from {ws.remote_address}: {recv}')


if __name__ == "__main__":
    client = Client()
    client.start()
