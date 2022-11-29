import asyncio
import json
import logging
import os
import time

import cv2
import toml
import websockets

from shared import image_util

config_path = '../config.toml'

conf = toml.load(config_path)['Server']
host: str = conf['host']
port: int = conf['port']
log_level: str = conf['log_level'].upper()
log_file: str = conf['log_file']

handler = logging.FileHandler(log_file)
handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s: [%(levelname)s] - %(message)s'))
logger = logging.getLogger('Server')
logger.setLevel(log_level)
logger.addHandler(handler)

CONNECTIONS = set()


async def register(ws):
    CONNECTIONS.add(ws)
    try:
        async for incoming in ws:
            logger.info(f'{ws.id}({ws.remote_address}): {incoming}')

            try:
                incoming = json.loads(incoming)
            except Exception as e:
                ws.send(f'ERROR - {e}')
                continue

            response = []
            if (text := incoming.get('text')) is not None:
                response.append(f'OK(text={text})')

            if (image := incoming.get('image')) is not None:
                try:
                    image = image_util.from_json_value(image)

                    if not os.path.exists('./tmp'):
                        os.makedirs('./tmp')

                    filename = f'./tmp/{time.time()}.png'
                    cv2.imwrite(filename, image)

                    response.append(f'OK(image={filename})')
                except Exception as e:
                    response.append(f'ERROR({e})')

            await ws.send(json.dumps(response))
    finally:
        CONNECTIONS.remove(ws)


async def main():
    async with websockets.serve(ws_handler=register, host=host, port=port, logger=logger):
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
