import asyncio
import logging

import toml
import websockets


class Server(websockets.WebSocketServer):
    def __init__(self, config_path: str = None):
        super().__init__()
        
        if config_path is None:
            config_path = '../config.toml'

        conf = toml.load(config_path)['Server']
        self.address: str = conf['address']
        self.port: int = conf['port']
        self.log_level: str = conf['log_level'].upper()
        self.log_file: str = conf['log_file']

        handler = logging.FileHandler(self.log_file)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s: [%(levelname)s] - %(message)s'))
        self.logger = logging.getLogger('Server')
        self.logger.setLevel(self.log_level)
        self.logger.addHandler(handler)

    def start(self):
        async def main():
            async with websockets.serve(self.loop, self.address, self.port, logger=self.logger):
                await asyncio.Future()

        asyncio.run(main())

    async def loop(self, ws, path):
        async for message in ws:
            self.logger.info(f'{ws.id}: {message}')
            await ws.send(message)


if __name__ == "__main__":
    server = Server()
    server.start()
