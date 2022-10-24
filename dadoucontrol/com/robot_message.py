import logging


class RobotMessage:

    def __init__(self, ws_client):
        self.ws_client = ws_client

    def send_legacy(self, key, move):
        if not key and not move:
            logging.error("key and move parameter empty")
            return
        if not key:
            key = "  "
        if not move:
            move = "   "

        self.ws_client.send(move+key)