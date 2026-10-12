"""Serve Fluidd static files and proxy Moonraker (HTTP + /websocket) on one origin.

Usage: fluidd_serve.py FLUIDD_DIR [port] [moonraker_port]   (needs tornado, see moonraker reqs)
"""
import sys

from tornado import httpclient, ioloop, web, websocket

root = sys.argv[1]
port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
mport = int(sys.argv[3]) if len(sys.argv) > 3 else 7125
UP = f"127.0.0.1:{mport}"
SKIP = {"host", "content-length", "transfer-encoding", "connection"}


class Http(web.RequestHandler):
    async def _go(self, *_):
        h = {k: v for k, v in self.request.headers.items() if k.lower() not in SKIP}
        body = self.request.body or None
        if body is None and self.request.method in ("POST", "PUT", "PATCH"):
            body = b""
        req = httpclient.HTTPRequest(
            f"http://{UP}{self.request.uri}", self.request.method, h, body,
            allow_nonstandard_methods=True, request_timeout=600)
        r = await httpclient.AsyncHTTPClient().fetch(req, raise_error=False)
        self.set_status(r.code if r.code != 599 else 502)
        for k, v in r.headers.get_all():
            if k.lower() not in SKIP and k.lower() != "content-encoding":
                self.add_header(k, v)
        if r.body:
            self.write(r.body)

    get = post = put = delete = patch = _go


class Ws(websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    async def open(self):
        self.up = await websocket.websocket_connect(
            f"ws://{UP}{self.request.uri}", on_message_callback=self._down)

    def _down(self, msg):
        if msg is None:
            self.close()
        else:
            self.write_message(msg, isinstance(msg, bytes))

    async def on_message(self, msg):
        await self.up.write_message(msg, isinstance(msg, bytes))

    def on_close(self):
        if getattr(self, "up", None):
            self.up.close()


pre = r"/(server|printer|machine|api|access|database|history|sensors|debug)/.*"
app = web.Application([
    (r"/websocket", Ws),
    (pre, Http),
    (r"/(.*)", web.StaticFileHandler, {"path": root, "default_filename": "index.html"}),
])
app.listen(port)
ioloop.IOLoop.current().start()
