#!/usr/bin/env python3
"""Static file server for the Galbot S1 web console (no ROS dependency).

    web_server.py --root <dir with index.html> [--port 8000] [--rosbridge-port 9090]

Also serves /config.js with the rosbridge port. Unknown arguments (roslaunch /
ros2 launch remapping args) are ignored, so it can be started as a ROS node.
"""
import argparse
import functools
import http.server
import os
import socket
import sys


class Handler(http.server.SimpleHTTPRequestHandler):
    rosbridge_port = 9090

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def do_GET(self):
        if self.path.split('?')[0] == '/config.js':
            body = ('window.GALBOT_CONFIG = {rosbridgePort: %d};\n' % self.rosbridge_port).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/javascript')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def log_message(self, fmt, *args):  # keep the launch output clean
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web'))
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--rosbridge-port', type=int, default=9090)
    args, _unknown = ap.parse_known_args([a for a in sys.argv[1:] if not a.startswith('__')])
    root = os.path.abspath(args.root)
    if not os.path.isfile(os.path.join(root, 'index.html')):
        sys.exit('web_server: index.html not found in %s' % root)
    Handler.rosbridge_port = args.rosbridge_port
    handler = functools.partial(Handler, directory=root)
    http.server.ThreadingHTTPServer.allow_reuse_address = True
    server = http.server.ThreadingHTTPServer(('0.0.0.0', args.port), handler)
    host = socket.gethostname()
    print('[galbot web] http://%s:%d  (or http://localhost:%d)  serving %s' % (host, args.port, args.port, root),
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
