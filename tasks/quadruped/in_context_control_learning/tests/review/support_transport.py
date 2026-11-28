"""Author bench: drive the unmodified participant stdio bridge; retain its actual public calls."""
from contextlib import ExitStack
import base64
import hashlib
import json
from pathlib import Path
import socketserver
import subprocess
import sys
from threading import Thread

from service import Handler


class Bridge:
    def __init__(self, runtime, output, task):
        self.stack = ExitStack()
        try:
            sock = Path("/run/visual_robotics/world.sock")
            server = self.stack.enter_context(socketserver.UnixStreamServer(str(sock), Handler))
            self.stack.callback(sock.unlink, missing_ok=True)
            server.runtime = runtime
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.stack.callback(thread.join)
            self.stack.callback(server.shutdown)
            self.log = self.stack.enter_context((output/"mcp.jsonl").open("x", buffering=1))
            self.proc = self.stack.enter_context(subprocess.Popen(
                [sys.executable, "-B", str(task/"environment/setup/payload/mcp_stdio.py")],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1))
            self.stack.callback(self.proc.stdin.close)
            self.seq = 0
            self.rpc("initialize", {"protocolVersion":"2024-11-05","capabilities":{},
                                    "clientInfo":{"name":"private-author-bench","version":"1"}})
            self.proc.stdin.write('{"jsonrpc":"2.0","method":"notifications/initialized"}\n')
            self.proc.stdin.flush()
            names = {t["name"] for t in self.rpc("tools/list", {})["tools"]}
            if names != {"robot_act","robot_observe","robot_advance","robot_recover"}:
                raise RuntimeError(f"unexpected participant tools: {names}")
        except BaseException:
            self.close()
            raise

    def rpc(self, method, params):
        self.seq += 1
        self.proc.stdin.write(json.dumps({"jsonrpc":"2.0","id":self.seq,"method":method,"params":params})+"\n")
        self.proc.stdin.flush()
        reply = json.loads(self.proc.stdout.readline())
        if reply["id"] != self.seq or "error" in reply:
            raise RuntimeError(reply)
        return reply["result"]

    def call(self, method, params):
        args = {"name":"robot_"+method, "arguments":params}
        content = self.rpc("tools/call", args)["content"]
        status = json.loads(content[0]["text"])
        images = [c["data"] for c in content[1:] if c["type"]=="image"]
        self.log.write(json.dumps({"id":self.seq, **args, "status":status,
            "png_sha256":[hashlib.sha256(base64.b64decode(im)).hexdigest() for im in images]})+"\n")
        return {**status, **({"images":images} if images else {})}

    def close(self):
        self.stack.close()
