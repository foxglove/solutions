import os 
import ExampleMsg_pb2
from base64 import standard_b64encode

with open(
    os.path.join(os.path.dirname(ExampleMsg_pb2.__file__), "ExampleMsg.bin"),
    "rb",
) as schema_bin:
    schema_base64 = standard_b64encode(schema_bin.read()).decode("ascii")

import asyncio
import time 
from foxglove_websocket import run_cancellable 
from foxglove_websocket.server import FoxgloveServer

async def main():
    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        chan_id = await server.add_channel(
            {
                "topic": "example_msg",
                "schemaName": "ExampleMsg",
                "encoding": "protobuf",
                "schema": schema_base64,
            }
        )

        i = 0 
        while True:
            i += 1
            await asyncio.sleep(0.2)
            await server.send_message(
                chan_id,
                time.time_ns(),
                ExampleMsg_pb2.ExampleMsg(msg="Hello!", count=i).SerializeToString(),
            )

if __name__ == "__main__":
    run_cancellable(main())