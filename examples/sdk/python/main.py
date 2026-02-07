import foxglove
import time 

foxglove.start_server()

with foxglove.open_mcap("example.mcap"):
    while True:
        foxglove.log("/hello", {"time": time.time()})
        time.sleep(0.03)
