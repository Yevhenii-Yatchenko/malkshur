"""Grab one frame from a Gazebo camera topic and save it as PNG."""
import asyncio
import sys

import numpy as np
import pygazebo
from pygazebo.msg import image_stamped_pb2
from PIL import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else \
    '/gazebo/default/down_cam/cam_link/nadir_camera/image'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/drone_project/logs/frame.png'


async def main():
    manager = await pygazebo.connect(('localhost', 11345))
    loop = asyncio.get_event_loop()
    fut = loop.create_future()

    def callback(data):
        msg = image_stamped_pb2.ImageStamped.FromString(data)
        if not fut.done():
            fut.set_result(msg)

    manager.subscribe(TOPIC, 'gazebo.msgs.ImageStamped', callback)
    msg = await asyncio.wait_for(fut, timeout=20)
    img = msg.image
    arr = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
    Image.fromarray(arr).save(OUT)
    print(f'saved {OUT}: {img.width}x{img.height} pixel_format={img.pixel_format}')


loop = asyncio.new_event_loop()
loop.run_until_complete(main())
