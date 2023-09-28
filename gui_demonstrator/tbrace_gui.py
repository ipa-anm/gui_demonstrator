import math
import threading
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import app, globals, run, ui
import asyncio

from std_msgs.msg import String


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.status = None
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.string_callback,
            10)
        #self.subscription  # prevent unused variable warning
        self.string = "Hier"

        with globals.index_client:
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label(self.string).classes('text-2xl')

                    self.status = ui.label()
                    ui.button('Start', on_click=self.compute)

    async def string_callback(self, msg) -> None:
        self.string = msg

    async def compute(self):
        while True:
            await asyncio.get_event_loop().run_in_executor(None, self.computation_step)


    def computation_step(self):
        time.sleep(0.1)
        self.status.text = self.string.data


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
# ROS2 uses a non-standard module name, so we need to specify it here
run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
