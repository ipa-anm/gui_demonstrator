import math
import threading
from pathlib import Path
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import asyncio
import subprocess
import os
from nicegui.events import ValueChangeEventArguments
import shlex
import os.path
import platform
import sys
import pygments
import signal
import cv2
import numpy as np
import concurrent.futures
import signal
import time
from fastapi import Response, APIRouter, FastAPI
import base64
# import plotly.graph_objects as go
from nicegui import app, globals, run, ui
# import plotly_tree as bt
from file_picker import start_webview
from message_filters import Subscriber
import sensor_msgs.msg as sensor_msg
from rclpy.qos import qos_profile_sensor_data
# import video as vid
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge


HEADERS = [
    ["UR Driver", "Pilz Driver", "Turn Off Robot"],
    ["Realsense", "Asus Xtion", "Turn Off Sensor"],
    ["Start Skillset: ", "Start Skillset(Color)", "Turn Off Skillset"],
    ["Start Behaviortree", "Stop Behaviortree"],
    ["Groot", "Rviz", "Turn Off Tools"]
]
COMMANDS = [
    ['ansible-playbook -i inventory.yaml playbook_start_components.yaml --tags "ur_driver"', "echo NOTHING HERE YET"],
    ['ansible-playbook -i inventory.yaml playbook_start_components.yaml --tags "launch_realsense"', "echo NOTHING HERE YET"],
    ['ansible-playbook -i inventory.yaml playbook_start_components.yaml --tags "aruco_marker_publish"',"echo NOTHING HERE YET"],
    ['ansible-playbook -i inventory.yaml playbook_control_bt.yaml  --extra-vars "bt_xml="'],['" loop=false" --tags "start_bt"'],
    ["ros2 run groot Groot", "rviz2", "ls"]
]

DEPLOY_BUTTON = 'curl -X POST \
 https://gitlab.cc-asp.fraunhofer.de/api/v4/projects/51998/jobs/3972498/play \
 --header "Content-Type: application/json" \
 --header "PRIVATE-TOKEN: vpyfJPCL6reDMUUzJ3Kq"'


TABS = ["Load Model", "Configure Application",
        "Monitor Application", "Add Tasks"]


class NiceGuiNode(Node):
    def __init__(self) -> None:
        super().__init__('Demonstrator')
        self._fastapi = FastAPI(
            version="v1",
        )
        self.process_pool_executor = concurrent.futures.ProcessPoolExecutor()
        self.processes = [(None,None,None), (None,None,None),(None,None,None),(None,None,None),(None,None,None)]
        self.image = None
        self.subscription = self.create_subscription(
            sensor_msg.Image, "/camera/color/image_raw", self.listener_callback, 1)
        # self.subscription = self.create_subscription(
        #     ur_msgs/msg/IOStates, "/camera/color/image_raw", self.listener_callback, 1)
        self.bt_file = None
        self.subscription
        app.add_api_route(
            path="/video/frame",
            endpoint=self.grab_video_frame,
            methods=["GET"],
        )

        self.br = CvBridge()

        with globals.index_client:
            with ui.footer(value=True).style('background-color: #25957b') as footer:
                ui.label(
                    'Demonstrator @ Fraunhofer IPA 2023').classes('absolute-center items-center')
            with ui.column().classes('w-full items-center'):
                ui.label('Demonstrator Testing Application').classes(
                    'text-h2 mt-10').style('color: #25957b')
                ui.label('ROS 2 Demonstrator 326').classes('text-h4 my-2')
                with ui.tabs().classes('w-1/2 justify-center').props('h2') as tabs:
                    for tab in TABS:
                        ui.tab(tab).classes(
                            'w-1/2 text-center').style('font-size: 20px;')
                with ui.tab_panels(tabs, value=TABS[0]):
                    with ui.tab_panel(TABS[0]):
                        ui.button('Deploy', color='#25957b', on_click=self.run_deploy).style(
                            'color: white; font-size: 50px; padding: 10px 30px; border-radius: 10px;').classes("my-10 justify-center")

                    with ui.tab_panel(TABS[1]):
                        ui.label("Choose the components for your presentation").classes(
                            "text-h4 my-2 text-center").style('color: #25957b')
                        with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 60vw; position: relative'):
                            with ui.row():
                                ui.label("Robot Driver:").classes(
                                    "text-h6 text-muted my-2")
                                ui.radio(HEADERS[0], value='Turn Off', on_change=self.run_command).props(
                                    'inline')
                            with ui.row():
                                ui.label("Sensor Driver:").classes(
                                    "text-h6 text-dark my-2")
                                ui.radio(HEADERS[1], value='Turn Off', on_change=self.run_command).props(
                                    'inline')
                            with ui.row():
                                ui.label("Algorithm:").classes(
                                    "text-h6 text-dark my-2")
                                ui.radio(HEADERS[2], value='Turn Off', on_change=self.run_command).props(
                                    'inline')
                            with ui.row():
                                ui.label("Tool:").classes(
                                    "text-h6 text-dark my-2")
                                ui.radio(HEADERS[4], value='Turn Off', on_change=self.run_command).props(
                                    'inline')
                            with ui.row():
                                ui.label("Behaviortree:").classes(
                                    "text-h6 text-dark my-2")
                                bt_button = ui.button('Choose .xml File', color='#25957b', on_click=self.choose_file)

                        ui.button('Start Application', color='#25957b').style(
                            'color: white; font-size: 20px; padding: 10px 30px; border-radius: 10px;').classes("my-10 justify-center")
                        ui.button('Stop Application', color='#25957b').style(
                            'color: white; font-size: 20px; padding: 10px 30px; border-radius: 10px;').classes("my-10 justify-center")

                    with ui.tab_panel(TABS[2]):
                        ui.label('Dashboard').classes(
                            'text-h4 my-2 text-center').style('color: #25957b')
                        with ui.row():
                            with ui.column():
                                with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 25vw; position: relative'):
                                    ui.label('Camera Capture').classes(
                                        'text-h6')
                                    # For non-flickering image updates an interactive image is much better than `ui.image()`.
                                    self.grab_video_frame()
                                    video_image = ui.interactive_image().classes('w-3/4 h-3/4')
                                    # A timer constantly updates the source of the image.
                                    # Because data from same paths are cached by the browser,
                                    # we must force an update by adding the current timestamp to the source.
                                    ui.timer(interval=0.1, callback=lambda: video_image.set_source(
                                      f'/video/frame?{time.time()}'))

                            with ui.column():
                                with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 25vw; position: relative'):
                                    ui.label('Information about the demonstrator: ').classes(
                                        'text-h6')

    async def choose_file(self):
        path = start_webview()
        self.bt_file = path
        ui.notify(
            f"A behaviortree file was successfully chosen with path {path}")
        ui.label(path).classes("mt-2")

    async def run_deploy(self):
        await self.run_ros2_command(DEPLOY_BUTTON)
        ui.notify("The deployment was successful")
    # Thanks to FastAPI's `app.get`` it is easy to create a web route which always provides the latest image from OpenCV.

    async def grab_video_frame(self) -> Response:
        loop = asyncio.get_running_loop()
        jpeg = await loop.run_in_executor(self.process_pool_executor, convert, self.image)
        # _, imencode_image = cv2.imencode('.jpg', self.image)
        # jpeg = imencode_image.tobytes()

        return Response(content=jpeg, media_type='image/jpeg')

    async def wait_for_process(self, process):
        await process.wait()

    async def run_command(self, event: ValueChangeEventArguments):
        """Run a command in the background and display the output in the pre-created dialog."""
        if event.value == None:
            print("No event registered")
        elif event.value == None:
            print("No event registered")
        elif event.value == 'Turn Off Robot':
            await self.shutdown_command(0)
        elif event.value == 'Turn Off Sensor':
            await self.shutdown_command(1)
        elif event.value == 'Turn Off Skillset':
            await self.shutdown_command(2)
        elif event.value == 'Turn Off Tools':
            await self.shutdown_command(3)
        else:
            print("Before event value")
            print(event.value)
            indices = next(((i, j) for i, sublist in enumerate(HEADERS) for j, value in enumerate(sublist) if value == event.value), None)
            await self.execute_command(COMMANDS[indices[0]][indices[1]], indices[0])

    async def shutdown_command(self,index):
        print(self.processes)
        print(self.processes[index][0])
        self.processes[index][0].cancel
        #self.processes[index][1].terminate()
        self.processes[index][1].send_signal(signal.SIGINT)

    async def execute_command(self, cmd, index):
        print(cmd)
        cmd = "ros2 launch ur5e_cell_moveit_config robot.launch.py"
        command = cmd
        print("Executing")
        process  = await asyncio.create_subprocess_exec(
            *shlex.split(command), cwd=os.path.dirname(os.path.abspath(__file__))
        )

        try:
            print("In event loop")
            # Create an event loop
            loop = asyncio.get_event_loop()
            # Create a task for running the command
            task = asyncio.create_task(
                self.wait_for_process(process))
            self.processes[index]= (task,process)
            print(self.processes)
        except asyncio.CancelledError:
            # The task was cancelled, terminate the process
            process.terminate()
            # Wait for the process to terminate
            await process.wait()

    async def disconnect(self) -> None:
        """Disconnect all clients from current running server."""
        for client in globals.clients.keys():
            await app.sio.disconnect(client)

    def handle_sigint(self, signum, frame) -> None:
        # `disconnect` is async, so it must be called from the event loop; we use `ui.timer` to do so.
        ui.timer(0.1, self.disconnect, once=True)
        # Delay the default handler to allow the disconnect to complete.
        ui.timer(1, lambda: signal.default_int_handler(
            signum, frame), once=True)

    def listener_callback(self, msg: sensor_msg.Image) -> None:
        self.image = self.br.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")
        # cv2.imshow("image", self.image)
        # key = cv2.waitKey(1)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def convert(frame):
    _, imencode_image = cv2.imencode('.jpg', frame)
    return imencode_image.tobytes()


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
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
       favicon='🤖', title='Robot Demonstrator 326')
