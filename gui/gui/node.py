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
from fastapi import Response
import base64
import plotly.graph_objects as go
from nicegui import app, globals, run, ui
import plotly_tree as bt




HEADERS = [
    ["UR Driver", "XYZ Driver", "Turn Off Driver"],
    ["Realsense", "RS_Marker", "Asus Xtion", "Turn Off Sensor"],
    ["Detect Red", "Detect Colors", "Turn Off Detection"],
    ["Static Scenario", "Marker", "Red", "Multiple Color", "Turn Off Scenario"],
    ["Groot", "Rviz", "Turn Off Tools"]
]
COMMANDS = [
    ["ros2 launch ur5e_cell_manipulation manipulation_driver.launch.py ", "ls", "ls"],
    ["ros2 launch ur5e_cell_manipulation manipulation_perception.launch.py ",
        "ros2 launch ur5e_cell_manipulation manipulation_perception_aruco.launch.py ", "ls", "ls"],
    ["ros2 run color_pose_estimation color_pose_estimation_unique",
        "ros2 run color_pose_estimation color_pose_estimation", "ls"],
    ["ros2 launch ur5e_cell_manipulation manipulation_static_scenario.launch.py", "ros2 launch ur5e_cell_manipulation manipulation_aruco_scenario.launch.py",
        "ros2 launch ur5e_cell_manipulation manipulation_color_unique_scenario.launch.py", "ros2 launch ur5e_cell_manipulation manipulation_color_scenario.launch.py", "ls"],
    ["ros2 run groot Groot", "rviz2", "ls"]
]

TABS = ["Model", "Configuration", "Monitor", "Add Tasks"]


# # We need an executor to schedule CPU-intensive tasks with `loop.run_in_executor()`.
# process_pool_executor = concurrent.futures.ProcessPoolExecutor()
# # In case you don't have a webcam, this will provide a black placeholder image.
# black_1px = 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAAXNSR0IArs4c6QAAAA1JREFUGFdjYGBg+A8AAQQBAHAgZQsAAAAASUVORK5CYII='
# placeholder = Response(content=base64.b64decode(black_1px.encode('ascii')), media_type='image/png')
# # OpenCV is used to access the webcam.
# video_capture = cv2.VideoCapture(0)

# app.add_static_files('/stl', 'static')


class NiceGuiNode(Node):
 
    def __init__(self) -> None:
        super().__init__('Demonstrator')
        self.task = None
        self.process = None
        self.video_frame = None
        with globals.index_client:
            with ui.footer(value=True).style('background-color: #25957b') as footer:
                ui.label('Demonstrator @ Fraunhofer IPA 2023').classes('absolute-center items-center')
            with ui.column().classes('w-full items-center'):
                ui.label('Demonstrator Testing Application').classes('text-h2 mt-10').style('color: #25957b')
                ui.label('ROS 2 Demonstrator 326').classes('text-h4 my-2')
                with ui.tabs().classes('w-1/2 justify-center').props('h2') as tabs:
                    for tab in TABS: 
                        ui.tab(tab).classes('w-1/2 text-center').style('font-size: 20px;')
                with ui.tab_panels(tabs, value=TABS[0]):
                    with ui.tab_panel(TABS[0]):
                            ui.button('Deploy', color='#25957b').style('color: white; font-size: 50px; padding: 10px 30px; border-radius: 10px;').classes("my-10 justify-center")
                        
                    with ui.tab_panel(TABS[1]):  
                        ui.label("Choose the components for your presentation").classes("text-h4 my-2 text-center").style('color: #25957b')  
                        with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 60vw; position: relative'):
                            with ui.row().classes('text-center'): 
                                ui.label("").classes("text-h6 text-muted")
                            with ui.row(): 
                                ui.label("Driver:").classes("text-h6 text-muted my-2")
                                ui.radio(HEADERS[0], value='Turn Off',on_change=self.run_command).props('inline')
                            with ui.row():
                                ui.label("Sensor:").classes("text-h6 text-dark my-2")
                                ui.radio(HEADERS[1], value='Turn Off',on_change=self.run_command).props('inline')
                            with ui.row():
                                ui.label("Detection:").classes("text-h6 text-dark my-2")
                                ui.radio(HEADERS[2], value='Turn Off',on_change=self.run_command).props('inline')
                            with ui.row():
                                ui.label("Scenario:").classes("text-h6 text-dark my-2")
                                ui.radio(HEADERS[3], value='Turn Off',on_change=self.run_command).props('inline')
                            with ui.row():
                                ui.label("Tool:").classes("text-h6 text-dark my-2")
                                ui.radio(HEADERS[4], value='Turn Off',on_change=self.run_command).props('inline')
                    with ui.tab_panel(TABS[2]):
                            ui.label('Dashboard').classes('text-h4 my-2 text-center').style('color: #25957b')
                            with ui.row():
                                with ui.column():
                                    with ui.card().classes('mt-10 mb-1 w-96 h-96').style('width: 30vw; position: relative'):
                                        ui.label('Camera Capture').classes('text-h6')
                                        # For non-flickering image updates an interactive image is much better than `ui.image()`.
                                        # self.grab_video_frame()
                                        #video_image = ui.interactive_image().classes('w-full h-full')
                                        # A timer constantly updates the source of the image.
                                        # Because data from same paths are cached by the browser,
                                        # we must force an update by adding the current timestamp to the source.
                                        #ui.timer(interval=0.1, callback=lambda: video_image.set_source(f'/video/frame?{time.time()}'))
                                    
                                with ui.column():
                                        fig = bt.create_tree()
                                        ui.plotly(fig).classes('mt-10 mb-1 w-20').style('width: 60vw; position: relative')

                                    #     
                                    #     with ui.row():
                                    #         with ui.column(): 
                                    #             with ui.card().classes('mx-10 mt-10 w-50 h-15').props('color=positive'):
                                    #                     ui.label('TASK1').classes('text-center')
                                    #         with ui.column():        
                                    #             with ui.card().classes('mx-10 mt-10 w-50 h-15').props('color=positive'):
                                    #                     ui.label('TASK2').classes('text-center')
                                    #         with ui.column():        
                                    #             with ui.card().classes('mx-10 mt-10 w-50 h-15').props('color=positive'):
                                    #                     ui.label('TASK3').classes('text-center')
                                    #         with ui.column():        
                                    #             with ui.card().classes('mx-10 mt-10 w-50 h-15').props('color=positive'):
                                    #                     ui.label('TASK4').classes('text-center')
                                    #         with ui.column():        
                                    #             with ui.card().classes('mx-10 mt-10 w-50 h-15').props('color=positive'):
                                    #                     ui.label('TASK5').classes('text-center')
                                 
                                            



                            with ui.row():
                                with ui.card().classes('mt-10 mb-1 w-20').style('width: 80vw; position: relative'):
                                        ui.label('Information about the demonstrator: ').classes('text-h6')
        
    # @app.get('/video/frame')
    # # Thanks to FastAPI's `app.get`` it is easy to create a web route which always provides the latest image from OpenCV.
    # async def grab_video_frame(self) -> Response:
    #     if not video_capture.isOpened():
    #         return placeholder
    #     loop = asyncio.get_running_loop()
    #     # The `video_capture.read` call is a blocking function.
    #     # So we run it in a separate thread (default executor) to avoid blocking the event loop.
    #     _, frame = await loop.run_in_executor(None, video_capture.read)
    #     if frame is None:
    #         return placeholder
    #     # `convert` is a CPU-intensive function, so we run it in a separate process to avoid blocking the event loop and GIL.
    #     jpeg = await loop.run_in_executor(process_pool_executor, convert, frame)
    #     return Response(content=jpeg, media_type='image/jpeg')

   
        
  
  
    async def wait_for_process(self, process):
        await process.wait()
    
    async def run_command(self, event: ValueChangeEventArguments):
        """Run a command in the background and display the output in the pre-created dialog."""
        if event.value=='Turn Off': 
                print(self.process)
                self.task.cancel()
                self.process.send_signal(signal.SIGINT)
                
        if event.value=="UR Driver":
            cmd = COMMANDS[0][0]
            setup_cmd = 'source ~/ws_ur5e/install/setup.bash '
            command = "ros2 launch ur5e_cell_manipulation manipulation_driver.launch.py"
            self.process = await asyncio.create_subprocess_exec(
                *shlex.split(command), cwd=os.path.dirname(os.path.abspath(__file__))
            )

            try:
                # Create an event loop
                loop = asyncio.get_event_loop()
                # Create a task for running the command
                self.task = asyncio.create_task(self.wait_for_process(self.process))
                print(self.task)
            except asyncio.CancelledError:
                # The task was cancelled, terminate the process
                self.process.terminate()
                # Wait for the process to terminate
                await self.process.wait()
          
                            
    def convert(self, frame: np.ndarray) -> bytes:
        _, imencode_image = cv2.imencode('.jpg', frame)
        return imencode_image.tobytes()

    async def disconnect(self) -> None:
        """Disconnect all clients from current running server."""
        for client in globals.clients.keys():
            await app.sio.disconnect(client)


    def handle_sigint(self,signum, frame) -> None:
        # `disconnect` is async, so it must be called from the event loop; we use `ui.timer` to do so.
        ui.timer(0.1, self.disconnect, once=True)
        # Delay the default handler to allow the disconnect to complete.
        ui.timer(1, lambda: signal.default_int_handler(signum, frame), once=True)


    async def cleanup(self) -> None:
        # This prevents ugly stack traces when auto-reloading on code change,
        # because otherwise disconnected clients try to reconnect to the newly started server.
        await self.disconnect()
        # Release the webcam hardware so it can be used by other applications again.
        video_capture.release()
        # The process pool executor must be shutdown when the app is closed, otherwise the process will not exit.
        process_pool_executor.shutdown()

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        app.on_shutdown(cleanup)
        # We also need to disconnect clients when the app is stopped with Ctrl+C,
        # because otherwise they will keep requesting images which lead to unfinished subprocesses blocking the shutdown.
        signal.signal(signal.SIGINT, handle_sigint)
        pass
    
app.on_startup(lambda: threading.Thread(target=ros_main).start())
run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', title='Robot Demonstrator 326')