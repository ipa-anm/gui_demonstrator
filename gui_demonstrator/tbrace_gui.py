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

from std_msgs.msg import Int16, Bool
TABS = ["RACE", "BESTLIST"]


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.status_points = None
        self.status_new_game = None
        self.status_time = None
        self.end_game = True
        self.points = 50
        self.new_game = True
        self.time = 200
        self.player = "Jonas"
        self.columns = [{'name': 'name', 'label': 'Name', 'field': 'name', 'required': True, 'align': 'left'},{'name': 'points', 'label': 'Points', 'field': 'points', 'sortable': True},]
        self.rows = [{'name': 'Anna', 'points': 30},]
        self.subscription = self.create_subscription(
            Int16,
            'points',
            self.int_callback,
            10)
        self.subscription = self.create_subscription(
            Int16,
            'time',
            self.time_callback,
            10)
        # self.subscription  # prevent unused variable warning
        with globals.index_client:
                    with ui.footer(value=True).style('background-color: #25957b') as footer:
                        ui.label(
                            'Demonstrator @ Fraunhofer IPA 2023').classes('absolute-center items-center')
                    with ui.column().classes('w-full items-center'):
                        ui.label('TURTLEBOT RACING').classes(
                            'text-h1 mt-10').style('color: #25957b')
                        ui.label('Attraction of 326').classes('text-h4 my-2')
                        with ui.tabs().classes('w-1/2 justify-center').props('h2') as tabs:
                            for tab in TABS:
                                ui.tab(tab).classes(
                                    'w-1/2 text-center').style('font-size: 20px;')
                        with ui.tab_panels(tabs, value=TABS[0]):
                            with ui.tab_panel(TABS[0]):
                                with ui.row():
                                    with ui.column():
                                        with ui.card().classes('mt-10 mb-1 w-32 h-96 bg-green-100 justify-center items-center').style('width: 40vw; position: relative'):
                                            ui.timer(0.1, self.compute_points)
                                            ui.label("Your Current Points").classes('text-h3')
                                            with ui.card().classes('mt-1 mb-1 bg-green-300 justify-center items-center').style('width: 10vw; position: relative'):
                                                self.status_points = ui.label().classes('text-h1')
                                    with ui.column():
                                        with ui.card().classes('mt-10 mb-1 w-32 h-96 bg-green-100 justify-center items-center').style('width: 40vw; position: relative'):
                                            ui.timer(0.1, self.compute_time)
                                            ui.label("Your Current Time").classes('text-h3')
                                            with ui.card().classes('mt-1 mb-1 bg-green-300 justify-center items-center').style('width: 10vw; position: relative'):
                                                self.status_time = ui.label().classes('text-h1')
                                                ui.timer(0.1, self.check_end_game)

                            with ui.tab_panel(TABS[1]):
                                 self.table = ui.table(columns=self.columns, rows=self.rows, row_key='name')


    async def int_callback(self, msg) -> None:
        self.points = msg.data

    async def bool_callback(self, msg) -> None:
        self.new_game = msg.data

    async def time_callback(self, msg) -> None:
        self.time = msg.data

    async def compute_points(self):
        print(self.points)
        self.status_points.text = self.points

    async def compute_time(self):
        print(self.time)
        self.status_time.text = self.time

    async def check_end_game(self):
            if (self.end_game==True and not self.time==0):
                final_points = 1000*(1/self.time)+self.points
                self.rows.append({'name': self.player, 'points': final_points})
                #self.table.add_rows({'name': self.player, 'points': final_points})
                self.status_points.text = 0
                self.status_time.text = 0
                self.points = 0
                self.time = 0
                final_points = 0

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
