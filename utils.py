import os
from IPython.display import display, HTML
from sidecar import Sidecar
from time import sleep
from ipywidgets import Button, Layout, GridBox, VBox, Box, FloatSlider, Checkbox, interact
from IPython.display import display, Markdown, IFrame, HTML
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

# Init the rclpy client
if not rclpy.ok():
    rclpy.init(args=None)

# Display remote desktop on sidecar tab
def display_desktop(anchor='right'):
    try:
        JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
    except KeyError:
        JUPYTERHUB_USER = None
    url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
    remote_desktop_url = f"{url_prefix}/desktop"
    sc = Sidecar(title='Desktop', anchor=anchor)
    with sc:
        # The inserted custom HTML and CSS snippets are to make the tab resizable
        display(HTML(f"""
            <style>
            body.p-mod-override-cursor div.iframe-widget {{
                position: relative;
                pointer-events: none;

            }}

            body.p-mod-override-cursor div.iframe-widget:before {{
                content: '';
                position: absolute;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: transparent;
            }}
            </style>
            <div class="iframe-widget" style="width: calc(100% + 10px);height:100%;">
                <iframe src="{remote_desktop_url}" width="100%" height="100%"></iframe>
            </div>
        """))

# Node to subscribe laserscan sensor data
class LaserScanSubscriber(Node):
    def __init__(self, callback):
        super().__init__('obstacle_avoider')

        # Create a subscriber to the LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            callback,
            10  # QoS profile depth
        )
        
        # Command message to publish
        self.cmd_vel_msg = Twist()

# Node to control robot moving
class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_msg = Twist()

    def publish_velocity(self, linear_x=None, angular_z=None):
        if linear_x is not None:
            self.cmd_vel_msg.linear.x = float(linear_x)
        if angular_z is not None:
            self.cmd_vel_msg.angular.z = float(angular_z)
        self.publisher_.publish(self.cmd_vel_msg)

# robot_steering, similar to "rqt_robot_steering"
def robot_steering(velocity_publisher):
    linear_x = FloatSlider(
        value=0,
        min=-0.5,
        max=0.5,
        step=0.05,
        description='Moving',
        orientation='vertical',
        readout=True,
        readout_format='.2f',
    )
        
    # slider for rotation velocity
    angular_z = FloatSlider(
        value=0,
        min=-1,
        max=1,
        step=0.1,
        description='Rotation',
        readout=True,
        readout_format='.2f',
    )

    linear_x.observe(lambda v: velocity_publisher.publish_velocity(linear_x=v['new']), names='value')
    angular_z.observe(lambda v: velocity_publisher.publish_velocity(angular_z=-v['new']), names='value')
    
    display(Markdown('#### Robot Steering:'))
    display(Box([linear_x, angular_z]))

