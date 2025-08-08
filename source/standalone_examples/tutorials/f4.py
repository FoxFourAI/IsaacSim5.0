import numpy as np
from isaacsim import SimulationApp


# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False}) # We need GUI

# Usd & Isaac core
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.stage import open_stage
from pxr import Sdf, UsdLux, UsdGeom
from isaacsim.core.prims import XFormPrim, RigidPrim

# Pegasus Sim imports
import omni.timeline
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import (
    ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from scipy.spatial.transform import Rotation

# Open environment
rivermark_path = "/home/nataliy/Downloads/isaac-sim-assets-1@4.2.0-rc.18+release.16044.3b2ed111/Assets/Isaac/4.2/Isaac/Environments/Outdoor/Rivermark/rivermark.usd"
open_stage(rivermark_path)

# Set up World
my_world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# Ground + Light
GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# Add cubes
visual_cube = VisualCuboid(
    prim_path="/visual_cube",
    name="visual_cube",
    position=np.array([0, 0, 10.0]),
    size=1,
    color=np.array([255, 255, 0]),
)

dynamic_cube = DynamicCuboid(
    prim_path="/dynamic_cube",
    name="dynamic_cube",
    position=np.array([0, 0, 25]),
    size=3,
    color=np.array([0, 255, 255]),
)

# Add a camera following cube
camera_path = "/FollowCam"
if not stage.GetPrimAtPath(camera_path):
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(camera_path))
    camera.AddTranslateOp()
    camera.AddRotateXYZOp()
camera_prim = XFormPrim(camera_path)

# --- PEGASUS SIM SETUP ---

# Pegasus Interface
pg = PegasusInterface()
pg._world = my_world  # reuse the existing world
timeline = omni.timeline.get_timeline_interface()

# Drone config (1 drone)
vehicle_id = 0
backend_config = ArduPilotMavlinkBackendConfig({
    "vehicle_id": vehicle_id,
    "ardupilot_autolaunch": True,
    "ardupilot_dir": pg.ardupilot_path,
    "ardupilot_vehicle_model": "gazebo-iris"
})

multirotor_config = MultirotorConfig()
multirotor_config.backends = [
    ArduPilotMavlinkBackend(config=backend_config),
    ROS2Backend(
        vehicle_id=vehicle_id,
        config={
            "namespace": f'drone',
            "pub_sensors": True,
            "pub_graphical_sensors": True,
            "pub_state": True,
            "sub_control": False,
            "pub_tf": True,
        }
    )
]

# Spawn the drone
Multirotor(
    f"/World/drone{vehicle_id}",
    ROBOTS['Iris'],
    vehicle_id,
    [0.0, 0.0, 2.0],  # spawn above ground
    Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
    config=multirotor_config
)

# Get drone prim
drone_path = f"/World/drone{vehicle_id}"
drone_prim = RigidPrim(drone_path)

# Reset world for drone physics
my_world.reset()

# --- MAIN LOOP ---
timeline.play()

while simulation_app.is_running():
    my_world.step(render=True)

    # Make camera follow the drone
    drone_pos, _ = drone_prim.get_world_pose()
    camera_pos = drone_pos + np.array([0.0, 2.0, 5.0])  # behind and above
    camera_prim.set_world_pose(position=camera_pos)

    # Make camera follow the dynamic cube
    # cube_pos = dynamic_cube.get_world_pose()[0]
    # camera_pos = cube_pos + np.array([0.0, 2.0, 5.0])
    # camera_prim.set_world_pose(position=camera_pos)
