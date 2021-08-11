import unittest
from beamngpy import BeamNGpy, Scenario, Road, Vehicle
from beamngpy.sensors import State, Damage, Camera, Timer
from trajectory_generator import generate_trajectory
from test_oracles import TargetAreaOracle
from shapely.geometry import Point, LineString, Polygon
from shapely.affinity import translate, rotate
from multiprocessing import Process, Queue




from sumoSimulation.cfg import simulationCfg
from sumoSimulation.simulation import Simulator



import common
from beamngpy import BeamNGpy, Scenario, Road, Vehicle
from beamngpy import ProceduralCone, ProceduralBump, ProceduralCube
from beamngpy import StaticObject
from time import sleep


 # Working with files in xml format
import xml.etree.cElementTree as etree 
from xml.etree import ElementTree

# Building a tree from the xml file
tree = ElementTree.parse('C:\ShineAutonomousTesting\ScenarioFactory\Code\scenarios\scenario-factory\ARG_Carcarana-4_1_T-1.xml')

# Extracting the root from the tree
root = tree.getroot()
# To get root from string we can use:
# root = ElementTree.fromstring(string_xml_data)

# Showing the root, tags and attributes
print(root)
print(root.tag, root.attrib)

# Going through all children from the tree
for child in root:
    print(child.tag, child.attrib)

# Showing the information from first child about its first child
print(root[0][0].text)

# Using iter showing the scores
for element in root.iter('scores'):
    score_sum = 0
    for child in element:
        score_sum += float(child.text)
    print(score_sum)




#Specify where BeamNG home and user are

BNG_HOME = "C:\ShineAutonomousTesting\BeamNG.tech"
BNG_USER = "C:\ShineAutonomousTesting\sbst-2021-tutorial\Code"


GROUND_LEVEL = -28.0
LANE_WIDTH = 4.0
CAR_LENGTH = 5


def generate_road():
    direction_of_the_road = (0, 0, +180.0)

    # Generate a road using a sequence of segments
    initial_location = Point(10, 10, GROUND_LEVEL)
    initial_rotation = 0
    road_segments = []
    # Fixed. Ensure all the rotations are the same...
    road_segments.append(
        {
            'trajectory_segments':
                [
                    {'type': 'straight', 'length': 20.0},
					{'type': 'turn', 'angle': 50.0, 'radius': 20.0},
					{'type': 'turn', 'angle': -400.0, 'radius': 100.0}
                ]
        }
    )
    road_segments.append(
        {
            'trajectory_segments':
                [
                    {'type': 'straight', 'length': 30.0},
                    {'type': 'turn', 'angle': -110.0, 'radius': 20.0},
                    {'type': 'turn', 'angle': -300.0, 'radius': 100.0}
                ]
        }
    )
    road_segments.append(
        {
            'trajectory_segments':
                [
                    {'type': 'straight', 'length': 4.0},
                    {'type': 'turn', 'angle': +45.0, 'radius': 12.0},
                    {'type': 'turn', 'angle': +45.0, 'radius': 24.0},
                    {'type': 'turn', 'angle': +45.0, 'radius': 48.0}
                ]
        }
    )
    road_spine = generate_trajectory(initial_location, initial_rotation, road_segments, SAMPLING_UNIT=50)
    # BeamNG road nodes are (x,y,z) + road_width
    return direction_of_the_road, [(tp[0], tp[1], GROUND_LEVEL, 8.0) for tp in road_spine]
	
	

class Test(unittest.TestCase):

    def test_beamng_ai(self):
        direction_of_the_road, road_nodes = generate_road()
        # This is the node at the beginning
        initial_location = (road_nodes[0][0], road_nodes[0][1], road_nodes[0][2])
        # This is the node at the end of the road
        destination = (road_nodes[-1][0], road_nodes[-1][1], road_nodes[-1][2])

        road = Road('tig_road_rubber_sticky', rid='road_1')
        road.nodes.extend(road_nodes)
		
        # If you want to see it,
	 
        road_visualizer = RoadVisualizer('tig', 'road_1')
        road_visualizer.visualize_road(road_1)
		
        # Create the scenario
        scenario = Scenario('tig', 'road_1')
        scenario.add_road(road)

        # Place the ego-car at the beginning of the road, in the middle of the right lane
        ego_position = translate(initial_location, +LANE_WIDTH * 0.5, 0.0)
        ego_position = translate(ego_position, 0.0, CAR_LENGTH)
        ego_vehicle = Vehicle('ego', model='etk800', licence='ego', color="green")
        scenario.add_vehicle(ego_vehicle, pos=(ego_position.x, ego_position.y, GROUND_LEVEL),
                             green=direction_of_the_road, green_quat=None)

        scenario.add_checkpoints([destination], [(1.0, 1.0, 1.0)], ids=["target_wp"])
		
		# Create a vehicle in front of the ego-car, in same lane, following the same direction
        # We use the current position of the ego-car
        heading_vehicle_position = translate(ego_position, +20.0, +lane_width)
        heading_vehicle = Vehicle('heading', model='autobello', licence='heading', color="yellow")
        scenario.add_vehicle(heading_vehicle, pos=(heading_vehicle_position.x, heading_vehicle_position.y, GROUND_LEVEL), rot=opposite_direction_of_the_road, rot_quat=None)

							 
										 
        # Create a vehicle in front of the ego, on the opposite lane, following the opposite direction
        opposite_vehicle_position = translate(ego_position, +10.0, +lane_width)
        opposite_vehicle = Vehicle('opposite', model='citybus', licence='opposite', color="white")

        scenario.add_vehicle(opposite_vehicle, pos=(opposite_vehicle_position.x, opposite_vehicle_position.y, GROUND_LEVEL), rot=direction_of_the_road, rot_quat=None)
		
        # Get current position and velocity via State
        state_sensor = State()
        ego_vehicle.attach_sensor('state', state_sensor)
        radius = 2 * LANE_WIDTH + 0.2
        # We place the target position for the test case just before the end of the road
        target_position = (road_nodes[-3][0], road_nodes[-3][1], road_nodes[-3][2])
        target_position_reached = TargetAreaOracle(target_position, radius, state_sensor)

        # Configure the sensors to monitor ego_car
        with BeamNGpy('localhost', 64256, home=BNG_HOME, user=BNG_USER) as bng:
            # Add some debug info
            bng.add_debug_spheres([target_position], [LANE_WIDTH], [(1, 1, 1, 0.2)])

            bng.set_steps_per_second(60)
            bng.set_deterministic()

            scenario.make(bng)

            bng.load_scenario(scenario)
            bng.start_scenario()
            bng.pause()

            bng.switch_vehicle(ego_vehicle)
            ego_vehicle.ai_set_mode('manual')
            ego_vehicle.ai_set_waypoint('target_wp')
            ego_vehicle.ai_drive_in_lane('true')


            speed_limit = 50 / 3.6
            ego_vehicle.ai_set_speed(speed_limit, mode='limit')

            while True:
                bng.step(60)
                ego_vehicle.poll_sensors()

                # Check Oracles
                if target_position_reached.check():
                    print("Car reached target location. Exit")
                    return


class shinetest(unittest.TestCase):
    # We drive the car using NVidia Dave2 - Credits to Precrime @ USI - Lugano
    # The driving agent runs in a separate process (we use the multiprocessing package)
    def test_driver_control_separate_process(self):

        # TODO This requires python 3.7 otherwise NVidia Driver will not work!
        # We use the Driving Agent from the DeepJanus project and that relies on a version
        # of tensorflow that is not available after Python 3.7

        direction_of_the_road, road_nodes = generate_road()
        # This is the node at the beginning
        initial_location = Point(road_nodes[0][0], road_nodes[0][1], road_nodes[0][2])
        # This is the node at the end of the road
        destination = (road_nodes[-1][0], road_nodes[-1][1], road_nodes[-1][2])

        road = Road('tig_road_rubber_sticky', rid='the_road')
        road.nodes.extend(road_nodes)

        scenario = Scenario('tig', 'shinetest')
        scenario.add_road(road)

        # Place the ego-car at the beginning of the road, in the middle of the right lane
        ego_position = translate(initial_location, +LANE_WIDTH * 0.5, 0.0)
        ego_position = translate(ego_position, 0.0, CAR_LENGTH)
        ego_vehicle = Vehicle('ego', model='etk800', licence='ego', color="green")
        scenario.add_vehicle(ego_vehicle, pos=(ego_position.x, ego_position.y, GROUND_LEVEL),
                             rot=direction_of_the_road, rot_quat=None)

        scenario.add_checkpoints([destination], [(1.0, 1.0, 1.0)], ids=["target_wp"])
		
        # Hardcoded coordinates
        

		
        direction_of_the_road = (0, 0 , 1, -1)
        opposite_direction_of_the_road = (0, 0, 1, 1)
        lane_width = 4.0
        length_car = 5.0 # approx
	

		# Create a vehicle in front of the ego-car, in same lane, following the same direction
        # We use the current position of the ego-car
        heading_vehicle_position = translate(ego_position, +20.0, +lane_width)
        heading_vehicle = Vehicle('heading', model='autobello', licence='heading', color="yellow")
        scenario.add_vehicle(heading_vehicle, pos=(heading_vehicle_position.x, heading_vehicle_position.y, GROUND_LEVEL), rot=opposite_direction_of_the_road, rot_quat=None)

							 
										 
        # Create a vehicle in front of the ego, on the opposite lane, following the opposite direction
        opposite_vehicle_position = translate(ego_position, +10.0, +lane_width)
        opposite_vehicle = Vehicle('opposite', model='citybus', licence='opposite', color="white")

        scenario.add_vehicle(opposite_vehicle, pos=(opposite_vehicle_position.x, opposite_vehicle_position.y, GROUND_LEVEL), rot=direction_of_the_road, rot_quat=None)      

			
            
        # Monitoring
        state_sensor = State()
        ego_vehicle.attach_sensor('state2', state_sensor)
        # We place the target position for the test case just before the end of the road
        target_position = (road_nodes[-4][0], road_nodes[-4][1], road_nodes[-4][2])
        radius = 2 * LANE_WIDTH + 0.2
        target_position_reached = TargetAreaOracle(target_position, radius, state_sensor)

        timer_sensor = Timer()
        ego_vehicle.attach_sensor('timer', timer_sensor)

        # Configure the sensors to monitor ego_car
        with BeamNGpy('localhost', 64256, home=BNG_HOME, user=BNG_USER) as bng:
            # Add some debug info
            bng.add_debug_spheres([target_position], [LANE_WIDTH], [(1, 1, 1, 0.2)])

            scenario.make(bng)

            bng.load_scenario(scenario)
            bng.start_scenario()
            bng.pause()

            bng.switch_vehicle(ego_vehicle)

            # Start the driver Using another venv
            print("Starting driver")
            # This is the h5 model that contains the trained weights for the AI
            model_file = ".\\self-driving-car-178-2020.h5"
            # Allows IPC and sync
            queue = Queue()

            driver_process = Process(target=drive_with_nvidia,
                                     # ego is the id of the vehicle
                                     args=(queue, 'ego', model_file, ))
            # Start in background
            driver_process.start()

            # Wait until the client started, max 30 sec
            print("Waiting for the driver to start...")
            queue.get(timeout=30)
            print("Ready...")
            # sleep(10)

            try:
                while True:
                    print("> Polling sensors")
                    ego_vehicle.poll_sensors()

                    # Use timer to filter out duplicate sensors
                    elapsed_time = timer_sensor.data['time']

                    if target_position_reached.check():
                        print("Car reached target location. Exit")
                        return

                    # Ensure monitoring is faster than driver
                    sleep(1)
            except Exception as e:
                print("ERROR", e)
            finally:
                # Kill the driver
                if driver_process.is_alive():
                    driver_process.terminate()
					

if __name__ == '__main__':
    unittest.main()