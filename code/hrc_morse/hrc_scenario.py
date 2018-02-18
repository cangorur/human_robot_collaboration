from morse.builder import *
from add_objects import *
import json

#import rospy

# reading the map and scenario configuration from the config json file:
scenario_config_file = os.path.join(os.path.dirname(__file__), '../configs/scenario_config.json')

# read configurations from JSON files
with open(scenario_config_file, 'r') as f:
    scenario_config = json.load(f)
create_package_pool(scenario_config['package_pool'])

create_conveyor(scenario_config['conveyor'])

# Creating a human model
human = Human()
human_x = scenario_config['conveyor']['location']['x'] + 2.7
human_y = scenario_config['conveyor']['location']['y'] - 0.74 # -0.5
human.translate(human_x, human_y, 0)
human.rotate(0, 0, -1.57)
human.add_overlay('ros', 'human_overlay.HumanControlAndMonitor')
human.disable_keyboard_control()
human.use_world_camera()
# Default interface
human.add_default_interface('ros')
# human.add_service('socket')
human.add_service('ros')


# Creating a PR2 robot model

robot = LocalizedPR2()
robot_x = scenario_config['conveyor']['location']['x'] + 2.7
robot_y = scenario_config['conveyor']['location']['y'] - 2.54 # -2.3
robot.translate(robot_x, robot_y, 0.05)
robot.rotate(0, 0, 1.57)
robot.add_overlay('ros', 'robot_overlay.RobotControlAndMonitor')
# Default interface
robot.add_default_interface('ros')
# Service
robot.add_service('ros')

create_tray("tray_human", "green", human_x - 0.8, human_y, -90)
create_tray("tray_robot", "green", robot_x - 0.8, robot_y, 90)
create_tray("tray_unprocessed", "red", scenario_config['conveyor']['location']['x'] + 4.17,
            scenario_config['conveyor']['location']['y'] - 1.6, -180) # -1.36

### Packages added below are to take a scenario picture ###

package1 = PassiveObject('pkgLight.blend', 'pkgLight')
package1.setgraspable()
package1.translate(scenario_config['conveyor']['location']['x'] + 0.7, scenario_config['conveyor']['location']['y'] - 1.6, 0.8)
# package1.translate(scenario_config['conveyor']['location']['x'] + 2.7, scenario_config['conveyor']['location']['y'] - 1.6, 0.8)
'''
package = PassiveObject('pkgLight.blend', 'pkgLight')
package.setgraspable()
package.translate(scenario_config['conveyor']['location']['x'], scenario_config['conveyor']['location']['y'] + 0.2, 0.6)

package2 = PassiveObject('pkgLight.blend', 'pkgLight')
package2.setgraspable()
package2.translate(human_x - 0.6, human_y, 1)

package3 = PassiveObject('pkgHeavy.blend', 'pkgHeavy')
package3.setgraspable()
package3.translate(robot_x - 0.6, robot_y, 0.6)

package4 = PassiveObject('pkgHeavy.blend', 'pkgHeavy')
package4.setgraspable()
package4.translate(robot_x - 0.6, robot_y, 1)

package5 = PassiveObject('pkgLight.blend', 'pkgLight')
package5.setgraspable()
package5.translate(scenario_config['conveyor']['location']['x'] + 4.1, scenario_config['conveyor']['location']['y'] - 1.6, 0.6)

package6 = PassiveObject('pkgHeavy.blend', 'pkgHeavy')
package6.setgraspable()
package6.translate(human_x - 1.7, scenario_config['conveyor']['location']['y'] - 1.6, 0.6)

package8 = PassiveObject('pkgHeavy.blend', 'pkgHeavy')
package8.setgraspable()
package8.translate(robot_x - 0.6, robot_y, 1.4)

package9 = PassiveObject('pkgLight.blend', 'pkgLight')
package9.setgraspable()
package9.translate(scenario_config['conveyor']['location']['x'], scenario_config['conveyor']['location']['y'] - 1.6, 1)
'''
###########################################################

# Import, configure and place a static object from 'kitchen_objects.blend'.
# cornflakes = PassiveObject("props/kitchen_objects", "Cornflakes")
# cornflakes.setgraspable()
# cornflakes.translate(scenario_config['conveyor']['location']['x'] - 0.3,
#                     scenario_config['conveyor']['location']['y'] + 0.2, 1)
# cornflakes.rotate(0, 0, 1.55)


# Set the environment
env = Environment('laas/grande_salle', fastmode = False)
env.set_camera_location([3.2, -3.7, 3.5])
env.set_camera_rotation([0.95, 0, -1.2])

# Set the environment
# env = Environment('laas/grande_salle', fastmode = False)
# env.set_camera_location([10, -3.7, 3.5])
# env.set_camera_rotation([0.95, 0, 1.2])
