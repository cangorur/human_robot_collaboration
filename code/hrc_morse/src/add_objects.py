import math
from morse.builder import *
from morse.builder.robots.conveyor import *


def create_tray_sensor(x, y, z, t_id):
    tray_robot = FakeRobot()
    tray_robot.add_default_interface('ros')
    tray_robot.translate(x, y, z)

    tray_sensor = Proximity()
    tray_sensor.add_stream('ros', 'tray_sensor_publisher.TraySensorPublisher', tray_id=t_id,
                           topic='/production_line/tray_sensors')
    tray_sensor.properties(Range=0.5, Track='Package')
    tray_sensor.frequency(1)
    tray_sensor.translate(0, 0, 0)

    tray_robot.append(tray_sensor)


def create_tray(tray_id, color, x, y, orientation):
    # determine model
    if color == 'green':
        tray_model_file = 'processed_tray.blend'
        tray_model_name = 'ProcessedTray'
    if color == 'red':
        tray_model_file = 'unprocessed_tray.blend'
        tray_model_name = 'UnprocessedTray'

    input_tray = PassiveObject(tray_model_file, tray_model_name)
    input_tray.translate(x, y, 0.0)
    input_tray.rotate(0.0, 0.0, math.radians(orientation))

    # add tray sensor
    create_tray_sensor(x, y, 0.3, tray_id)


def create_conveyor(conveyor_config):

    if conveyor_config['type'] == "L":
        conveyor1 = Conveyor(conveyor_config['id'] + '/printer_part')
        conveyor1.translate(conveyor_config['location']['x'], conveyor_config['location']['y'], 0)
        conveyor2 = Conveyor(conveyor_config['id'] + '/assembly_part1')
        conveyor2.translate(conveyor_config['location']['x'] + 0.75, conveyor_config['location']['y'] - 1.6, 0) # -1.36
        conveyor2.rotate(z=1.57)
        conveyor3 = Conveyor(conveyor_config['id'] + '/assembly_part2')
        conveyor3.translate(conveyor_config['location']['x'] + 0.75 + 1.9, conveyor_config['location']['y'] - 1.6, 0) #-1.36
        conveyor3.rotate(z=1.57)
        
        conveyor1.add_overlay('ros', 'conveyor_overlay.ConveyorControlOverlay')
        conveyor1.add_service('socket')  # as all the bands are using the same blend file, they share the global parameters
        conveyor2.add_overlay('ros', 'conveyor_overlay.ConveyorControlOverlay')
        conveyor2.add_service('socket')  # as all the bands are using the same blend file, they share the global parameters
        conveyor3.add_overlay('ros', 'conveyor_overlay.ConveyorControlOverlay')
        conveyor3.add_service('socket')  # as all the bands are using the same blend file, they share the global parameters

    else:  # tip to tip connection of the belts
        conveyor1 = Conveyor(conveyor_config['id'] + '/printer_part')
        conveyor1.translate(conveyor_config['location']['x'], conveyor_config['location']['y'], 0)
        conveyor2 = Conveyor(conveyor_config['id'] + '/assembly_part')
        conveyor2.translate(conveyor_config['location']['x'], conveyor_config['location']['y'] - 1.8, 0)
        conveyor1.add_overlay('ros', 'conveyor_overlay.ConveyorControlOverlay')
        conveyor1.add_service('socket')  # as all the bands are using the same blend file, they share the global parameters
        conveyor2.add_overlay('ros', 'conveyor_overlay.ConveyorControlOverlay')

    
def create_package_pool(package_config):
    # this is to stack all the packages into a given location
    number_lightPkgs = package_config['amount']['light']
    number_heavyPkgs = package_config['amount']['heavy']

    package_length = 0.251
    height = 4
    p_pos = package_config['location']
    x_positions = list(frange(p_pos['x'], p_pos['x'] + 1, package_length))
    y_positions = list(frange(p_pos['y'], p_pos['y'] + 2, package_length))

    package_count = 0
    prefix = 'pkg'
    for y in y_positions:
        for x in x_positions:
            z = 0.15
            for i in range(height):
                if package_count < number_lightPkgs:
                    type_str = prefix + "Light"
                    add_package(type_str, type_str + '_' + str(package_count + 1), x, y, z)
                    package_count = package_count + 1
                    z = z + package_length
                elif package_count < number_lightPkgs + number_heavyPkgs:
                    type_str = prefix + "Heavy"
                    add_package(type_str, type_str + '_' + str(package_count + 1), x, y, z)
                    package_count = package_count + 1
                    z = z + package_length
                else:
                    break

def add_package(package_type, package_name, x, y, z):
    # TODO adjust blender file path to package type
    package = PassiveObject(package_type + '.blend', package_type)
    package.setgraspable()
    package.name = package_name
    package.translate(x, y, z)

def frange(start, stop, step):
    i = start
    while i <= stop:
        yield i
        i += step
