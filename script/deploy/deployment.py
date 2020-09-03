import boto3
import sys

from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

client = boto3.client('robomaker')
cloudformation = boto3.resource('cloudformation')
stack = cloudformation.Stack('intdash-robomaker-sample-deploy')

def get_robot_arn(name):
    try:
        response = client.list_robots()
        #print(response)
        for robot in response['robots']:
            if (robot['name'] == name):
                return robot['arn']
        raise "Robot not found"
    except Exception as e:
        logger.error(e)
        exit(1)

def get_robot_application_arn(name):
    try:
        response = client.list_robot_applications(
            filters = [
                {
                    "name" : 'name',
                    "values": [name],
                }
            ]
        )
        return response['robotApplicationSummaries'][0]['arn']
    except Exception as e:
        logger.error(e)

def create_fleet(name):
    try:
        response = client.create_fleet(
            name=name
            )
        return response['arn']
    except Exception as e:
        logger.error(e)

def register_robot(fleet_arn, robot_arn):
    try:
        reponse = client.register_robot(
            fleet=fleet_arn,
            robot=robot_arn
            )
    except Exception as e:
        logger.error(e)

def create_robot_application_version(robot_application_arn):
    try:
        response = client.create_robot_application_version(
            application = robot_application_arn
        )
        return response['version']
    except client.exceptions.LimitExceededException:
        logger.error("Robot Application versions exceeds limits. Please remove version from AWS RoboMaker Console")
    except Exception as e:
        logger.error(e)
        
def create_deployment_job(robot_application_arn, fleet_arn, version, launchConfig):
    try:
        response = client.create_deployment_job(
            fleet=fleet_arn,
            deploymentApplicationConfigs=[
                {
                    'application': robot_application_arn,
                    'applicationVersion': version,
                    'launchConfig': launchConfig
                },
            ]
        )
    except Exception as e:
        logger.error(e)       


if __name__ == '__main__':
    mode = None
    try:
        if (len(sys.argv) < 2):
            raise "Target is not described"
        if (sys.argv[1] == "turtlebot3"):
            fleet_name= "intdash_sample_robot_fleet"
            robot_name = "IntdashSampleRobot"
            robot_application_name = "intdash_sample_robot"
            launchConfig = {
                        'packageName': 'launcher',
                        'launchFile': 'turtlebot3_raspi.launch',
                        'environmentVariables': {
                            'INTDASH_EDGE_ROOT': '/home/ggc_user/intdash',
                            'TURTLEBOT3_MODEL' : 'waffle_pi'
                        }
                    }
        elif (sys.argv[1] == "turtlebot3_rviz"):
            fleet_name= "intdash_sample_robot_fleet"
            robot_name = "IntdashSampleRobot"
            robot_application_name = "intdash_sample_robot"
            launchConfig = {
                        'packageName': 'launcher',
                        'launchFile': 'turtlebot3_raspi_rviz.launch',
                        'environmentVariables': {
                            'INTDASH_EDGE_ROOT': '/home/ggc_user/intdash',
                            'TURTLEBOT3_MODEL' : 'waffle_pi'
                        }
                    }
        elif (sys.argv[1] == "controller"):
            fleet_name= "intdash_sample_controller_fleet"
            robot_name="IntdashSampleController"
            robot_application_name = "intdash_sample_controller"
            launchConfig = {
                        'packageName': 'launcher',
                        'launchFile': 'controller.launch',
                        'environmentVariables': {
                            'INTDASH_EDGE_ROOT': '/home/ggc_user/intdash',
                        }
                    }
        else:
            raise "invalid name"
    except Exception as e:
        logger.error(e)
        exit(1)

    robot_arn = get_robot_arn(robot_name)
    fleet_arn = create_fleet(fleet_name)
    register_robot(fleet_arn, robot_arn)

    robot_application_arn = get_robot_application_arn(robot_application_name)
    version = create_robot_application_version(robot_application_arn)
    
    create_deployment_job(robot_application_arn, fleet_arn, version, launchConfig)