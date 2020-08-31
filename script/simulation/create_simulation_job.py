import sys
import boto3

from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

client = boto3.client('robomaker')
iam_client = boto3.client('iam')

cloudformation = boto3.resource('cloudformation')
stack = cloudformation.Stack('intdash-robomaker-sample')

def get_simulation_application_arn(name):
    try:
        response = client.list_simulation_applications(
            filters = [
                {
                    "name" : 'name',
                    "values": [name],
                }
            ]
        )
        return response['simulationApplicationSummaries'][0]['arn']
    except Exception as e:
        logger.error(e)

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
        
def get_iam_role_arn(name):
    try:
        response = iam_client.get_role(RoleName=name)
        return response['Role']['Arn']
    except Exception as e:
        logger.error(e)

def create_simulation_job(robot_application_name, simulation_application_name, iam_role_name, s3Bucket, subnets, securityGroups,
                            launch_config):
    try:
        robot_application_arn = get_robot_application_arn(robot_application_name)
        simulation_application_arn = get_simulation_application_arn(simulation_application_name)
        iam_role_arn = get_iam_role_arn(iam_role_name)

        response = client.create_simulation_job(
            outputLocation={
                's3Bucket': s3Bucket,
            },
            loggingConfig={
                'recordAllRosTopics': True
            },
            maxJobDurationInSeconds=3600,
            iamRole=iam_role_arn,
            failureBehavior='Fail',
            robotApplications=[
                {
                    'application': robot_application_arn,
                    'launchConfig': launch_config
                },
            ],
            simulationApplications=[
                {
                    'application': simulation_application_arn, 
                    'launchConfig': {
                        'packageName': 'launcher',
                        'launchFile': 'turtlebot3.launch',
                        'environmentVariables': {
                            'TURTLEBOT3_MODEL': 'waffle_pi'
                        }
                    }
                },
            ],
            tags={
                'Project': 'RoboMaker'
            },
            vpcConfig={
                'subnets': [
                    subnets[0], subnets[1]
                ],
                'securityGroups': [
                    securityGroups
                ],
                'assignPublicIp': True
            }
        )
    except Exception as e:
        logger.error("Create simulation job failed")
        logger.error(e)

def create_simulation_job_rviz(simulation_application_name, iam_role_name, s3Bucket, subnets, securityGroups):
    try:
        simulation_application_arn = get_simulation_application_arn(simulation_application_name)
        iam_role_arn = get_iam_role_arn(iam_role_name)

        response = client.create_simulation_job(
            outputLocation={
                's3Bucket': s3Bucket,
            },
            loggingConfig={
                'recordAllRosTopics': True
            },
            maxJobDurationInSeconds=3600,
            iamRole=iam_role_arn,
            failureBehavior='Fail',
            simulationApplications=[
                {
                    'application': simulation_application_arn, 
                    'launchConfig': {
                        'packageName': 'launcher',
                        'launchFile': 'rviz.launch',
                        'environmentVariables': {
                            'TURTLEBOT3_MODEL': 'waffle_pi',
                            'INTDASH_EDGE_ROOT': '/home/robomaker/intdash',
                        }
                    }
                },
            ],
            tags={
                'Project': 'RoboMaker'
            },
            vpcConfig={
                'subnets': [
                    subnets[0], subnets[1]
                ],
                'securityGroups': [
                    securityGroups
                ],
                'assignPublicIp': True
            }
        )
    except Exception as e:
        logger.error("Create simulation job failed")
        logger.error(e)




if __name__ == "__main__":
    mode = None
    try:
        if (len(sys.argv) < 2):
            raise "Target is not described"
        if (sys.argv[1] == "turtlebot3"):
            mode="turtlebot3"
            launch_config = {
                        'packageName': 'launcher',
                        'launchFile': 'turtlebot3_simulation.launch',
                        'environmentVariables': {
                            'TURTLEBOT3_MODEL': 'waffle_pi',
                            'INTDASH_EDGE_ROOT': '/home/robomaker/intdash',
                        },
                    }
        elif (sys.argv[1] == "turtlebot3_intdash_bridge"):
            mode = sys.argv[1]
            launch_config = {
                        'packageName': 'launcher',
                        'launchFile': 'turtlebot3_simulation_intdash_bridge.launch',
                        'environmentVariables': {
                            'TURTLEBOT3_MODEL': 'waffle_pi',
                            'INTDASH_EDGE_ROOT': '/home/robomaker/intdash',
                        },
                    }
        elif (sys.argv[1] == "rviz"):
            mode = "rviz"
        else:
            raise "Not defined"
    except Exception as e:
        logger.error(e)
        exit(1)
    
    try:
        outputs = stack.outputs
        subnets = []
        securityGroups = None
        s3_bucket_name = None
        iam_role_name = None
        for d in outputs:
            if (d['OutputKey'] == 'IntdashRoboMakerSampleSubnet1'):
                subnets.append(d['OutputValue'])
            elif (d['OutputKey'] == 'IntdashRoboMakerSampleSubnet2'):
                subnets.append(d['OutputValue'])
            elif (d['OutputKey'] == 'DefaultSecurityGroupID'):
                securityGroups = d['OutputValue']
            elif (d['OutputKey'] == 'IntdashRoboMakerS3BucketName'):
                s3_bucket_name = d['OutputValue']
            elif (d['OutputKey'] == 'IntdashRoboMakerSampleRole'):
                iam_role_name = d['OutputValue']
    except Exception as e:
        logger.error(e)

    if (mode == "turtlebot3" or mode == "turtlebot3_intdash_bridge"):
        create_simulation_job(
                robot_application_name="intdash_sample_robot",
                simulation_application_name='intdash_sample_simulation',
                iam_role_name=iam_role_name,
                s3Bucket=s3_bucket_name,
                subnets = subnets,
                securityGroups=securityGroups,
                launch_config=launch_config
            )
    elif (mode == "rviz"):
        create_simulation_job_rviz(
                simulation_application_name='intdash_sample_rviz',
                iam_role_name=iam_role_name,
                s3Bucket=s3_bucket_name,
                subnets = subnets,
                securityGroups=securityGroups,
            )
    
