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
s3 = boto3.resource('s3')

cloudformation = boto3.resource('cloudformation')
stack = cloudformation.Stack('intdash-robomaker-sample')

def robot_application_exist(s3Bucket, s3Key):
    try:
        s3.Object(s3Bucket, s3Key).load()
        return True
    except Exception as e:
        logger.error(e)
        return False

def get_robot_application_sources(robot_arn, architecture):
    try:
        response = client.describe_robot_application(
            application=robot_arn,
        )
        sources = []
        for s in response['sources']:
            if (s['architecture'] != architecture and 
            robot_application_exist(s['s3Bucket'], s['s3Key'])):
                sources.append(s)
        return sources            
    except Exception as e:
        logger.error(e)
        return None

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

def create_robot_application(name, s3Bucket, s3Key):
    sources = [
        {
            's3Bucket': s3Bucket,
            's3Key': s3Key,
            'architecture': 'X86_64'
        }
    ]
    try:
        response = client.create_robot_application(
                name=name,
                sources=sources,
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': 'Kinetic'
                }
            )
    except client.exceptions.ResourceAlreadyExistsException:
        application = get_robot_application_arn(name)
        tmp = get_robot_application_sources(application, "X86_64")
        if (not (tmp is None)):
            sources.extend(tmp)
        response = client.update_robot_application(
            application=application,
            sources=[
                {
                    's3Bucket': s3Bucket,
                    's3Key': s3Key,
                    'architecture': 'X86_64'
                }
            ],
            robotSoftwareSuite={
                'name': 'ROS',
                'version': 'Kinetic'
            }
        )
    except Exception as e:
        logger.error(e)

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

def create_simulation_application(name, s3Bucket, s3Key, simulationSoftwareSuite):
    try:
        if (simulationSoftwareSuite['name'] == 'RosbagPlay'):
            response = client.create_simulation_application(
                name=name,
                sources=[
                    {
                        's3Bucket': s3Bucket,
                        's3Key': s3Key,
                        'architecture': 'X86_64'
                    },
                ],
                simulationSoftwareSuite=simulationSoftwareSuite,
    
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': 'Kinetic'
                }
            )
        else:
            response = client.create_simulation_application(
                name=name,
                sources=[
                    {
                        's3Bucket': s3Bucket,
                        's3Key': s3Key,
                        'architecture': 'X86_64'
                    },
                ],
                simulationSoftwareSuite=simulationSoftwareSuite,
    
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': 'Kinetic'
                },
                renderingEngine={
                    'name': 'OGRE',
                    'version': '1.x'
                }
            )

    except client.exceptions.ResourceAlreadyExistsException:
        application=get_simulation_application_arn(name)
        if (simulationSoftwareSuite['name'] == 'RosbagPlay'):
            response = client.update_simulation_application(
                application=application,
                sources=[
                    {
                        's3Bucket': s3Bucket,
                        's3Key': s3Key,
                        'architecture': 'X86_64'
                    },
                ],
                simulationSoftwareSuite=simulationSoftwareSuite,
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': 'Kinetic'
                }
            )
        else:
            response = client.update_simulation_application(
                application=application,
                sources=[
                    {
                        's3Bucket': s3Bucket,
                        's3Key': s3Key,
                        'architecture': 'X86_64'
                    },
                ],
                simulationSoftwareSuite={
                    'name': 'Gazebo',
                    'version': '7'
                },
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': 'Kinetic'
                },
                renderingEngine={
                    'name': 'OGRE',
                    'version': '1.x'
                }
            )

    except Exception as e:
        logger.error(e)



if __name__ == "__main__":
    try:
        if (len(sys.argv) < 2):
            raise "Target is not described"
        if (sys.argv[1] == "turtlebot3"):
            robot_application_name = "intdash_sample_robot"
            robot_application_s3_key = robot_application_name
            simulation_application_name = "intdash_sample_simulation"
            simulation_application_s3_key = simulation_application_name
            simulation_software_suite = {
                'name': 'Gazebo',
                'version': '7'
            }
        elif (sys.argv[1] == "rviz"):
            robot_application_name = None
            simulation_application_name = "intdash_sample_rviz"
            simulation_application_s3_key = simulation_application_name
            simulation_software_suite = {
                'name': 'RosbagPlay',
                'version': 'Kinetic'
            }
        else:
            raise "Not defined"
    except Exception as e:
        logger.error(e)
        exit(1)

    try:
        s3_bucket_name = None
        outputs = stack.outputs
        for d in outputs:
            if (d['OutputKey'] == 'IntdashRoboMakerS3BucketName'):
                s3_bucket_name = d['OutputValue']

        bundle_info = []
        if (not (robot_application_name is None)):
            bundle_info.append(
            {
                'workspace': 'turtlebot3_teleop_robot_ws',
                's3Key' : robot_application_s3_key
            }
            )
        
        bundle_info.append(
                {
                    'workspace': 'turtlebot3_teleop_simulation_ws',
                    's3Key'    : simulation_application_s3_key
                }
            )

        for info in bundle_info:
            s3.meta.client.upload_file("../../{}/bundle/output.tar".format(info['workspace']), s3_bucket_name, "{}/output.tar".format(info['s3Key']))
    except Exception as e:
        logger.error(e)
    
    if (not (robot_application_name is None)):
        create_robot_application(
            robot_application_name,
            s3_bucket_name, 
            "{}/output.tar".format(robot_application_s3_key)
            )
        
    create_simulation_application(
            simulation_application_name, 
            s3_bucket_name,
            "{}/output.tar".format(simulation_application_s3_key),
            simulation_software_suite
        )
