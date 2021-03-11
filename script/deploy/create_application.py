import boto3
import os
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
stack = cloudformation.Stack('intdash-robomaker-sample-deploy')

def get_ros_distro():
    return os.environ['ROS_DISTRO'].capitalize()

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

def create_robot_application(name, s3Bucket, s3Key):
    sources = [
        {
            's3Bucket': s3Bucket,
            's3Key': s3Key,
            'architecture': 'ARMHF'
        }
    ]

    try:
        response = client.create_robot_application(
                name=name,
                sources=sources,
                robotSoftwareSuite={
                    'name': 'ROS',
                    'version': get_ros_distro()
                }
            )
    except client.exceptions.ResourceAlreadyExistsException:
        application = get_robot_application_arn(name)
        tmp = get_robot_application_sources(application, "ARMHF")
        if (not (tmp is None)):
            sources.extend(tmp)

        response = client.update_robot_application(
            application=application,
            sources=sources,
            robotSoftwareSuite={
                'name': 'ROS',
                'version': get_ros_distro()
            }
        )
    except Exception as e:
        logger.error(e)



if __name__ == "__main__":
    mode = None
    try:
        if (len(sys.argv) < 2):
            raise "Target is not described"
        if (sys.argv[1] == "turtlebot3"):
            workspace = "turtlebot3_teleop_robot_ws"
            s3Key = 'intdash_sample_robot_armhf'
            robot_application_name = "intdash_sample_robot"
        elif (sys.argv[1] == "controller"):
            workspace = "controller_ws"
            s3Key = 'intdash_sample_controller_armhf'
            robot_application_name = "intdash_sample_controller"
    except Exception as e:
        logger.error(e)
        exit(1)
    
    try:
        s3_bucket_name = None
        outputs = stack.outputs
        for d in outputs:
            if (d['OutputKey'] == 'IntdashRoboMakerSampleDeployS3BucketName'):
                s3_bucket_name = d['OutputValue']
        
        bundle_info = [
            {
                'workspace': workspace,
                's3Key' : s3Key
            }
            ]
    
        for info in bundle_info:
            s3.meta.client.upload_file("../../{}/armhf_bundle/output.tar".format(info['workspace']), s3_bucket_name, "{}/output.tar".format(info['s3Key']))
    except Exception as e:
        logger.error(e)
    
    create_robot_application(
        robot_application_name,
        s3_bucket_name, 
        "{}/output.tar".format(s3Key)
        )

