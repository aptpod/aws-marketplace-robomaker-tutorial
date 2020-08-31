import boto3

from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

client = boto3.client('cloudformation')

def create_simulation_resource(resource_file_name, name):
    try:
        with open(resource_file_name) as f:
            template_body = f.read()
            response = client.create_stack(
                StackName=name,
                TemplateBody=template_body,
                Capabilities=['CAPABILITY_NAMED_IAM']
            )
        
    except client.exceptions.AlreadyExistsException:
        logger.error("Stack has already been created")
    except Exception as e:
        logger.error(e)

if __name__ == "__main__":
    resource_file_name = "aws_resource.yaml"
    stack_name = "intdash-robomaker-sample"
    create_simulation_resource(resource_file_name, stack_name)