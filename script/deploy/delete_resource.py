import boto3

from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

client = boto3.client('cloudformation')

def delete_simulation_resource(name):
    try:
        response = client.delete_stack(
            StackName=name, 
        )
        
    except client.exceptions.AlreadyExistsException:
        logger.error("Stack has already been created")
    except Exception as e:
        logger.error(e)

if __name__ == "__main__":
    stack_name = "intdash-robomaker-sample-deploy"
    delete_simulation_resource(stack_name)