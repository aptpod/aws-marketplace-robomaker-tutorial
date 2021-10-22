import json
import boto3

from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False

client = boto3.client('secretsmanager')

def delete_secret(name):
    logger.info("delete secrets: {}".format(name))
    try:
        client.delete_secret(
            SecretId=name
        )
    except Exception:
        pass

def create_secret(name, secrets):
    try:
        secrets_str = json.dumps(secrets)

        l = client.list_secrets(
            Filters=[
                    {
                        'Key':'name',
                        'Values': [name]
                    }
                ]
            )
        if (len(l['SecretList']) != 0):
            client.update_secret(
                SecretId=l['SecretList'][0]['ARN'],
                SecretString=secrets_str
                )
            logger.info("create secrets: {}".format(name))
        else:
            response = client.create_secret(
                Name=name,
                SecretString=secrets_str
                )
    except Exception as e:
        logger.error(e)


def register_secrets(file_name):
    with open(file_name, 'r') as f:
        conf = json.load(f)

    for secret in conf:
        if (secret["host"] == ""):
            delete_secret(secret["secrets_name"])
        else:
            secret_to_register = {"host":secret["host"], "my_id":secret["my_id"], "my_secret":secret["my_secret"]}
            if ("ctlr_id" in secret):
                secret_to_register["ctlr_id"] = secret["ctlr_id"]
            if ("dst_id" in secret):
                secret_to_register["dst_id"] = secret["dst_id"]
            print(secret_to_register)
            create_secret(secret["secrets_name"], secret_to_register)

if __name__ == '__main__':
    file_name="secrets.conf"
    register_secrets(file_name)