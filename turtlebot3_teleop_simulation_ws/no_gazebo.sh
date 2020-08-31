#!/bin/bash
export BUCKET_NAME="robomaker-stable-devel"  # <= バンドルを保存する S3バケットの名前 
export S3KEY="rosbag_test"  # <= バンドルを保存する S3バケットのキー名
export ROLE_ARN="arn:aws:iam::263480619405:role/RoboMakerStableDevel"  # <= シミュレーションを実行する際の IAM ロールの ARN
export LOGBASED_SIM_APP_NAME="rosbag_only_test2"  # <= 任意のシミュレーションアプリケーションの名前 turtlebot_logbased_simulation_app  など
export PACKAGE_NAME="launcher"  # <= 1で作成した launch ファイルが属しているパッケージ
export LAUNCH_FILE="rviz_test_no_gazebo.launch"  # <= 1で作成した launch ファイル

#simulation_ws 配下のバンドルファイルをS3にアップロード。パスは任意に変更してください。
aws s3 cp ./bundle/output.tar s3://${BUCKET_NAME}/${S3KEY}/simulation_ws/bundle/output.tar

#Create log-based simulation app (This time, we use normal simulation application bundle for log-based simulation application,too)
aws robomaker create-simulation-application \
  --name $LOGBASED_SIM_APP_NAME \
  --sources "s3Bucket=$BUCKET_NAME,s3Key=${S3KEY}/simulation_ws/bundle/output.tar,architecture=X86_64" \
  --simulation-software-suite "name=RosbagPlay,version=Kinetic" \
  --robot-software-suite "name=ROS,version=Kinetic"

#Find ARN name for robot application and log-based simulation app
sim_app=`aws robomaker list-simulation-applications | grep -Po '"\K(arn:.*'${LOGBASED_SIM_APP_NAME}'.*)"' | sed 's/.$//'`

template='{"iamRole":"%s",
    "outputLocation":{"s3Bucket":"%s","s3Prefix":"%s"},
    "maxJobDurationInSeconds":3600,
    "failureBehavior":"Fail",
    "simulationApplications":[{"application":"%s","applicationVersion":"$LATEST",
        "launchConfig":{"packageName":"%s","launchFile":"%s",
        "environmentVariables": {"TURTLEBOT3_MODEL": "waffle_pi"}}}],
    "maxJobDurationInSeconds": 3600,
  "vpcConfig": {
    "subnets": [
      "subnet-03d07560f248c91ed",
      "subnet-003a5442b56501dcc"
    ],
    "securityGroups": [
      "sg-0b6905171499f0c83"
    ],
    "assignPublicIp": true
  },
  "tags": {
    "Project": "RoboMaker"
  }
}'

json_params=$(printf "$template" "$ROLE_ARN" "$BUCKET_NAME" "$S3KEY" "$sim_app" "$PACKAGE_NAME" "$LAUNCH_FILE" )

echo $json_params
aws robomaker create-simulation-job --cli-input-json "$json_params"