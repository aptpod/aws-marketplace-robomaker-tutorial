example to launch intdash_edge_wrapper


```xml
  <!-- intdash edge wrapper for x86_64 -->
  <include file="$(find intdash_edge_wrapper)/launch/intdash_edge_wrapper.launch">
    <arg name="intdash_edge_path" value="$(find intdash_edge)"/>
    <arg name="manager_conf_file" value="manager.conf"/>
    <arg name="intdash_working_dir" value="$(env HOME)/intdash"/>
    <arg name="aws_secrets_manager_secret_name" value="intdash-robomaker-sample-simulation"/>
  </include>

```

- intdash_edge_path: path to intdash_edge ROS node
- manager_conf_file: the name of config file. This file shuold be placed in intdash_edge_path/opt/vm2m/etc
- intdash_working_dir: the place to put ./opt/vm2m
- aws_secrets_manager_secret_name: The name of secrets stored in AWS Secrets Manager 
