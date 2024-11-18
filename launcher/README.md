# roslaunch like launching of many processes via toml config file

```
[[arg]]
name = "topic"
default = "/test"

[[arg]]
name = "value"
default = "test1"

[[cmd]]
name = "rostopic pub string"
command = "rostopic"
args = ["pub", "$(arg topic)", "std_msgs/String", "\"data: '$(arg value)'\"", "-r", "1"]

[[cmd]]
name = "rostopic echo string"
command = "rostopic"
args = ["echo", "$(arg topic)"]
```

```
launcher examples/ros_pub_echo.toml topic:=foo value:=blah
```
