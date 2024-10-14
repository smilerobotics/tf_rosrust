```
ROS_PACKAGE_PATH=`rospack find geometry_msgs`:`rospack find tf2_msgs`:`rospack find sensor_msgs`:`rospack find std_msgs`:`rospack find actionlib_msgs` cargo build --release -p tf_roslibrust
```

https://github.com/Carter12s/roslibrust/discussions/177

https://github.com/smilerobotics/tf_rosrust/pull/55

```
export ROS_PACKAGE_PATH=`rospack find geometry_msgs`:`rospack find tf2_msgs`:`rospack find std_msgs`:`rospack find actionlib_msgs`
```

## tf echo

Get the transform from one frame to another like tf2_utils echo.py:

```
echo frame1 frame2
```

# tf publisher

```
cargo run --release --bin tf_publisher -- tf_roslibrust/examples/transforms.toml
```

# tf2tfs

```
cargo run --release --bin tf_publisher -- tf_roslibrust/examples/transforms.toml
cargo run --release --bin tf2tfs -- tf_roslibrust/examples/tf2tfs.toml
```

Then try `tf_echo test0 test1` and `tf_echo test0 test1_zerod` to see if echoed values are as expected.
