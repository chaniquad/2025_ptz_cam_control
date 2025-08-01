# ptz_camera_control

--------------------------------------------------------------------------------------------------------------------------------

# 1) 먼저 좌표를 이미지쪽에서 퍼블리쉬해준다고 가정하고 해당 좌표를 퍼블리쉬해준다.좌표는 변경 가능하다.

ros2 topic pub /target_point geometry_msgs/PointStamped   "{header: {frame_id: 'base_link'}, point: {x: 1.6, y: -3.0, z: 9.0}}"


# 2) 좌표를 바라보게하는 코드를 실행.

 ros2 run ptz_playground pan_tilt_controller

 -------------------------------------------------------------------------------------------------------------------------------

  ## 3) 수동 조절 gui띄우는 법
  
 ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args  -p robot_description:="$(< ~/ptz_ws/src/ptz_urdf/urdf/pan_tilt.urdf)"
  
