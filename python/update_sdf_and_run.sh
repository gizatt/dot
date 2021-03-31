xacro4sdf ../models/dot_control.sdf.xmacro
python run_direct_joint_teleop.py --yaml_path=../models/dot_servo_config.yaml --meshcat=default --interactive

