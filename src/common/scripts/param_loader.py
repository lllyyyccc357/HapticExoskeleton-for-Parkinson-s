#!/usr/bin/env python
import rospy
import yaml
import os

def load_params(yaml_path):
    with open(yaml_path, 'r') as file:
        params = yaml.safe_load(file)
        for key, value in params.items():
            rospy.set_param(key, value)
        rospy.loginfo("Parameters loaded from %s", yaml_path)

if __name__ == '__main__':
    rospy.init_node('common_param_loader')
    yaml_file = rospy.get_param('~config_file', '')
    if yaml_file and os.path.exists(yaml_file):
        load_params(yaml_file)
    else:
        rospy.logerr("Invalid or missing YAML path: %s", yaml_file)
