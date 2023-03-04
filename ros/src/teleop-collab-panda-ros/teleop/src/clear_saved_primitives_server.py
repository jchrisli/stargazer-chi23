#!/usr/bin/env python

import rospy
import os
import glob
from teleop_msgs.srv import ClearMP, ClearMPResponse


class ClearPrimitives(object):
    def __init__(self):
        self.node = rospy.init_node("clear_primitives_server_node")
        self.clear_service = rospy.Service("clear_primitives_service", ClearMP, self.clear_saved)
        #self.primitive_save_dir = "\\wsl.localhost\\Ubuntu-18.04\\home\karthikm\\catkin_ws\\src\\teleop-collab-panda-ros\\teleop\\src\\saved_promps"
        self.promp_save_dir = "/home/karthikm/catkin_ws/src/teleop-collab-panda-ros/teleop/src/saved_promps/"
        self.dmp_save_dir = "/home/karthikm/catkin_ws/src/ros_dmp/dmp/weights/"
        
        print(self.promp_save_dir)
        print(self.dmp_save_dir)
        rospy.spin()

    def clear_saved(self, request):
        # PROMP
        if request.primitive_name.data == "":
            # Clear all
            '''for root, dirs, files in os.walk(self.promp_save_dir):
                for filename in files:
                    if filename.endswith(".json"):
                        os.remove(self.promp_save_dir + filename)'''
            self.delete_all_files(self.promp_save_dir, ".json")
            self.delete_all_files(self.dmp_save_dir, ".yaml")

        else:
            # Only clear that primitive
            self.delete_file(self.promp_save_dir, request.primitive_name.data, ".json")
            self.delete_file(self.dmp_save_dir, request.primitive_name.data, ".yaml")
            #primitive_filename = self.primitive_save_dir + request.primitive_name.data + ".json"
            #if os.path.isfile(primitive_filename):
                #os.remove(primitive_filename)

        response = ClearMPResponse()
        response.success.data = True

        return response
    
    def delete_all_files(self, save_dir, file_type):
        # Clear all
        for root, dirs, files in os.walk(save_dir):
            for filename in files:
                if filename.endswith(file_type):
                    os.remove(save_dir + filename)

    def delete_file(self, save_dir, requested_file, file_type):
        primitive_filename = save_dir + requested_file + file_type

        if os.path.isfile(primitive_filename):
            os.remove(primitive_filename)


def main():
    ClearPrimitives()


if __name__ == '__main__':
    main()