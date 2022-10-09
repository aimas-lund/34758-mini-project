#!/usr/bin/env python

import rospy, tf, random, os
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState, GetWorldProperties
from geometry_msgs.msg import *
from pubobject import publish_object

class Spawner:

    _DIR_NAME = os.path.dirname(__file__)
    _WORLD = "world"
    _CUBE_NAME = "cube"
    _BUCKET_NAME = "bucket"

    _ORIENT = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))
    _BUCKET_POSE = Pose(Point(x=0.53, y=-0.23, z=0.78), _ORIENT)

    def __init__(self):
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")

        print("Initializing rospy service proxies...")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)


    def spawn_models(self):
        self.spawn_cubes()
        self.spawn_bucket()

        
    def spawn_cubes(self):
        cube_urdf_path = os.path.join(self._DIR_NAME, "../urdf/cube.urdf")
        with open(cube_urdf_path, "r") as f:
            product_xml = f.read()

        num_of_cubes = random.randint(2,6)

        for num in xrange(0,num_of_cubes):
            bin_y   =   random.uniform(0,0.5)
            bin_x   =   random.uniform(0,0.5)
            item_name   =   "{}{}".format(self._CUBE_NAME, num)

            print("Spawning model:%s", item_name)
            item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   self._ORIENT)
            self.spawn_model(item_name, product_xml, "", item_pose, self._WORLD)

            #Publish cube
            publish_object(item_name,item_pose)


    def spawn_bucket(self, pose=_BUCKET_POSE):
        bucket_urdf_path = os.path.join(self._DIR_NAME, "../urdf/bucket.urdf")
        with open(bucket_urdf_path, "r") as f:
            product_xml = f.read()

        print("Spawning model:%s", self._BUCKET_NAME)
        self.spawn_model(self._BUCKET_NAME, product_xml, "", pose, self._WORLD)
        #Publish bucket
        publish_object(self._BUCKET_NAME, pose)


    def despawn_cubes(self):
        model_list = self.world_properties()
        for model in model_list.model_names:
            if self._CUBE_NAME not in model:
                continue

            try:
                self.delete_model(str(model))
            except:
                print("Could not find model '{}' in Gazebo".format(str(model)))


if __name__ == '__main__':
    spawner = Spawner()
    spawner.spawn_cubes()
    spawner.spawn_bucket()
