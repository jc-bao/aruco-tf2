#!/usr/bin/env python  
import rospy
import numpy as np

import tf2_ros
from tf import transformations as tf_tr
import geometry_msgs.msg
import fiducial_msgs.msg

class Calibrate:
    def __init__(self, world_tag_id):
        self.world_tag_id = world_tag_id
        self.max_len = 100
        self.reset()

    def reset(self):
        self.pos_list = []
        self.rot_list = []

    def handle_camera(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header = msg.header
        for trans in msg.transforms:
            if trans.fiducial_id == self.world_tag_id:
                t.header.frame_id = "tag"+str(trans.fiducial_id)
                t.child_frame_id = "camera_link"
                t.transform = (trans.transform)
                pos = [t.transform.translation.x, t.transform.translation.y, \
                    t.transform.translation.z]
                ori = [t.transform.rotation.x, t.transform.rotation.y, \
                    t.transform.rotation.z, t.transform.rotation.w ]
                transform = tf_tr.concatenate_matrices(\
                    tf_tr.translation_matrix(pos), \
                        tf_tr.quaternion_matrix(ori))
                inversed_transform = tf_tr.inverse_matrix(transform)
                self.pos_list.append(tf_tr.translation_from_matrix(inversed_transform))
                self.rot_list.append(tf_tr.quaternion_from_matrix(inversed_transform))
                if len(self.pos_list) ==  self.max_len:
                    rospy.set_param('/camera_pos', np.mean(self.pos_list, axis = 0).tolist())
                    rospy.set_param('/camera_rot', np.mean(self.rot_list, axis = 0).tolist())
                    self.reset()
                    rospy.logdebug('camera calibrate done')


if __name__ == '__main__':
    rospy.init_node('camera_calibrate')
    world_tag_id = rospy.get_param('world_tag_id')
    calirate = Calibrate(world_tag_id=world_tag_id)
    rospy.Subscriber('/fiducial_transforms', # Topic
                     fiducial_msgs.msg.FiducialTransformArray, # Msg type
                     calirate.handle_camera)
    rospy.spin()