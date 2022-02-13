#!/usr/bin/env python  
import rospy

import tf2_ros
from tf import transformations as tf_tr
import geometry_msgs.msg
import fiducial_msgs.msg

def handle_camera(msg, world_tag_id):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header = msg.header
    for trans in msg.transforms:
        if trans.fiducial_id == world_tag_id:
            t.header.frame_id = "tag"+str(trans.fiducial_id)
            t.child_frame_id = "camera_link"
            t.transform = (trans.transform)
            pos = [t.transform.translation.x, t.transform.translation.y, \
                t.transform.translation.z]
            ori = [t.transform.rotation.x, t.transform.rotation.y, \
                t.transform.rotation.z, t.transform.rotation.w ]
            # pos = t.transform.translation
            # ori = t.transform.rotation
            transform = tf_tr.concatenate_matrices(\
                tf_tr.translation_matrix(pos), \
                    tf_tr.quaternion_matrix(ori))
            inversed_transform = tf_tr.inverse_matrix(transform)
            t.transform.translation.x = tf_tr.translation_from_matrix(inversed_transform)[0]
            t.transform.translation.y = tf_tr.translation_from_matrix(inversed_transform)[1]
            t.transform.translation.z = tf_tr.translation_from_matrix(inversed_transform)[2]
            t.transform.rotation.x = tf_tr.quaternion_from_matrix(inversed_transform)[0]
            t.transform.rotation.y = tf_tr.quaternion_from_matrix(inversed_transform)[1]
            t.transform.rotation.z = tf_tr.quaternion_from_matrix(inversed_transform)[2]
            t.transform.rotation.w = tf_tr.quaternion_from_matrix(inversed_transform)[3]
        else:
            t.header.frame_id = "camera_link"
            t.child_frame_id = "tag"+str(trans.fiducial_id) 
            t.transform = (trans.transform)
        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_fix_broadcaster')
    world_tag_id = rospy.get_param('~world_tag_id')
    rospy.Subscriber('/fiducial_transforms', # Topic
                     fiducial_msgs.msg.FiducialTransformArray, # Msg type
                     handle_camera, world_tag_id)
    rospy.spin()