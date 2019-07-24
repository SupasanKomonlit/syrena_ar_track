#!/usr/bin/env python2
# FILE			: filter_tf.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 24 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This code I will design to manage filter of tf message by receive tf on tf listener
#   and I will broadcast on tf broadcast
#   This code will use average all data and export data

# REFERENCE
#   ref1    : http://docs.ros.org/melodic/api/tf/html/python/transformations.html

from __future__ import print_function, division

import tf
import rospy
import numpy as np
import numpy.matlib as npm
import roslib
from syrena_utils.window import Window

_TFAVERAGE_FILTER_PRINT_PARAMETER_ = False
_TFAVERAGE_FILTER_PRINT_NEW_DATA_ = False
_TFFILTER_PRINT_RAW_DATA_ = False

class TFAverageFilter:

    # Function __init__ will manage about create variable of this object
    def __init__( self ):

        self.time_duration = rospy.get_param( '~time_duration' , 1.0 )

        self.parent_frame = rospy.get_param( '~old_parent_frame' , "rayfin_optical_frame" )

        self.old_child_frame = rospy.get_param( '~old_child_frame' , "ar_pose_marker6" )

        self.new_child_frame = rospy.get_param( '~new_child_frame' , "filter_marker" )

        self.listener = tf.TransformListener()

        self.sender = tf.TransformBroadcaster()

        self.buffer = Window( duration = self.time_duration )

        self.last_original = None

        # This variable use to protect about send data whicj erver broadcaster
        self.new_data = False

    # This functio will broadcaster loop of tf from parent_frame to new_child_frame
    def add_data( self ):
        # We will use loop tomanage about that you will see warning when I can't get tf for you
        while( not rospy.is_shutdown() ):   
            try:
                original = self.listener.lookupTransform( self.parent_frame
                    , self.old_child_frame
                    , rospy.Time(0) )
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as e :
                rospy.logerr( repr(e) )
                rospy.sleep( 0.2 )
                continue
            if( _TFAVERAGE_FILTER_PRINT_NEW_DATA_ ):
                print( "New data : " , original )

            if not np.array_equal(self.last_original,original):
                self.buffer.add( original )

            self.last_original = list(original)
            break

    def broadcast_loop( self ):
        while( not rospy.is_shutdown() ):
            self.add_data()
            self.filter_funtion()
            if( self.new_data ):
                self.sender.sendTransform( self.linear
                    , self.quaternion
                    , rospy.get_rostime()
                    , self.new_child_frame
                    , self.parent_frame )
                self.new_data = False
            rospy.sleep(0.05)

    def filter_funtion( self ):
        raw_data = self.buffer.get_data()
        if( _TFFILTER_PRINT_RAW_DATA_ ):
            print( "=================================================================" )
            print( "RAW DATA : " , raw_data )
            print( "=================================================================" )
        length_data = len( raw_data )
        average_quaternion = (0, 0, 0, 1 )
        weight = 1
        summation = [0, 0, 0]
        count = 0
        if( length_data != 0 ):
            average_quaternion = self.averageQuaternions( np.array(map(lambda x:x[1],raw_data)) )
            self.new_data = True
            ting_data = np.array(map(lambda x:x[0],raw_data))
            self.linear = list(np.median(ting_data,axis=0))
            self.quaternion = average_quaternion

    def averageQuaternions(self , Q):
        # Number of quaternions to average
        M = Q.shape[0]
        A = npm.zeros(shape=(4,4))

        for i in range(0,M):
            q = Q[i,:]
            # multiply q with its transposed version q' and add A
            A = np.outer(q,q) + A

        # scale
        A = (1.0/M)*A
        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)
        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:,0].A1)

if __name__=="__main__":
    rospy.init_node( "filter_tf" )
    TF_filter = TFAverageFilter()
    TF_filter.broadcast_loop()
     
