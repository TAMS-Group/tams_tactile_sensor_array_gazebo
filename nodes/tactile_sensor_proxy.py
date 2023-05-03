#! /usr/bin/env python 
#
# ROS node to emulate the output data from our "tams_tactile_sensor_array"
# hardware sensors [1,2,3] using a simulated force/contact sensor in Gazebo.
#
# Subscribes to wrench (geometry_msgs/WrenchStamped) and contact information,
# applies exponential averaging filter to dampen the simulation artifacts,
# and finally publishes a TactileSensorArrayData message that matches the
# format of the real hardware sensor.
#
# On the original hardware sensor (using 10-bit ADCs), each sensor cell
# value is within the range 0..1023, where 0 corresponds to the idle
# sensor and 1023 to full force. However, the raw-to-force characteristic
# of the sensor is non-linear and depends on the material/velostat foil,
# the sensor cell and electrode geometry, and also the amplifier details.
# 
# For now, we just map incoming Gazebo data (in Newtons) to integers
# using a linear scale factor "fscale", but clamp to the interval 0..1023.
# Feel free to add piecewise-linear or polynomial/rational/spline interpolation
# yourself.
# 
# The force scaling factor ("fscale") must be set as a node parameter 
# but also can be changed at runtime by publishing a "fscale <value>"
# messaage to the "~command" topic.
#
# Note that you may want a negative fscale value, e.g. a sensor on the
# floor might record negative values due to gravity being (0,0,-9.81),
# but the hardware only outputs positive values.
# 
# For visualization of the simulated tactile sensor, you would typically
# use the <code>tactile_sensor_visualizer.py</code> node from this ROS package.
# Alternatively, you could also run the <code>tactile_visualizer.py</code>
# from the <code>tams_tactile_sensor_array</code> package, which uses
# OpenCV2 functions to create a smoothed image (cubic-interpolation) 
# using the JET colormap (blue->yellow-red) of the incoming raw data.
#
# 2023.05.02 - created
#
# For documentation and code see:
# [1] github.com/TAMS-Group/tams_tactile_sensor_array
# [2] github.com/TAMS-Group/tams_tactile_sensor_array_hardware
# [3] Niklas Fiedler, Philipp Ruppel, Yannick Jonetzko, Norman Hendrich, Jianwei Zhang, 
#     Low-Cost Fabrication of Flexible Tactile Sensor Arrays, HardwareX, 2022. 
#     doi: 10.1016/j.ohx.2022.e00372
#
# (c) 2023 fnh, hendrich@informatik.uni-hamburg.de
#

import collections
import math
import numpy as np
import sys

import rospy
import std_msgs.msg
import geometry_msgs.msg
import gazebo_msgs.msg
import tams_tactile_sensor_array.msg  # want TactileSensorArrayData.msg

from math import sqrt



verbose = 0
fscale = 1.0  # force scaling 
alpha  = 0.02 # (strong) exponential averaging: 0.02 current 0.98 previous
wrench_data = None
contacts_data = None



class SensorCellSubscriber:
    '''
    helper class that subscribes to the Gazebo wrench and contacts
    topics for a single sensor cell. We extract the current effective
    z-force from the messages and store at the corresponding index
    into the global wrench_data and contacts_data arrays.
    '''

    def clamp( self, value, lower, upper ):
        if   value > upper: return upper
        elif value < lower: return lower
        else:               return value

    def wrench_callback( self, msg ):
        global verbose, wrench_data, fscale, alpha
        if verbose > 10:
            print( "... wrench_callback[" + str(iy) + "," + str(ix) + "]: \n ... " + str(msg))
        force = msg.wrench.force

        # do we want magnitude of force or normal force only?
        # wrench_data[ self.iy, self.ix ] = sqrt( force.x*force.x + force.y*force.y + force.z*force.z )
        old_data = wrench_data[ self.iy, self.ix ]
        new_data = force.z*fscale
        filtered = alpha*new_data + (1-alpha)*old_data
        clamped  = self.clamp( filtered, 0, 1023 )
        wrench_data[ self.iy, self.ix ] = clamped
        if force.z != 0.0 and verbose > 5:
            print( "... wrench_callback  [" + str(self.iy) + "," + str(self.ix) + "]= " + str( force.z*fscale ) )
        return

    def contacts_callback( self, msg ):
        global verbose, contacts_data, alpha
        if verbose > 10:
            print( "... contacts_callback[" + str(iy) + "," + str(ix) + "]: \n ... " + str(msg))
        ''' each contact message can include multiple contacts, each of which in turn
            with the names of contacting links, multiple contact points, multiple
            contact wrenches, multiple penetration depths, and the resulting total wrench
            gazebo_msgs/ContactState[]:
            states[]
              string info
              string collision1_name
              string collision2_name
              geometry_msgs/Wrench[] wrenches
              geometry_msgs/Wrench total_wrench
              geometry_msgs/Vector3[] contact_positions
              geometry_msgs/Vector3[] contact_normals
              float64[] depths
        '''
        fx = 0.0
        fy = 0.0
        fz = 0.0
        n_contacts = len(msg.states)

        if n_contacts > 0 and verbose > 6:
            print( "... found (multple) contacts: " )
            for i in range( n_contacts ):
                n1 = msg.states[i].collision1_name 
                n2 = msg.states[i].collision2_name 
                print( "... ... " + n1 + " <-> " + n2 )

        for i in range( n_contacts ):
            fx += msg.states[i].total_wrench.force.x
            fy += msg.states[i].total_wrench.force.y
            fz += msg.states[i].total_wrench.force.z

        # do we want magnitude of force or normal force only?
        # wrench_data[ self.iy, self.ix ] = sqrt( force.x*force.x + force.y*force.y + force.z*force.z )

        if (fz != 0.0) and verbose > 3:
            print( "... contacts_callback X [" + str(self.iy) + "," + str(self.ix) + "]= " + str( fz*fscale ) )

        old_data = contacts_data[ self.iy, self.ix ]
        filtered = (alpha*fz*fscale) + (1-alpha)*old_data
        clamped  = self.clamp( filtered, 0, 1023 )
        contacts_data[ self.iy, self.ix ] = clamped
        return

    def __init__(self, iy, ix, basename ):
        global verbose
        # create two ROS subscribers, one for WrenchStamped and one for
        # Gazebo ContactState
        if verbose > 5:
            print( "... SensorCellSubscriber.init: " )
        self.iy = iy
        self.ix = ix
        self.basename = basename
        self.wrench_subscriber = rospy.Subscriber( basename + '/wrench',
                                                   geometry_msgs.msg.WrenchStamped,
                                                   self.wrench_callback )

        # Gazebo publishes ContactsState, but the topics are called ...contact 
        #self.contacts_subscriber = rospy.Subscriber( basename + '/contact',
        #                                           gazebo_msgs.msg.ContactsState,
        #                                           self.contacts_callback )
        return




def publish_tactile_data( msg ): 
    '''
    called as a Timer callback (but we ignore the incoming timer message).
    Construct the visualization messages corresponding to the most recent wrench
    and contacts data and publish the markers.
    '''
    global verbose, sensor_frame, sensor_name, ny, nx, sx, sy, sensor_id
    global wrench_publisher, contacts_publisher
    global wrench_data, contacts_data

    msg = tams_tactile_sensor_array.msg.TactileSensorArrayData()
    msg.header.frame_id = sensor_frame
    msg.header.stamp    = rospy.Time.now()

    msg.sensor_id = sensor_id
    msg.sensor_data_width = nx   # number of taxels along the x-axis, uint8 
    msg.sensor_data_height = ny  # number of taxels along the y-axis, uint8
    msg.data = []                # int32 each

    if verbose > 5:
        print( "... publish_tactile_data[ " + str(ny) + "," + str(nx) + "]..." )

    for ix in range( nx ):
        for iy in range( ny ):
            value = int( wrench_data[iy,ix] )
            msg.data.append( value )
            if verbose > 5:
                print( "... wrench_data[ " + str(iy) + "," + str(ix) + "] = " + str( value ))

    tactile_data_publisher.publish( msg )

    '''
    for iy in range( ny ):
        for ix in range( nx ):
            value = int( contacts_data[iy,ix] )
            msg.data.append( value )
            if verbose > 5:
                print( "... contacts_data[ " + str(iy) + "," + str(ix) + "] = " + str( contacts_data[iy,ix] ))

    tactile_data_publisher.publish( msg )
    '''
    return 


def command_callback( msg ):
    global alpha, fscale, verbose
    print( "... received user command: '" + msg.data + "'..." )
    tokens = msg.data.split()
    try:
        if tokens == None: 
            return
        elif tokens[0] == "help":
            print( "... available commands are: " )
            print( "alpha <0..1>    - new weight factor for exponential averaging filter" )
            print( "fscale <value>  - new force scaling factor" )
            print( "verbose <level> - new debugging verbosity level" )
            return
        elif len(tokens) < 2: 
            return
        elif tokens[0] == "alpha": 
            alpha = float( tokens[1] )
            print( "... new alpha (exponential averaging) is: " + str( alpha ))
        elif tokens[0] == "fscale":
            fscale = float( tokens[1] )
            print( "... new force scaling factor fscale: " + str( fscale ))
        elif tokens[0] == "verbose":
            verbose = int( tokens[1] )
            print( "... new verbosity value: " + str( verbose ))
        else:
            print( "... unknown command, ignored" )
    finally:
        pass # don't crash here
    return 

        
if __name__ == '__main__':
    rospy.init_node( 'tactile_sensor_proxy', anonymous=False )
    #
    # get node params, (ny*nx) or (1*nx) or one (1*1) cells
    #
    verbose      = rospy.get_param( '~verbose' )
    sensor_name  = rospy.get_param( '~sensor_name' )
    sensor_frame = rospy.get_param( '~sensor_frame' )
    ny           = rospy.get_param( '~ny' ) # number of cells, matrix if > 0
    nx           = rospy.get_param( '~nx' ) # number of cells, array if > 0
    sx           = rospy.get_param( '~sx' ) # cell-size x
    sy           = rospy.get_param( '~sy' ) # cell-size y
    fscale       = rospy.get_param( '~fscale' ) # scaling factor for wrench arrow length
    rate         = rospy.get_param( '~rate' ) # in Hz
    sensor_id    = rospy.get_param( '~sensor_id' ) # 0, 1, 2, ...

    # create two visualization marker array publishers, one for wrench arrows
    # and one for color-encoded sensor rectangles.
    # 
    tactile_data_publisher = rospy.Publisher(  \
         '~tactile_data', tams_tactile_sensor_array.msg.TactileSensorArrayData, queue_size=2 )

    # instead of fighting with dynamic reconfigure, we offer a command topic
    # that allows the user to change parameters
    command_subscriber = rospy.Subscriber( '~command', std_msgs.msg.String, command_callback )

    # create sensor subscribers, two (wrench+contact) for each sensor cell;
    # the names created by our URDF are like this:
    # //tactile1_sensor_cell/wrench           (single cell "tactile1")
    # /tactile_array1_6_sensor_cell/contact   (linear array "tactile_array1" ny=0 nx>0)
    # /tactile_matrix1_4_3_sensor_cell/wrench (matrix array "tactile_matrix1" ny>0 nx>0)
    # 
    subscribers = []
    if ny > 0:
        wrench_data = np.zeros( (ny, nx) ) # hate python variable type overloading 
        contacts_data = np.zeros( (ny, nx) ) # see above
        for iy in range( ny ):
            for ix in range( nx ):
                basename = sensor_name + "_" + str(iy) + "_" + str(ix) + "_sensor_cell"
                subscribers.append( SensorCellSubscriber( iy, ix, basename ) )

    elif nx > 0:
        ny = 1
        wrench_data = np.zeros( (1, nx) )
        contacts_data = np.zeros( (1, nx) )
        for ix in range( nx ):
                basename = sensor_name + "_" + str(ix) + "_sensor_cell"
                subscribers.append( SensorCellSubscriber( 0, ix, basename ) )

    else: # single sensor
        nx = 1
        ny = 1
        wrench_data = np.zeros( (1,1) )
        contacts_data = np.zeros( (1,1) )
        basename = sensor_name + "_sensor_cell"
        subscribers.append( SensorCellSubscriber( 0, 0, basename ) )

    # trigger tactile data publishing at the given rate
    # 
    vis_timer = rospy.Timer( rospy.Duration( 1.0/rate), publish_tactile_data )

    # now just wait for incoming messages and let our child threads process them
    # 
    rospy.spin()

# end tactile_sensor_proxy
