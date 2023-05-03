#! /usr/bin/env python 
#
# ROS node to visualize tactile/contact sensors in rviz. This is typically
# used to visualize our Gazebo simulation models of tactile sensors in
# grasping tasks.
#
# Subscribes to wrench (geometry_msgs/WrenchStamped) and creates a 
# VisualizationMarker message with red arrows to visualize the incoming 
# wrenches, similar to the rviz Wrench display, with arrow length 
# proportional to the incoming values. Max-value can be set as a node
# parameter.
# 
# Also subscribes to contact messages (gazebo_msgs/ContactsState) and 
# creates a second VisualizationMarker message with color-encoded
# rectangles just above the tactile sensor cells. Color encoding
# uses brightness (dark->bright) and optionally rainbow colormaps.
#
# 2023.05.01 - created
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
import visualization_msgs.msg

from math import sqrt



verbose = 0    # verbosity level for debugging
fscale = 1.0   # force scaling factor
alpha  = 0.01  # (strong9 exponential averaging: 1% current 99% previous 
wrench_data = None
contacts_data = None
vis_marker_msg = visualization_msgs.msg.MarkerArray()



class SensorCellSubscriber:
    '''
    helper class that subscribes to the Gazebo wrench and contacts
    topics for a single sensor cell. We extract the current effective
    z-force from the messages and store at the corresponding index
    into the global wrench_data and contacts_data arrays.
    '''

    def read_temperature_sensor_data(self):
        # Here you read the data from your sensor
        # And you return the real value
        self.temperature = 30.0

    def wrench_callback( self, msg ):
        global verbose, wrench_data, fscale, alpha
        if verbose > 10:
            print( "... wrench_callback[" + str(iy) + "," + str(ix) + "]: \n ... " + str(msg))
        force = msg.wrench.force

        # do we want magnitude of force or normal force only?
        # wrench_data[ self.iy, self.ix ] = sqrt( force.x*force.x + force.y*force.y + force.z*force.z )
        wrench_data[ self.iy, self.ix ] = alpha*force.z*fscale + (1.0-alpha)*wrench_data[ self.iy, self.ix ]
        if force.z != 0.0 and verbose > 5:
            print( "... wrench_callback  [" + str(self.iy) + "," + str(self.ix) + "]= " + str( force.z*fscale ) )
        return

    def contacts_callback( self, msg ):
        global verbose, contacts_data, alpha
        if verbose > 10:
            print( "... contacts_callback[" + str(iy) + "," + str(ix) + "]: \n ... " + str(msg))
        ''' 
            each contact message can include multiple contacts, each of which in turn
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

        contacts_data[ self.iy, self.ix ] = (alpha*fz*fscale) + (1-alpha)*contacts_data[ self.iy, self.ix ]


        if (fz != 0.0) and verbose > 3:
            print( "... contacts_callback X [" + str(self.iy) + "," + str(self.ix) + "]= " + str( fz*fscale ) )
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
        self.contacts_subscriber = rospy.Subscriber( basename + '/contact',
                                                   gazebo_msgs.msg.ContactsState,
                                                   self.contacts_callback )
        return




def publish_markers( msg ): 
    '''
    called as a Timer callback (but we ignore the incoming timer message).
    Construct the visualization messages corresponding to the most recent wrench
    and contacts data and publish the markers.
    '''
    global verbose, sensor_frame, sensor_name, ny, nx, sx, sy
    global wrench_publisher, contacts_publisher
    global wrench_data, contacts_data

    # for the Wrench markers, we would like to use arrows, but we want to support
    # large arrays and unfortunately there is no pre-defined ArrowList marker type.
    # 
    # We could use LINE_LIST markers, one line each extending from its sensor cell,
    # but this looks bad. So, using an array of CYLINDERs after all. Note that the
    # cylinders are centered at the origin, so we need to shift them a bit in z
    # direction.
    # 
    # The base position coincides with the center of the sensor cell,
    # (ix*sx, iy*sy, 0) and the length corresponds to the measured total force on
    # that cell.
    # Line lists use the points member of the visualization_msgs/Marker message. 
    # It will draw a line between each pair of points, so 0-1, 2-3, 4-5, ...
    #
    if verbose > 5:
        print( "... publish_markers[ " + str(ny) + "," + str(nx) + "]..." )

    wrench_vis_markers = visualization_msgs.msg.MarkerArray()


    for iy in range( ny ):
        for ix in range( nx ):
            if verbose > 5:
                print( "... wrench_data[ " + str(iy) + "," + str(ix) + "] = " + str( wrench_data[iy,ix] ))
            # 
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = sensor_frame # base pose for rendering
            marker.type    = visualization_msgs.msg.Marker.CYLINDER
            marker.action  = visualization_msgs.msg.Marker.ADD
            marker.ns      = sensor_name + "_contacts_" 
            marker.id      = 13 + (iy*nx) + (ix)
            marker.scale.x = 0.5*min( sx, sy ) # fit onto cell
            marker.scale.y = 0.5*min( sx, sy ) # unused for LINE_LIST
            # cylinders are centered for height scale.z
            # 
            fz = -0.01*wrench_data[iy,ix]
            marker.scale.z = fz                # unused for LINE_LIST
            marker.color.r = 0.8
            marker.color.g = 0.3
            marker.color.b = 0.1
            marker.color.a = 1.0
            marker.pose.position.x = ix*sx
            marker.pose.position.y = iy*sy
            marker.pose.position.z = fz/2
            marker.pose.orientation.w = 1         # default quaternion: z up

            wrench_vis_markers.markers.append( marker )
            ''' for LINE_LIST
            marker.points = []
            p1 = geometry_msgs.msg.Point()
            p1.x = ix*sx
            p1.y = iy*sy
            p1.z = 0.0
            p2 = geometry_msgs.msg.Point()
            p2.x = ix*sx
            p2.y = iy*sy
            p2.z = 0.05 - 0.1*wrench_data[iy,ix]
            marker.points.append( p1 )
            marker.points.append( p2 )
            '''

    wrench_publisher.publish( wrench_vis_markers )

    contacts_vis_markers = visualization_msgs.msg.MarkerArray()
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = sensor_frame # base pose for rendering
    marker.type    = visualization_msgs.msg.Marker.CUBE_LIST
    marker.action  = visualization_msgs.msg.Marker.ADD
    marker.ns      = sensor_name
    marker.id      = 14
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = 0.01
    marker.color.r = 0.8
    marker.color.g = 0.4
    marker.color.b = 0.4
    marker.color.a = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1         # default quaternion: z up
    marker.points = []
    marker.colors = []

    for iy in range( ny ):
        for ix in range( nx ):
            if verbose > 5:
                print( "... contacts_data[ " + str(iy) + "," + str(ix) + "] = " + str( contacts_data[iy,ix] ))

            p1 = geometry_msgs.msg.Point()
            p1.x = ix*sx
            p1.y = iy*sy
            p1.z = 0.0
            c1 = std_msgs.msg.ColorRGBA()
            c1.r = 0.0
            c1.g = min( 0.2 + 0.1*wrench_data[iy,ix], 1.0 )
            c1.b = 0.0
            c1.a = 1.0 # opaque
            marker.points.append( p1 )
            marker.colors.append( c1 )

    contacts_vis_markers.markers.append( marker )
    contacts_publisher.publish( contacts_vis_markers )
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
    rospy.init_node( 'tactile_sensor_visualizer', anonymous=False )
    #
    # get node params, ny*nx or 0*nx or one (0*0) cells
    #
    verbose      = rospy.get_param( '~verbose' )
    print( "verbose= " + str(verbose) )
    # sys.exit( 1 )

    sensor_name  = rospy.get_param( '~sensor_name' )
    sensor_frame = rospy.get_param( '~sensor_frame' )
    ny           = rospy.get_param( '~ny' ) # number of cells, matrix if > 0
    nx           = rospy.get_param( '~nx' ) # number of cells, array if > 0
    sx           = rospy.get_param( '~sx' ) # cell-size x
    sy           = rospy.get_param( '~sy' ) # cell-size y
    fscale       = rospy.get_param( '~fscale' ) # scaling factor for wrench arrow length
    rate         = rospy.get_param( '~rate' ) # in Hz

    # create two visualization marker array publishers, one for wrench arrows
    # and one for color-encoded sensor rectangles.
    # 
    wrench_publisher   = rospy.Publisher( '~wrench_markers', visualization_msgs.msg.MarkerArray, queue_size=2 )
    contacts_publisher = rospy.Publisher( '~contacts_markers', visualization_msgs.msg.MarkerArray, queue_size=2 )

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

    # trigger redraws (marker publishing) at the given rate
    # 
    vis_timer = rospy.Timer( rospy.Duration( 1.0/rate), publish_markers )

    # now just wait for incoming messages
    # 
    rospy.spin()

# end tactile_sensor_visualizer
