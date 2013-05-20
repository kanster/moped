#!/usr/bin/env python
################################################################################
#                                                                              
# mayavi_plotting.py: Drawing primitives for mayavi 
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" Mayavi Plotting - Drawing primitives for numpy and mayavi. 

Created by Alvaro Collet on 07/10/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""

import numpy as np
from enthought.mayavi import mlab
from tf_format import tf_format

# ---------------------------------------------------------------------------- #
def show_model(model, type = 'all', IdxCameras = None):
    """ Visualize data from a bundler model.

    Usage: show_model(model, type, IdxCameras)

    Input:
    model - Moped model to visualize
    type{'all'} - Choose what you want to see: 'pts', 'cam' or 'all'.
    IdxCameras{None} - If type = {'all', 'cam'}, options is a list of camIDs that
        specify which camera(s) to display. If not given, the default if to 
        display all cameras used in the training process.

    Output:
    -NONE-
    """

    if IdxCameras is None:
        IdxCameras = range(model.nViews)
    
    # Our figure
    fig = mlab.figure()
   
    # Display only the points   
    if type.lower() == 'pts':
        # Get a proper scale for the cameras: the std after removing mean
        scale = np.mean( np.std(model.pts3D - \
                                np.mean(model.pts3D, axis=1)[:, None]) )   
        mlab.points3d(model.pts3D[0,:], \
                      model.pts3D[1,:], \
                      model.pts3D[2,:], \
                      scale_mode='none', scale_factor=0.02, \
                      color = (0,0,1), figure = fig)
        
    elif type.lower() == 'cam':
        scale = 0.1 
        for i in IdxCameras:
            # To use draw_camera we need the camera to world transf.
            R, t = tf_format('iRT', model.cam_poses[:, i])
            draw_camera(R, t, scale, figure = fig); # Draw cameras
        
    # Draw 3D points and cameras
    elif type.lower() == 'all':   
        # Get a proper scale for the cameras: the std after removing mean
        scale = np.mean( np.std(model.pts3D - \
                                np.mean(model.pts3D, axis=1)[:, None]) )

        for i in IdxCameras:
            # To use draw_camera we need the camera to world transf.
            R, t = tf_format('iRT', model.cam_poses[:, i])
            draw_camera(R, t, scale, figure = fig); # Draw cameras
        
        mlab.points3d(model.pts3D[0,:], \
                      model.pts3D[1,:], \
                      model.pts3D[2,:], \
                      scale_mode='none', scale_factor=0.02, \
                      color = (0,0,1), figure = fig)
        
    # Oops
    else: 
        print('Unknown option')

    mlab.view(0,0, distance = scale*50, focalpoint = np.r_[0, 0, 0])
    mlab.show() 


# ---------------------------------------------------------------------------- #
def draw_camera (R, t, scale = 0.1, opt = 'fancy', figure = None):
    """ Draw a camera in world coords according to its pose 
    If R = id and t = [0 0 0]' show a camera in the world 
    center pointing towards the +Z axis.

    Usage: draw_camera (R, t, scale, opt);

    Input:
    R - 3x3 Rotation matrix or 3x1 Rodrigues rotation vector (camera to
       world rotation)
    t - 3x1 Translation vector (camera to world translation)
    scale - (optional) scaling parameter, in meters. Default: 0.1.
    opt - Visualization option: (default: 'pyr') 
         'pyr' shows an inverted pyramid in the camera direction
         'axis' shows the 3 axis (red for X, green for Y, blue for Z)
    """

    # Generate figure if none given
    if figure == None:
        figure = mlab.figure()

    # Enforce 2d column vector of translation
    t = np.atleast_2d(t.ravel()).T

    # Five points that define the pyramid
    # Careful, because np.c_ somehow transposes this matrix, this is 3x6.
    pyr_points = scale * np.c_[[   0,    0, 0], \
                               [-0.4, -0.4, 1], \
                               [ 0.4, -0.4, 1], \
                               [ 0.4,  0.4, 1], \
                               [-0.4,  0.4, 1], \
                               [np.nan, np.nan, np.nan]]

    pyr_tri_side = np.c_[[0, 1, 2], \
                         [0, 2, 3], \
                         [0, 3, 4], \
                         [0, 1, 4]].T

    pyr_tri_front = np.c_[[1, 2, 3], \
                          [1, 3, 4]].T

    # Four points that define the axis in 3-space
    axis_points = scale * np.c_[[0, 0, 0], \
                                [1, 0, 0], \
                                [0, 1, 0], \
                                [0, 0, 1]]

    # Order in which to draw the points, so that it looks like 3 axis
    axis_idx_x = np.r_[1, 2] - 1
    axis_idx_y = np.r_[1, 3] - 1
    axis_idx_z = np.r_[1, 4] - 1

    # Some color constants
    RED   = (1,0,0)
    GREEN = (0,1,0)
    BLUE  = (0,0,1)
    
    if opt == 'pyr':  
        # Rotate pyramid and plot it
        tx_pyr = np.dot(R, pyr_points) + \
                 np.tile( t, (1, pyr_points.shape[1]) )

        mlab.triangular_mesh(tx_pyr[0, :],  \
                             tx_pyr[1, :],  \
                             tx_pyr[2, :],  \
                             pyr_tri_side,        \
                             color = BLUE,        \
                             figure = figure)
        mlab.draw() 
        mlab.triangular_mesh(tx_pyr[0, :],  \
                             tx_pyr[1, :],  \
                             tx_pyr[2, :],  \
                             pyr_tri_front,       \
                             color = GREEN,       \
                             figure = figure)       


    elif opt == 'axis':
        # Rotate the 3 axis and plot them
        tx_axis = np.dot(R, axis_points) + \
                  np.tile( t, (1, axis_points.shape[1]) )

        mlab.plot3d(tx_axis[0, axis_idx_x], \
                    tx_axis[1, axis_idx_x], \
                    tx_axis[2, axis_idx_x], \
                    color = RED, \
                    tube_radius = .003, \
                    figure = figure)
        
        mlab.plot3d(tx_axis[0, axis_idx_y], \
                    tx_axis[1, axis_idx_y], \
                    tx_axis[2, axis_idx_y], \
                    color = GREEN, \
                    tube_radius = .003, \
                    figure = figure)

        mlab.plot3d(tx_axis[0, axis_idx_z], \
                    tx_axis[1, axis_idx_z], \
                    tx_axis[2, axis_idx_z], \
                    color = BLUE, \
                    tube_radius = .003, \
                    figure = figure)

    elif opt == 'fancy':
        # Rotate pyramid and plot it
        tx_pyr = np.dot(R, pyr_points) + \
                 np.tile( t, (1, pyr_points.shape[1]) )

        mlab.triangular_mesh(tx_pyr[0, :],  \
                             tx_pyr[1, :],  \
                             tx_pyr[2, :],  \
                             pyr_tri_side,        \
                             color = BLUE,        \
                             figure = figure)

        mlab.triangular_mesh(tx_pyr[0, :],  \
                             tx_pyr[1, :],  \
                             tx_pyr[2, :],  \
                             pyr_tri_front,       \
                             color = GREEN,       \
                             figure = figure)     
        
        # Rotate the 3 axis and plot them
        tx_axis = np.dot(R, axis_points) + \
                  np.tile( t, (1, axis_points.shape[1]) )

        mlab.plot3d(tx_axis[0, axis_idx_x], \
                    tx_axis[1, axis_idx_x], \
                    tx_axis[2, axis_idx_x], \
                    color = RED, \
                    tube_radius = .003, \
                    figure = figure)
        
        mlab.plot3d(tx_axis[0, axis_idx_y], \
                    tx_axis[1, axis_idx_y], \
                    tx_axis[2, axis_idx_y], \
                    color = GREEN, \
                    tube_radius = .003, \
                    figure = figure)


    else: 
        print('Unrecognized option');


