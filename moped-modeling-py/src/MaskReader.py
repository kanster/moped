################################################################################
#                                                                              
# MaskReader.py: module to load and use masks for filtering 2D points 
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" MaskReader - Load and use masks for filtering 2D points 

Created by Alvaro Collet on 06/27/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""

import numpy as np
import utils
import os

################################################################################
#
# BaseMask class --> If you have a custom mask, you should inherit from BaseMask
#
################################################################################
class BaseMask(object):
    """BaseMask - Basic functionality to read a mask and filter points. """

    name = ''
    """ Mask name. """

    path = ''
    """ Mask path. """

    data = None
    """ Numpy array containing the mask data. """

    # ------------------------------------------------------------------------ #
    def __init__(self, name = '', path = ''):
        """ Initialize mask structure, and if given a name/path, load data. """

        data = self.load(name, path)

    # ------------------------------------------------------------------------ #
    def load(self, name = None, path = None):
        """ load data from mask, and update name/path if necessary.
        
            Usage: Mask = BaseMask()
                   Mask.load(name, path)

            Input: 
                name - Mask name
                path - Mask path

            Output:
                -NONE- but self.data is updated with the mask data
        """

        if name is not None:
            self.name = name

        if path is not None:
            self.path = path

        # BaseMask assumes we're reading masks from an image type
        file = os.path.join(self.path, self.name)
        if os.path.exists(file):
            self.data = np.asarray(Image.open(file))

    # ------------------------------------------------------------------------ #
    def filter(self, pts2D):
        """ Filter image points based on loaded mask. 
        
        Usage: Mask = BaseMask('mask.png')
               FilteredPts = Mask.filter(pts)
        
        Input:
            pts - 2-by-N numpy array of 2D points

        Output:
            FilteredPts - 2-by-N numpy array of 2D points. Those that agree 
            with the mask (i.e., those points [x, y] s.t. mask[x,y] = True)
            remain the same, while invalid points are zeroed.
        """
        
        # Find points that are not zero in image space
        idxPts = utils.find( np.logical_and(pts2D[0, :] > 0, pts2D[1,:] > 0) )

        # Get rounded and flattened (1D) pixel indexes for our 2D points
        idxPtsFlat = utils.ravel_index(pts2D[::-1, idxPts].astype(int), \
                                       self.data.shape)

        # Find invalid 2D points by comparing them to the mask
        idxInvalidPts = utils.find( self.data.flat[idxPtsFlat] == 0 )

        # Set invalid points to zero
        pts2D[:, idxPts[idxInvalidPts]] = 0

        return pts2D

################################################################################
#
# MatMask class --> Custom read mask from matlab .mat file
#
################################################################################
class MatMask(BaseMask):
    """MatMask - Read mask from Matlab and filter points. """

    # ------------------------------------------------------------------------ #
    def load(self, name = None, path = None):
        """ load data from mask, and update name/path if necessary. """

        if name is not None:
            self.name = name

        if path is not None:
            self.path = path

        # BaseMask assumes we're reading masks from an image type
        file = os.path.join(self.path, self.name)
        if os.path.exists(file):
            self.data = utils.load(file, 'mask', Matlab_compatible = True)


################################################################################
#
# ImgMask class 
#
################################################################################

# ImgMask is the same as BaseClass so far...
ImgMask = BaseMask

def MaskFromType(type):
    """ Mask selector from mask type. Returns most suitable MaskReader class for 
    your needs given the image type.

    Usage: MaskReader = MaskFromType(type)

    Input:
        type - mask file extension. Accepted types are 'jpg', 'png', 'bmp' and
               'mat'.
    Output:
        MaskReader - Mask reader class associated with your mask type.
        Currently, it is either ImgMask or MatMask.
    """

    # in case we enter something that is not a type, keep the last three chars
    type = type[-3:]

    if type in ['jpg', 'png', 'bmp']:
        return ImgMask

    elif type in ['mat']:
        return MatMask

