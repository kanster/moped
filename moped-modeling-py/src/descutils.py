#!/usr/bin/env python
################################################################################
#                                                                              
# descutils.py: common feature utilities to apply on images 
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" descutils.py

Created by Alvaro Collet on 06/27/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""

import glob
import os

# Feature types
FEAT_SIFT = 1
FEAT_SURF = 2

################################################################################
#
# BaseFeature class
#
################################################################################
class BaseFeature(object):
    """ Virtual class that defines the common interface for our feature
    descriptors.
    """
    name = ''
    """ Feature name"""

    key_length = None
    """ Keypoint length (in Sift, keypoint = [x y angle scale]"""
    
    desc_length = None
    """ Descriptor length"""

    id = None
    """ Feature identifier """

    # ------------------------------------------------------------------------ #
    def __init__(self, name=None, key_length = None, \
                 desc_length = None, id = None):
        """ Init function."""
        self.name = name
        self.key_length = key_length
        self.desc_length = desc_length
        self.id = id
        
    # ------------------------------------------------------------------------ #
    def run(self, file):
        """ Extract features from image file.

        Usage: result = Feature.run(file)

        Input:
            file - string containing image name

        Output:
            result - feature output, in whatever format it understands (could
                be a matrix, raw text, etc...)
        """
        # You need to implement this in your subclass
        pass

    # ------------------------------------------------------------------------ #
    def dump(self, result, out_file):
        """ Dump features to disk.

        Usage: Feature.dump(result, out_file)

        Input:
            result - feature output, in whatever format was output in 'run'.
            out_file - string containing output file name. 

        Output:
            -NONE- but your features will presumably be saved in your format
            of choice.
        """
        # You need to implement this in your subclass
        pass

    # ------------------------------------------------------------------------ #
    def call(self, file, out_file):
        """ Extract features from image file and dump them into file (run+dump)

        Usage: Feature.call(file, out_file)

        Input:
            file - string containing image name
            out_file - string containing output keypoints name

        Output:
            -NONE- but results are stored into file
        """
        # You need to implement this in your subclass
        pass

    # ------------------------------------------------------------------------ #
    def load(file):
        """ Load features from disk.

        Usage: keypoints, descriptors = Feature.load(file)

        Input:
            file - string containing file name with keypoints to load

        Output:
            keypoints - N-by-M matrix, containing M keypoints 
            descriptors - K-by-M matrix, containing M keypoint descriptors 
        """
        # You need to implement this in your subclass
        pass


# --------------------------------------------------------------------------- #
def createDescFilename(file, desc_name):
    """ Simple utility to generate output feature names from image files.

    Usage: out_file = createDescFileName(input_file, feature.name)

    Input:
        input_file - String with Input filename
        desc_name - String with feature name (e.g. 'SIFT', 'SURF')
    
    Output:
        out_file - output filename (e.g.: file.SIFT.key)
    """
    # Get rid of file extension
    fpath, fname_ext = os.path.split(file)
    fname, ext = os.path.splitext(fname_ext)

    return fname + '.' + desc_name + '.key'

# --------------------------------------------------------------------------- #
def RunFolder(feat, in_dir, out_dir = None, in_pattern='*.jpg', \
              overwrite = False):
    """ Execute a feature object on a set of files within a folder.

    Usage: in_list, out_list = descutils.RunFolder(desc, in_dir, in_pattern, \
                                                   out_dir, overwrite)
           in_list, out_list = descutils.RunFolder(desc, '/tmp/images', \
                                                   /tmp/model, '*.jpg')

    Input:
        feat - Object of class 'Feature'. For each file in the folder,
            we call feature.dump(feature.run(file), out_file)
        in_dir - Input folder where images are taken
        out_dir - Output folder where keypoint feature images are stored 
            (If None, we set out_dir = in_dir)
        in_pattern - pattern of files to search for (default: '*.jpg')
        overwrite{false} - If true, force feature.run to be called on every
        file, even if there is already a file
        
    Output:
        in_list - List of files we executed our feature in
        out_list - List of output filenames containing feature lists
    """
    
    out_dir = in_dir if out_dir is None else out_dir

    # Find files
    img_list = glob.glob(os.path.join(in_dir, in_pattern))
    img_list.sort() 

    keys_list = list()
    # Run features on files
    for file in img_list:
        output_name = os.path.join(out_dir, createDescFilename(file, \
                                                               feat.name))
        keys_list.append(output_name)

        # If file does not exist, do this
        if (not os.path.exists(output_name) \
           and not os.path.exists(output_name + '.gz')) \
           or overwrite:
            feat.call(file, output_name)
            # result = feat.run(file)
            # feat.dump(result, output_name)
        else:
            print ("There is already a keypoints file for " + output_name)
    
    return img_list, keys_list



