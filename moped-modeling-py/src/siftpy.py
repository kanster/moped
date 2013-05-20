#!/usr/bin/env python
################################################################################
#                                                                              
# sift.py: descriptor interface for SIFT
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
# Dependencies: sudo apt-get install imagemagick
#
################################################################################
""" sift.py

Created by Alvaro Collet on 06/27/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""

import descutils
import os
import subprocess
import numpy as np

################################################################################
#
# Sift class derived from BaseFeature
#
################################################################################
class Sift(descutils.BaseFeature):
    
    executable = ''

    def __init__(self, name=None, executable=None):
        """ By default, we assume the SIFT executable is in the same folder
        as this file.

        Usage: Sift(name = 'SIFT', executable = '/home/sift/sift')

        NOTE: executable must point to the sift binary! By default, we assume
        sift.py and the binary are in the same folder.
        """

        # Call BaseFeature initializer
        super(Sift, self).__init__(name, key_length = 4, desc_length = 128, \
                                   id = descutils.FEAT_SIFT)
                                   

        if executable is None:
            # Get path for this script
            path = os.path.abspath(__file__)
            self.executable = os.path.join( os.path.dirname(path), 'sift' )
        else:
            self.executable = executable

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
        # Simple interface to use on current SIFT binary
        if __debug__:
            print ("Finding " + self.name + " descriptors for " + file)

        # Convert to pgm format
        os.system("mogrify -format pgm " + file)

        # Get name for pgm file... "file[:-4] + '.pgm'" would also work.
        fname, ext = os.path.splitext(file)
        file_pgm = fname + '.pgm'

        # Call SIFT binary
        with open(file_pgm, 'r') as fp_pgm:
            with open(out_file, 'w') as fp_out:
                out = subprocess.call(self.executable, \
                                      stdin = fp_pgm,  \
                                      stdout = fp_out, \
                                      stderr = subprocess.PIPE)
        
        # Zip output
        os.system ('gzip ' + out_file)

        # Clean up afterwards
        os.system('rm ' + file_pgm)
        
        return out

    # ------------------------------------------------------------------------ #
    def run(self, file):
        """ Extract features from image file.

        Usage: result = Feature.run(file)

        Input:
            file - string containing image name

        Output:
            result - feature output, in whatever format it understands (could
                be a matrix, raw text, etc...)

        NOTE: Better to use call() instead of run()...
        """
        import commands

        # Simple interface to use on current SIFT binary
        if __debug__:
            print ("Finding " + self.name + " descriptors for " + file)

        # Convert to pgm format
        os.system("mogrify -format pgm " + file)

        # Get name for pgm file... "file[:-4] + '.pgm'" would also work.
        fname, ext = os.path.splitext(file)
        file_pgm = fname + '.pgm'

        # Call SIFT binary
        out = commands.getoutput(self.executable + ' < ' + file_pgm)
        
        # Remove first line
        # out = out[out.find('\n')+1:]

        # Clean up afterwards
        os.system('rm ' + file_pgm)
        return out

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
        with open(out_file, 'w') as fp:
            fp.write(result)
        
        os.system ('gzip ' + out_file)

    # ------------------------------------------------------------------------ #
    def load(self, gz_file):
        """Decompress .KEY.GZ file, import SIFT data and compress

        Usage: locs, desc = load(filename)
        
        Input:
            filename - gzipped SIFT filename to be read
        
        Output:
            locs - 4-by-K matrix, in which each row has the 4 values for a
                keypoint location (X, Y, scale, orientation).  The 
                orientation is in the range [-PI, PI] radians.
            desc - a 128-by-K matrix, where each row gives an invariant
                descriptor for one of the K keypoints.  The descriptor is a vector
                of 128 values normalized to unit length.
        """

        # Make sure it's a .key.gz file
        if gz_file[-7:] != '.key.gz':
            raise Exception, 'File type must be of type .key.gz'

        # Unzip file
        os.system('gunzip ' + gz_file)
        file = gz_file[:-3]

        # Open file.key and check its header
        with open(file, 'rt') as fp:
            
            line = fp.readline().split()
            num_desc = int(line[0])
            desc_len = int(line[1])

            # Read first line to get sizes
            if num_desc:
                # Keypoint locations
                locs_tmp = np.fromstring(fp.readline(), sep=' ')
                locs = np.zeros((num_desc, len(locs_tmp)))
                locs[0,:] = locs_tmp

                # Descriptor
                desc = np.zeros((num_desc, desc_len))
                cur_len = 0
                while cur_len < desc_len:
                    desc_tmp = np.fromstring(fp.readline(), sep=' ')
                    desc[0, cur_len:cur_len+len(desc_tmp)] = desc_tmp
                    cur_len += len(desc_tmp)
            else:
                 # Error, we couldn't read any descriptors
                 locs = np.zeros((0,))
                 desc = np.zeros((0,))


            # Parse file.key
            for i in range(num_desc):
                # First line is row, col, scale, ori
                vector = np.fromstring(fp.readline(), sep = ' ') 
                locs[i, :] = vector

                # Following lines contain the descriptor
                cur_len = 0
                while cur_len < desc_len:
                    vector = np.fromstring(fp.readline(), sep=' ')
                    desc[i, cur_len:cur_len+len(vector)] = vector
                    cur_len += len(vector)
                
                # Normalize each input vector to unit length
                desc[i, :] /= np.linalg.norm(desc[i, :])

        # Restore gzipped file
        os.system('gzip ' + file)

        return locs, desc

