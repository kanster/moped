#!/usr/bin/env python
################################################################################
#                                                                              
# MopedModeling.py: module to compute and export models for MOPED. 
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" MopedModeling - Generate 3D models for MOPED. 

Created by Alvaro Collet on 06/27/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""

# Global imports
import numpy as np
import inspect
import os
import sys
import glob
import yaml
from scipy.cluster import vq

# Import our custom modules
try:
    import roslib; roslib.load_manifest('moped-modeling')
except ImportError:
    print 'Could not import paths from ROS, please make sure the \
            following modules are available in your PYTHONPATH: \
            meanshift, tf_format, Camera'  

# Custom imports 
import Bundler
import descutils
import meanshift  # We use k-means now 
import tf_format
import MaskReader
import utils
from Camera import Camera

################################################################################
#
# Config class
#
################################################################################
class Config(object):
    """Config - Read information from YAML file to start MopedModeling
    from the command line."""
   
    name = ''
    """ Desired object name. If not given, we use the folder name."""

    img_dir = ''
    """ Image folder. Must be FULL path (or ./) """

    out_dir = ''
    """ Output folder. Must be FULL path (or ./) """

    cam_file = ''
    """ YAML file that contains camera parameters. 
        Check Camera.py for details """

    options_file = ''
    """ YAML file that contains algorithm options for MopedModeling. """

    output_xml_file = ''
    """ Output XML file to use by MOPED (should have extension '.moped.xml')"""

    # ------------------------------------------------------------------------ #
    def __init__(self, config_file = 'config.yaml'):
        """ Create a Moped modeling Config object with default fields. 
        
            Usage: config = Config(config_file = 'config.yaml')

            Output:
                config - Object of class Config()
        """

        self.name = ''
        self.img_dir = ''
        self.out_dir = ''
        self.cam_file = ''
        self.options_file = ''
        self.output_xml_file = ''

        # If there is an options file, it will overwrite the defaults 
        if config_file is not None:
            self.load(config_file)

    # ------------------------------------------------------------------------ #
    def load(self, config_file):
        """ Load a Moped-modeling YAML config file.
        
            Usage: Success = Config.load(config_file = 'config.yaml')

            Output:
                Success - True if we read the file successfully, False if not.
                -OTHER- fields are updated in-place
        """
        try:
            with open(config_file, 'r') as fp:
                data = yaml.load(fp)
                
                for key in data:
                    if hasattr(self, key):
                        setattr(self, key, data[key])

                # If we didn't assign a name, we do it now
                if self.name == '':
                    _, self.name = os.path.split(self.img_dir)

            return True # Return true if we succeeded
        
        except IOError:  
            return False # Return false if we didn't succeed

    # ------------------------------------------------------------------------ #
    def dump(self, config_file = 'config.yaml'):
        """ Export a Moped-modeling Config object into a YAML file.
        
            Usage: opts.dump(config_file = 'config.yaml')

            Output:
                -NONE- but the file 'config_file' is created and populated.
                If the file already exists, it will be overwritten.
        """

        with open(config_file, 'w') as fp:
            yaml.dump(self.__dict__, fp)



################################################################################
#
# Options class
#
################################################################################
class Options(object):
    """Options - Storage class for Moped Modeling options. """

    reproj_th = 2.5 
    """ Filter points with high reprojection error """

    min_matched_views = 3
    """ Minimum number of matching views of a keypoint to be added to the 3D
        model. """

    descriptors = None
    """ Dictionary containing the descriptors names and modules to use for
        Moped-modeling. See descriptor_template.py for details."""

    mask_suffix = None 
    """ Pattern that masks must match in order for us to find them. """
  
    # ------------------------------------------------------------------------ #
    def __init__(self, yaml_file = 'options_modeling.yaml'):
        """ Create a Moped modeling options object with default fields. 
        
            Usage: opts = Options(yaml_file = 'options_modeling.yaml')

            Output:
                opts - Object of class Options()
        """

        self.reproj_th = 2.5
        self.min_matched_views = 3
        self.descriptors = {'SIFT': 'sift'} # Descriptor name and module name
        self.mask_suffix = '*_mask.png'
        
        # If there is an options file, it will overwrite the defaults 
        if yaml_file is not None:
            self.load(yaml_file)

    # ------------------------------------------------------------------------ #
    def load(self, yaml_file):
        """ Load a Moped-modeling YAML options file.
        
            Usage: Success = opts.load(yaml_file = 'options.yaml')

            Output:
                Success - True if we read the file successfully, False if not.
                -OTHER- fields are updated in-place
        """
        try:
            with open(yaml_file, 'r') as fp:
                data = yaml.load(fp)
                
                for key in data:
                    if hasattr(self, key):
                        setattr(self, key, data[key])
            return True # Return true if we succeeded
        
        except IOError:  
            return False # Return false if we didn't succeed

    # ------------------------------------------------------------------------ #
    def dump(self, yaml_file):
        """ Export a Moped-modeling Options object into a YAML file.
        
            Usage: opts.dump(yaml_file = 'options.yaml')

            Output:
                -NONE- but the file 'yaml_file' is created and populated.
                If the file already exists, it will be overwritten.
        """

        with open(yaml_file, 'w') as fp:
            yaml.dump(self.__dict__, fp)

    
################################################################################
#
# PointInfo class
#
################################################################################
class PointInfo(object):
    """PointInfo - Store extra information about keypoint. """

    locs = None
    """ 4-vector of SIFT x, y, scale, orientation """

    desc = None
    """ 128-vector SIFT descriptor. """

    cam_id = None
    """ ID of cameras that see this point """

    # ------------------------------------------------------------------------ #
    def __init__(self, locs = None, desc = None, cam_id = None):
        """ Initialize PointInfo with locs, desc, cam_id."""
        self.locs = locs
        self.desc = desc
        self.cam_id = cam_id
        

################################################################################
#
# MopedModel class
#
################################################################################
class MopedModel(object):
    """MopedModel - Definition of a MOPED model in python, with utilities to
       generate models from images and their masks.
    """

    name = ""
    """ Object name """

    version = ""
    """ Model structure version, e.g. "Bundler v1" """

    nViews = None
    """ Number of views K used in the creation of this model. """

    pts3D = None
    """ 3-by-M array of M 3D points resulting of the SFM algorithm """

    cam_poses = None
    """ 6-by-K array of 6DOF camera poses [r1 r2 r3 t1 t2 t3]'
        where [r1 r2 r3] is a rodrigues rotation vector. This
        is a WORLD TO CAMERA transformation: x = K[R t]X. """

    desc = None
    """ list with N descriptors (potentially variable length), one for 
        each point in pts3D. Each descriptor is an ndarray."""
 
    desc_name = None
    """ dict with descriptor name for each descriptor, such that:
        Features[ desc_name[desc_type[i]] ] <--> desc[i]."""   

    desc_type = None
    """ list with descriptor type for each descriptor, such that:
        Features[ desc_type[i] ] <--> desc[i]."""

    cam = None
    """ Camera() object with internal camera parameters """

    pts2D = None
    """ 2-by-M-by-K array of M 2D points [X Y] in K views. A 
        correspondence in views I and J is shown when the points 
        (:, k, I) and (:, k, J) both are non-zero. A value
        of [0 0] means the point is not present in that view. """

    opts = None
    """ Options structure that specifies how this model was created. """

    Features = None
    """ Dictionary of {'FEATURE_NAME': FEATURE_OBJ}. Each FEATURE_OBJ must
    be an object instance of class descutils.BaseFeature or a subclass."""
    
    # Extra information from Bundler (optional) ---------------------------

    color3D = None
    """ 3-by-N array of RGB colors of each 3D point """

    keys = None
    """ N-by-M matrix of 2D-3D correspondences as read from Bundler """

    num_views = None 
    """ 1-by-N Number of images in which each 3D point appears """

    pt_info = None
    """ 1-by-N structure with extra info about all 2D points and descriptors
    found for each 3D point (unnecessary for recognition, for now) """

    avg_err = None 
    """ 1-by-N array of average reprojection error for each 3D point """

    cam_nPoints = None
    """ K-by-1 array with the number of good 2D keypoints of each camera """

    # ------------------------------------------------------------------------ #
    def __init__(self, opts = None):
        """ Create a Moped model with empty fields. 
        
            Usage: model = MopedModel()

            Output:
                model - Instance of class MopedModel.
        """
        if opts is not None:
            self.opts = opts
        else:
            self.opts = Options()
        
        # Initialize dictionary of features
        self.init_features()

    # ------------------------------------------------------------------------ #
    def init_features(self, opts = None):
        """ Initialize dictionary of features from options.

        Usage: model.init_features(opts)

        Input:
            opts - Options object. 'opts.descriptors' must be initialized.

        Output:
            -NONE- but self.Features should be initialized
        """
        if opts is None:
            opts = self.opts
        else:
            self.opts = opts
        
        # Erase old stuff
        self.Features = dict()

        for desc_name in opts.descriptors:
            module = __import__(opts.descriptors[desc_name]) 
             
            # Silly module introspection to verify there is a class called
            # just like our descriptor, but case insensitive
            class_name = [cls for cls in dir(module) \
                          if cls.lower() == desc_name.lower()]

            # If we don't find the class, continue with next descriptor
            if len( class_name ) == 0:
                print ("Error: descriptor class " + desc_name + " not found.")
                continue

            # Get class name and instantiate it
            desc_cls = getattr(module, class_name[0])
            self.Features[desc_name] = desc_cls(name = desc_name)

        # Were we able to load at least one descriptor?
        if len(self.Features) == 0:
            raise Exception, "Could not load any descriptor class, aborting..."


    # ------------------------------------------------------------------------ #
    def Process(self, name, img_dir, out_dir, cam, opts = None, \
                call_bundler = True):
        """ Create Moped model using (internally) the Bundler software

        Usage: model = MopedModel()
               model.Process(name, img_dir, out_dir, cam, opts)
 
        Input:
        name - Model name
        img_dir - Path to image directory containing N JPG images
        out_dir - Directory where to store all relevant data for a model.
                     WARNING: BOTH IMAGE_DIR and OUTPUT_DIR need to be FULL 
                     PATHS!! (or ./, which is also fine).
        cam - object of class Camera() containing the camera's intrinsic
              parameters, both linear and non-linear  (same format as the
              Bouguet's Camera Calibration Toolbox). If the input images have
              been previously undistorted, use zeros for the non-linear
              parameters (recommended).  At least, cam.KK, cam.width and
              cam.height must be set.  
        opts - (optional) MopedModeling.Options() object. If not given, defaults
              will be used
 
        Output:
            model - Processed MopedModel object. Use model.export() to generate
                    an xml file to use with MOPED.
        """

        # If we receive an options file, initialize features
        if opts is not None:
            self.init_features(opts)
        else:
            opts = self.opts

        # Use default bundler output file
        bundler_file = 'bundle.out';

        # Verify camera parameters are properly set
        if cam.width is None or cam.height is None or not any(cam.KK):
            raise Exception, "Camera width, height or KK not properly \
                              initialized"

        # Extract [multiple] descriptors from images
        for feat_name in self.Features:
            img_list, key_list = \
                    descutils.RunFolder(self.Features[feat_name], img_dir)

        # Run Bundler
        if call_bundler:
            if __debug__: 
                print('Running Bundler')
            Bundler.runCustom(img_list, key_list, out_dir, cam)

        if __debug__:
            print('Importing camera poses and points from Bundler...')
        cam_poses, scene, image_names = \
                Bundler.read_data(os.path.join(out_dir, bundler_file), \
                                  cam.size(), \
                                  read_pts = True, \
                                  read_images = True)

        self.name = name
        self.version = 'Bundler v0.4'
        self.cam = cam
        self.cam_poses = cam_poses
        self.nViews = cam_poses.shape[1]
        self.opts = opts

        self.pts2D = scene['pts2D']
        self.pts3D = scene['pts3D']
        self.color3D = scene['color3D']
        self.num_views = scene['num_views']
        self.keys = scene['keys']

        # Filter points behind the camera
        for i in range (self.nViews):
            pts2D_view = self.pts2D[:, :, i]
            idx_view = utils.find(pts2D_view[0, :] > 0)
            # Avoid errors if no points in this view
            if idx_view.size > 0:
                proj_pts2D = utils.ProjectPts(self.pts3D[:, idx_view],\
                                              self.cam_poses[:,i], \
                                              self.cam.KK, \
                                              filter_negZ = True)
                not_in_front = np.isnan(proj_pts2D[0,:]) 
                if any(not_in_front):
                    self.pts2D[:, idx_view[not_in_front], i] = 0 
        
        # ------------------------------------------------------------------- #
        # If segmentation data is available, filter points outside the mask
        mask_list = [(img[:-4] + opts.mask_suffix) for img in image_names]

        # Get mask reader for our mask type
        MyMask = MaskReader.MaskFromType(opts.mask_suffix)

        # Filter each view, keep only points within the mask
        for i, mask_name in enumerate(mask_list):
            if os.path.exists(mask_name):
                # Read mask and filter points in-place
                Mask = MyMask(name = mask_name)
                self.pts2D[:, :, i] = Mask.filter( self.pts2D[:, :, i] )

        # Filter points with less than a certain number of views
        self.filter( min_views = self.opts.min_matched_views )

        # Move pts3D to be (roughly) in the world center
        # mu = np.mean(self.pts3D, axis = 1)
        # self.transform(np.eye(3), -mu)
        self.InitialAlignment(scale = 0.15) # 0.15m of max length

        # Add descriptor info to model
        if __debug__:
            print('Reading features for all images...')
        key_gz_list = [(key + '.gz') for key in key_list]
        self.loadDescriptorsInfo(key_gz_list)

        # Filter points with a high reprojection error
        self.filter(reproj_th = self.opts.reproj_th)

    # ------------------------------------------------------------------------ #
    def import_from_bundler(self, name, img_dir, out_dir, cam, opts = None):
        """ Same as model.Process, but no call to execute Bundler, just read
          from bundle.out.

        Usage: model = MopedModel()
               model.import_from_bundler(name, img_dir, out_dir, cam, opts)
 
        Input:
        name - Model name
        img_dir - Path to image directory containing N JPG images
        out_dir - Directory where to store all relevant data for a model.
                     WARNING: BOTH IMAGE_DIR and OUTPUT_DIR need to be FULL 
                     PATHS!! (or ./, which is also fine).
        cam - object of class Camera() containing the camera's intrinsic
              parameters, both linear and non-linear  (same format as the
              Bouguet's Camera Calibration Toolbox). If the input images have
              been previously undistorted, use zeros for the non-linear
              parameters (recommended).  At least, cam.KK, cam.width and
              cam.height must be set.  
        opts - (optional) MopedModeling.Options() object. If not given, defaults
              will be used
 
        Output:
            model - Processed MopedModel object. Use model.export() to generate
                    an xml file to use with MOPED.
        """

        return self.Process(name, img_dir, out_dir, cam, opts, \
                            call_bundler = False)

    # ------------------------------------------------------------------------ #
    def getNumViews(self):
        """ Compute number of views of each 3D point

        Usage:  num_views = model.getNumViews()

        Input:
            -NONE- but self.pts2D is used

        Output:
            num_views - N-array containing the number of views in which each
                        3D point appears. Self.num_views is updated as well.
        """

        # Compute number of views of each 2D points
        self.num_views = np.sum( np.sum(self.pts2D, axis = 0) != 0, 1 )
        return self.num_views

    # ------------------------------------------------------------------------ #
    def getValidPtsInView(self, view, output_type='bool'):
        """ Compute indexes for the valid 2D points in view 'view'

        Usage:  idxValidPts = model.getValidPtsInView(view)

        Input:
            view - Index of the view for which we want the valid point indexes.
            output_type{'bool'} - 'bool' (bool array of valid/invalid pts) or 
                                  'idx' (indexes of the valid points).

        Output:
            idxValidPts - valid point indexes for view 'view', i.e. those
                s.t. self.pts2D[:, idxValidPts, view] != 0 
        """

        idxBoolPts = np.logical_and(self.pts2D[0, :, view] > 0, \
                                    self.pts2D[1, :, view] > 0) 
        if output_type == 'bool':
            return idxBoolPts
        elif output_type == 'idx':
            return utils.find(idxBoolPts)

    # ------------------------------------------------------------------------ #
    def getAverageErr(self):
        """Compute average reprojection error of pts3D into pts2D
		
        Usage:  avg_err = model.getAverageErr()
		
		Input:
           -NONE- but self.pts2D and self.pts3D are used.

		Output:
		    avg_err - N-array of average reprojection errors of pts3D among 
               all views. Each point's error is normalized by the number of
               views it appears in. self.avg_err is also updated.
        """
        TotalErr = np.zeros(self.pts3D.shape[1])

        for view in range(self.nViews):
            # Weights: 1 for points that appear in the image, zero otherwise
            idx_valid = self.getValidPtsInView(view)
            # Project 3D points onto the image plane
            proj_pts2D = utils.ProjectPts(self.pts3D[:, idx_valid], \
                                          self.cam_poses[:, view],  \
                                          self.cam.KK)
            # Reprojection error for each point
            ErrView = np.sqrt( np.sum( ( self.pts2D[:, idx_valid, view] - \
                                         proj_pts2D )**2, axis = 0 ))
            TotalErr[idx_valid] += ErrView
 
        # Count how many 2D views a pts3D appears
        num_views = self.getNumViews()  

        self.avg_err = TotalErr / num_views.astype(float)
        # Average error per view
        return self.avg_err 


    # ------------------------------------------------------------------------ #
    def getNumPointsInCam(self, idxCam = None):
        """ Compute number of 2D points are visible from each camera

        Usage:  cam_nPoints = model.getNumPointsInCam()
                cam_nPoints = model.getNumPointsInCam(idxCam = 1)
                cam_nPoints = model.getNumPointsInCam(idxCam = np.r_[0, 1, 4])

        Input:
            -NONE- but self.pts2D is used

        Output:
            cam_nPoints - N-array containing the number of points visible
                from each camera. Self.cam_nPoints is updated as well.
        """
        # Number of points in each camera
        if idxCam is None:
            cam_nPoints = np.sum( np.sum(self.pts2D, axis = 0) != 0, \
                                      axis = 0 )
            self.cam_nPoints = cam_nPoints
        else:
            cam_nPoints = np.sum( np.sum(self.pts2D[:, :, idxCam], \
                                              axis = 0) != 0, axis = 0)

        return cam_nPoints

    # ------------------------------------------------------------------------ #
    def filter(self, idx = None, reproj_th = None, min_views = None):
        """ Remove points in the model according to condition.

        Usage:  model.filter(idx = idxList)
                model.filter(reproj_th = max_reproj_error)
                model.filter(min_views = min_matched_views)

        Input:
            idx: filter according to list of indexes that will survive 
                 the filtering
            reproj_th: filter points with high reprojection error
            min_views: filter points with small number of views
            NOTE: If more than one option is given, the results are
                unpredictable. Don't do it! One at a time!

        Output:
            -NONE- but the internal structures are all filtered in place
        """
        
        def check_arg(self, name):
            return hasattr(self, name) and getattr(self, name) is not None

        # Filter points by index
        if idx is not None:
            # If list of bools, ensure we also have the indexes
            if idx.dtype == np.bool:
                idx_nobool = utils.find(idx)
            else:
                idx_nobool = idx

            # Original number of points
            orig_pts = self.pts3D.shape[1]

            # Remove them from the model
            if check_arg(self, 'pts2D'):
                self.pts2D = self.pts2D[:, idx, :]
            if check_arg(self, 'pts3D'):
                self.pts3D = self.pts3D[:, idx]
            if check_arg(self, 'desc'):
                new_desc = list()
                for i in idx_nobool:
                    new_desc.append( self.desc[i] )
                self.desc = new_desc

            # Extended bundler models
            if check_arg(self, 'keys') and self.keys.shape[0] == orig_pts:
                self.keys = self.keys[idx, :]
            else:
                if __debug__:
                    print("Couldn't filter keys")
                    
            if check_arg(self, 'color3D') and self.color3D.shape[1] == orig_pts:
                self.color3D = self.color3D[:, idx]
            else:
                if __debug__:
                    print("Couldn't filter color3D")

            if check_arg(self, 'pt_info') and len(self.pt_info) == orig_pts:
                new_pt_info = list()
                for i in idx_nobool:
                    new_pt_info.append( self.pt_info[i] )
                self.pt_info = new_pt_info
            else:
                if __debug__:
                    print("Couldn't filter pt_info")

            if check_arg(self, 'num_views') and self.num_views.size == orig_pts:
                self.num_views = self.num_views[idx]
            else:
                if __debug__:
                    print("Couldn't filter num_views")

            if check_arg(self, 'avg_err') and self.avg_err.size == orig_pts:
                self.avg_err = self.avg_err[idx]
            else:
                if __debug__:
                    print("Couldn't filter avg_err")

            return 

        # Filter points with a high reprojection error
        if reproj_th is not None:
            # Choose points to keep
            valid_pts = self.getAverageErr() < reproj_th

            # Just keep idx_valid_pts in the model
            self.filter(idx = valid_pts)

            return

        # Filter points with less than a certain number of views
        if min_views is not None:
            # Find points with >= min_views and keep them
            valid_pts = self.getNumViews() >= min_views 
            
            # Just keep idx_valid_pts in the model
            self.filter(idx = valid_pts)

            return

    # ------------------------------------------------------------------------ #
    def InitialAlignment(self, scale = 0.15):
        """ Compute SVD and align object to be in a certain coordinate frame.
        
        Usage: model.InitialAlignment(scale)

        Input:
            scale - Desired scale for object. Scale is defined as the length
            along the leading eigenvector, in meters.
        """


        pts3D = self.pts3D

        # Compute eigenvecs and rotate according to them
        pc, evals, mean = utils.pca(pts3D, remove_mean = True)
        pts3D_rot = np.dot(pc.T, pts3D)

        # Find length according to max eigenvector
        mins = np.min(pts3D_rot, axis=1)
        maxs = np.max(pts3D_rot, axis=1)
        max_length = maxs[0] - mins[0]
        
        # Rotation matrix is the covariance matrix, but we want Z as the leading
        # eigenvector:
        rot = np.c_[-pc[2], pc[1], pc[0]]

        # Transform model to have zero mean, reasonable scale and rotation.
        self.transform(rot, np.dot(rot, -mean), float(scale) / max_length)


    # ------------------------------------------------------------------------ #
    def transform(self, R, t, scale = 1):
        """ Apply rigid 3D transformation to model.
        This function is useful to align a MOPED model (arbitrary axes) so that
        it corresponds to a known scale and axis. The fields 'pts3D' and
        'cam_poses' are updated.

        Usage: model.transform (R, t, scale=1)
        
        Input:
        R - 3x3 Rotation matrix or 3x1 Rodrigues rotation vector that rotates
               from the model coord frame to the desired coord frame.
               Alternatively, if only 2 parameters are given, 3-by-4 or 4-by-4 
               transformation matrices are also accepted.
        t - 3x1 Translation vector (model to world translation)
        scale - Scaling parameter. Default: 1.

        Output:
        -NONE- but model is updated in place (fields 'pts3D' and 'cam_poses')
        
        Alternatively, if only 2 parameters are given, 3-by-4 or 4-by-4 
        transformation matrices are also valid 
        """

        # Build 4-by-4 projection matrix from args ----------------------------
        # This is what we are doing internally:
        # Proj = np.r_[ scale * np.c_[R, t], [[0, 0, 0, 1]] ]
        # InvProj = np.r_[ scale * np.c_[R.T, -np.dot(R.T, t)], [[0,0,0,scale]] ]
        Proj = tf_format.tf_format('4x4', R, t)
        Proj[:-1,:] *= scale
        InvProj = tf_format.tf_format('i4x4', R, t) * scale
        
            
        # Apply transformation to pts3D ---------------------------------------
        if self.pts3D is not None and self.pts3D.shape[1] > 0:
            # Use homogeneous coords
            pts3D = np.r_[self.pts3D, np.ones((1, self.pts3D.shape[1]))]
            pts3D = np.dot(Proj, pts3D)
            self.pts3D = pts3D[:3, :]

        # Apply transformation to cameras -------------------------------------
        # Camera poses are stored using camera-to-world transformations, we 
        # need to invert the projection matrix for this to work --> 
        # we use InvProj

        cposes = self.cam_poses
        for i in range(cposes.shape[1]):

            # Extract camera projection matrix
            p_cam = tf_format.tf_format('4x4', cposes[:, i])

            # Transform camera projection matrix
            new_p_cam = np.dot(p_cam, InvProj)
            
            # Make sure it's a true rotation!
            [u, s, vT] = np.linalg.svd(new_p_cam[:3,:3])
            cposes[:3, i] = tf_format.rodrigues( np.dot(u,vT) ).ravel()
            cposes[3:, i] = new_p_cam[:3, 3]

        self.cam_poses = cposes 

    # ------------------------------------------------------------------------ #
    def loadDescriptorsInfo(self, files):
        """ Load info from (potentially) multiple descriptors to model.

        Usage: model.loadDescriptorsInfo(files)
        
        Input:
            files - List of input files

        Output:
            -NONE- but model is updated in place (fields 'pt_info' and 'desc')
        """
        EXT = '.key.gz'

        locs = list()
        desc = list()
        self.desc_type = np.zeros( (self.pts3D.shape[1],) )
        desc_type = list()
        self.desc = list()

        # Build mini-dict of feature names
        self.desc_name = dict()
        for feat in self.Features:
            self.desc_name[self.Features[feat].id] = feat 

        for f in files:
            # Get descriptor name
            _, desc_name = os.path.splitext( f[:f.find(EXT)] )
            desc_name = desc_name[1:]

            # Load descriptors from file
            if __debug__:
                print("Loading descriptors from {0}".format(f) )
            k, d = self.Features[desc_name].load( f )

            # Save them in the list
            nFeats = k.shape[0]
            locs.append( k )
            desc.append( d )
            desc_type.append( self.Features[desc_name].id )


        
        # Add info about descriptors to the model
        self.pt_info = list()
        for i in range(self.pts3D.shape[1]):
            # Images that see this 3D point
            idx_views = utils.find( self.keys[i,:] != 0 )

            if idx_views.size == 0:
                continue

            # All views of this point should come from the same descriptor type
            self.desc_type[i] = desc_type[ idx_views[0] ]

            # keypoint and descriptor lengths for this particular point
            key_length  = self.Features[ self.desc_name[self.desc_type[i]] ].key_length
            desc_length = self.Features[ self.desc_name[self.desc_type[i]] ].desc_length
            
            # Initialize keypoint & descriptor matrices
            pt_locs = np.zeros( (idx_views.size, key_length) )
            pt_desc = np.zeros( (idx_views.size, desc_length) )
            
            # Copy points to their matrices
            for j, idx_view in enumerate(idx_views):
                pt_locs[j, :] = locs[idx_view][self.keys[i, idx_view], :]
                pt_desc[j, :] = desc[idx_view][self.keys[i, idx_view], :]
            
            pt_info = PointInfo(pt_locs, pt_desc, idx_views)
            self.pt_info.append(pt_info)

            # Create some mean desc for compatibility with current modeling
            cluster_labels, desc_centroid = meanshift.meanShift(pt_desc, 10) 
            
            # Normalize descriptor
            self.desc.append( desc_centroid.ravel() / \
                              np.linalg.norm(desc_centroid.ravel()) )


    # ------------------------------------------------------------------------ #
    def export_xml(self, filename, full_export = False):
        """Export SFM model to file in XML format
 
        Usage: model.export_xml(filename, full_export = False)
  
        Input:
        filename - Text file to write to. It should finish in '.moped.xml'
        full_export{False} - if True, export EVERYTHING from a model
            (Points3D, Observations, and Cameras). If False,
            export only Points3D (default).
  
        Output:
        -NONE- Just the output file.
  
        File format:
        <Model name="" version="Bundler v0.4"> version defines the model creator 
            <Points>
                <Point p3d="x;y;z" nviews="" avg_err="" color="R;G;B" desc_type="SIFT" desc="a;b;c;...">
                    <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation" desc="a;b;c;...">
                    <Observation ...>
                    <Observation ...>
                <Point p3d="x;y;z" nviews="" avg_err="" desc_type="SIFT" desc="a;b;c;...">
                    <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation" desc="a;b;c;...">
                    <Observation ...>
                    <Observation ...>
                ...
            </Points>
            <Cameras K="fx; fy; cx; cy;">
                <Camera id="n" rot_type="quat" rot="w;x;y;z" tx="T(1);T(2);T(3)">
                ...
            </Cameras>
        </Model>
        """
    
        # Private functions to write blocks of text
        # --------------------------
        def print_openrave(f, model):
            # print_openrave - OpenRAVE data
            # For compatibility only...
            f.write( '  <Openrave>\n')
            f.write( '    <name>{0}</name>\n'.format(model.name))
            f.write( '    <xml>{0}</xml>\n'.format(model.or_xml))
            f.write( '    <transf>')
            for n in model.or_transf.flat[:]:
                f.write('{0:6f}'.format(n))
            f.write( '</transf>\n')
            f.write( '  </Openrave>\n')

        # --------------------------
        def print_Points(f, model, full_export):
            # print_Points - Print all Point3D entries

            f.write( '  <Points>\n')
            for i in range(model.pts3D.shape[1]):
                print_Point(f, model, i)
                
                if full_export:
                    for j in range(model.pt_info[i].desc.shape[0]):
                        print_observ(f, model.pt_info[i], j, \
                                     self.desc_name[self.desc_type[i]])
                f.write( '</Point>\n');
            
            f.write( '  </Points>\n');

        # --------------------------
        def print_observ(f, pt, idx_pt, desc_name):
            # <Observation camera_id="n" desc_type="SIFT" loc="x;y;scale;orientation"
            # desc="a;b;c;...">
            f.write( '      <Observation ');
            f.write( 'camera_id="{0}" '.format(pt.cam_id[idx_pt]))
            f.write( 'desc_type="{0}" '.format(desc_name))
            f.write( 'loc="')
            for l in pt.locs[idx_pt, :].ravel():
                f.write('{0:6f} '.format(l))
            f.write( '" ')
            f.write( 'desc="')
            for d in pt.desc[idx_pt, :].ravel():
                f.write( '{0:6f} '.format(d))
            f.write( '"/>\n')

        # --------------------------
        def print_Point(f, model, idx_pt):
            # <Point p3d="x;y;z" nviews="" avg_err="" color="R;G;B" desc_type="SIFT"
            # desc="a;b;c;...">
            f.write( '    <Point ');
            f.write( 'p3d="{0:6f} {1:6f} {2:6f}" '.format(model.pts3D[0, idx_pt], \
                                                          model.pts3D[1, idx_pt], \
                                                          model.pts3D[2, idx_pt]))
            f.write( 'nviews="{0:d}" '.format(model.num_views[idx_pt]))
            f.write( 'avg_err="{0:6f}" '.format(model.avg_err[idx_pt]))
            f.write( 'color="{0} {1} {2}" '.format(model.color3D[0,idx_pt], \
                                                   model.color3D[1,idx_pt], \
                                                   model.color3D[2,idx_pt]))
            f.write( 'desc_type="{0}" '\
                        .format(model.desc_name[ model.desc_type[idx_pt] ]))
            f.write( 'desc="')
            for d in model.desc[idx_pt].ravel():
                f.write( '{0:6f} '.format(d))
            f.write( '">\n')

        # --------------------------
        def print_Cameras(f, model):
            # print_Cameras - Print all Camera entries

            f.write( '  <Cameras>\n')
            for idx, cam in enumerate(model.cam_poses.T):
                print_Camera(f, cam, idx)
            f.write( '  </Cameras>\n')

        # --------------------------
        def print_Camera(f, cpose, idx_cam):
            # print_Camera - Camera entry

            f.write( '    <Camera ')
            f.write( 'id="{0}" '.format(idx_cam))
            f.write( 'rot_type="quat" ')
            q_t = tf_format.tf_format('quat', cpose)
            f.write( 'rot="')
            for val in q_t[:4].ravel():
                f.write( '{0:6f} '.format(val))
            f.write( '" ')
            f.write( 'tx="')
            for val in q_t[4:].ravel():
                f.write( '{0:6f} '.format(val))
            f.write( '"/>\n')

        # --------------------------
        # Print data to file

        # First, update structures
        self.getNumViews()
        self.getNumPointsInCam()
        self.getAverageErr()

        with open(filename, 'w') as f:
            f.write('<Model name="{0}" version="{1}">\n'.format(self.name, \
                                                                self.version) )
            # print_openrave(f, model)
            print_Points(f, self, full_export)
            if full_export:
                print_Cameras(f, self)
            f.write('</Model>\n')

    # ----------------------------------------------------------------------- #
    def show(self, type = 'all'):
        """ Show model using mayavi. Requires mayavi_plotting.py. 
        
        Usage: model.show( type = 'all' )
               model.show()

        Input:
            type{'all'} - Choose what you want to see: 'pts', 'cam' or 'all'.
        """
        import mayavi_plotting

        mayavi_plotting.show_model(self, type)

    # ----------------------------------------------------------------------- #
    def dump(self, filename):
        """ Dump model into file using pickle.
        
        Usage: model.dump(filename = 'FILENAME.pickle.zip')

        Input:
            filename - File to dump model into. Should end in '.pickle.zip'.
        """

        utils.save(filename, {'model': self}, zipped=True)

