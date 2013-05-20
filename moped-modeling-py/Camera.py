################################################################################
#                                                                              
# Camera.py: Python module containing a Calibrated Camera class
#
# Alvaro Collet
# acollet@cs.cmu.edu
#
################################################################################
""" Camera.py: Python module containing Calibrated Camera and Calibrated
               Image classes

    List of functions:
    Camera() - Class constructor for class Camera
    CImage() - Class constructor for class CImage
    """
import numpy as np
import yaml
from tf_format import tf_format

################################################################################
#
# Camera class
#
################################################################################
class Camera(object):
    """Camera - Calibrated Camera Class for Python
        
        Usage: cam = Camera(pose=x1, K=x2, KK=x2, ...)

    Input:
    -NONE-

    Output:
    cam - Object instance containing all necessary info about each camera. 
          There exist the following fields:
       .name - String containing optional name for a camera (e.g. 'left').
       .pose - 6-by-1 6DOF camera pose [r1 r2 r3 t1 t2 t3]'
                where [r1 r2 r3] is a rodrigues rotation vector. This
                is a WORLD TO CAMERA transformation.
       .R - 3-by-3 rotation matrix, WORLD TO CAMERA
       .T - 3-by-1 translation vector, WORLD TO CAMERA
       .Rinv - 3-by-3 rotation matrix, CAM TO WORLD
       .Tinv - 3-by-1 translation vector, CAM TO WORLD
       .K - 3-by-3 intrinsics camera matrix
       .KK - 4-by-1 intrinsics camera parameters [fx fy cx cy]
       .Kinv - 4-by-1 inverse intrinsics camera parameters
       .dist - 5-by-1 distortion parameters [r1 r2 t1 t2 r3]
       .M - 4-by-4 projection matrix, WORLD TO CAMERA
       .Minv - 4-by-4 projection matrix, CAM TO WORLD
       .width - Camera width
       .height - Camera height

       NOTE: All matrices are numpy arrays.
       """

    name = ''
    pose = np.zeros((6,1))
    R = np.eye(3)
    T = np.zeros((3,1))
    Rinv = np.eye(3)
    Tinv = np.zeros((3,1))
    K = np.zeros((3,3))
    KK = np.zeros((4,1))
    KKinv = np.zeros((4,1))
    dist = np.zeros((5,1))
    rect = np.zeros((3,3))
    M = np.eye(4)
    Minv = np.eye(4)
    width = 640
    height = 480

    # ----------------------------------------------------------------------- #
    # Reclass 
    @classmethod
    def reclass(cls, obj):
        """ Update the object class type. Useful if pickled object is outdated
            and want to refresh the class methods without copying data.

            Usage: New_Class.reclass(obj)

            Input:
                obj - Object to have its classed renewed to 'New_Class'
            Output:
                -NONE- The change is done in place
        """
        obj.__class__ = cls 

    # ------------------------------------------------------------------------ #
    def __init__(self, **kwargs):
        """ Initialize class. 
        
        Usage: cam = Camera(yaml_file = 'camera.yaml')
               cam = Camera(KK = KK, pose = pose, ...)
               
        Please use (K or KK), (pose or M), or dist as initial args"""
        
        self.update(**kwargs)

    # ------------------------------------------------------------------------ #
    def load_yaml(self, yaml_file):
        """ Load camera data from yaml file. The file should contain, at
            least, the field K, width, and height. """
        
        with open(yaml_file, 'r') as fp:
            data = yaml.load(fp)

        # Recover numpy arrays
        if data.has_key('K'):
            data['K'] = np.array(data['K']).reshape((3,3))
        if data.has_key('dist'):
            data['dist'] = np.array(data['dist'])
        if data.has_key('M'):
            data['M'] = np.array(data['M']).reshape((4,4))
        if data.has_key('rect'):
            data['rect'] = np.array(data['rect']).reshape((3,3))

        self.update(**data)
    
    # ------------------------------------------------------------------------ #
    def dump_yaml(self, yaml_file):
        """ Save camera data into yaml file.

            Usage: cam.dump_yaml('yaml_file.yaml')
        """
        
        data = dict()
        data['name'] = self.name
        
        # Export matrices as strings for easier readability
        data['K'] = self.K.ravel().tolist()
        data['dist'] = self.dist.ravel().tolist()
        data['M'] = self.M.ravel().tolist()
        data['rect'] = self.rect.ravel().tolist()
        data['width'] = self.width
        data['height'] = self.height

        with open(yaml_file, 'w') as fp:
            yaml.dump(data, fp)

    
    # ------------------------------------------------------------------------ #
    def update(self, **kwargs):
        """ Update value(s) in class. Please use K or KK, pose or M, dist, \
             and/or rect as input args.

        Usage: update(**kwargs)
        Examples:
                update(KK=np.array([1000., 1000., 320., 240.]))
                update(pose=np.array([0., 1.5, 0., 0.1, 0.2, 0,5]))
        """
        
        # Assign attributes
        for arg in kwargs:

            if arg == 'yaml_file':
                self.load_yaml(kwargs[arg])

            elif arg == 'name':
                self.name = kwargs[arg]

            elif arg == 'KK':
                self.KK = kwargs[arg].copy()
                self.K = np.eye(3)
                self.K[0,0] = self.KK[0]
                self.K[1,1] = self.KK[1]
                self.K[0,2] = self.KK[2]
                self.K[1,2] = self.KK[3]
                self.KKinv = np.array([1/self.KK[0], 1/self.KK[1], \
                                -self.KK[2]/self.KK[0], -self.KK[3]/self.KK[1]]);

            elif arg == 'K':
                self.K = kwargs[arg].copy()
                self.KK = np.zeros([4,1])
                self.KK[0] = self.K[0,0]
                self.KK[1] = self.K[1,1]
                self.KK[2] = self.K[0,2]
                self.KK[3] = self.K[1,2]
                self.KKinv = np.array([1/self.KK[0], 1/self.KK[1], \
                                -self.KK[2]/self.KK[0], -self.KK[3]/self.KK[1]]);

            elif arg == 'pose' or arg == 'M':
                pose = kwargs[arg].copy()
                self.pose = tf_format('rod', pose)
                self.M = tf_format('4x4', pose)
                self.Minv = tf_format('i4x4', pose)
                self.R, self.T = tf_format('RT', pose)
                self.Rinv, self.Tinv = tf_format('iRT', pose)

            elif arg == 'dist':
                # Ensure dimensions are (5,1)
                dist = kwargs[arg].copy()
                self.dist = np.zeros((5,1))
                self.dist.flat[:dist.size] = dist
            
            elif arg == 'rect':
                self.rect = kwargs[arg].copy()

            elif arg == 'width':
                self.width = kwargs[arg]

            elif arg == 'height':
                self.height = kwargs[arg]

            else:
                raise KeyError, 'The argument %s cannot be updated. Please \
                                 use K, KK, pose, M, dist or rect as initial \
                                 args.' % arg
                           

    # ------------------------------------------------------------------------ #
    def copy(self):
        """COPY - Copy all data to a new class"""

        return Camera(name=self.name, K=self.K, pose=self.pose, dist=self.dist,\
                      rect=self.rect)

    # ------------------------------------------------------------------------ #
    def size(self):
        """ Return tuple (width, height) with camera size, in pixels."""

        return (self.width, self.height)

################################################################################
#
# CImage class
#
################################################################################
import Image
class CImage(object):
    """CImage - A Calibrated Image class.
       Attributes:
           .img - PIL Image
           .numpy_img - same as .img, with a numpy interface
           .cam - Calibrated Camera class, that contains:
                .K - 3-by-3 intrinsic matrix
                .KK - 4-by-1 intrinsic parameters [fx fy cx cy]
                .M - 4-by-4 transformation matrix [R T; 0 1] world-to-cam
                .pose - 6-by-1 vector of camera pose world-to-cam [r1 r2 r3 x y
                        z], where [r1 r2 r3] is a rodrigues rotation vector.
    """
    img = None
    numpy_img = None
    cam = Camera()
    type = 'img'
    id = ''


    # ----------------------------------------------------------------------- #
    # Reclass 
    @classmethod
    def reclass(cls, obj):
        """ Update the object class type. Useful if pickled object is outdated
            and want to refresh the class methods without copying data.

            Usage: New_Class.reclass(obj)

            Input:
                obj - Object to have its classed renewed to 'New_Class'
            Output:
                -NONE- The change is done in place
        """
        obj.__class__ = cls 

    # ------------------------------------------------------------------------ #
    def __init__(self, img=None, cam=None):
        # Regular interface
        if type(img) is str:
            self.img = Image.open(img)
        elif hasattr(img, 'copy'):
            self.img = img.copy()
        else:
            self.img = img

        # Numpy interface 
        if Image.isImageType(self.img):
            self.numpy_img = np.asarray(self.img).copy()
            self.img = Image.fromarray(self.numpy_img)
        elif type(self.img) == np.ndarray:
            self.numpy_img = self.img
            self.img = Image.fromarray(self.numpy_img)
        else:
            self.numpy_img = None
        
        if cam is not None:
            self.cam = cam.copy()

    # ------------------------------------------------------------------------ #
    def __getstate__(self):
        """Implemented in order to use cPickle"""
        D = self.__dict__.copy()
        del D['img']
        return D

    # ------------------------------------------------------------------------ #
    def __setstate__(self, D):
        """Implemented in order to use cPickle"""
        self.__dict__ = D
        self.img = Image.fromarray(self.numpy_img)
        
    # ------------------------------------------------------------------------ #
    def copy(self):
        """Create a copy of a class instance"""

        return CImage(self.numpy_img, self.cam)

    # ------------------------------------------------------------------------ #
    def show(self):
        """show - Show image on screen."""
        self.img.show()
