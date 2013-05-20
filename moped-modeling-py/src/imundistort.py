################################################################################
#                                                                              
# imundistort.py
# Utility to undistort images (alone, or in batch mode)
#
# Copyright: Carnegie Mellon University.
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" Imundistort.py - Undistort images in python

    Available functions:
        undistort_image - Undistort single image
        undistort_folder - Undistort entire folder
"""

import libimundistort
import numpy as np
import pyublas
import shutil
import Image
import os
import glob

# --------------------------------------------------------------------------- #
def undistort_image(img, cam_K, cam_dist):
    """ Python wrapper of cvUndistort2 to remove nonlinear distortion

    Usage:  out_img = imundistort(input_img, cam_K, cam_dist)

    Input:
    img - Numpy array with color or grayscale image to be undistorted. 
    cam_K - 4-by-1 numpy double array with intrinsic camera params 
           [fx fy cx cy].
    cam_dist - 4-by-1 or 5-by-1 double array with radial and tangential
                distortion parameters [r^2 r^4 t^2 t^4].

    Output:
    out_img - Output undistorted image.
    """
    return libimundistort.undistort(img.astype(np.uint8), cam_K, cam_dist)


# --------------------------------------------------------------------------- #
def imundistort_folder(img_dir, cam_K, cam_dist=None, output_size=None):
    """IMUNDISTORT_FOLDER - Undistort a whole folder of images (and masks)

      Usage:  imundistort_folder(img_dir, cam_K, cam_dist, output_size)

      Input:
      img_dir - Folder where images are contained. It can be either a wildcard
          pattern (e.g.: '/usr/*.jpg') or a folder name ('/usr'). If it is a folder
          name, the default filenames to search are '*.jpg'. Associated masks
          are assumed to be 'XXX_mask.png'.
      cam_K - 4-by-1 internal camera parameters, [fx fy cx cy]
      cam_dist - 5-by-1 Camera distortion parameters (Bouguet's tool)
      output_size - (Optional) Resize images (and masks) when saving them,
          AFTER undistorting them. Format: (width, height)

      Output:
      -NONE-

    USEFUL TIP: If you want to rescale a set of images and masks without
    undistorting them, you can use:
       imundistort_folder(img_dir, cam_K, zeros(5,1), output_size)
    """


    # Check if we received a string
    if isinstance(img_dir, str):
        if os.path.isdir(img_dir):
            search_path = os.path.join(img_dir, '*.jpg')
        else:
            search_path = img_dir
            img_dir, pattern = os.path.split(search_path)

        files = glob.glob(search_path)
    elif isinstance(img_dir, list):
    # A list of file names?
        files = img_dir
        img_dir, _ = os.path.split(files[0])
    else:
        raise Exception, "I need either a folder name or a list of files!"


    new_file = lambda f: os.path.join(img_dir, f)
    backup_file = lambda f: os.path.join(img_dir, 'original', f)

    # Make folder for backups
    if not os.path.isdir(os.path.join(img_dir, 'original')):
        os.mkdir(os.path.join(img_dir, 'original'))

    for f in files:
        print('Undistorting {0}...'.format(f))
        
        # Copy file to 'original'
        if not os.path.isfile(backup_file(f)):
            shutil.copyfile(new_file(f), backup_file(f))
        
        # Undistort
        img = Image.open(new_file(f))
        if cam_dist is not None:
            new_img = undistort_image(np.asarray(img), cam_K, cam_dist)
            img = Image.fromarray(new_img)
        
        if output_size is not None:
            img = img.resize(output_size)
        
        # Write to file again
        with open(new_file(f), 'w') as fp:
            img.save(fp)
        
        # Is there an associated mask?
        mask_name = f[:-4] + '_mask.png'
        if os.path.isfile(mask_name):
            print('Undistorting {0}...'.format(mask_name))
            
            # Copy file to 'original'
            if not os.path.isfile(backup_file(mask_name)):
                shutil.copyfile(new_file(mask_name), backup_file(mask_name))
            
            # Undistort
            mask = Image.open(new_file(mask_name))
            if cam_dist is not None:
                new_mask = undistort_image(np.asarray(mask), cam_K, cam_dist)
                mask = Image.fromarray(new_mask)
      
            if output_size is not None:
                mask = mask.resize(output_size)
            
            # Hard threshold (bilevel image --> mode 1)
            mask.convert(mode=1)
            
            # Save undistorted/transformed mask
            mask.save(new_file(mask_name)) 
