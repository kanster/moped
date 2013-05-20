#!/usr/bin/env python
################################################################################
#                                                                              
# generate_model.py: Command-line interface for MopedModeling.py 
#
# Copyright: Carnegie Mellon University
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" generate_model.py - Command-line interface for MopedModeling.py

Created by Alvaro Collet on 09/07/2011
Copyright (c) 2011 Carnegie Mellon University. All rights reserved.
"""
# System modules
import sys

# Import our custom modules
try:
    import roslib; roslib.load_manifest('moped-modeling')
except ImportError:
    print 'Could not import paths from ROS, please make sure the \
            following modules are available in your PYTHONPATH: \
            meanshift, tf_format, Camera'  

import MopedModeling
from Camera import Camera

# --------------------------------------------------------------------------- #
# read options from yaml file and process file
if __name__ == "__main__":

    # We accept exactly one argument: the YAML config file
    if len(sys.argv) != 2:
        print('Usage: ./generate_model.py CONFIG_FILE.YAML')
    else:
        # Read config file 
        config = MopedModeling.Config(sys.argv[1])
        
        # Read opts and camera files
        opts = MopedModeling.Options(yaml_file = config.options_file)
        cam = Camera(yaml_file = config.cam_file)
      
        # Generate model
        model = MopedModeling.MopedModel()
        model.Process(name = config.name, \
                      img_dir = config.img_dir, \
                      out_dir = config.out_dir, \
                      cam = cam, \
                      opts = opts)

        # Export options: XML or pickle?
        # XML
        model.export_xml(filename = config.output_xml_file)

        # Pickle --> in order to read it from disk:
        # # model = utils.load(filename, 'model', zipped=True)
        model.dump(filename = model.name + '.pickle.zip')

        # Uncomment for model Visualization using mayavi
        # mayavi_plotting.show_model(model)

