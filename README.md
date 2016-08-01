# MOPED
The MOPED framework: Object recognition and pose estimation for manipulation


It recognizes objects from point-based features (e.g. SIFT, SURF) and their geometric relationships extracted from rigid 3D models of objects. The global MOPED framework requires seven steps to recognize objects:

- Feature extraction
- Feature matching
- Feature clustering
- Hypothesis generation
- Pose clustering
- Hypothesis refinement
- Pose recombination


This code is structured in two modules: first, a ROS-agnostic library called libmoped, in which all code for the 7-step algorithm is implemented; and second, ROS-enabled wrapper code that utilizes libmoped to read images from the network and to publish the detected objects.
Remove SVN backup
From CMU
