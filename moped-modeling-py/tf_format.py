#!/usr/bin/env python
# encoding: utf-8
"""
tf_format.py

Conversion module between multiple rigid transformations. This module accepts 
the following formats both as input and output: rodrigues (3-by-1 vector), 
rodrigues + T (6-by-1 vector), R, R and T, quaternion + T, 3-by-4 and 4-by-4
matrices, and roll-pitch-yaw. It converts from and to any of these.

The main function of this module is tf_format(output_type, param1, [param2]). 
See this function's help for more details.

Created by Alvaro Collet on 10/10/2009
Copyright (c) 2009 Carnegie Mellon University. All rights reserved.
"""

# Import modules
try:
    import roslib; roslib.load_manifest('tf_format')
    from cv import Rodrigues2 as _cvRodrigues2
    from cv import CreateMat as _CreateMat
    from cv import fromarray as _fromarray
    from cv import CV_64F as _CV_64F
except ImportError:
    print 'Could not import ROS/OpenCV, so the rodrigues function is not there'
import numpy as _n
import math as _math


# Main function
def tf_format(output_type, param1, param2=None):
    """TF_FORMAT - Convert rotation + tx matrices into vectors and vice versa.
    This function is useful to change the expression of rigid
    transformations. It accepts the following formats both as input and
    output: rodrigues (3-by-1), rodrigues + T (6-by-1), R, R and T, quaternion
    and T, 3-by-4, 4-by-4, rpy, rpy+T. It performs conversions between all of them.
    All numerical inputs are expected to be numpy arrays.
    
    Usage:  output = tf_format(output_type, param1, [param2])
    
    Input:
    output_type - It can be the following options:
        '3x4' - Output is [R T].
        '4x4' - Output is [R T; 0 0 0 1]
        'RT'  - Output is a 3-by-3 rotation matrix R and, if given, T (two 
                separate parameters)
        'rod' - Output is [r1 r2 r3] (Rodrigues rotation vector). If translation
                is given, output is [r1 r2 r3 tx ty tz], where [r1 r2 r3] is a
                Rodrigues rotation vector.
        'quat' - Output is [q1 q2 q3 q4], where q1 is the scale parameter. If 
                 translation is given, output is [q1 q2 q3 q4 tx ty tz].
        'rpy' - Output is [roll pitch yaw].
        
        NOTE: Adding an 'i' in front of output_type returns the INVERSE
        TRANSFORMATION of the input. E.g.:
        [Ri, Ti] = tf_format('iRT', rod_pose)
        [R, T] = tf_format('RT', rod_pose)
        The outputs of these functions relate as: Ri = R'; Ti = -R'*T;
        
        NOTE 2: If input type is 'rpy', this function needs to be called as:
        output = tf_format(output_type, array([r, p, y]), 'rpy').
        This way, we avoid confusion with a rodrigues rotation vector.
        
    param1 - It can be either a 3x1 rotation vector, a 3x3 rotation matrix,
             a 4x1 quaternion, a 3x4 projection matrix or a 4x4 projective 
             matrix.
    param2 - (optional) 3x1 translation vector [tx ty tz].
    
    Output:
    out1, out2 - Output parameters after conversion, as requested with
                 'output_type'.
    
    Examples:
            rodrigues_vec = tf_format('rod', R, T)
            R, T = tf_format('RT', rodrigues_vec)
            P = tf_format('3x4', rodrigues_vec) # P is 3-by-4
            quat = tf_format('quat', R)
            [quat, T] = tf_format('quat', P) # P is 3-by-4
            (...)
    
    Alvaro Collet
    acollet@cs.cmu.edu
    """ 
     
    # Parse input arguments
    if param2 is not None: 
        use_tx = True
    else:
        use_tx = False
    
    # Transform any kind of input parameter to R, T ---------------------------
    nDim = param1.size
    T = None
    
    # Rotation vector or rpy
    if nDim == 3:
        if param2 == 'rpy':
            R = rpy2rot(param1)
        else:
            # Rotation vector
            R = rodrigues(param1)
            if use_tx:
                T = param2    
    
    # Quaternion
    elif nDim == 4:  
        R = quat2rot(param1);
        if use_tx:
            T = param2
    
    # Rodrigues + tx
    elif nDim == 6:
        if param2 == 'rpy':
            R = rpy2rot(param1[0:3])
            T = param1[3:]
        else:
            # Rotation vector
            R = rodrigues(param1[0:3])
            T = param1[3:]
    
    # Quaternion + tx
    elif nDim == 7:
        R = quat2rot(param1[0:4])
        T = param1[4:]

    # Rotation matrix
    elif nDim == 9:
        R = param1
        if use_tx:
            T = param2

    # 3-by-4 or 4-by-4 
    elif nDim == 12 or nDim == 16:
        R = param1[0:3,0:3]
        T = param1[0:3,3]
            
    else:
        print 'Unknown input types'
        return
    
    # Optional inversion of the transformation --------------------------------
    if output_type[0].lower() == 'i':
        R = R.T
        T = -_n.dot(R,T)
        output_type = output_type[1:]

        
    # Transform input parameters into output parameters -----------------------
    if output_type.lower() == 'rod':
        # Output is [r1 r2 r3 tx ty tz]
        rod = rodrigues(R)
        if T is not None: 
            out1 = _n.r_[rod.ravel(), T.ravel()]
        else:
            out1 = rod
            
    elif output_type.lower() == 'quat': 
        # Quaternion and T
        quat = rot2quat(R)           
        if T is not None:
            out1 = _n.r_[quat, T.ravel()]
        else:
            out1 = quat
            
    elif output_type.lower() == 'rt':                 
        # 3x3 Rotation matrix and T
        out1 = R
        out2 = T
        return out1, out2
    
    elif output_type.lower() == '3x4':            
        # 3x4 projection matrix
        out1 = _n.c_[R, T]
            
    elif output_type.lower() == '4x4':  
        out1 = _n.r_[ _n.c_[R,T], _n.array([[0, 0, 0, 1]]) ]     
        
    elif output_type.lower() == 'rpy':
        # Output is [r p y tx ty tz]
        rpy = rot2rpy(R)
        if T is not None: 
            out1 = _n.r_[rpy, T]
        else:
            out1 = rpy
            
    else:
        print 'Unknown output type'
        return

    return out1

def rodrigues(input):
    """rodrigues - Transform Rotation matrix into rodrigues vector and viceversa.
    
    Usage: rod = rodrigues(Rmat)
           Rmat = rodrigues(rod)
    
    Input:
    Rmat - 3-by-3 Rotation matrix
    
    Output:
    rod - 3-by-1 rodrigues rotation vector
    """
    try:
        if input.size == 3:
            out = _CreateMat(3, 3, _CV_64F)
            inp = _n.array(input[:, None], dtype=float).copy()
        else:
            out = _CreateMat(3, 1, _CV_64F)
            inp = _n.array(input, dtype=float).copy()

        _cvRodrigues2(_fromarray(inp), out)
        rod = _n.fromstring(out.tostring()).reshape(out.rows, out.cols)
        return rod
    except:
        raise TypeError, "You called rodrigues, but the CV module could not \
                be loaded"

def rot2quat(R):
    """ROT2QUAT - Transform Rotation matrix into normalized quaternion.
    
    Usage: q = rot2quat(R)
    
    Input:
    R - 3-by-3 Rotation matrix
    
    Output:
    q - 4-by-1 quaternion, with form [w x y z], where w is the scalar term.
    """
    # By taking certain sums and differences of the elements
    # of R we can obtain all products of pairs a_i a_j with
    # i not equal to j. We then get the squares a_i^2 from
    # the diagonal of R.
    a2_a3 = (R[0,1] + R[1,0]) / 4
    a1_a4 = (R[1,0] - R[0,1]) / 4
    a1_a3 = (R[0,2] - R[2,0]) / 4
    a2_a4 = (R[0,2] + R[2,0]) / 4
    a3_a4 = (R[1,2] + R[2,1]) / 4
    a1_a2 = (R[2,1] - R[1,2]) / 4
  
    D = _n.array([[+1, +1, +1, +1],
               [+1, +1, -1, -1],
               [+1, -1, +1, -1],
               [+1, -1, -1, +1]]) * 0.25
               
    aa = _n.dot(D, _n.r_[_n.sqrt(_n.sum(R**2) / 3), _n.diag(R)])
  
    # form 4 x 4 outer product a \otimes a:
    a_a = _n.array([[aa[0], a1_a2, a1_a3, a1_a4],
                 [a1_a2, aa[1], a2_a3, a2_a4],
                 [a1_a3, a2_a3, aa[2], a3_a4],
                 [a1_a4, a2_a4, a3_a4, aa[3]]])
    
    # use rank-1 approximation to recover a, up to sign.
    U, S, V = _n.linalg.svd(a_a)
    q = U[:, 0] 
    # q = _n.dot(_math.sqrt(S[0]), U[:, 0]) # Use this if you want unnormalized quaternions 
    
    return q


def quat2rot(q):
    """QUAT2ROT - Transform quaternion into rotation matrix
    
    Usage: R = quat2rot(q)
    
    Input:
    q - 4-by-1 quaternion, with form [w x y z], where w is the scalar term.
    
    Output:
    R - 3-by-3 Rotation matrix
    """    
    
    q = q / _n.linalg.norm(q)
         
    w = q[0]; x = q[1];  y = q[2];  z = q[3]
    
    x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w
    xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z
    wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z
    
    R = _n.array([[w2+x2-y2-z2, xy-wz, xz+wy],
               [xy+wz, w2-x2+y2-z2, yz-wx],
               [xz-wy, yz+wx, w2-x2-y2+z2]])
    return R


def rot2rpy(R):
    """ROT2RPY - Transform rotation matrix to roll, pitch, yaw.    
    Rotations are around fixed-axes. Roll is around x-axis, pitch is around 
    y-axis, yaw is around z-axis. 
    Rotations' order is: first roll, then pitch, then yaw.
    
    Usage: rpy = rot2rpy(R)
    
    Input:
    R - 3-by-3 rotation matrix
    
    Output:
    rpy - 3-by-1 vector with [roll, pitch, yaw] values
    """

    A = R[1,2] + R[0,1]
    B = R[0,2] - R[1,1]
    sinpitch = -R[2,0]
    if sinpitch == 1:
    	yawplusroll = 0
    else:
    	yawplusroll = _n.arctan2(A/(sinpitch-1), B/(sinpitch-1))

    A = R[1,2] - R[0,1]
    B = R[0,2] + R[1,1]
    if sinpitch == -1:
    	yawminusroll = 0
    else:
    	yawminusroll = _n.arctan2(A/(sinpitch+1), B/(sinpitch+1))

    # Output variable
    rpy = _n.r_[0., 0., 0.]
    
    rpy[0] = (yawplusroll - yawminusroll) / 2.0
    rpy[1] = _n.arctan2(-R[2,0], R[0,0] * _math.cos(rpy[0]) + R[1,0] * _math.sin(rpy[0]))
    rpy[2] = (yawplusroll + yawminusroll) / 2.0
    return rpy


def rpy2rot(rpy):
    """ROT2RPY - Transform [roll, pitch, yaw] to rotation matrix
    Rotations are around fixed-axes. Roll is around x-axis, pitch is around 
    y-axis, yaw is around z-axis. 
    Rotations' order is: first roll, then pitch, then yaw.
    
    Usage: R = rpy2rot(rpy)
    
    Input:
    rpy - 3-by-1 array with [roll, pitch, yaw] values
    
    Output:
    R - 3-by-3 rotation matrix
    """
    r = rpy[0]; p = rpy[1]; y = rpy[2]
    s = _math.sin
    c = _math.cos
     
    R = _n.array([
        [c(y)*c(p),   c(y)*s(p)*s(r)-s(y)*c(r),   c(y)*s(p)*c(r)+s(y)*s(r)],
        [s(y)*c(p),   s(y)*s(p)*s(r)+c(y)*c(r),   s(y)*s(p)*c(r)-c(y)*s(r)],
        [-s(p),       c(p)*s(r),                  c(p)*c(r)]])

    return R

def rotmat(angle, dir):
        """ROTMAT - Create a rotation matrix with ANGLE in dir DIR.

        Usage: H = rotmat(angle, dir)

        Input:
        angle - angle in radians
        dir - Axis of rotation: 'x', 'y' or 'z' or any 3-vector to define axis.

        Output:
        R - 3-by-3 rotation matrix 
        Examples:
        H = rotmat(angle, 'x');
        H = rotmat(angle, [0 1 0]);
        H = rotmat(angle, 'z');
        """

        if isinstance(dir, basestring):
            if dir == 'x':
                v = _n.array([[1], [0], [0]])
            elif dir == 'y':
                v = _n.array([[0], [1], [0]])
            elif dir == 'z':
                v = _n.array([[0], [0], [1]])
        else:
            v = dir / _n.linalg.norm(dir)

        # Rotation across arbitrary direction (see wikipedia)
        H = _n.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) * \
            _math.sin(angle) + _math.cos(angle)*(_n.eye(3,3)-_n.dot(v,v.T)) + _n.dot(v, v.T)

        # Normalize
        U, S, VT = _n.linalg.svd(H)
        return _n.dot(U, VT)


def test_tf():
    R = rotmat(0.3, _n.r_[1, 0.25, 0.5])
    T = _n.r_[0.1, 0.2, 0.3]
    M = _n.c_[R, T]
    print "Example rotation matrix + translation: " 
    print M

    q = tf_format('quat', M)
    print "Convert RT to quaternion + translation: "
    print q
    
    rod = tf_format('rod', q)
    print "Convert q to rodrigues + translation: "
    print rod
    
    rpy = tf_format('rpy', rod)
    print "Convert rod to rpy + translation: "
    print rpy
    
    M2 = tf_format('3x4', rpy, 'rpy')
    print "Convert rpy to rotation matrix + translation: "
    print M2


if __name__ == "__main__":
    test_tf()
    
