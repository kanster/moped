################################################################################
#                                                                              
# utils.py
#
# Copyright: Carnegie Mellon University 
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
""" Utils.py: Python module containing some useful utilities 
    
    List of functions:
    FIND - Find indices in array that satisfy condition, and return array.
    LOAD - Load variable(s) from file (using scipy)
    SAVE - Save variable(s) to file (using scipy)
    CV2ARRAY - OpenCV to Numpy array conversion
    ARRAY2CV - Numpy array to OpenCV conversion
    PIL2ARRAY - PIL Image to Numpy array conversion
    ARRAY2PIL - Numpy array to PIL Image conversion
    PROJECTPTS - Project 3D points onto a 2D image plane
    C_PROJECTPTS - Speeded up version of PROJECTPTS
    OUT_OF_BOUNDS - Check if n-dim vectors are within [min, max] bounds.
    BREAKPOINT - Set breakpoint. Opens an IPDB window in next exception.
    BREAKNOW - Set unconditional breakpoint. Opens IPDB when executed.
    INITSHELL - Open IPython embedded shell when executed (within program).
    PROJECTPTSOUT - Use inverse projection to map 2D pts to 3D rays.
    DISTRAY3D - Compute Euclidean distances from 3D points to 3D rays
    PCA - Compute PCA decomposition of a matrix
    CHI2 - Compute the chi-square distance between two arrays/histograms.
    RAVEL_INDEX - Compute flat indexes given ND indexes
    C_sub2ind - Speeded up version of RAVEL_INDEX
    UNRAVEL_INDEX - Compute ND indexes given flat indexes
    C_ind2sub - Speeded up version of UNRAVEL_INDEX
    C_GraphDistances - Compute euclidean distances between vertexs in graph
    IMFILTER - Filter image I with kernel K, using FFT
    SizedDict - Dictionary with limited number of entries
    CachedDict - Dictionary with limited number of entries and timeout.
    MERGE_IMAGES - Put all images in folder in a single large image
    CIRCLE - Create meshgrid in circle shape (numpy)
    IMREADDEPTH - Read depth image from PNG file
    """

from libutils import C_ProjectPts, C_sub2ind, C_ind2sub, C_find
from libutils import C_ProjectPtsOut, C_distRay3D_fast, C_GraphDistances

def find(condition):
    """FIND - Find indices in array that satisfy condition, and return array.

    Usage: idx = find(condition)
    
    Input:
    condition - array of logic values (e.g. condition = array > 4)
    
    Output:
    idx - array of indices such that 'condition(idx) = True'
    """
    import numpy as np
    # return (condition.ravel().nonzero())[0]
    return C_find(condition.ravel().astype(np.uint8))


# ------------------------------------------------------------------------ # 
def load(file, var_name=None, Matlab_compatible=False, zipped=False):
    """LOAD - Load MAT variable(s) from file (using scipy), and return them.

    Usage: var = load(f, 'var', Matlab_compatible=False)
           dict_vars = load(f, Matlab_compatible=False)

    Input:
        f - Either file handler or filename to read variables from
        Matlab_compatible {False} - if True, uses scipy. If False, uses cPickle.
        zipped{False} - If True, uncompress .pickle.zip file. Only works
            if given a filename.
    
    Output:
        v1, v2, ... - List of variables in file
    """
    # If it finished in .zip, don't care what 'zipped' is: it's a zipped file
    if len(file) > 4 and file[-4:] == '.zip':
        zipped = True

    if zipped:
        import os
        path, name = os.path.split(file)
        os.system('unzip -jqo ' + file + ' -d /tmp')
        file = os.path.join('/tmp', name[:-4]) 

    if Matlab_compatible:
        from scipy.io import loadmat as LOAD
    else:
        from cPickle import load as LOAD

    if type(file) == str:
        with open(file, 'r') as f:
            X = LOAD(f)
    else:
        X = LOAD(file)

    if zipped:    
        # Clean up aftewards
        os.system('rm -f ' + file)

    if var_name == None:
        return X 
    else:
        return X[var_name]


# ------------------------------------------------------------------------ # 
def save(file, dict, Matlab_compatible=False, zipped=False):
    """SAVE - Save MAT variable(s) to file (using cPickle or scipy).

    Usage: save(f, {'var1': var1}, [Matlab_compatible=False], zipped=False)
           save(f, dict)


    Input:
    f - Either file handler or filename to save variables to. If a filename
        is given, an extension '.pickle' or '.mat' will be added it your
        file has no extension.
    dict - Dictionary containing variable names and data to store
    Matlab_compatible {False} - if True, uses scipy. If false, uses cPickle.
    zipped{False} - If True, compress file after saving it, so that you get
        a .pickle.zip file.

    Output:
        -NONE- but a file is saved to disk
    """
    # If it finished in .zip, don't care what 'zipped' is: it's a zipped file
    if len(file) > 4 and file[:-4] == '.zip':
        zipped=True

    # Verify if file must be zipped
    if zipped:
        import os
        # Get output name and folder
        nonzipped_file, ext = os.path.splitext(file)
        if ext == '.zip':
            zipped_file = file
            file = nonzipped_file
        else:
            zipped_file = file + '.zip'

    if Matlab_compatible:
        # Use Scipy
        from scipy.io import savemat as SAVE
        SAVE(file, dict)
    else:
        # Use cPickle (better)
        from cPickle import dump as SAVE


        if type(file) == str:
            file = file + '.pickle' if file.find('.') == -1 else file
            with open(file, 'w') as f:
                SAVE(dict, f, protocol=2)
        else:
            SAVE(dict, file, protocol=2)

    # Delete nonzipped file and replace it with zipped version
    if zipped:
        if os.path.exists(zipped_file):
            os.system('rm ' + zipped_file)
        os.system('zip -jq ' + zipped_file + ' ' + file)
        os.system('rm ' + file)

    return

# ------------------------------------------------------------------------ # 
def save_gzip(file, dict):
    """SAVE - Save MAT variable(s) to file (using cPickle and gzip).

    Usage: save(f, {'var1': var1})
           save(f, dict)

    Input:
    f - filename to save variables to. If a filename
        is given, an extension '.pickle' or '.mat' will be added it your
        file has no extension.
    dict - Dictionary containing variable names and data to store

    Output:
        -NONE- but a file is saved to disk
    """
    import gzip, cPickle

    fp = gzip.open(file, 'wb')
    cPickle.dump(dict, fp, protocol=2)
    fp.close()

    return

# ------------------------------------------------------------------------ # 
def load_gzip(file, var_name=None):
    """load_gzip - Load variable(s) from file, and return them.

    Usage: var = load(f, 'var')
           dict_vars = load(f)

    Input:
        f - Either file handler or filename to read variables from.
        var_name - If given, name of variable to extract. Otherwise, return
            dictionary.
    
    Output:
        var - Either variable (if var_name is not None) or dictionary with
            all variables.
    """

    import gzip, cPickle

    fp = gzip.open(file, 'rb')
    X = cPickle.load(fp)

    if var_name == None:
        return X 
    else:
        return X[var_name]
    return

# ------------------------------------------------------------------------ # 
def sqrDist(x, y):
    """sqrDist - Compute distances between column vectors in x and y.
    
    Usage: dist = sqrDist(x, y)

    Input:
    x - M-by-N matrix of N column vectors of dim M
    y - M-by-K matrix of K column vectors of dim M

    Output:
    dist - N-by-K matrix of euclidean distances, s.t. 
            dist[i,j] = sqrt(sum((x[:,i]-y[:,j])**2)
    """
    import numpy as np

    # This does not work with 1-dim arrays, quick fix to ensure 2-dim
    if np.size(x.shape) == 1:
        x = x[:,None]
    if np.size(y.shape) == 1:
        y = y[:,None]

    # Faster for big matrices with low dimensionality
    if x.shape[1]<10:
        d = np.zeros( (x.shape[1],y.shape[1]), dtype = x.dtype)
        for i in xrange(x.shape[0]):
            diff2 = x[i,:,None] - y[i,:]
            diff2 **= 2
            d += diff2
        np.sqrt(d,d)
        return d

    # For small matrices with big dimensionality, this is faster
    else:
        d = np.dot(x.T,y)
        d *= -2.0
        d += (x*x).sum(0)[:,None]
        d += (y*y).sum(0)
        # Rounding errors occasionally cause negative entries in d
        d[d<0] = 0
        # in place sqrt
        np.sqrt(d,d)
        return d


# ------------------------------------------------------------------------ # 
def cv2array(im, cv, rgb2bgr=True):
    '''cv2array(im, cv) - Convert OpenCV array to Numpy array
       
        Usage: numpy_arr = cv2array(cv_arr, cv, rgb2bgr)

        rgb2bgr{False} - If true, switch image channels from RGB to BGR or
                         viceversa
        NOTE: We pass OpenCV as a module 'cv' because I don't want to load it
              with the rest of the 'utils' library.
        NOTE2: Be careful, because OpenCV often loads images as BGR. Set
               RGB2BGR=True if you want the channels to be switched as well.
    '''
    import numpy as np

    depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
    }

    # Optional channel switching
    if rgb2bgr:
        cv.CvtColor(im, im, cv.CV_BGR2RGB)

    a = np.fromstring(
         im.tostring(),
         dtype=depth2dtype[im.depth],
         count=im.width*im.height*im.nChannels)
    a.shape = (im.height,im.width,im.nChannels)
    return a

# ------------------------------------------------------------------------ # 
def array2cv(a, cv):
    '''array2cv(numpy_arr, cv) - Convert OpenCV array to Numpy array
       
        Usage: cv_arr = array2cv(numpy_arr, cv)

        NOTE: We pass OpenCV as a module 'cv' because I don't want to load it
              with the rest of the 'utils' library.
    '''

    dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
    }
    try:
        nChannels = a.shape[2]
    except:
        nChannels = 1
    
    cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]),
                                 dtype2depth[str(a.dtype)],
                                 nChannels)
    cv.SetData(cv_im, a.tostring(),
               a.dtype.itemsize*nChannels*a.shape[1])
    
    return cv_im

# ------------------------------------------------------------------------ # 
def array2pil(a, copy=False):
    '''array2pil(numpy_arr, copy) - Convert PIL image to Numpy array
       
        Usage: PIL_img = array2pil(numpy_arr)

        Input:
            numpy_arr - N-by-M numpy array
            copy{False} - Copy data or just generate a view of numpy_arr
    '''

    import Image
    img = Image.fromarray(a)

    if copy:
        return img.copy()
    else:
        return img

# ------------------------------------------------------------------------ # 
def pil2array(a, copy=False):
    '''pil2array(pil_img, copy) - Convert PIL image to Numpy array
       
        Usage: numpy_arr = array2pil(pil_img, copy=False)

        Input:
            pil_img - N-by-M-by-K PIL image
            copy{False} - Copy data or just generate a view of pil_img

        Output:
            numpy_arr - N-by-M-by-K numpy NDarray
    '''

    import numpy as np
    img = np.asarray(a)

    if copy:
        return img.copy()
    else:
        return img


# ------------------------------------------------------------------------ # 
def ProjectPts(pts3D, cam_pose, KK, filter_negZ=True):
    """ ProjectPts - Project 3D points to camera

    Usage: pts2D = ProjectPts(pts3D, cam_pose, KK, [filter_negZ])

    Input:
        pts3D - 3-by-N numpy array of points
        cam_pose - 6-by-1 or 4-by-4 numpy array of camera extrinsic
                   parameters (world-to-image)
        KK - Camera Intrinsic parameters [fx fy cx cy]
        filter_negZ{True} - If true, filters all points behind the camera
                            (Z < 0) and sets them to [nan nan].

    Output:
        pts2D - 2-by-N numpy array of points [x y] in pixel coords as 
                projected in the image plane of CalibImage.
    """
    import numpy as np
    from tf_format import tf_format
    
    # If there are no points in, no points out 
    if not pts3D.shape[1]:
        return np.array([])

    pts2D = np.zeros([2, pts3D.shape[1]])

    # Get rotation and translation
    R, T = tf_format('RT', cam_pose)

    # Transform points in world coords to camera coords
    tx_pts3D = np.dot(R, pts3D) + T[:,None]

    # Perspective projection (I use abs(Z) because I don't want points
    # behind the camera
    pts2D[0, :] = KK[0] * tx_pts3D[0,:] / tx_pts3D[2,:] + KK[2]
    pts2D[1, :] = KK[1] * tx_pts3D[1,:] / tx_pts3D[2,:] + KK[3]

    if filter_negZ:
        negZ = tx_pts3D[2,:] < 0
        pts2D[:, negZ] = np.nan 
    
    return pts2D

# ------------------------------------------------------------------------ #
def out_of_bounds(arr, bounds):
    """OUT_OF_BOUNDS - Find all values that are out of bounds in 2d-arrays,
        in particular 2D and 3D points.

        Usage: idx = out_of_bounds(arr, bounds)

        Input:
            arr - K-by-N numpy array containing k-dim vectors in each column
            bounds - K-by-2 array of [min, max] values for each dimension

        Output:
            idx - N-by-1 array of boolean values, where idx[i] = True 
                  represents a vector with one or more values out of bounds.
                  In other words:
                  idx[i] = False   iff arr[i,k] >= bounds[k,0] \
                                   or arr[i,k] <= bounds[k,1], for all k
        """

    import numpy as np

    idx = np.ones(arr.shape[1], dtype=np.bool)

    for i, k in enumerate(bounds):
        idx = np.logical_and (idx, \
              np.logical_and (arr[i,:] >= k[0], arr[i,:] <= k[1]))

    return ~idx

# ------------------------------------------------------------------------ #
def breakpoint():
    """BREAKPOINT - Set breakpoint. Opens an IPDB window when the next
        exception is raised.
        
        Usage: breakpoint()

        NOTE: In order for the breakpoints to do anything, you need to 
        execute your script with 'ipython -pdb YOUR_SCRIPT.py'.

    """
    from IPython.Debugger import Tracer

    Tracer()

# ------------------------------------------------------------------------ #
def breaknow():
    """BREAKNOW - Set unconditional breakpoint. Raises an exception and
        opens the IPDB shell.
        
        Usage: breaknow()

        NOTE: In order for the breakpoints to do anything, you need to 
        execute your script with 'ipython -pdb YOUR_SCRIPT.py'.

    """
    from IPython.Debugger import Tracer

    Tracer()
    raise KeyError, "You set an unconditional breakpoint here, so we stop."

# ------------------------------------------------------------------------ #
def initshell():
    """INITSHELL - Open IPython embedded shell when executed .

        Usage: shell = initshell()
               shell()

        NOTE: These two steps are necessary so that you see the parent's
              workspace, not the one inside initshell() function.

    """
    from IPython.Shell import IPShellEmbed

    return IPShellEmbed()

# ------------------------------------------------------------------------ #
def ProjectPtsOut(pts2D, KK, cam_pose):
    """projectPtsOut - Use inverse projection to map pts in 2D to 3D rays.

    Usage: vec3D, cam_center = projectPtsOut(pts2D, KK, cam_pose);

    Input:
    pts2D - 2-by-N array of 2D points to be backprojected
    KK - internal camera parameters: [fx fy px py]. 
    cam_pose - 6-by-1 camera pose [r1 r2 r3 t1 t2 t3]' (world-to-camera),
               where [r1 r2 r3] is a rodrigues rotation vector.

    Output:
    vec3D - 3-by-N array of 3D vectors backprojected from the camera plane
    cam_center - 3-by-1 point that works as camera center. All rays
        backprojected from the camera pass through cam_center and vec3D, 
        i.e.:  line(lambda) = cam_center + lambda*vec3D;
    """

    from tf_format import tf_format
    import numpy as np

    Kinv = np.array([1/KK[0], 1/KK[1], -KK[2]/KK[0], -KK[3]/KK[1]])

    # We need cam-to-world R,T, so we use inverse-RT 'iRT'
    R, T = tf_format('iRT', cam_pose)

    cam_center = T

    # Vectors in camera coords
    vec3D = np.ones((3, pts2D.shape[1]))
    vec3D[0, :] = Kinv[0] * pts2D[0,:] + Kinv[2]
    vec3D[1, :] = Kinv[1] * pts2D[1,:] + Kinv[3]

    # Normalize vectors
    vec3D = vec3D / np.sqrt(np.sum(vec3D**2, axis=0))
    
    # Put vectors in world coords (just rotate them)
    vec3D = np.dot(R, vec3D)

    return vec3D, cam_center

# ------------------------------------------------------------------------ #
def distRay3D(pts3D, rays3D, cam_pose):
    """distRay3D - Compute distances from 3D point(s) to 3D ray(s).
    For a ray3D = [v, c], the distance to a point X = RP+t is computed as:
    dist = || (I-vv')(X-c) ||;

    Usage: dist = distRay3D(cam_pose, pts3D, rays3D, output);

    Input:
    pts3D - 3-by-N array of 3D points to compute distances from.
    rays3D - 6-by-N array of rays [v(1:3,:); c(4:6,:)] to project pts3D to.
            v is the ray direction (a vector) and c is a point in the
            ray (e.g. the camera center).
    cam_pose - 6-by-1 camera pose [r1 r2 r3 t1 t2 t3]' (world-to-camera),
               where [r1 r2 r3] is a rodrigues rotation vector.

    Output:
    dist - N-by-1 distance vector. Dist(i) = dist(pts3D(:,i), rays3D(:,i)).

    NOTE: This function computes 1 point to 1 ray distances, NOT all rays
    to all points!
    """    
    from tf_format import tf_format
    import numpy as np

    if rays3D.ndim < 2:
        rays3D = np.atleast_2d(rays3D).T

    R, T = tf_format('RT', cam_pose)

    # Transform points according to camera position
    tx_pts3D = np.dot(R, pts3D) + T[:,None]

    # Pcentered --> X-c
    Pcentered = tx_pts3D - rays3D[3:, :]

    # dotp --> v'(X-c)
    dotp = np.sum(rays3D[0:3, :] * Pcentered, axis=0)

    # This is it: dist = (I-vv')(X-c) = (X-c) - vv'(X-c)
    dist = Pcentered - (rays3D[0:3, :] * dotp)
    return np.sqrt(np.sum(dist**2, axis=0))

    # Optional: for points in front of the camera, the magnitude of the
    # dot product (1-dotp) should be greater than zero
    #mag_dotp = 1 - dotp / np.sqrt(np.sum(Pcentered**2)); # mag_dotp >= 0
    #mag_dotp = np.max(mag_dotp - 0.1, axis=0); # If 1-dotp < 0.1, consider it 0

# ------------------------------------------------------------------------ #
def pca(arr, k=None, max_energy=None, remove_mean=True, norm_evals=True):
    """pca - Compute Principal Component Analysis on data matrix

    Usage: pc, evals, mean = utils.pca(arr, remove_mean=True)
           pc, evals = utils.pca(arr, remove_mean=False)
           pc, evals = utils.pca(arr, k=10, remove_mean=False)
           pc, evals = utils.pca(arr, max_energy=0.7, remove_mean=False)

    Input:
        arr - M-by-N array of column vectors (N vectors of M-dim)
        k - Number of principal components to keep. Either k or max_energy
            should be specified, otherwise all principal components are
            returned.
        max_energy - Maximum energy (all energy = 1) that we want to keep in
                     PCA to do dimensionality reduction.
        remove_mean{True} - Remove data mean before computing PCA
        norm_evals{True} - If True, normalize eigenvalues so that they all sum
                           to 1.

    Output:
        pc - M-by-K array of principal components
        evals - Eigenvalues corresponding to the principal component, sorted.
        mean - M-by-1 matrix containing the mean of 'arr' along axis=1
    """    
    
    import numpy as np

    if remove_mean:
        mean = np.mean(arr, axis=1)
        arr_nomean = arr - mean[:, None]
    else:
        mean = 0
        arr_nomean = arr

    # I thought s were the singular values, but it gives the eigenvals
    U, s, Vh = np.linalg.svd(np.cov(arr_nomean))

    if not k and max_energy:
        energy = np.cumsum(s)/np.sum(s)
        k = np.argmax(energy > max_energy)+1 if max_energy < 1 else s.size

    if norm_evals:
        s /= np.sum(s)

    # Truncate PCA matrix
    if k == 1:
        pc = U[:,0,None]
    elif k:
        pc = U[:,:k]
    else:
        pc = U

    if remove_mean:
        return pc, s, mean
    else:
        return pc, s


# ------------------------------------------------------------------------ #
def q_score(arr, sigma2 = 1, metric = 'L2sq', sum_all=True):
    """q_score - Compute robust q-score from array of differences. For each
    col vector, the q_score is defined as the Cauchy distribution:
        q_score(col) = 1/(1 + metric(col)/sigma2)

    Usage: q_score(arr, sigma = 1, metric = 'L2', sum_all=False)

    Input:
        arr - M-by-N array of column vectors (N vectors of M-dim)
        sigma2{1} - Normalization factor in Cauchy distribution
        metric{'L2sq'} - 'L1', 'L2', 'L2sq', or your own metric (lambda
            function) to average all dimensions from each M-dim vector.
        sum_all{True} - sum all q_scores of all vectors

    Output:
        score - 1-by-N array (or scalar, if sum_all=True) containing computed
                q_scores for each column vector.
    """
    import numpy as np

    if metric == 'L2sq':
        d = lambda x: np.sum(x**2)
    elif metric == 'L2':
        d = lambda x: np.sqrt(np.sum(x**2))
    elif metric == 'L1':
        d = lambda x: np.sum(np.abs(x))
    else:
        d = metric

    # Adapt data if necessary
    if type(arr) is not np.ndarray:
        arr = np.array([arr])
    if len(arr.shape) == 1:
        arr = arr[None,:]

    q_sc = lambda col: 1. / (1 + d(col)/sigma2)
    q_score = np.zeros(arr.shape[1])
        
    for i, row in enumerate(arr.T):
        q_score[i] = q_sc(row)

    if sum_all:
        return np.sum(q_score)
    else:
        return q_score


# ------------------------------------------------------------------------ #
def chi2(arr1, arr2):
    """chi2 - Compute the chi-square distance between two arrays/histograms.
    This function operates over flattened arrays.
    
    Usage: chi2_dist = utils.chi2(arr1, arr2)

    Input:
        arr1, arr2 - M-by-N arrays/histograms to compute chi-square distances
                     over. You might want to normalize them first for better
                     results...
    
    Output:
        chi2_dist - Result of comparing the two arrays. Result=0 if the two
            arrays are identical, and the bigger this result is, the more
            dissimilar these arrays are.
    """

    import numpy as np
    return np.sum( (arr1 - arr2)**2 / (arr1 + arr2 + np.finfo(np.float).eps) )


# ------------------------------------------------------------------------ #
def ann(arr1, arr2, n_neighs=1, eps=0.):
    """ANN - Approximate nearest neighbor search using kd-Trees (and Scipy).
    The search is performed from arr2 to arr1, that is, for each point in
    arr2, we search its k nearest neighbors in arr1.

    Usage: nn_dist, nn_idx = utils.ann(arr1, arr2, n_neighs, eps)

    Input:
        arr1 - M-by-N array of column points (N points of M dims)
        arr2 - M-by-L array of query points (K points of M dims)
        n_neighs{1} - Number of nearest neighbors to compute
        eps{0.0} - Approximate search. The kth returned value is guaranteed
            to be no further than (1+eps) times the distance to the real
            nearest neighbor.

    Output:
        nn_dist - (n_neighs)-by-L array of distances to arr1
        nn_idx - (n_neiggs)-by-L array of locations (indexes) to points
                 in arr1 such that:
                    dist(arr1[:,nn_idx[i]], i]) = nn_dist[:,i]
    """
    import scipy.spatial as SP 

    kd_arr1 = SP.KDTree(arr1.T.copy())
    nn_dist, nn_idx = kd_arr1.query(arr2.T, n_neighs, eps)

    return nn_dist, nn_idx


# ------------------------------------------------------------------------ #
def ravel_index(idx, dims):
    """ravel_index - From ND-indexes, compute their flattened 1d equivalent
        indexes (equivalent to sub2ind in Matlab).

        Usage: flat_idx = ravel_index(idx, dims)

        Input:
            idx - N-by-M array of N-dimensional indexes
            dims - N-by-1 array containing indexed array dimensions

        Output:
            flat_idx - N-by-1 array containing flat indexes, such that
                       array[idx[0,:], idx[1,:]..., idx[N,:]] = \
                       array.flat[flat_idx]
    """
    import numpy as np

    offsets = np.cumprod(dims[1:][::-1])[::-1]
    return (np.sum(idx[:-1,:]*offsets[:,None], axis=0) + idx[-1,:]).astype(int)

# ------------------------------------------------------------------------ #
def unravel_index(flat_idx, dims):
    """ravel_index - From flat indexes, compute their ND-equivalent
        indexes (equivalent to ind2sub in Matlab). This function is
        equivalent to numpy.unravel_index, but works with arrays of indexes.
        

        Usage: idx = unravel_index(flat_idx, dims)

        Input:
            flat_idx - N-by-1 array containing flat indexes, such that
                array[idx[:,0], idx[:,1]..., idx[:,N]] = \
                array.flat[flat_idx]
            dims - M-by-1 array containing indexed array dimensions

        Output:
            idx - N-by-M array of N-dimensional indexes
    """
    import numpy as np

    idx = np.zeros((np.array(dims).size, flat_idx.size), dtype=int)
    offsets = np.cumprod(dims[1:][::-1])[::-1]

    for i, off in enumerate(offsets):
        idx[i,:] = flat_idx / off
        flat_idx = np.remainder(flat_idx, off) 

    idx[-1,:] = flat_idx
    return idx

# ------------------------------------------------------------------------ #
def show(arr, shape=None):
    """show - Create new temp figure and show array contents as image.

        Usage: show(arr, shape=None) 

        Input:
            arr - Either numpy nd-array or PIL image.
            shape - (optional) if arr is a flat array, shape is a tuple
                    with the desired shape, in (rows, cols) format.
            
        Output:
            -NONE- but a figure is created.
    """
    import Image
    import numpy as np

    if shape:
        arr = arr.reshape(shape)

    if type(arr) is np.ndarray:
        Image.fromarray( (arr.astype(float) * 255 / arr.max()).astype(np.uint8) ).show()
    else:
        try:
            arr.show()
        except AttributeError:
            print "Unknown array type"


# ------------------------------------------------------------------------ #
def obj2arr(ObjList, field):
    """obj2arr - Generate numpy array from a field in a list of objects.
    E.g.: exp[0].ratio = 0.4; exp[1].ratio = 0.5; exp[2].ratio = 0.8
    arr = obj2arr(exp, 'ratio') --> arr = [0.4, 0.5, 0.8]

    Usage: arr = obj2arr(ObjList, field)

    Input:
        ObjList - Iterable of N object instances
        field - Field in each object instance to extract in array, each
            containing a scalar value.

    Output:
        arr - If 'field' contains scalars, output is a np.ndarray of N 
              floats. Otherwise, it is a list.
    """
    import numpy as np
    l = list()
    for O in ObjList:
        l.append(getattr(O, field))

    try:
        out = np.array(l, dtype=np.float) 
    except ValueError:
        out = l

    return out 

# ------------------------------------------------------------------------ #
def arr2obj(ObjList, field, iter):
    """obj2arr - Copy iterable[i] into ObjList[i].field.

    Usage: ObjList = arr2obj(ObjList, field, iterable)

    Input:
        ObjList - Iterable of N object instances
        field - Field in each object instance to set.
        iter - iterable, must have same len() than ObjList.

    Output:
        ObjList - Iterable of N object instances with updated field.
    """

    if len(iter) != len(ObjList):
        raise ValueError, "arr and ObjList must have same length"
    for i, O in enumerate(ObjList):
        setattr(O, field, iter[i])

    return ObjList

# ------------------------------------------------------------------------ #
def dict2arr(ObjList, field):
    """obj2arr - Generate array from 'field' in a list of dictionaries.
    E.g.: exp[0]['ratio'] = 0.4; exp[1]['ratio'] = 0.5;
          exp[2]['ratio'] = 0.8
    arr = dict2arr(exp, 'ratio') --> arr = [0.4, 0.5, 0.8]

    Usage: arr = dict2arr(ObjList, field)

    Input:
        ObjList - Iterable of N object instances, each dictionary-like
        field - Field in each dictionary to extract in array, each
            containing a scalar value.

    Output:
        arr - If 'field' contains scalars, output is a np.ndarray of N 
              floats. Otherwise, it is a list.
    """
    import numpy as np
    l = list()
    for O in ObjList:
        l.append(O[field])

    try:
        out = np.array(l, dtype=np.float) 
    except ValueError:
        out = l

    return out 

# ------------------------------------------------------------------------ #
def arr2dict(ObjList, field, iter):
    """arr2dict - Copy iterable[i] into ObjList[i].field.

    Usage: ObjList = arr2obj(ObjList, field, iterable)

    Input:
        ObjList - Iterable of N object instances
        field - Field in each dict instance.
        iter - iterable, must have same len() than ObjList.

    Output:
        ObjList - Iterable of N dict instances with updated field.
    """

    if len(iter) != len(ObjList):
        raise ValueError, "arr and ObjList must have same length"
    for i, O in enumerate(ObjList):
        O[field] = iter[i]

    return ObjList 

# ------------------------------------------------------------------------ #
def lock_file(filename, create_folder=True, check_existence=True):
    """lock_file - Create a lock for filename (filename.lock)

    Usage: Success = lock_file(filename, create_folder, check_existence) 

    Input:
        filename - file to create a lock on
        create_folder{True} - True: If folder where file must reside does 
                              not exist, create it.
        check_existence{True} - True: If file already exists, don't lock it

    Output:
        Success - Return True if we created the lock successfully. Return
                  False if lock already created or if we can't create it
                  for some reason (folder did not exist, or file or lock 
                  already existed).
    """
    import os.path

    # Get output name and folder
    folder, name = os.path.split(filename)
    
    # Does this path exist?
    if not os.path.exists(folder):
        if create_folder:
            try:
                import os
                os.mkdir(folder)
            except:
                pass
        else:
            print folder + " does not exist."
            return False

    # Does this experiment exist?
    if check_existence and os.path.exists(filename):
        print filename + " already exists."
        return False
        
    # Is it locked?
    elif os.path.exists(filename + '.lock'):
        print filename + " is already locked."
        return False

    # Output file does not exist. Lock it so that nobody else works on it.
    else:
        open(filename + '.lock', 'w').close()
        return True

# ------------------------------------------------------------------------ #
def unlock_file(filename):
    """unlock_file - Delete a lock for filename (filename.lock)

    Usage: Success = unlock_file(filename) 

    Input:
        filename - file to release a lock on

    Output:
        Success - Return True if we deleted the lock successfully. Return
                  False if lock was not there or couldn't be deleted.
    """
    import os

    try:
        os.remove(filename + '.lock')
        return True
    except:
        return False

# ------------------------------------------------------------------------ #
def imfilter(I, K, cache=None):
    """
    Filter image I with kernel K.

    Image color values outside I are set equal to the nearest border color on I.

    To filter many images of the same size with the same kernel more efficiently, use:

    >>> cache = []
    >>> filter(I1, K, cache)
    >>> filter(I2, K, cache)
    ...

    An even width filter is aligned by centering the filter window around each given
    output pixel and then rounding down the window extents in the x and y directions.
    """
    import numpy as np

    def roundup_pow2(x):
        y = 1
        while y < x:
            y *= 2
        return y

    I = np.asarray(I)
    K = np.asarray(K)

    if len(I.shape) == 3:
        s = I.shape[0:2]
        L = []
        ans = np.concatenate([filter(I[:,:,i], K, L).reshape(s+(1,))
                                 for i in range(I.shape[2])], 2)
        return ans
    if len(K.shape) != 2:
        raise ValueError('kernel is not a 2D array')
    if len(I.shape) != 2:
        raise ValueError('image is not a 2D or 3D array')

    s = (roundup_pow2(K.shape[0] + I.shape[0] - 1),
         roundup_pow2(K.shape[1] + I.shape[1] - 1))
    Ipad = np.zeros(s)
    Ipad[0:I.shape[0], 0:I.shape[1]] = I

    if cache is not None and len(cache) != 0:
        (Kpad,) = cache
    else:
        Kpad = np.zeros(s)
        Kpad[0:K.shape[0], 0:K.shape[1]] = np.flipud(np.fliplr(K))
        Kpad = np.fft.rfft2(Kpad)
        if cache is not None:
            cache[:] = [Kpad]

    Ipad[I.shape[0]:I.shape[0]+(K.shape[0]-1)//2,:I.shape[1]] = I[I.shape[0]-1,:]
    Ipad[:I.shape[0],I.shape[1]:I.shape[1]+(K.shape[1]-1)//2] = I[:,I.shape[1]-1].reshape((I.shape[0],1))

    xoff = K.shape[0]-(K.shape[0]-1)//2-1
    yoff = K.shape[1]-(K.shape[1]-1)//2-1
    Ipad[Ipad.shape[0]-xoff:,:I.shape[1]] = I[0,:]
    Ipad[:I.shape[0],Ipad.shape[1]-yoff:] = I[:,0].reshape((I.shape[0],1))

    Ipad[I.shape[0]:I.shape[0]+(K.shape[0]-1)//2,I.shape[1]:I.shape[1]+(K.shape[1]-1)//2] = I[-1,-1]
    Ipad[Ipad.shape[0]-xoff:,I.shape[1]:I.shape[1]+(K.shape[1]-1)//2] = I[0,-1]
    Ipad[I.shape[0]:I.shape[0]+(K.shape[0]-1)//2,Ipad.shape[1]-yoff:] = I[-1,0]
    Ipad[Ipad.shape[0]-xoff:,Ipad.shape[1]-yoff:] = I[0,0]

    ans = np.fft.irfft2(np.fft.rfft2(Ipad) * Kpad, Ipad.shape)

    off = ((K.shape[0]-1)//2, (K.shape[1]-1)//2)
    ans = ans[off[0]:off[0]+I.shape[0],off[1]:off[1]+I.shape[1]]

    return ans


################################################################################
#
# SizedDict class
#
################################################################################
class SizedDict(dict):
    """ Sized dictionary without timeout. Optional parameter 'size' when 
    initializing the dictionary is the global size of the dictionary. After
    'size' entries are written, the dictionary overwrites the oldes entry."""

    def __init__(self, size=10):
        """ Initialization procedure.

        Usage: d = SizedDict(size=10)

        Input:
            size - Maximum dictionary size. Unbounded dict: size=Inf. But if 
            you want it unbounded, you might as well use a regular dictionary.
        """

        dict.__init__(self)
        self._maxsize = size
        self._stack = []

    def __setitem__(self, name, value):
        if len(self._stack) >= self._maxsize:
            self.__delitem__(self._stack[0])
            del self._stack[0]
        self._stack.append(name)
        return dict.__setitem__(self, name, value)

    # Recommended but not required:
    def get(self, name, default=None, do_set=False):
        try:
            return self.__getitem__(name)
        except KeyError:
            if default is not None:
                if do_set:
                    self.__setitem__(name, default)
                return default
            else:
                raise

################################################################################
#
# CacheDict class
#
################################################################################
class CacheDict(dict):
    """ Sized dictionary with a timeout. Optional parameter 'size' when 
    initializing the dictionary is the global size of the dictionary. After
    'size' entries are written, the dictionary overwrites the oldes entry.
    Optional parameter 'timeout' specifies how long a dictionary entry is kept
    before deleting it."""

    def __init__(self, size=10, timeout=None):
        """ Initialization procedure.

        Usage: d = SizedDict(size=10, timeout=None)

        Input:
            size - Maximum dictionary size.
            timeout - maximum time an entry is kept
            """
        dict.__init__(self)
        self._maxsize = size
        self._stack = []
        self._timeout = timeout

    def __setitem__(self, name, value, timeout=None):
        if len(self._stack) >= self._maxsize:
            self.__delitem__(self._stack[0])
            del self._stack[0]
        if timeout is None:
            timeout = self._timeout
        if timeout is not None:
            timeout = time.time() + timeout
        self._stack.append(name)
        dict.__setitem__(self, name, (value, timeout))

    def get(self, name, default=None):
        try:
            focus = self.__getitem__(name)
            if focus[1] is not None:
                if focus[1] < time.time():
                    self.__delitem__(name)
                    self._stack.remove(name)
                    raise KeyError
            return focus[0]
        except KeyError:
            return default


# ------------------------------------------------------------------------ #
def merge_images(pattern, crop_size = None, output_filename = None):
    """merge_images - Merge all images in a folder into a single file, removing
    white space for each of them.

    Usage: out_file = merge_images (pattern = './images/*.jpg', 
                                    output_filename = 'img.png',
                                    crop_size = (200,200))

    Input:
        pattern - Folder to use. A list of filenames is also accepted.
        output_filename - Filename to save image into (if None, do not store).
        crop_size{None} - tuple with dimensions to crop around ROI center of
            each image.

    Output:
        out_file - Merged image.
    """
    import os, glob, Image
    import numpy as np
    
    # Default number of columns
    maxColumns = 8

    if ~isinstance(pattern, list):
        files = glob.glob(pattern)
    else:
        files = pattern

    if crop_size is None:
        try:
            img = Image.open(files[0])
            crop_size = img.size
        except:
            print("Could not open file")
            return

    nCols = min(maxColumns, len(files))
    nRows = 1 + len(files) / maxColumns
    BigImg = np.zeros((crop_size[1] * nRows, crop_size[0] * nCols, 3))

    for idx, f in enumerate(files):
        c = idx % nCols
        r = idx / nCols

        img_tmp = Image.open(f)
        img = np.asarray(img_tmp)

        # find first used row
        for first_row, row in enumerate(img):
            if np.any(row):
                break

        # find first used col
        for first_col in range(img.shape[1]):
            if np.any(img[:, first_col, :]):
                break

        last_row = min(img.shape[0]-1, first_row + crop_size[0])
        last_col = min(img.shape[1]-1, first_col + crop_size[1])
        
        BigImg[crop_size[0]*r:(crop_size[0]*r + last_row - first_row), \
               crop_size[1]*c:(crop_size[1]*c + last_col - first_col), :] = \
                img[first_row:last_row, first_col:last_col, :]

    if output_filename is not None:
        Image.fromarray(BigImg.astype(np.uint8)).save(output_filename)

    return BigImg

# ------------------------------------------------------------------------ #
def circle(radius, rounding=True):
    """
    circle - Create meshgrid in circular shape.

    Usage: idx_X, idx_Y = circle(radius, rounding = True)

    Input:
        radius - Circle radius
        rounding{True} - Round indexes so that they are integers
    Output:
        idx_X, idx_Y - X and Y indexes in meshgrid.
    """
    import numpy as np

    # Create linear indexes
    y, x = np.ogrid[-radius: radius, -radius: radius]

    # 2D indexes
    idx_X, idx_Y = np.meshgrid(x.ravel(), y.ravel())

    # Circular condition
    index = x**2 + y**2 <= radius**2

    if rounding:
        idx_XX = np.ceil(idx_X[index]).astype(np.int)
        idx_YY = np.ceil(idx_Y[index]).astype(np.int)
    else:
        idx_XX = idx_X[index]
        idx_YY = idx_Y[index]

    return idx_XX, idx_YY


# ------------------------------------------------------------------------ #
def imreadDepth(imgname, conversion_rate = 1000):
    """ Load depth image from png file. The png file must be uint16 format
        and filled with depth measurements in mm.

        Usage: depth = imreadDepth(imgname, conversion_rate = 1000)

        Input:
            imgname - Name to depth image (must finish in .png)
            conversion_rate{1000} - Units multiplier with respect to meters.
                If image is in mm (default), then conversion_rate=1000
        Output:
            depth - DepthImage (subclass of numpy) of depth values. Each
                value is a float, in *meters*.
    """

    import cv
    import numpy as np
    from PCloud2 import DepthImage

    cvDepth = cv.LoadImage (imgname, cv.CV_LOAD_IMAGE_UNCHANGED)
    depth = np.fromstring (cvDepth.tostring(), dtype = np.uint16)
    depth = depth.astype( np.float ) / conversion_rate # We want depth map in meters
    depth = depth.reshape((cvDepth.height, cvDepth.width))

    return DepthImage(depth)

# ------------------------------------------------------------------------ #
def countbits(number):
    """ Count the number of set bits in the binary form of 'number'.

    Usage: nbits = countbits (number)

    Input:
        number - integer to count for bits
    Output:
        nbits - number of active bits (=1) in 'number'
    """

    return bin(number).count('1')


# ------------------------------------------------------------------------ #
def enumerate_binomial(m, n, nFirst = None):
    """ Enumerate the combinations of (m choose n). If nFirst is not none,
    the enumeration stops after finding the nFirst combinations.

    Usage: combi = enumerate_binomial(m, n, nFirst)

    Input: 
        m - Pool of elements
        n - Subsets of elements to be chosen
        nFirst{None} - Number of combinations to return. If None, return
        all combinations.
    """
    # Important libraries
    import numpy as np

    # Get max number of combinations
    max_number = 2**m
    nFirst = max_number if nFirst == None else nFirst

    combi = list()
    for i in range(max_number):
        # Is this a combination of n numbers?
        if countbits(i) == n:
            # Parse binary form and get the binary indices
            strbits = bin(i)
            num = np.zeros((n,))
            numid = 0
            for idx, b in enumerate(strbits[::-1]):
                if b == '1':
                    num[numid] = idx
                    numid += 1
            
            # Append combination to list of combinations
            combi.append(num)
            
            if len(combi) >= nFirst:
                return combi

    return combi

