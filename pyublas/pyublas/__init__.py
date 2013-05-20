import numpy
import pyublas._internal




has_sparse_wrappers = pyublas._internal.has_sparse_wrappers
unstrided_size = pyublas._internal.unstrided_size
strided_size = pyublas._internal.strided_size

set_trace = pyublas._internal.set_trace




# type code-related  -----------------------------------------------------------
def _dtype_name(dtype):
    if dtype in [numpy.float64, numpy.float, float, numpy.dtype(numpy.float64)]:
        return "Float64"
    elif dtype in [numpy.complex128, numpy.complex, complex, numpy.dtype(numpy.complex128)]:
        return "Complex128"
    else:
        raise RuntimeError("Invalid dtype specified")




class ParameterizedType(object):
    """
    A base class for "types" that depend on a dtype.

    This is a rather internal class.
    """

    def __init__(self, name, use_dict=None):
        if use_dict is None:
            use_dict = pyublas._internal.__dict__
        self.Name = name

        type_dict = {}
        for tc in DTYPES_AND_ALIASES:
            type_dict[tc] = use_dict[name + _dtype_name(tc)]
        self.TypeDict = type_dict

    def is_a(self, object):
        try:
            return isinstance(object, self(object.dtype))
        except AttributeError:
            return False

    def __str__(self):
        return self.Name

    def __call__(self, dtype):
        return self.TypeDict[dtype]

    def make(self, dtype, *args, **kwargs):
        return self.TypeDict[dtype](*args, **kwargs)

    def get_name(self):
        return self.Name
    name = property(get_name)




DTYPES_AND_ALIASES = [
    numpy.dtype(numpy.float64),
    numpy.dtype(numpy.complex128),
    numpy.float64,
    numpy.complex128,
    numpy.float,
    numpy.complex,
    float,
    complex
    ]
DTYPES = [ float, complex ]




if has_sparse_wrappers():
    SparseBuildMatrix = ParameterizedType("SparseBuildMatrix")
    SparseExecuteMatrix = ParameterizedType("SparseExecuteMatrix")
else:
    SparseBuildMatrix = None
    SparseExecuteMatrix = None




# python-implemented methods --------------------------------------------------
FLAVORS = [
        SparseBuildMatrix,
        SparseExecuteMatrix,
        ]




def _add_python_methods():
    def _is_number(value):
        try: 
            complex(value)
            return True
        except AttributeError:
            return False
        except TypeError:
            return False




    def _is_matrix(value):
        try: 
            return value.flavor in FLAVORS
        except NameError:
            return False




    # stringification ---------------------------------------------------------
    def _wrap_vector(strs, max_length=80, indent=8*" ", first_indent=0):
        result = ""
        line_length = first_indent
        for i, s in enumerate(strs):
            item_length = len(s) + 2
            if line_length + item_length > max_length:
                line_length = len(indent)
                result += "\n%s" % indent
            line_length += item_length
            result += s
            if i != len(strs) - 1:
                result += ", "
        return result

    def _stringify_sparse_matrix(array, num_stringifier, max_length=80):
        strs = []
        last_row = -1
        for i, j in array.indices():
            if i != last_row:
                current_row = []
                strs.append((i, current_row))
                last_row = i

            current_row.append("%d: %s" % (j, num_stringifier(array[i,j])))

        result = ""
        for row_idx, (i,row) in enumerate(strs):
            indent = 10+len(str(row_idx))
            result += "%d: {%s}" % (i, _wrap_vector(row, 
                indent=(indent + 4)*" ", first_indent=indent, max_length=max_length))
            if row_idx != len(strs) - 1:
                result += ",\n" + 8 * " "
        return "sparse({%s},\n%sshape=%s, flavor=%s)" % (
            result, 7*" ",repr(array.shape), array.flavor.name)

    def _str_sparse_matrix(array): return _stringify_sparse_matrix(array, str)
    def _repr_sparse_matrix(array): return _stringify_sparse_matrix(array, repr)

    # equality testing --------------------------------------------------------
    def _equal(a, b):
        try:
            if type(a) != type(b):
                return False
            if a.shape != b.shape:
                return False
            diff = a - b
            for i in diff.indices():
                if diff[i] != 0:
                    return False
            return True
        except AttributeError:
            return False

    def _not_equal(a, b):
        return not _equal(a, b)





    # add the methods ---------------------------------------------------------
    def get_returner(value):
        # This routine is necessary since we don't want the lambda in
        # the top-level scope, whose variables change.
        return lambda self: value

    for dtype in DTYPES:
        for f in FLAVORS:
            co = f(dtype)
            co.__eq__ = _equal
            co.__ne__ = _not_equal
            
            co.__pos__ = lambda x: x

            co.flavor = property(get_returner(f))
            co.dtype = property(get_returner(dtype))
            co.typecode = get_returner(dtype)

        # stringification -----------------------------------------------------
        for f in FLAVORS:
            co = f(dtype)
            co.__str__ = _str_sparse_matrix
            co.__repr__ = _repr_sparse_matrix
            co.format = _stringify_sparse_matrix




if has_sparse_wrappers():
    _add_python_methods()




# public interface  ------------------------------------------------------------
if has_sparse_wrappers():
    def zeros(shape, dtype=float, flavor=SparseBuildMatrix):
        """Return a zero-filled array."""
        return flavor(dtype)(*shape)





    def asarray(data, dtype=None, flavor=None):
        """Construct an array from data.
        
        Same as array(), except that a copy is made only when necessary.
        """

        try:
            given_flavor = data.flavor
            given_dtype = data.dtype
        except AttributeError:
            # not handling a pylinear array--leave that to the general array
            # constructor.
            raise ValueError("asarray's first argument must be a PyUblas array")

        if flavor is None:
            flavor = given_flavor

        if dtype is None:
            dtype = given_dtype

        if given_dtype == dtype and given_flavor == flavor:
            return data

        return flavor(dtype)(data)




    def sparse(mapping, shape=None, dtype=None, flavor=SparseBuildMatrix):
        """Create a sparse Array from (two-level) nested
        mappings (e.g. dictionaries).

        Takes into account the given dtype  and  flavor.   If  None  are  specified,
        the   minimum   that   will   accomodate   the   given   data   are    used.
        If shape is unspecified, the smallest size  that  can  accomodate  the  data
        is used.

        See array() for valid dtype and flavors.
        """

        def get_biggest_type(mapping, prev_biggest_type=float):
            for row in mapping.values():
                for val in row.values():
                    if isinstance(val, complex):
                        prev_biggest_type = complex
            return prev_biggest_type

        if dtype is None:
            dtype = get_biggest_type(mapping)

        if shape is None:
            height = max(mapping.keys()) + 1
            width = 1
            for row in mapping.values():
                width = max(width, max(row.keys())+1)

            shape = height, width

        mat = flavor(dtype)(shape[0], shape[1])
        for i, row in mapping.iteritems():
            for j, val in row.iteritems():
                mat[i,j] = val
        return mat




    def permutation_matrix(to_indices=None, from_indices=None, h=None, w=None,
            dtype=float, flavor=SparseExecuteMatrix):
        """Return a permutation matrix.

        If to_indices is specified, the resulting permutation 
        matrix P satisfies the condition

        P * e[i] = e[to_indices[i]] for i=1,...,len(to_indices)

        where e[i] is the i-th unit vector. The height of P is 
        determined either implicitly by the maximum of to_indices
        or explicitly by the parameter h.

        If from_indices is specified, the resulting permutation 
        matrix P satisfies the condition

        P * e[from_indices[i]] = e[i] for i=1,...,len(from_indices)
        
        where e[i] is the i-th unit vector. The width of P is
        determined either implicitly by the maximum of from_indices
        of explicitly by the parameter w.

        If both to_indices and from_indices is specified, a ValueError
        exception is raised.
        """
        if to_indices is not None and from_indices is not None:
            raise ValueError("only one of to_indices and from_indices may "
                    "be specified")

        if to_indices is not None:
            if h is None:
                h = max(to_indices)+1
            w = len(to_indices)
        else:
            if w is None:
                w = max(from_indices)+1
            h = len(from_indices)

        result = zeros((h,w), flavor=SparseBuildMatrix, dtype=dtype)

        if to_indices is not None:
            for j, i in enumerate(to_indices):
                result.add_element(i, j, 1)
        else:
            for i, j in enumerate(from_indices):
                result.add_element(i, j, 1)

        return asarray(result, flavor=flavor)
else:
    def zeros(shape, dtype=float, flavor=SparseBuildMatrix):
        """Unavailable."""
        return None

    def asarray(data, dtype=None, flavor=None):
        """Unavailable."""
        return None

    def sparse(mapping, shape=None, dtype=None, flavor=SparseBuildMatrix):
        """Unavailable."""
        return None

    def permutation_matrix(to_indices=None, from_indices=None, h=None, w=None,
            dtype=float, flavor=SparseExecuteMatrix):
        """Unavailable."""
        return None




# C++ interface utilities -----------------------------------------------------
def why_not(val, dtype=float, matrix=False, row_major=True):
    from warnings import warn
    import numpy
    if not isinstance(val, numpy.ndarray):
        warn("array is not a numpy object")
    elif dtype != val.dtype:
        warn("array has wrong dtype (%s)" % str(val.dtype))
    elif not val.flags.aligned:
        warn("array is not aligned")
    elif not val.flags.aligned:
        warn("array is not aligned")
    elif val.dtype.byteorder not in ["=", "|"]:
        warn("array does not have the right endianness (%s)" % val.dtype.byteorder)
    elif matrix:
        if len(val.shape) != 2:
            warn("array rank is not 2")
        else:
            if val.strides[1] == val.itemsize:
                # array is row-major
                if not row_major:
                    warn("array is row-major, but column-major arrays are accepted")
                if not val.flags.c_contiguous:
                    warn("array is row-major, but rows are not contiguous")
            elif val.strides[0] == val.itemsize:
                # array is column-major
                if row_major:
                    warn("array is column-major, but row-major arrays are accepted")
                if not val.flags.f_contiguous:
                    warn("array is column-major, but columns are not contiguous")
            else:
                warn("array is not contiguous")
    return val
