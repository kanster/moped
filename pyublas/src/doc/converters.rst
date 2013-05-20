.. highlight:: c++

.. _frompython:

Automatic From-Python Conversion
================================

Shapes, Ranks and Sizes
-----------------------

Two simple rules:

* Any shape of array can be converted to a :ctype:`numpy_vector`.

* Only 2D arrays of the right element order (i.e. column-/row-major) can be
  converted to :ctype:`numpy_matrix`.

If a multi-dimensional array is converted to a :ctype:`numpy_vector`,
the data in the vector will be a flattened representation of that
vector.

References and Pointers
-----------------------

PyUblas registers from-Python converters only for *rvalues*. This means that
PyUblas will not successfully wrap functions that expect a
:ctype:`numpy_vector` lvalue, i.e. a reference or a pointer::

  void do_stuff1(numpy_vector<double> &v) { ... }       // (1) WILL NOT WORK
  void do_stuff2(numpy_vector<double> *v) { ... }       // (2) WILL NOT WORK
  void do_stuff3(const numpy_vector<double> &v) { ... } // (3) OK, can't modify v
  void do_stuff4(numpy_vector<double> v) { ... }        // (4) OK, can modify v

Note that versions (1) and (2) will successfully compile, but any invocation to
``do_stuff1`` and ``do_stuff2`` from Python will simply fail, with Boost.Python
reporting that no valid overload could be found. Version (3) is fine, because
const references count as rvalues. Version (4) is the recommended version if
you want to modify `v` in-place. Recall that :ctype:`numpy_vector` is a handle class
for an existing, separate chunk of numpy data.

For a similar reason, you may only use Boost.Python's ``extract`` facility on
non-references and non-pointers of :ctype:`numpy_vector` type::

  boost::python::object o;
  
  // (1) WRONG, will throw exception
  numpy_vector<double> &u1 = boost::python::extract<numpy_vector<double> &>(o);
  
  // (2) WRONG, will throw exception
  const numpy_vector<double> &u1 = boost::python::extract<const numpy_vector<double> &>(o);
  
  // (3) OK
  numpy_vector<double> u1 = boost::python::extract<numpy_vector<double> >(o);

Since there is no underlying :ctype:`numpy_vector` instance, no reference can be
extracted that would outlive the `extract<>()` constructor call. Therefore (1)
and (2) are invalid. Because of the reference semantics, (3) is probably what
you want anyway.

.. note:: 

    :ctype:`numpy_vector` also has a constructor accepting a
    ``boost::python::handle<>`` (which is a thin wrapper around a ``PyObject *``).
    This may also be used to conveniently construct a :ctype:`numpy_vector` from a
    known Python instance.


.. _slices:

What about slices?
------------------

Numpy arrays can (and often do) represent slices of bigger arrays.
PyUblas deals with these arrays just fine, but if your slices are
non-contiguous, a few issues arise.

What are non-contiguous slices?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For a 1D array, the only way to get a non-contiguous slice is
to specify a stride::

    >>> import numpy
    >>> a = numpy.arange(10)
    >>> a
    array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    >>> a[::2]
    array([0, 2, 4, 6, 8])

For a 2D array, there are more fun ways of getting non-contiguous
data::

    >>> b = a.reshape((3,3))
    >>> b
    array([[0, 1, 2],
           [3, 4, 5],
           [6, 7, 8]])
    >>> b[1:] # contiguous
    array([[3, 4, 5],
           [6, 7, 8]])
    >>> b[:,1:] # not contiguous
    array([[1, 2],
           [4, 5],
           [7, 8]])

Same concept, but different appearance for Fortran ordering::

    >>> c = a.reshape((3,3), order="F")
    >>> c
    array([[0, 3, 6],
           [1, 4, 7],
           [2, 5, 8]])
    >>> c[1:] # not contiguous
    array([[1, 4, 7],
           [2, 5, 8]])
    >>> c[:,1:] # contiguous
    array([[3, 6],
           [4, 7],
           [5, 8]])

.. note::

  Negative strides are supported as of PyUblas 0.93.

What happens to non-contiguous slices?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Since PyUblas directly exposes NumPy's internal data storage area via
:ctype:`numpy_vector`, the in-between elements that are omitted from
the slice suddenly show up again. This could be prevented, at the
cost of forcing the use of strided iterators. I believe that this
would add an unreasonable performance penalty to the average use
case. Therefore, this is not the default behavior.

There are four ways of dealing with this situation:

* Receive an argument of type :ctype:`numpy_strided_vector` instead of
  :ctype:`numpy_vector` from Python. The smallest stride will then 
  automatically be respected (and you will incur the strided-iterator
  speed penalty).

* By invoking the :cfunc:`numpy_vector::as_strided` member function,
  you can obtain a view of the vector that takes the numpy array's
  smallest stride into account, making it *seem* contiguous.

* You can access the array exclusively through the 
  :cfunc:`numpy_vector::sub` family of member functions.
  These take the striding into account, too.

* You can obtain stride information by calling 
  :cfunc:`numpy_vector::strides` and do the striding manually.

The PyUblas test suite explores many of these corner cases that
arise here. You're welcome to take a look.

.. warning::

    Ublas, the C++ side of PyUblas, only provides single-dimensional strides. 
    Multi-dimensional :mod:`numpy` slices (such as ``zeros((5,5))[:3,:3]``)
    can easily become too complex to be represented using 1D slices.
    In this case, the first two ways mentioned above will fail with a 
    :exc:`ValueError`.

Does :ctype:`numpy_matrix` support non-contiguous arrays?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

No.

.. _nullconversion:

Can I pass *None* for an argument that gets converted to :ctype:`numpy_vector`?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

No. But you can wrap the type in :ctype:`invalid_ok`, which enables this
behavior. In this case, the resulting vector will be invalid. See
:cfunc:`numpy_vector::is_valid` and :cfunc:`numpy_matrix::is_valid`

Troubleshooting
---------------

See :ref:`faq-overload-failure`

