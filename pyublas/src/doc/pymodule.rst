The :mod:`pyublas` Python module
================================

.. module:: pyublas

The main purpose of the :mod:`pyublas` module is to make the automatic to-
and from-Python converters available upon being imported.

Debugging Tools
---------------

.. function:: why_not(val, [dtype=float, [matrix=False, [row_major=True]]])

    Issue a warning if the array *val* will not successfully convert to a
    :ctype:`numpy_vector` or :ctype:`numpy_matrix`.  Return ``val`` unchanged.

    When debugging an overload failure, simply insert a call to this function in
    the argument list of the failing call::

        do_stuff(pyublas.why_not(myarray))

.. function:: set_trace(enable)

    If set to true, prints diagnostic messages upon each failed vector
    conversion explaining what went wrong. (Note that each argument
    may go through a number of failed conversions before the correct
    one is found.)

Sparse Matrix wrappers
----------------------

**CAUTION:** In PyUblas version 0.91 and later, the sparse wrappers are an
optional feature that has to be enabled at `configure` time with the option
:option:`--with-sparse-wrappers`. You may check for their presence using this
function:

.. function:: has_sparse_wrappers()

    Return a bool indicating whether PyUblas was compiled with sparse matrix wrappers.

In addition to :ctype:`numpy_vector` and :ctype:`numpy_matrix`, PyUblas also wraps Ublas's
powerful sparse matrix machinery into Python objects. The interface to these
functions is somewhat like `numpy`'s own and is found in the `pyublas` name
space.

Here's a brief demo::

    import numpy
    import pyublas

    a = pyublas.zeros((5,5), flavor=pyublas.SparseBuildMatrix, dtype=float)

    a[4,2] = 19

    b = numpy.random.randn(2,2)
    a.add_block(2, 2, b)

    a_fast = pyublas.asarray(a, flavor=pyublas.SparseExecuteMatrix)

    vec = numpy.random.randn(5)

    res = a_fast * vec

    print a_fast
    print res

This prints something like::

    sparse({2: {2: 0.774217588463, 3: -1.5320702452},
            3: {2: 0.118048365647, 3: 1.05028340411},
            4: {2: 19.0}},
           shape=(5, 5), flavor=SparseExecuteMatrix)
    [ 0.          0.         -0.60793048  0.13384055 -8.28513612]

The `SparseBuildMatrix` flavor is designed for fastest possible assembly of
sparse matrices, while the `SparseExecuteMatrix` flavor is made for the fastest
possible matrix-vector product. There's much more functionality here--don't be
afraid to peek into the source code.

