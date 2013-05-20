.. highlight:: c++

The C++ Vector and Matrix Types
===============================

PyUblas defines Ublas-compatible C++ types, namely
:ctype:`numpy_vector` and :ctype:`numpy_matrix`. These are defined in
:file:`pyublas/numpy.hpp`.

Introduction
------------

:ctype:`numpy_vector` is derived from Ublas's ``vector`` and may be used in the
same places as the latter, in the same ways.  There is only one difference:

  :ctype:`numpy_vector` will not copy your data unless you ask it to.

:ctype:`numpy_vector` internally uses Numpy arrays to store data.  Unlike the
native Ublas ``vector<T>``, :ctype:`numpy_vector` has reference or "handle"
semantics: Copy-constructing and assigning only makes a new reference to the
existing vector's data. Unlike native Ublas, it will not make a copy. Consider
this example::

  numpy_vector<double> a(5);
  std::fill(a.begin(), a.end(), 17);
    
  numpy_vector<double> b(a); // only references a's data
  b[2] = 0;

  std::cout << a[2] << std::endl; // prints "0".

If you *want* the data to be copied, you must say so explicitly::

  numpy_vector<double> c(a.size());
  c.assign(a); // actually copies a's data
  c[2] = 17;

  std::cout << a[2] << std::endl; // still prints "0".
  std::cout << c[2] << std::endl; // prints "17".

If you follow these simple rules, it's easy to write code that works the same
for both :ctype:`ublas::vector` and :ctype:`numpy_vector`. (Note that here, as
in the rest of this documentation, the same discussion applies without change
to :ctype:`numpy_matrix` as well.) 

.. warning::

  The `assign` method does not resize its target to the size of its operand--this is something you have to do by hand if you use the above recipe.

.. _indexing-speed:

You should be aware of another difference: Indexed access to
:ctype:`numpy_vector` is much slower than iterator access. Iterators achieve
the same speed as "regular" Ublas, while indexed access adds some extra
instructions to find the real start of the array in the presence of negative
slices. As is true of much of the rest of C++: 

.. note:: 

  Use iterators whenever possible.

Also see :ref:`speed-faq`

Reference Documentation
-----------------------

``#include <pyublas/numpy.hpp>``

.. ctype:: numpy_array
  
    ``template <class ValueType>``, in namespace ``pyublas``.

    Only members that are not already part of the 
    `Boost.Ublas "Storage" Concept <http://www.boost.org/doc/libs/1_35_0/libs/numeric/ublas/doc/storage_concept.htm>`_
    are shown.

    Public type definitions::

      typedef std::size_t size_type;
      typedef std::ptrdiff_t difference_type;
      typedef ValueType value_type;
      typedef const ValueType &const_reference;
      typedef ValueType &reference;
      typedef const ValueType *const_pointer;
      typedef ValueType *pointer;

    .. cfunction:: constructor numpy_array()
                   explicit_constructor numpy_array(size_type n)
                   constructor numpy_array(size_type n, const value_type &v)
                   constructor numpy_array(int ndim, const npy_intp *dims)
                   constructor numpy_array(const boost::python::handle<> &obj)
        
        Construct a new :ctype:`numpy_array`. If you use the
        empty constructor, the array is in an invalid state until
        :cfunc:`numpy_array::resize` is called. Calling any other
        member function will result in undefined behavior.

    .. cfunction:: size_type numpy_array::ndim()

        A ``const`` member function.
        
    .. cfunction:: const npy_intp *numpy_array::dims()

        A ``const`` member function.

    .. cfunction:: const npy_intp *numpy_array::strides()

        A ``const`` member function.

    .. cfunction:: npy_intp numpy_array::itemsize()

        A ``const`` member function.

    .. cfunction:: bool numpy_array::writable()

        A ``const`` member function.

    .. cfunction:: void numpy_array::reshape(int ndim, const npy_intp *dims, NPY_ORDER order=NPY_CORDER)

    .. cfunction:: value_type *numpy_array::data()

    .. cfunction:: const value_type *numpy_array::data()

        A ``const`` member function.

    .. cfunction:: boost::python::handle<> numpy_array::handle() 

        Return a :ctype:`handle` to the underlying Numpy array
        object. If the array is unitialized, the function may
        return a handle to *None*.

        A ``const`` member function.

.. ctype:: numpy_vector

    ``template <class ValueType>``, in namespace ``pyublas``.

    .. cfunction:: constructor numpy_vector()
                   constructor numpy_vector(const numpy_array<ValueType> &s)
                   constructor numpy_vector(int ndim, const npy_intp *dims)
                   explicit_constructor numpy_vector(typename super::size_type size)
                   constructor numpy_vector(size_type size, const value_type &init)
                   constructor numpy_vector(const numpy_vector &v)
                   constructor numpy_vector(const boost::numeric::ublas::vector_expression<AE> &ae)
                   constructor numpy_vector(int ndim, const npy_intp *dims, const boost::numeric::ublas::vector_expression<AE> &ae)

        Construct a new :ctype:`numpy_vector` instance.

        The ``(ndim, dims)`` constructor form can be used to specify
        the Python-side shape of the array at construction time.
        This is extended by the ``(ndim, dims, ae)`` form, which allows
        to specify the vector expression from which the vector is
        initialized, along with its Python-side shape. Both the
        initializer and the Python-side shape are assumed to yield
        identical vector sizes.

        Observe that PyObject handles are implicitly convertible
        to :ctype:`numpy_array`, so that you can invoke the 
        constructor simply by feeding it a ``boost::python::handle``.

        If you use the empty constructor, the vector is in an invalid
        state until :cfunc:`numpy_vector::resize` is called. In this state,
        calling :cfunc:`numpy_vector::is_valid()`, :cfunc:`numpy_vector::size()`
        and :cfunc:`numpy_vector::resize()` is allowed. Calling any other 
        member function results in undefined behavior.

    .. cfunction:: size_type numpy_vector::ndim()

        Return the number of dimensions of this array.

        A ``const`` member function.
        
    .. cfunction:: const npy_intp *numpy_vector::dims()

        Return an array of :cfunc:`numpy_vector::ndim` entries,
        each of which is the size of the array along one dimension. 
        in *elements*. 

        A ``const`` member function.

    .. cfunction:: const npy_intp *numpy_vector::strides()

        Return an array of :cfunc:`numpy_vector::ndim` entries,
        each of which is the stride along one dimension, in 
        *bytes*. Divide by :cfunc:`numpy_vector::itemsize` 
        to convert this to element-wise strides.

        A ``const`` member function.

    .. cfunction:: npy_intp numpy_vector::itemsize()
        
        Return the size (in bytes) of each element of the array.

        A ``const`` member function.
    .. cfunction:: bool numpy_vector::writable()

        A ``const`` member function.

    .. cfunction:: ValueType &numpy_vector::sub(npy_intp i) 
                   ValueType &numpy_vector::sub(npy_intp i, npy_intp j) 
                   ValueType &numpy_vector::sub(npy_intp i, npy_intp j, npy_intp k) 
                   ValueType &numpy_vector::sub(npy_intp i, npy_intp j, npy_intp k, npy_intp l) 

        Return the element at the index (i), (i,j), (i,j,k),
        (i,j,k,l). It is up to you to ensure that the array
        has the same number of dimensions, otherwise the results
        are undefined.

        Also available as ``const`` member functions.

    .. cfunction:: void numpy_vector::reshape(int ndim, const npy_intp *dims, NPY_ORDER order=NPY_CORDER)
        
        Same operation as :func:`numpy.reshape`.

    .. cfunction:: boost::numeric::ublas::vector_slice<numpy_vector> numpy_vector::as_strided()
        
        Return a view of the array that seems contiguous, by 
        only looking at every :cfunc:`numpy_vector::min_stride`'th 
        element.

        Also available as a ``const`` member function.

    .. cfunction:: boost::vector<ValueType> &numpy_vector::as_ublas() 

        Downcast this instance to the underlying 
        ``boost::numeric::ublas::vector<ValueType>``.

        Also available as a ``const`` member function.

    .. cfunction:: boost::python::handle<> numpy_vector::to_python()

        Return a Boost.Python ``handle`` (which is essentially an
        auto-refcounting ``PyObject *``) to the underlying Numpy
        array.  If the matrix is empty, the function may return a 
        handle to *None*.

        A ``const`` member function.

.. ctype:: numpy_strided_vector

    ``template <class ValueType>``, in namespace ``pyublas``.

    If you use this type as a argument type in a function called from Python,
    the converted vector will respect non-contiguous slices automatically.
    See :ref:`slices`

    Inherits from :ctype:`boost::numeric::ublas::vector_slice`.

    .. warning::

        Ublas only provides single-dimensional strides. 
        Multi-dimensional :mod:`numpy` slices (such as ``zeros((5,5))[:3,:3]``)
        can easily become too complex to be represented using these slices.
        In this case, the from-Python conversion fails with a :exc:`ValueError`.

    .. cfunction:: constructor numpy_strided_vector(const numpy_array<ValueType> &s)
                   constructor numpy_strided_vector(const numpy_strided_vector &v)
                   constructor numpy_strided_vector(numpy_vector<ValueType> &v, boost::numeric::ublas::slice const &s)
                   constructor numpy_strided_vector(const boost::numeric::ublas::vector_expression<AE> &ae)

        Observe that PyObject handles are implicitly convertible
        to :ctype:`numpy_array`, so that you can invoke the 
        constructor simply by feeding it a ``boost::python::handle``.

    .. cfunction:: size_type numpy_strided_vector::ndim()

        Return the number of dimensions of this array.

        A ``const`` member function.
        
    .. cfunction:: const npy_intp *numpy_strided_vector::dims()

        Return an array of :cfunc:`numpy_strided_vector::ndim` entries,
        each of which is the size of the array along one dimension. 
        in *elements*. 

        A ``const`` member function.

    .. cfunction:: const npy_intp *numpy_strided_vector::strides()

        Return an array of :cfunc:`numpy_strided_vector::ndim` entries,
        each of which is the stride along one dimension, in 
        *bytes*. Divide by :cfunc:`numpy_strided_vector::itemsize` 
        to convert this to element-wise strides.

        A ``const`` member function.

    .. cfunction:: npy_intp numpy_strided_vector::min_stride()

        The smallest stride used in the underlying array, in bytes.
        Divide by :cfunc:`numpy_strided_vector::itemsize` to convert this to
        element-wise strides.

        A ``const`` member function.

    .. cfunction:: npy_intp numpy_strided_vector::itemsize()
        
        Return the size (in bytes) of each element of the array.

        A ``const`` member function.

    .. cfunction:: bool numpy_strided_vector::writable()

        A ``const`` member function.

    .. cfunction:: ValueType &numpy_strided_vector::sub(npy_intp i) 
                   ValueType &numpy_strided_vector::sub(npy_intp i, npy_intp j) 
                   ValueType &numpy_strided_vector::sub(npy_intp i, npy_intp j, npy_intp k) 
                   ValueType &numpy_strided_vector::sub(npy_intp i, npy_intp j, npy_intp k, npy_intp l) 

        Return the element at the index (i), (i,j), (i,j,k),
        (i,j,k,l). It is up to you to ensure that the array
        has the same number of dimensions, otherwise the results
        are undefined.

        Also available as ``const`` member functions.

    .. cfunction:: boost::numeric::ublas::vector_slice<numpy_vector<ValueType> > &numpy_vector::as_ublas() 

        Downcast this instance to the underlying 
        ``boost::numeric::ublas::vector<ValueType>``.

        Also available as a ``const`` member function.

    .. cfunction:: boost::python::handle<> numpy_vector::to_python()

        Return a Boost.Python ``handle`` (which is essentially an
        auto-refcounting ``PyObject *``) to the underlying Numpy
        array.  If the matrix is empty, the function may return a 
        handle to *None*.

        A ``const`` member function.


.. ctype:: numpy_matrix

    ``template <class ValueType, class Orientation=boost::numeric::ublas::row_major>``, 
    in namespace ``pyublas``.

    .. cfunction:: constructor numpy_matrix()
                   constructor numpy_matrix(size_type size1, size_type size2)
                   constructor numpy_matrix(size_type size1, size_type size2, const value_type &init)
                   constructor numpy_matrix(size_type size1, size_type size2, const array_type &data)
                   constructor numpy_matrix(const typename super::array_type &data)
                   constructor numpy_matrix(const numpy_matrix &m)
                   constructor numpy_matrix(const boost::numeric::ublas::matrix_expression<AE> &ae)

        Observe that PyObject handles are implicitly convertible
        to :ctype:`numpy_array`, so that you can invoke the 
        constructor simply by feeding it a ``boost::python::handle``.

        If you use the empty constructor, the matrix is in an invalid
        state until :cfunc:`numpy_matrix::resize` is called. Calling any
        other member function will result in undefined behavior.

    .. cfunction:: boost::matrix<ValueType, Orientation> &numpy_matrix::as_ublas() 

        Also available as a ``const`` member function.

    .. cfunction:: boost::python::handle<> numpy_matrix::to_python()

        Return a :ctype:`handle` to the underlying Numpy array
        object. If the matrix is empty, the function may
        return a handle to *None*.

        A ``const`` member function.

.. ctype:: invalid_ok

    ``template <class Contained>``, in namespace ``pyublas``.

    *Contained* can be :ctype:`numpy_vector` or :ctype:`numpy_matrix`.
    If arguments of this type are converted from Python, they will also accept
    the value *None*. In that case, the resulting *Contained* will be invalid if
    *None* is passed in. See :ref:`nullconversion`

    .. cfunction:: Contained &invalid_ok::operator*()

        Return a reference to the *Contained* array.

    .. cfunction:: Contained *invalid_ok::operator->()

        Return a pointer to the *Contained* array.

Interacting with Boost.Bindings
-------------------------------

PyUblas contains special code to support interacting with the `Boost.Bindings
<http://mathema.tician.de/software/boost-bindings>`_ library.

If you want to activate this support, define the macro 
:cmacro:`PYUBLAS_HAVE_BOOST_BINDINGS` before including :file:`pyublas/numpy.hpp`.

Boost.Bindings works seamlessly with :ctype:`numpy_vector`. For 
:ctype:`numpy_matrix`, you need to explicitly downcast it to the
:ctype:`ublas::matrix` type. You may do so by simply calling the
:cfunc:`as_ublas` method.
