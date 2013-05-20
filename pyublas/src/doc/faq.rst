Frequently Asked Questions
==========================

Where do the headers get installed?
-----------------------------------

Its easiest to find this location using this Python code snippet::

    from imp import find_module
    file, pathname, descr = find_module("pyublas")
    from os.path import join
    return join(pathname, "..", "include")

Since PyUblas requires numpy, you'll need a similar snippet for that, too::

    from imp import find_module
    file, pathname, descr = find_module("numpy")
    from os.path import join
    return join(pathname, "core", "include")

What about 0-dimensional arrays?
--------------------------------

0-dimensional arrays are supported by PyUblas, they can be converted
to :ctype:`numpy_vector` instances of length 1.

.. _faq-overload-failure:

My wrapped function is not found by Boost.Python. Help!
-------------------------------------------------------

Even if you heed all the advice in :ref:`frompython`, Boost.Python
will sometimes still complain that no valid overload can be found.
Common reasons include:

* You expect to be handling a float array, but you actually got an int (or
  different dtype) array. To be consistent, PyUblas will not copy-and-cast in
  this situation. If necessary, use :func:`numpy.asarray` in your Python code.
  (I'm debating whether to add C++ syntax to say "copying is ok here". If you
  have input, let me know.)

* If converting a matrix, the row/column-major setting may not match what Ublas
  is expecting. By default, :ctype:`numpy_matrix` is row-major, as is :class:`numpy.array`
  when creating a 2D array.

The function :func:`pyublas.why_not` can help you debug these cases.

.. _speed-faq:

Gaah! Why is this garbage so slow?
----------------------------------

Ok, let's substantiate this discussion somehwat. I've obtained
the following numbers using :file:`test/strided_speed.py` on my
1.7GHz Pentium M. You can run that file yourself for comparison.

PyUblas with Boost.Ublas in its default configuration obtains the
following speeds::

    test_ublas_speed: 0.023819s
    test_unstrided_speed: 0.152608s
    test_strided_speed: 0.234522s

All these tests measure a certain number of large in-place
vector-scalar multiplications. The assumption is that performance
for most other vector-vector operations will be very similar.

``test_ublas_speed`` measures the performance of that operation for
:ctype:`boost::numeric::ublas::vector`, ``test_unstrided_speed`` for
:ctype:`numpy_vector`, and ``test_strided_speed`` for 
:ctype:`numpy_strided_vector`. Now, you'll say, that's scary, because, 
unlike what's promised :ref:`here <indexing-speed>`, the unstrided
:ctype:`numpy_vector` actually is about an order of magnitude slower
than native Ublas. This is due to the fact that Ublas uses indexed
access for dense vector/matrix operations by default.

This default can be changed, however, by defining
:cmacro:`BOOST_UBLAS_USE_ITERATING`, in which case the timings
are pretty much as promised::

    test_ublas_speed: 0.031008s
    test_unstrided_speed: 0.034083s
    test_strided_speed: 0.205794s

If you configure PyUblas with :option:`--use-iterators`, it will
define :cmacro:`BOOST_UBLAS_USE_ITERATING` while it is being compiled.
Note however that you still need to define this macro when compiling
your own code.

.. note:: 

    Unfortunately, Boost 1.35 shipped with code that breaks when
    :cmacro:`BOOST_UBLAS_USE_ITERATING` is defined. I have submitted a `patch
    <http://lists.boost.org/MailArchives/ublas/2008/07/2872.php>`_ to the Ublas
    folks to fix this.
 
The final question is, then, why ``test_strided_speed`` is still about
an order of magnitude slower than the other two. The answer is that
Ublas will always use indexing access in
:ctype:`boost::numeric::vector_slice`, from which
:ctype:`numpy_strided_vector` is derived.

User-visible Changes
====================

PyUblas 0.93
------------

* Negative strides are supported. Slice handling was cleaned up and should be 
  correct now.

* :class:`invalid_ok` was added.

* :class:`numpy_strided_vector` was added as another way of transparently dealing
  with non-contiguous slices.

* :cfunc:`numpy_vector::min_stride` is gone. It was ill-specified and not capable
  of doing what it promised to do.

* :mod:`numpy` forces every C/C++ module that uses its functionality to call
  :cfunc:`import_array`. PyUblas has a clever mechanism that does this for you.
  This mechanism was not correct previously, it would often fail when a 
  particular piece of code was not inlined.

Acknowledgements
================

PyUblas lives through contributions from users like you! The following
people have been kind enough to contribute their changes back to PyUblas:

* Neal Becker provided lots of feedback and a good bit of code.
* Bryan Silverthorn added :ctype:`strided_vector` to-Python conversion.
* Joshua Napoli made PyUblas compatible with MSVC and made PyUblas buildable
  with Boost.Build.

Thanks to all of you! (Any omission here? If so, please let me know.)

Licensing
=========

PyUblas is licensed to you under the MIT/X Consortium license:

Copyright (c) 2008 Andreas Klöckner

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

