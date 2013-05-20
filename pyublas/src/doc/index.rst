Welcome to PyUblas's documentation!
===================================

PyUblas solves one main difficulty of developing hybrid numerical codes in
Python and C++: It integrates two major linear algebra libraries across the two
languages, namely `numpy <http://www.numpy.org>`_ and `Boost.Ublas
<http://www.boost.org/doc/libs/1_35_0/libs/numeric/ublas/doc/index.htm>`_. In Python, 
you are working with native numpy arrays, whereas in C++, PyUblas lets you work
with matrix and vector types immediately derived from and closely integrated with
Ublas. And best of all: There's no copying at the language boundary.

PyUblas is built using and meant to be used with 
`Boost Python <http://www.boost.org/doc/libs/1_35_0/libs/python>`_.

PyUblas also has its own `web page <http://mathema.tician.de/software/pyublas>`_.

Show me! I need examples!
-------------------------

Ok, here's a simple sample extension:

.. code-block:: c++

    #include <pyublas/numpy.hpp>

    pyublas::numpy_vector<double> doublify(pyublas::numpy_vector<double> x)
    {
      return 2*x;
    }

    BOOST_PYTHON_MODULE(sample_ext)
    {
      boost::python::def("doublify", doublify);
    }

and some Python that uses it::

    import numpy
    import sample_ext
    import pyublas # not explicitly used--but makes converters available

    vec = numpy.ones((5,), dtype=float)
    print vec
    print sample_ext.doublify(vec)

and this is what gets printed::

    [ 1.  1.  1.  1.  1.]
    [ 2.  2.  2.  2.  2.]

Table of Contents
-----------------

.. toctree::
    :maxdepth: 2

    installing
    cpptypes
    converters
    wrapping
    pymodule
    faq

* :ref:`genindex`

.. TODO write about slice handling on conversion

