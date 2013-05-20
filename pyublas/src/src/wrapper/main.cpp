//
// Copyright (c) 2008 Andreas Kloeckner
//
// Permission to use, copy, modify, distribute and sell this software
// and its documentation for any purpose is hereby granted without fee,
// provided that the above copyright notice appear in all copies and
// that both that copyright notice and this permission notice appear
// in supporting documentation.  The authors make no representations
// about the suitability of this software for any purpose.
// It is provided "as is" without express or implied warranty.
//




#include <boost/python.hpp>




void pyublas_expose_converters();
void pyublas_expose_sparse_build();
void pyublas_expose_sparse_execute();




namespace
{
  bool has_sparse_wrappers()
  {
#ifdef HAVE_SPARSE_WRAPPERS
    return true;
#else
    return false;
#endif
  }
}




BOOST_PYTHON_MODULE(_internal)
{
  pyublas_expose_converters();
#ifdef HAVE_SPARSE_WRAPPERS
  pyublas_expose_sparse_build();
  pyublas_expose_sparse_execute();
#endif

  boost::python::def("has_sparse_wrappers", has_sparse_wrappers);
}
