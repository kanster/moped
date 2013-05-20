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




#include "array.hpp"




template <typename ValueType>
static void exposeAll(ValueType, const std::string &python_eltypename)
{
  expose_matrix_type(ublas::coordinate_matrix<
      ValueType, ublas::column_major>(), 
      "SparseBuildMatrix", python_eltypename);
}




void pyublas_expose_sparse_build()
{
  EXPOSE_ALL_TYPES;
}
