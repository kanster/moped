//
// Copyright (c) 2004-2006
// Andreas Kloeckner
//
// Permission to use, copy, modify, distribute and sell this software
// and its documentation for any purpose is hereby granted without fee,
// provided that the above copyright notice appear in all copies and
// that both that copyright notice and this permission notice appear
// in supporting documentation.  The authors make no representations
// about the suitability of this software for any purpose.
// It is provided "as is" without express or implied warranty.
//




#ifndef HEADER_SEEN_HELPERS_HPP
#define HEADER_SEEN_HELPERS_HPP



#include <complex>
#include <pyublas/generic_ublas.hpp>
#include <boost/numeric/ublas/vector.hpp>




namespace helpers {
namespace ublas = boost::numeric::ublas;




template <class Function>
struct reverse_binary_function : 
public std::binary_function<typename Function::second_argument_type,
typename Function::first_argument_type, typename Function::result_type>
{
  typename Function::result_type
  operator()(
      const typename Function::second_argument_type &a2, 
      const typename Function::first_argument_type &a1) const
  {
    return Function()(a1, a2);
  }
};




// decomplexify ---------------------------------------------------------------
template <typename T>
struct decomplexify
{
  typedef T type;
};

template <typename ELT>
struct decomplexify<std::complex<ELT> >
{
  typedef ELT type;
};




// complexify -----------------------------------------------------------------
template <typename T>
struct complexify
{
  typedef std::complex<T> type;
};

template <typename ELT>
struct complexify<std::complex<ELT> >
{
  typedef std::complex<ELT> type;
};




// isComplex ------------------------------------------------------------------
template <typename T>
inline bool isComplex(const T &)
{
  return false;
}




template <typename T2>
inline bool isComplex(const std::complex<T2> &)
{
  return true;
}




// conjugate ------------------------------------------------------------------
template <typename T>
inline T conjugate(const T &x)
{
  return x;
}




template <typename T2>
inline std::complex<T2> conjugate(const std::complex<T2> &x)
{
  return conj(x);
}




// conjugate_if ---------------------------------------------------------------
template <typename T>
inline T conjugate_if(bool do_it, const T &x)
{
  return x;
}




template <typename T2>
inline std::complex<T2> conjugate_if(bool do_it, const std::complex<T2> &x)
{
  return do_it ? conj(x) : x;
}




// absolute_value -------------------------------------------------------------
template <typename T>
inline T absolute_value(const T &x)
{
  return fabs(x);
}




template <typename T2>
inline T2 absolute_value(const std::complex<T2> &x)
{
  return sqrt(norm(x));
}




// absolute_value_squared -----------------------------------------------------
template <typename T>
inline T absolute_value_squared(const T &x)
{
  return fabs(x)*fabs(x);
}




template <typename T2>
inline T2 absolute_value_squared(const std::complex<T2> &x)
{
  return norm(x);
}




// end namespaces -------------------------------------------------------------
}




#endif

