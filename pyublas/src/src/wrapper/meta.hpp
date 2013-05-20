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




#ifndef HEADER_SEEN_META_HPP
#define HEADER_SEEN_META_HPP




#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>

// #include <helpers.hpp>
#include <pyublas/generic_ublas.hpp>




using namespace boost;
namespace ublas = boost::numeric::ublas;




using pyublas::is_vector;




// change_value_type
template <typename MatrixType, typename NewValueType>
struct change_value_type { 
};

template <typename OLD, typename NEW>
struct change_value_type<ublas::unbounded_array<OLD>, NEW>
{ typedef ublas::unbounded_array<NEW> type; };

template <typename OldValueType, typename NewValueType>
struct change_value_type<ublas::matrix<OldValueType>, NewValueType>
{ typedef ublas::matrix<NewValueType> type; };

template <typename OLD, typename NEW, typename F, std::size_t IB, typename IA>
struct change_value_type<ublas::coordinate_matrix<OLD, F, IB, IA>, NEW>
{ typedef ublas::coordinate_matrix<NEW, F, IB, IA> type; };

template <typename OLD, typename NEW, typename F, std::size_t IB, typename IA>
struct change_value_type<ublas::compressed_matrix<OLD, F, IB, IA>, NEW>
{ typedef ublas::compressed_matrix<NEW, F, IB, IA> type; };

template <typename OldValueType, typename NewValueType>
struct change_value_type<ublas::vector<OldValueType>, NewValueType>
{ typedef ublas::vector<NewValueType> type; };




// generic instantiation infrastructure ---------------------------------------
template <typename Exposer, typename ValueType>
static void exposeForAllSimpleTypes(const std::string &python_eltname, const Exposer &exposer, ValueType)
{
  exposer.expose("Matrix" + python_eltname, ublas::matrix<ValueType>());
  exposer.expose("SparseExecuteMatrix" + python_eltname, 
      ublas::compressed_matrix<ValueType, 
      ublas::column_major, 0, ublas::unbounded_array<int> >());
  exposer.expose("SparseBuildMatrix" + python_eltname, 
      ublas::coordinate_matrix<ValueType, ublas::column_major>());
}




template <typename Exposer, typename T>
static void exposeForAllMatrices(const Exposer &exposer, T)
{
  exposeForAllSimpleTypes("Float64", exposer, T());
}




template <typename Exposer, typename T>
static void exposeForAllMatrices(const Exposer &exposer, std::complex<T>)
{
  exposeForAllSimpleTypes("Complex128", exposer, std::complex<T>());
}




template <typename Exposer>
static void exposeForAllMatrices(const Exposer &exposer)
{
  exposeForAllMatrices(exposer, double());
  exposeForAllMatrices(exposer, std::complex<double>());
}




template <typename Exposer,typename T>
static void exposeForMatricesConvertibleTo(const Exposer &exposer, T)
{
  exposeForAllMatrices(exposer, T());
}




template <typename Exposer,typename T>
static void exposeForMatricesConvertibleTo(const Exposer &exposer, std::complex<T>)
{
  exposeForAllMatrices(exposer);
}




#endif
