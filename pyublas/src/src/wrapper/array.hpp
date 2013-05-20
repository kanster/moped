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



#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4018 ) // signed/unsigned mismatch
#endif

#include <complex>
#include <string>
#include <cmath>
#include <functional>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include <pyublas/python_helpers.hpp>
#include <pyublas/numpy.hpp>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/triangular.hpp>

#include "meta.hpp"
#include "helpers.hpp"




using boost::python::class_;
using boost::python::handle;
using boost::python::borrowed;
using boost::python::enum_;
using boost::python::self;
using boost::python::def;
using helpers::decomplexify;




namespace {
// op wrappers ----------------------------------------------------------------
template <class Array, class Operator>
Array wrapUnaryOp(const Array &op)
{
  return Operator()(op);
}




template <class Array, class Operator>
Array wrapBinaryOp(const Array &op1, const Array &op2)
{
  return Operator()(op1, op2);
}




// helpers --------------------------------------------------------------------
template <typename T>
pyublas::minilist<T> getMinilist(const python::object &tup)
{
  unsigned len = python::extract<T>(tup.attr("__len__")());

  pyublas::minilist<T> result;
  for (unsigned i = 0; i < len; ++i)
    result.push_back(python::extract<T>(tup[i]));
  return result;
}




template <typename T>
python::tuple getPythonShapeTuple(const pyublas::minilist<T> &ml)
{
  if (ml.size() == 1)
    return python::make_tuple(ml[0]);
  else
    return python::make_tuple(ml[0], ml[1]);
}




template <typename T>
python::object getPythonIndexTuple(const pyublas::minilist<T> &ml)
{
  if (ml.size() == 1)
    return python::object(ml[0]);
  else
    return python::make_tuple(ml[0], ml[1]);
}




// shape accessors ------------------------------------------------------------
template <typename MatrixType>
inline unsigned getLength(const MatrixType &m)
{
  return m.size1();
}




template <typename MatrixType>
inline python::object getShape(const MatrixType &m)
{
  return getPythonShapeTuple(pyublas::getShape(m));
}




template <typename MatrixType>
inline void setShape(MatrixType &m, const python::tuple &new_shape)
{
  pyublas::setShape(m,getMinilist<typename MatrixType::size_type>(new_shape));
}




// iterator interface ---------------------------------------------------------
template <typename MatrixType>
struct python_matrix_key_iterator
{
  typename pyublas::matrix_iterator<MatrixType> m_iterator, m_end;

  python_matrix_key_iterator *iter()
  {
    return this;
  }

  python::object next()
  {
    if (m_iterator == m_end)
    {
      PyErr_SetNone(PyExc_StopIteration);
      throw python::error_already_set();
    }

    python::object result = getPythonIndexTuple(m_iterator.index());
    ++m_iterator;
    return result;
  }

  static python_matrix_key_iterator *obtain(MatrixType &m)
  {
    std::auto_ptr<python_matrix_key_iterator> it(new python_matrix_key_iterator);
    it->m_iterator = pyublas::begin(m);
    it->m_end = pyublas::end(m);
    return it.release();
  }
};




template <typename MatrixType, typename _is_vector = typename is_vector<MatrixType>::type >
struct python_matrix_value_iterator
{
  const MatrixType                      &m_matrix;
  typename MatrixType::size_type        m_row_index;

  python_matrix_value_iterator(const MatrixType &matrix)
  : m_matrix(matrix), m_row_index(0)
  {
  }

  python_matrix_value_iterator *iter()
  {
    return this;
  }

  handle<> next()
  {
    if (m_row_index >= m_matrix.size1())
    {
      PyErr_SetNone(PyExc_StopIteration);
      throw python::error_already_set();
    }

    return pyublas::handle_from_new_ptr(
        new pyublas::numpy_vector<typename MatrixType::value_type>(
          ublas::row(m_matrix, m_row_index++)));
  }

  static python_matrix_value_iterator *obtain(MatrixType &m)
  {
    return new python_matrix_value_iterator(m);
  }
};




template <typename MatrixType>
struct python_matrix_value_iterator<MatrixType, mpl::true_>
{
  const MatrixType                      &m_matrix;
  typename MatrixType::size_type        m_row_index;

  python_matrix_value_iterator(const MatrixType &matrix)
  : m_matrix(matrix), m_row_index(0)
  {
  }

  python_matrix_value_iterator *iter()
  {
    return this;
  }

  typename MatrixType::value_type next()
  {
    if (m_row_index >= m_matrix.size())
    {
      PyErr_SetNone(PyExc_StopIteration);
      throw python::error_already_set();
    }

    return m_matrix(m_row_index++);
  }

  static python_matrix_value_iterator *obtain(MatrixType &m)
  {
    return new python_matrix_value_iterator(m);
  }
};




// element accessors ----------------------------------------------------------
struct slice_info
{
  bool m_was_slice;
  Py_ssize_t m_start;
  Py_ssize_t m_end;
  Py_ssize_t m_stride;
  Py_ssize_t m_length;
};




void translateIndex(PyObject *slice_or_constant, slice_info &si, int my_length)
{
  si.m_was_slice = PySlice_Check(slice_or_constant);
  if (si.m_was_slice)
  {
    if (PySlice_GetIndicesEx(reinterpret_cast<PySliceObject *>(slice_or_constant),
          my_length, &si.m_start, &si.m_end, &si.m_stride, &si.m_length) != 0)
      throw python::error_already_set();
  }
  else
  {
    bool valid = false;
    long index;
#if PY_VERSION_HEX < 0x03000000
    if (PyInt_Check(slice_or_constant))
    {
      index = PyInt_AS_LONG(slice_or_constant);
      valid = true;
    }
#endif
    if (!valid && PyLong_Check(slice_or_constant))
    {
      index = PyLong_AsLong(slice_or_constant);
      if (index == -1 && PyErr_Occurred())
        throw python::error_already_set();
      valid = true;
    }

    if (!valid)
      throw std::out_of_range("invalid index object");

    if (index < 0)
      index += my_length;
    if (index < 0)
      throw std::out_of_range("negative index out of bounds");
    if (index >= my_length)
      throw std::out_of_range("index out of bounds");
    si.m_start = index;
    si.m_end = index + 1;
    si.m_stride = 1;
    si.m_length = 1;
  }
}




template <typename MatrixType>
handle<> getElement(/*const*/ MatrixType &m, handle<> index)
{
  typedef
    pyublas::numpy_vector<typename MatrixType::value_type>
    vector_t;
  typedef
    typename MatrixType::value_type
    value_t;
  typedef
    ublas::basic_slice<typename MatrixType::size_type> slice_t;


  if (PyTuple_Check(index.get()))
  {
    // we have a tuple
    if (PyTuple_GET_SIZE(index.get()) != 2)
      PYUBLAS_PYERROR(IndexError, "expected tuple of size 2");

    slice_info si1, si2;
    translateIndex(PyTuple_GET_ITEM(index.get(), 0), si1, m.size1());
    translateIndex(PyTuple_GET_ITEM(index.get(), 1), si2, m.size2());

    if (!si1.m_was_slice && !si2.m_was_slice)
      return pyublas::handle_from_rvalue(value_t(m(si1.m_start, si2.m_start)));
    else if (!si1.m_was_slice)
      return pyublas::handle_from_new_ptr(new vector_t(
            ublas::matrix_vector_slice<MatrixType>(m,
              slice_t(si1.m_start, 0,            si2.m_length),
              slice_t(si2.m_start, si2.m_stride, si2.m_length))));
    else if (!si2.m_was_slice)
      return pyublas::handle_from_new_ptr(new vector_t(
                  ublas::matrix_vector_slice<MatrixType>(m,
                      slice_t(si1.m_start, si1.m_stride, si1.m_length),
                      slice_t(si2.m_start, 0,            si1.m_length))));
    else
    {
      return pyublas::handle_from_new_ptr(
          new MatrixType(
            subslice(m,
                si1.m_start, si1.m_stride, si1.m_length,
                si2.m_start, si2.m_stride, si2.m_length)));
    }
  }
  else
  {
    slice_info si;
    translateIndex(index.get(), si, m.size1());

    if (!si.m_was_slice)
      return pyublas::handle_from_new_ptr(new vector_t(row(m, si.m_start)));
    else
      return pyublas::handle_from_new_ptr(
          new MatrixType(
            subslice(m,
              si.m_start, si.m_stride, si.m_length,
              0, 1, m.size2())
              ));
  }
}




template <typename MatrixType>
void setElement(MatrixType &m, handle<> index, python::object &new_value)
{
  typedef
    pyublas::numpy_vector<typename MatrixType::value_type>
    vector_t;
  typedef
      typename MatrixType::value_type m_value_t;
  typedef
      typename MatrixType::size_type m_size_t;
  typedef
      ublas::basic_slice<m_size_t> slice_t;


  python::extract<typename MatrixType::value_type> new_scalar(new_value);
  python::extract<const vector_t &> new_vector(new_value);
  python::extract<const MatrixType &> new_matrix(new_value);

  if (PyTuple_Check(index.get()))
  {
    // we have a tuple
    if (PyTuple_GET_SIZE(index.get()) != 2)
      PYUBLAS_PYERROR(IndexError, "expected tuple of size 2");

    slice_info si1, si2;
    translateIndex(PyTuple_GET_ITEM(index.get(), 0), si1, m.size1());
    translateIndex(PyTuple_GET_ITEM(index.get(), 1), si2, m.size2());

    if (new_scalar.check())
    {
      // scalar broadcast
      subslice(m,
            si1.m_start, si1.m_stride, si1.m_length,
            si2.m_start, si2.m_stride, si2.m_length) =
        ublas::scalar_matrix<m_value_t>(si1.m_length, si2.m_length, new_scalar());
    }
    else if (new_vector.check())
    {
      const vector_t &new_vec(new_vector());
      if (si1.m_length == 1)
      {
        // replace row
        if (new_vec.size() != si2.m_length)
          PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

        ublas::matrix_vector_slice<MatrixType>(m,
            slice_t(si1.m_start, 0,            si2.m_length),
            slice_t(si2.m_start, si2.m_stride, si2.m_length)) = new_vec;
      }
      else if (si2.m_length == 1)
      {
        // replace column
        if (new_vector().size() != si1.m_length)
          PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

        ublas::matrix_vector_slice<MatrixType>(m,
            slice_t(si1.m_start, si1.m_stride, si1.m_length),
            slice_t(si2.m_start, 0,            si1.m_length)) = new_vec;
      }
      else
      {
        // broadcast vector across matrix
        ublas::matrix_slice<MatrixType> my_slice(m,
              slice_t(si1.m_start, si1.m_stride, si1.m_length),
              slice_t(si2.m_start, si2.m_stride, si2.m_length));

        if (new_vec.size() != my_slice.size2())
          PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

        for (m_size_t i = 0; i < my_slice.size1(); ++i)
          row(my_slice, i) = new_vector();
      }
    }
    else if (new_matrix.check())
    {
      // no broadcast
      const MatrixType &new_mat = new_matrix();
      if (int(new_mat.size1()) != si1.m_length ||
          int(new_mat.size2()) != si2.m_length)
        PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

      subslice(m,
        si1.m_start, si1.m_stride, si1.m_length,
        si2.m_start, si2.m_stride, si2.m_length) = new_mat;
    }
    else
      PYUBLAS_PYERROR(ValueError, "unknown type in element or slice assignment");
  }
  else
  {
    slice_info si;
    translateIndex(index.get(), si, m.size1());

    if (new_scalar.check())
      subslice(m,
          si.m_start, si.m_stride, si.m_length,
          0, 1, m.size2()) =
        ublas::scalar_matrix<m_value_t>(si.m_length, m.size2(), new_scalar());
    else if (new_vector.check())
    {
      vector_t new_vec = new_vector();

      if (si.m_length == 1)
      {
        if (new_vec.size() != m.size2())
          PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

        row(m,si.m_start) = new_vec;
      }
      else
      {
        // broadcast vector across matrix
        if (new_vec.size() != m.size2())
          PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

        for (m_size_t i = si.m_start; i < si.m_end; i += si.m_stride)
          row(m, i) = new_vec;
      }
    }
    else if (new_matrix.check())
    {
      const MatrixType &new_mat = new_matrix();

      if (int(new_mat.size1()) != si.m_length ||
          int(new_mat.size2()) != m.size2())
        PYUBLAS_PYERROR(ValueError, "submatrix is wrong size for assignment");

      project(m,
          ublas::basic_slice<typename MatrixType::size_type>(si.m_start, si.m_stride, si.m_length),
          ublas::basic_slice<typename MatrixType::size_type>(0, 1, m.size2())) = new_mat;
    }
    else
      PYUBLAS_PYERROR(ValueError, "unknown type in element or slice assignment");
  }
}




// pickling -------------------------------------------------------------------
template <typename MatrixType>
struct sparse_pickle_suite : python::pickle_suite
{
  static
  python::tuple
  getinitargs(const MatrixType &m)
  {
    return getPythonShapeTuple(pyublas::getShape(m));
  }




  static
  python::object
  getstate(MatrixType &m)
  {
    pyublas::matrix_iterator<MatrixType>
      first = pyublas::begin(m),
      last = pyublas::end(m);

    python::list result;
    while (first != last)
    {
      result.append(python::make_tuple(getPythonIndexTuple(first.index()),
                                       typename MatrixType::value_type(*first)));
      first++;
    }

    return result;
  }




  static
  void
  setstate(MatrixType &m, python::object entries)
  {
    unsigned len = python::extract<unsigned>(entries.attr("__len__")());
    for (unsigned i = 0; i < len; i++)
    {
      pyublas::insert_element(
        m,
        getMinilist<typename MatrixType::size_type>(
          python::extract<python::tuple>(entries[i][0])),
        python::extract<typename MatrixType::value_type>(entries[i][1]));
    }
  }
};




template <typename PythonClass, typename WrappedClass>
void exposePickling(PythonClass &pyclass, WrappedClass)
{
  pyclass.def_pickle(sparse_pickle_suite<WrappedClass>());
}




// universal functions --------------------------------------------------------
template <typename MatrixType>
inline MatrixType *copyNew(const MatrixType &m)
{
  return new MatrixType(m);
}




template <typename MatrixType>
handle<> hermite_matrix(const MatrixType &m)
{
  return pyublas::handle_from_new_ptr(new MatrixType(herm(m)));
}




template <typename MatrixType>
handle<> transpose_matrix(const MatrixType &m)
{
  return pyublas::handle_from_new_ptr(new MatrixType(trans(m)));
}




template <typename MatrixType>
struct realWrapper
{
  typedef
    typename change_value_type<MatrixType,
      typename decomplexify<typename MatrixType::value_type>::type>::type
    result_type;

  inline static handle<> apply(const MatrixType &m)
  {
    return pyublas::handle_from_new_ptr(new result_type(real(m)));
  }
};




template <typename MatrixType>
struct imagWrapper
{
  typedef
    typename change_value_type<MatrixType,
      typename decomplexify<typename MatrixType::value_type>::type>::type
    result_type;

  inline static handle<> apply(const MatrixType &m)
  {
    return pyublas::handle_from_new_ptr(new result_type(imag(m)));
  }
};




template <typename MatrixType>
struct conjugateWrapper
{
  typedef MatrixType result_type;

  inline static result_type *apply(const MatrixType &m)
  {
    return new result_type(conj(m));
  }
};


template<typename MatrixType>
struct add_element_inplace_helper
{
  typedef typename MatrixType::size_type size_type;
  typedef typename MatrixType::value_type value_type;

  void operator()(MatrixType &mat,
    size_type i,
    size_type j,
    value_type x)
  {
    mat(i,j) += x;
  }
};

template<typename V>
struct add_element_inplace_helper<ublas::coordinate_matrix<V, ublas::column_major> >
{
  typedef unsigned int size_type;
  typedef V value_type;

  void operator()(ublas::coordinate_matrix<V, ublas::column_major> &mat,
    size_type i,
    size_type j,
    value_type x)
  {
    mat.append_element(i, j, x);
  }
};




template <typename MatrixType>
inline
void add_element_inplace(MatrixType &mat,
    typename add_element_inplace_helper<MatrixType>::size_type i,
    typename add_element_inplace_helper<MatrixType>::size_type j,
    typename add_element_inplace_helper<MatrixType>::value_type x)
{
  add_element_inplace_helper<MatrixType>()(mat, i, j, x);
}




template <typename MatrixType, typename SmallMatrixType>
void add_block(MatrixType &mat,
    typename MatrixType::size_type start_row,
    typename MatrixType::size_type start_column,
    const SmallMatrixType &small_mat)
{
  typedef typename SmallMatrixType::size_type index_t;

  pyublas::matrix_iterator<const SmallMatrixType>
    first = pyublas::begin(small_mat),
    last = pyublas::end(small_mat);

  while (first != last)
  {
    const pyublas::minilist<index_t> index = first.index();
    add_element_inplace(mat,
        start_row+index[0],
        start_column+index[1],
        *first++);
  }
}




template <typename MatrixType, typename SmallMatrixType>
void add_scattered(MatrixType &mat,
    python::object row_indices_py,
    python::object column_indices_py,
    const SmallMatrixType &small_mat)
{
  using namespace boost::python;

  typedef typename SmallMatrixType::size_type index_t;
  std::vector<index_t> row_indices;
  std::vector<index_t> column_indices;
  copy(
      stl_input_iterator<index_t>(row_indices_py),
      stl_input_iterator<index_t>(),
      back_inserter(row_indices));
  copy(
      stl_input_iterator<index_t>(column_indices_py),
      stl_input_iterator<index_t>(),
      back_inserter(column_indices));

  if (row_indices.size() != small_mat.size1()
      || column_indices.size() != small_mat.size2())
    throw std::runtime_error("sizes don't match");

  pyublas::matrix_iterator<const SmallMatrixType>
    first = pyublas::begin(small_mat),
    last = pyublas::end(small_mat);

  while (first != last)
  {
    const pyublas::minilist<index_t> index = first.index();
    add_element_inplace(mat,
        row_indices[index[0]],
        column_indices[index[1]],
        *first++);
  }
}




template <typename MatrixType, typename SmallMatrixType>
void add_scattered_with_skip(MatrixType &mat,
    python::object row_indices_py,
    python::object column_indices_py,
    const SmallMatrixType &small_mat)
{
  using namespace boost::python;

  typedef typename SmallMatrixType::size_type index_t;
  std::vector<index_t> row_indices;
  std::vector<index_t> column_indices;
  copy(
      stl_input_iterator<index_t>(row_indices_py),
      stl_input_iterator<index_t>(),
      back_inserter(row_indices));
  copy(
      stl_input_iterator<index_t>(column_indices_py),
      stl_input_iterator<index_t>(),
      back_inserter(column_indices));

  if (row_indices.size() != small_mat.size1()
      || column_indices.size() != small_mat.size2())
    throw std::runtime_error("sizes don't match");

  pyublas::matrix_iterator<const SmallMatrixType>
    first = pyublas::begin(small_mat),
    last = pyublas::end(small_mat);

  while (first != last)
  {
    const pyublas::minilist<index_t> index = first.index();
    unsigned dest_row = row_indices[index[0]];
    unsigned dest_col = column_indices[index[1]];
    if (dest_row >= 0 && dest_col >= 0)
      add_element_inplace(mat, dest_row, dest_col, *first);
    ++first;
  }
}




// wrapper for stuff that is common to vectors and matrices -------------------
template <typename MatrixType>
typename MatrixType::value_type
sum(MatrixType &mat)
{
  pyublas::matrix_iterator<MatrixType>
    first = pyublas::begin(mat),
    last = pyublas::end(mat);

  typename MatrixType::value_type result = 0;
  while (first != last)
    result += *first++;
  return result;
}




template <typename MatrixType>
typename helpers::decomplexify<typename MatrixType::value_type>::type
abs_square_sum(MatrixType &mat)
{
  pyublas::matrix_iterator<MatrixType>
    first = pyublas::begin(mat),
    last = pyublas::end(mat);

  typedef
    typename helpers::decomplexify<typename MatrixType::value_type>::type
    real_type;
  real_type result = 0;
  while (first != last)
    result += helpers::absolute_value_squared(*first++);
  return result;
}




template <typename PythonClass, typename WrappedClass>
void exposeElementWiseBehavior(PythonClass &pyclass, WrappedClass)
{
  typedef WrappedClass cl;
  typedef typename cl::value_type value_type;
  pyclass
    .def("copy", copyNew<cl>,
        python::return_value_policy<python::manage_new_object>(),
        "Return an exact copy of the given Array.")
    .def("clear", &cl::clear,
        "Discard Array content and fill with zeros, if necessary.")

    .add_property(
      "shape",
      (python::object (*)(const cl &)) getShape,
      (void (*)(cl &, const python::tuple &)) setShape,
      "Return a shape tuple for the Array.")
    .add_property(
      "__array_shape__",
      (python::object (*)(const cl &)) getShape)
    .def("__len__", (unsigned (*)(const cl &)) getLength,
        "Return the length of the leading dimension of the Array.")
    .def("swap", &cl::swap)

    .def("__getitem__", (handle<> (*)(/*const*/ cl &, handle<>)) getElement)
    .def("__setitem__", (void (*)(cl &, handle<>, python::object &)) setElement)
    ;

  // unary negation
  pyclass
    .def("__neg__", wrapUnaryOp<cl, std::negate<cl> >)
    ;

  // container - container
  pyclass
    .def(self += self)
    .def(self -= self)

    .def("sum", sum<cl>,
        "Return the sum of the Array's entries.")
    .def("abs_square_sum", abs_square_sum<cl>)
    ;

  exposePickling(pyclass, WrappedClass());
}




template <typename PythonClass, typename WrappedClass>
void expose_iterator(PythonClass &pyclass, const std::string &python_typename, WrappedClass)
{
  typedef
    python_matrix_value_iterator<WrappedClass>
    value_iterator;

  typedef
    python_matrix_key_iterator<WrappedClass>
    key_iterator;

  pyclass
    .def("__iter__", &value_iterator::obtain,
        python::return_value_policy<python::manage_new_object,
        python::return_internal_reference<> >())
    .def("indices", &key_iterator::obtain,
        python::return_value_policy<python::manage_new_object,
        python::return_internal_reference<> >(),
        "Return an iterator over all non-zero index pairs of the Array.")
    ;

  class_<key_iterator>
    ((python_typename + "KeyIterator").c_str(), python::no_init)
    .def("next", &key_iterator::next)
    .def("__iter__", &key_iterator::iter,
        python::return_self<>())
    ;

  class_<value_iterator>
    ((python_typename + "ValueIterator").c_str(), python::no_init)
    .def("next", &value_iterator::next)
    .def("__iter__", &value_iterator::iter,
        python::return_self<>())
    ;
}




// matrix wrapper -------------------------------------------------------------
template <typename MatrixType>
handle<> multiply_matrix_base(
    const MatrixType &mat,
    python::object op2,
    bool reverse)
{
  python::extract<MatrixType> op2_mat(op2);
  if (op2_mat.check())
  {
    const MatrixType &mat2 = op2_mat();
    if (mat.size2() != mat2.size1())
      throw std::runtime_error("matrix sizes don't match");
    if (!reverse)
      return pyublas::handle_from_new_ptr(new MatrixType(prod(mat, mat2)));
    else
      return pyublas::handle_from_new_ptr(new MatrixType(prod(mat2, mat)));
  }

  typedef
    pyublas::numpy_vector<typename MatrixType::value_type>
    vector_t;

  python::extract<vector_t> op2_vec(op2);
  if (op2_vec.check())
  {
    const vector_t &vec = op2_vec();
    if (mat.size2() != vec.size())
      throw std::runtime_error("matrix size doesn't match vector");

    vector_t result(mat.size1());

    if (!reverse)
      ublas::axpy_prod(mat, vec, result, /*init*/ true);
    else
      ublas::axpy_prod(vec, mat, result, /*init*/ true);
    return pyublas::handle_from_rvalue(result);
  }

  python::extract<typename MatrixType::value_type> op2_scalar(op2);
  if (op2_scalar.check())
  {
    return pyublas::handle_from_new_ptr(new MatrixType(mat * op2_scalar()));
  }

  return handle<>(borrowed(Py_NotImplemented));
}




template <typename MatrixType>
handle<> multiply_matrix(const MatrixType &mat, python::object op2)
{
  return multiply_matrix_base<MatrixType>(mat, op2, false);
}




template <typename MatrixType>
handle<> rmultiply_matrix(const  MatrixType &mat, python::object op2)
{
  return multiply_matrix_base<MatrixType>(mat, op2, true);
}




template <typename MatrixType>
handle<> multiply_matrix_inplace(python::object op1, python::object op2)
{
  python::extract<MatrixType> op2_mat(op2);
  if (op2_mat.check())
  {
    MatrixType &mat = python::extract<MatrixType &>(op1);
    const MatrixType &mat2 = op2_mat();
    if (mat.size2() != mat2.size1())
      throw std::runtime_error("matrix sizes don't match");

    // FIXME: aliasing!
    mat = prod(mat, mat2);

    return pyublas::handle_from_object(op1);
  }

  python::extract<typename MatrixType::value_type> op2_scalar(op2);
  if (op2_scalar.check())
  {
    MatrixType &mat = python::extract<MatrixType &>(op1);
    mat *= op2_scalar();
    return pyublas::handle_from_object(op1);
  }

  return handle<>(borrowed(Py_NotImplemented));
}




template <typename MatrixType>
void matrixSimplePushBack(MatrixType &m,
                          typename MatrixType::size_type i,
                          typename MatrixType::size_type j,
                          const typename MatrixType::value_type &el)
{
  m(i, j) = el;
}




template <typename MatrixType>
void matrixSimpleAppendElement(MatrixType &m,
                               typename MatrixType::size_type i,
                               typename MatrixType::size_type j,
                               const typename MatrixType::value_type &el)
{
  m(i, j) += el;
}




template <typename MatrixType>
void insertElementWrapper(MatrixType &m,
                   typename MatrixType::size_type i,
                   typename MatrixType::size_type j,
                   const typename MatrixType::value_type &el)
{
  m.insert_element(i, j, el);
}




template <typename WrappedClass, typename SmallMatrix, typename PythonClass>
void expose_add_scattered(PythonClass &pyclass)
{
  using python::arg;

  pyclass
    .def("add_block", add_block<WrappedClass, SmallMatrix>,
        (arg("self"), arg("start_row"), arg("start_column"), arg("small_mat")),
        "Add C{small_mat} to self, starting at C{start_row,start_column}.")
    .def("add_scattered", add_scattered<WrappedClass, SmallMatrix>,
        (arg("self"), arg("row_indices"), arg("column_indices"), arg("small_mat")),
        "Add C{small_mat} at intersections of C{row_indices} and "
        "C{column_indices}.")
    .def("add_scattered_with_skip", add_scattered_with_skip<WrappedClass, SmallMatrix>,
        (arg("self"), arg("row_indices"), arg("column_indices"), arg("small_mat")),
        "Add C{small_mat} at intersections of C{row_indices} and "
        "C{column_indices}. Entries of C{row_indices} or C{column_indices} "
        "may be negative to skip this row or column.")
    ;
}




template <typename PythonClass, typename WrappedClass>
void exposeMatrixConcept(PythonClass &pyclass, WrappedClass)
{
  typedef typename WrappedClass::value_type value_type;

  exposeElementWiseBehavior(pyclass, WrappedClass());

  pyclass
    .add_property("H", hermite_matrix<WrappedClass>,
        "The complex-conjugate transpose of the Array.")
    .add_property("T", transpose_matrix<WrappedClass>,
        "The transpose of the Array.")

    // products
    .def("__mul__", multiply_matrix<WrappedClass>)
    .def("__rmul__", rmultiply_matrix<WrappedClass>)
    .def("__imul__", multiply_matrix_inplace<WrappedClass>)
    ;
}




template <typename PythonClass>
struct matrix_converter_exposer
{
  PythonClass &m_pyclass;

public:
  matrix_converter_exposer(PythonClass &pyclass)
  : m_pyclass(pyclass)
  {
  }

  template <typename MatrixType>
  void expose(const std::string &python_mattype, MatrixType) const
  {
    m_pyclass
      .def(python::init<const MatrixType &>());
  }
};




template <typename PYC, typename MT>
void expose_matrix_specialties(PYC, MT)
{
}




template <typename PYC, typename VT, typename L, std::size_t IB, typename IA, typename TA>
void expose_matrix_specialties(PYC &pyclass, ublas::compressed_matrix<VT, L, IB, IA, TA>)
{
  typedef ublas::compressed_matrix<VT, L, IB, IA, TA> cl;

  pyclass
    .def("complete_index1_data", &cl::complete_index1_data,
        "Fill up index data of compressed row storage.")
    .def("set_element_past_end", &cl::push_back,
        "(i,j,x) Set a[i,j] = x assuming no element before i,j in lexical ordering.")
    .add_property("nnz", &cl::nnz,
        "The number of structural nonzeros in the matrix")
    ;

  pyclass
    .def("__add__", wrapBinaryOp<cl, std::plus<cl> >)
    .def("__sub__", wrapBinaryOp<cl, std::minus<cl> >)
    ;
}




template <typename PYC, typename VT, typename L, std::size_t IB, typename IA, typename TA>
void expose_matrix_specialties(PYC &pyclass, ublas::coordinate_matrix<VT, L, IB, IA, TA>)
{
  typedef ublas::coordinate_matrix<VT, L, IB, IA, TA> cl;

  pyclass
    .def("sort", &cl::sort,
        "Make sure coordinate representation is sorted.")
    .def("set_element", insertElementWrapper<cl>,
        "(i,j,x) Set a[i,j] = x.")
    .def("set_element_past_end", &cl::push_back,
        "(i,j,x) Set a[i,j] = x assuming no element before i,j in lexical ordering.")
    .def("add_element", &cl::append_element,
        "(i,j,x) Set a[i,j] += x.")
    .add_property("nnz", &cl::nnz,
        "The number of structural nonzeros in the matrix")
    ;

  expose_add_scattered<cl, pyublas::numpy_matrix<VT> >(pyclass);
  expose_add_scattered<cl, cl >(pyclass);
  expose_add_scattered<cl,
    ublas::compressed_matrix<VT, ublas::column_major, 0,
    ublas::unbounded_array<int> > >(pyclass);

  pyclass
    .def("__add__", wrapBinaryOp<cl, std::plus<cl> >)
    .def("__sub__", wrapBinaryOp<cl, std::minus<cl> >)
    ;
}




template <typename WrappedClass>
void expose_matrix_type(WrappedClass, const std::string &python_typename, const std::string &python_eltypename)
{
  std::string total_typename = python_typename + python_eltypename;
  typedef class_<WrappedClass> wrapper_class;
  wrapper_class pyclass(total_typename.c_str());

  pyclass
    .def(python::init<typename WrappedClass::size_type,
        typename WrappedClass::size_type>())
    ;

  exposeMatrixConcept(pyclass, WrappedClass());
  expose_iterator(pyclass, total_typename, WrappedClass());
  exposeForMatricesConvertibleTo(matrix_converter_exposer<wrapper_class>(pyclass),
      typename WrappedClass::value_type());
  expose_matrix_specialties(pyclass, WrappedClass());
}




#define EXPOSE_ALL_TYPES \
  exposeAll(double(), "Float64"); \
  exposeAll(std::complex<double>(), "Complex128"); \



} // private namespace



#ifdef _MSC_VER
#pragma warning( pop )
#endif


// EMACS-FORMAT-TAG
//
// Local Variables:
// mode: C++
// eval: (c-set-style "stroustrup")
// eval: (c-set-offset 'access-label -2)
// eval: (c-set-offset 'inclass '++)
// c-basic-offset: 2
// tab-width: 8
// End:

