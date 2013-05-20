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




#include <pyublas/numpy.hpp>
#include <boost/python/converter/implicit.hpp>
#include <boost/python/converter/registry.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <boost/numeric/ublas/io.hpp>




using namespace pyublas;
namespace py = boost::python;
namespace ublas = boost::numeric::ublas;




namespace
{
  bool trace_conversion = false;

  template<class T>
  typename numpy_vector<T>::size_type unstrided_size(const numpy_vector<T> &v)
  { return v.size(); }

  template<class T>
  typename numpy_strided_vector<T>::size_type strided_size(const numpy_strided_vector<T> &v)
  { return v.size(); }



  template <class TargetType>
  struct array_converter_base
  {
    typedef typename TargetType::value_type value_type;
    typedef TargetType target_type;

    static void construct(
        PyObject* obj,
        py::converter::rvalue_from_python_stage1_data* data)
    {
      void* storage = ((py::converter::rvalue_from_python_storage<target_type>*)data)->storage.bytes;

      new (storage) target_type(py::handle<>(py::borrowed(obj)));

      // record successful construction
      data->convertible = storage;
    }

    static void construct_invalid_ok(
        PyObject* obj,
        py::converter::rvalue_from_python_stage1_data* data)
    {
      typedef invalid_ok<target_type> inv_ok;
      void* storage = ((py::converter::rvalue_from_python_storage<inv_ok>*)data)->storage.bytes;

      new (storage) inv_ok(target_type(py::handle<>(py::borrowed(obj))));

      // record successful construction
      data->convertible = storage;
    }

    template <class RealTgtType>
    static void construct_indirect(
        PyObject* obj,
        py::converter::rvalue_from_python_stage1_data* data)
    {
      void* storage = ((py::converter::rvalue_from_python_storage<target_type>*)data)->storage.bytes;

      new (storage) RealTgtType(target_type(py::handle<>(py::borrowed(obj))));

      // record successful construction
      data->convertible = storage;
    }

    struct to_python
    {
      static PyObject* convert(target_type const &v)
      {
        return v.to_python().release();
      }
    };

    template <class OriginalType>
    struct indirect_to_python
    {
      static PyObject* convert(OriginalType const &v)
      {
        target_type copied_v(v);
        return copied_v.to_python().release();
      }
    };
  };




  template <class VectorType>
  struct vector_converter : public array_converter_base<VectorType>
  {
    private:
      typedef array_converter_base<VectorType> super;

    public:
      static void construct_strided(
          PyObject* obj,
          py::converter::rvalue_from_python_stage1_data* data)
      {
        typedef numpy_strided_vector<typename super::value_type> strided_vec;
        void* storage = ((py::converter::rvalue_from_python_storage<strided_vec>*)data)->storage.bytes;

        typename super::target_type vec(py::handle<>(py::borrowed(obj)));
        new (storage) strided_vec(vec, vec.stride_slice());

        // record successful construction
        data->convertible = storage;
      }


      static void *check(PyObject* obj)
      {
        if (!PyArray_Check(obj))
        {
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as vec: not a numpy array") % obj
              << std::endl;
          return 0;
        }

        if (!is_storage_compatible<typename super::value_type>(obj))
        {
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as vec: not storage-compatible with %2%")
                  % obj % typeid(typename super::value_type).name()
              << std::endl;

          return 0;
        }


        return obj;
      }

      static void *check_invalid_ok(PyObject* obj)
      {
        if (obj == Py_None)
          return obj;

        return check(obj);
      }

      // needs to be overridden to copy strided version
      template <class RealTgtType>
      static void construct_indirect(
          PyObject* obj,
          py::converter::rvalue_from_python_stage1_data* data)
      {
        void* storage = ((py::converter::rvalue_from_python_storage<
              typename super::target_type>*)data)->storage.bytes;

        new (storage) RealTgtType(
            typename super::target_type(
              py::handle<>(py::borrowed(obj))
              ).as_strided()
            );

        // record successful construction
        data->convertible = storage;
      }
  };




  template <class MatrixType>
  struct matrix_converter : public array_converter_base<MatrixType>
  {
    private:
      typedef array_converter_base<MatrixType> super;

    public:
      static void *check(PyObject* obj)
      {
        if (!PyArray_Check(obj))
        {
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as mat: not a numpy array") % obj
              << std::endl;
          return 0;
        }

        if (!is_storage_compatible<typename super::value_type>(obj))
        {
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as mat: not storage-compatible with %2%")
                  % obj % typeid(typename super::value_type).name()
              << std::endl;

          return 0;
        }

        if (PyArray_NDIM(obj) != 2)
        {
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as mat: not 2-dimensional") % obj
              << std::endl;
          return 0;
        }

        if (PyArray_STRIDE(obj, 1) == PyArray_ITEMSIZE(obj))
        {
          // row-major
          if (!is_row_major(typename MatrixType::orientation_category()))
          {
            if (trace_conversion)
              std::cerr
                << boost::format("obj %1% rejected as mat: not row-major") % obj
                << std::endl;
            return 0;
          }

          if (!PyArray_CHKFLAGS(obj, NPY_C_CONTIGUOUS))
          {
            if (trace_conversion)
              std::cerr
                << boost::format("obj %1% rejected as row-major mat: not C-contiguous") % obj
                << std::endl;
            return 0;
          }
        }
        else if (PyArray_STRIDE(obj, 0) == PyArray_ITEMSIZE(obj))
        {
          if (is_row_major(typename MatrixType::orientation_category()))
          {
            if (trace_conversion)
              std::cerr
                << boost::format("obj %1% rejected as mat: not column-major") % obj
                << std::endl;
            return 0;
          }

          if (!PyArray_CHKFLAGS(obj, NPY_F_CONTIGUOUS))
          {
            if (trace_conversion)
              std::cerr
                << boost::format("obj %1% rejected as column-major mat: not Fortran-contiguous") % obj
                << std::endl;
            return 0;
          }
        }
        else
        {
          // no dim has stride == 1
          if (trace_conversion)
            std::cerr
              << boost::format("obj %1% rejected as mat: no unit-size stride") % obj
              << std::endl;
          return 0;
        }

        return obj;
      }

      static void *check_invalid_ok(PyObject* obj)
      {
        if (obj == Py_None)
          return obj;
        return check(obj);
      }
  };




  const PyTypeObject *get_PyArray_Type()
  { return &PyArray_Type; }

  template <class Converter>
  void register_array_converter()
  {
    py::converter::registry::push_back(
        &Converter::check
        , &Converter::construct
        , py::type_id<typename Converter::target_type>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        , &get_PyArray_Type
#endif
        );

    py::converter::registry::push_back(
        &Converter::check_invalid_ok
        , &Converter::construct_invalid_ok
        , py::type_id<invalid_ok<typename Converter::target_type> >()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        , &get_PyArray_Type
#endif
        );

    py::to_python_converter<
      typename Converter::target_type, typename Converter::to_python>();
  }




  template <class Converter>
  void register_vector_converter()
  {
    py::converter::registry::push_back(
        &Converter::check
        , &Converter::construct_strided
        , py::type_id<numpy_strided_vector<typename Converter::value_type> >()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        , &get_PyArray_Type
#endif
        );

    register_array_converter<Converter>();
  }




  template <class Converter, class RealTgtType>
  void register_indirect_array_converter()
  {
    py::converter::registry::push_back(
        &Converter::check
        , &Converter::template construct_indirect<RealTgtType>
        , py::type_id<RealTgtType>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        , &get_PyArray_Type
#endif
        );

    py::to_python_converter<
      RealTgtType, typename Converter::template indirect_to_python<RealTgtType> >();
  }



  // array scalars ------------------------------------------------------------
  template <class T>
  const PyTypeObject *get_array_scalar_typeobj()
  {
    return (PyTypeObject *) PyArray_TypeObjectFromType(get_typenum(T()));
  }

  template <class T>
  void *check_array_scalar(PyObject *obj)
  {
    if (obj->ob_type == get_array_scalar_typeobj<T>())
      return obj;
    else
      return 0;
  }

  template <class T>
  static void convert_array_scalar(
      PyObject* obj,
      py::converter::rvalue_from_python_stage1_data* data)
  {
    void* storage = ((py::converter::rvalue_from_python_storage<T>*)data)->storage.bytes;

    // no constructor needed, only dealing with POD types
    PyArray_ScalarAsCtype(obj, reinterpret_cast<T *>(storage));

    // record successful construction
    data->convertible = storage;
  }




  // main exposer -------------------------------------------------------------
  template <class T>
  void expose_converters()
  {
    // conversion of array scalars
    py::converter::registry::push_back(
        check_array_scalar<T>
        , convert_array_scalar<T>
        , py::type_id<T>()
#ifndef BOOST_PYTHON_NO_PY_SIGNATURES
        , get_array_scalar_typeobj<T>
#endif
        );

    // conversion of arrays
    typedef numpy_array<T> ary;
    typedef numpy_vector<T> vec;
    typedef numpy_matrix<T, ublas::row_major> rm_mat;
    typedef numpy_matrix<T, ublas::column_major> cm_mat;

    typedef vector_converter<ary> ary_converter;
    typedef vector_converter<vec> vec_converter;
    typedef matrix_converter<cm_mat> cm_mat_converter;
    typedef matrix_converter<rm_mat> rm_mat_converter;

    register_array_converter<ary_converter>();
    register_vector_converter<vec_converter>();
    register_array_converter<cm_mat_converter>();
    register_array_converter<rm_mat_converter>();

    register_indirect_array_converter
      <vec_converter, ublas::vector<T> >();
    register_indirect_array_converter
      <vec_converter, ublas::bounded_vector<T, 2> >();
    register_indirect_array_converter
      <vec_converter, ublas::bounded_vector<T, 3> >();
    register_indirect_array_converter
      <vec_converter, ublas::bounded_vector<T, 4> >();
    register_indirect_array_converter
      <rm_mat_converter, ublas::matrix<T, ublas::row_major> >();
    register_indirect_array_converter
      <cm_mat_converter, ublas::matrix<T, ublas::column_major> >();

    py::to_python_converter<
        numpy_strided_vector<T>,
        typename vector_converter<numpy_strided_vector<T> >::to_python>();

    py::def("unstrided_size", unstrided_size<T>);
    py::def("strided_size", strided_size<T>);
  }



  void set_trace(bool t)
  {
    trace_conversion = t;
  }
}




void pyublas_expose_converters()
{
  expose_converters<bool>();
  expose_converters<npy_byte>();
  expose_converters<npy_ubyte>();
  expose_converters<npy_short>();
  expose_converters<npy_ushort>();
  expose_converters<npy_int>();
  expose_converters<npy_uint>();
  expose_converters<npy_long>();
  expose_converters<npy_ulong>();
  expose_converters<npy_longlong>();
  expose_converters<npy_ulonglong>();
  expose_converters<npy_float>();
  expose_converters<npy_double>();
  expose_converters<std::complex<float> >();
  expose_converters<std::complex<double> >();
#if HAVE_LONG_DOUBLE            // defined in pyconfig.h
  expose_converters<npy_longdouble>();
  expose_converters<std::complex<long double> >();
#endif

  py::def("set_trace", set_trace);
}
