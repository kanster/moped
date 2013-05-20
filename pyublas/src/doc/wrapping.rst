Wrapping C++ code with PyUblas
==============================

Data members of type :ctype:`numpy_vector`
------------------------------------------

If the piece of software you are wrapping contains :ctype:`numpy_vector`
structure or class members, note that you cannot use Boost.Python's standard
``class_<>::def_readonly()`` or ``class_<>::def_readwrite()`` mechanism to expose
these. This is again because these functions expect to be able to return an
lvalue to Python when a member is read. Since there is no lvalue to-Python
converter for :ctype:`numpy_vector`, this will fail at runtime.

PyUblas contains a mechanism to circumvent this limitation::

  struct my_vector_container 
  {
    const numpy_vector<double> my_ro_data_member;
    numpy_vector<double> my_rw_data_member;
  };

  BOOST_PYTHON_MODULE(memberdemo)
  {
    class_<my_vector_container>("MyVectorContainer")
      .def(pyublas::by_value_ro_member(
           "my_ro_data_member",
           &my_vector_container::my_ro_data_member))
      .def(pyublas::by_value_rw_member(
           "my_rw_data_member",
           &my_vector_container::my_rw_data_member))
      ;
  }

**CAUTION:** If you are exposing a member that is originally found in a base
class of the class you are exposing, you will get errors when accessing that
member from Python. (such as "Argument types ... did not match C++
signature") This is because the C++ pointer-to-member type only records the
type of the base class that Boost.Python knows nothing about. Therefore, the
getters and setters implicitly used by the ``by_value_rw/ro_member`` code expect
the base class as their first argument, and conversion fails. For regular
members, Boost.Python contains sophisticated logic to fix this. Unfortunately,
at this point, this logic is marked as ``private``, and is therefore inaccessible
to PyUblas.
