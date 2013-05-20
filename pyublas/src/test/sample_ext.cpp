#include <pyublas/numpy.hpp>

pyublas::numpy_vector<double> doublify(pyublas::numpy_vector<double> x)
{
  return 2*x;
}

BOOST_PYTHON_MODULE(sample_ext)
{
  boost::python::def("doublify", doublify);
}
