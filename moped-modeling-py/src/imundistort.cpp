/*********************************************************************
* imundistort
* Non-linear image undistortion, code taken from old version of OpenCV
*
* Written by Alvaro Collet  	acollet@cs.cmu.edu
* Copyright: Carnegie Mellon University
*
* DEPENDENCIES: This library requires boost.python and pyUblas.
*
*********************************************************************/
#include <Python.h>
#include <boost/python.hpp>
#include <pyublas/numpy.hpp>
#include <opencv/cv.h>
#include "cvundistort.h"

using namespace pyublas;

pyublas::numpy_vector<unsigned int> 
undistort(
          pyublas::numpy_vector<unsigned int> pydata,
          pyublas::numpy_vector<double> pyKK,
          pyublas::numpy_vector<double> pydist
          )
{
  const npy_intp *dims = pydata.dims();
  const numpy_array<unsigned int> py_img(pydata.to_python());
  const numpy_array<double> dist(pydist.to_python());

  int channels = pydata.ndim() == 3 ? dims[2] : 1;

  IplImage *cv_img = cvCreateImageHeader(cvSize(dims[1], dims[0]), IPL_DEPTH_8U, channels);
  IplImage *cv_out_img = cvCreateImageHeader(cvSize(dims[1], dims[0]), IPL_DEPTH_8U, channels);
  numpy_vector<unsigned int> out_img;

  out_img.resize(py_img.size());
  out_img.reshape(pydata.ndim(), dims);
  cvSetData(cv_img, (unsigned int *) &pydata[0], dims[1]);
  cvSetData(cv_out_img, (unsigned int *) &out_img[0], dims[1]);

  double KK[9] = {pyKK[0], 0, pyKK[2], 0, pyKK[1], pyKK[3], 0, 0, 1};
  CvMat FullKK = cvMat(3, 3, CV_64FC1, KK);
  CvMat dist_coeffs = cvMat(4, 1, CV_64FC1, (double *)&dist[0]);

  cvUndistort(cv_img, cv_out_img, &FullKK, &dist_coeffs);
  
  cvReleaseImageHeader(&cv_img);
  cvReleaseImageHeader(&cv_out_img);
  return out_img;
}

BOOST_PYTHON_MODULE(libimundistort)
{
  boost::python::def("undistort", undistort);
}
