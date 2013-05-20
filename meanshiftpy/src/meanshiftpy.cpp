/*********************************************************************
* meanshiftpy
* Port of MeanShift to python
* Written by Alvaro Collet  	acollet@cs.cmu.edu
* Original C/Matlab code from Piotr Dollar
* 
* DEPENDENCIES: This library requires boost.python and pyUblas.
*
*********************************************************************/

/*********************************************************************
* DATESTAMP  29-Sep-2005  2:00pm
* Piotr's Image&Video Toolbox      Version 1.0   
* Written and maintained by Piotr Dollar    pdollar-at-cs.ucsd.edu 
* Adapted from code by: Sameer Agarwal || sagarwal-at-cs.ucsd.edu
* Please email me if you find bugs, or have suggestions or questions! 
*********************************************************************/

#include <math.h>
#include <string.h>
#include <Python.h>
#include <boost/python.hpp>
#include <pyublas/numpy.hpp>

using namespace pyublas;

/* MIN MAX PI */
#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.14159265


/*
 * Calculates the mean of all the points in data that lie on a sphere of 
 * radius^2==radius2 centered on the vector x.  x has p dimensions, data has n 
 * points of dimension p.  The pointer *mean will point to the array containing
 * the mean, and the return is the number of points that were used to calculate
 * the mean.  
 */
/* had to remove 'inline' because my compiler sucks */
static int meanvector( double *x, double *data, int p, int n, double radius2, double *mean)
{
    int i, j;
    float value;
    int counter = 0;
    int npoints = 0;

    for(j=0;j<p;j++) mean[j]=0.0;

    /* loop over points in data */
    for (i = 0; i < n; i++) {
        value = 0.0;
        
        /* get distance of point to x */
        for (j = 0; j < p; j++) {
            value += (x[j] - data[counter]) * (x[j] - data[counter]);
            counter++;
        }

        /* include point in average if its d to x is within radius2 */
        if (value < radius2) {
            counter = counter - p;
            for (j = 0; j < p; j++) {
                mean[j] += data[counter];
	            counter++;
            }
            npoints++;
        }
    }
    
    /* get mean by dividing by number of points */
    for (j = 0; j < p; j++) mean[j] /= npoints; 
    return npoints;
}




/******************************************************************************/
/*
 * Squared euclidean distance between two vectors.
 */
/* had to remove 'inline' because my compiler sucks */
double dist (double *A, double *B, int n)
{
    double result = 0.0;
    int i;
    for (i = 0; i < n; i++)
        result += (A[i] - B[i]) * (A[i] - B[i]);
    return result;
}

/******************************************************************************/
/*
 * See accompanying m file for more info.
 *
 * data             - p x n column matrix of data points
 * p               - dimension of data points
 * n               - number of data points
 * radius           - radius of search windo    
 * rate             - gradient descent proportionality factor
 * maxiter          - max allowed number of iterations
 * blur             - specifies which mode of the algorithm you want to use 
 * clusterlabels    - labels for each cluster
 *
 * means_final      - output (final clusters)
 */
static void meanshift(double data[], int p, int n, double radius, double rate, 
                        int maxiter, int blur, double clusterlabels[], double *means_final)
{
    /* VARIABLE DECLARATIONS */
    float radius2;      /* radius*radius */
    int itercount;      /* number of iterations */
    double *mean;       /* mean vector */
    int npoints;        /* number of points used to calc mean vector */
    int i, j;           /* looping variables */
    int delta = 1;      /* indicator if change occurred between iterations */
    int *deltas;        /* indicator if change occurred between iterations per item */
    
    /* The calculated means in the current and for next iteration */
    double *means_current;
    double *means_next;
    
    /* If blur adata points to means_current else it points to data */
    double *adata;
    
    
    /* The following are needed in the assignment of cluster labels */
    int *consolidated; 
    int nclusterlabels = 1;
    
    
    /* INITIALIZATION */
    means_current = (double *) malloc (sizeof (double) * p * n);
    means_next = (double *) malloc (sizeof (double) * p * n);
    mean = (double *) malloc (sizeof (double) * p);
    consolidated = (int *) malloc (sizeof (int) * n);
    deltas = (int *) malloc (sizeof (int) * n);
    for (i = 0; i < n; i++) deltas[i] = 1;
        
    radius2 = radius * radius;
    means_current = (double *) memcpy(means_current, data, p * n * sizeof (double));
    if (blur) adata = means_current; else adata = data;


    /* MAIN LOOP */
    itercount = 0;  
    while ((itercount < maxiter) && delta) {
       
        for (i = 0; i < n; i++) {
            if( deltas[i] ) {
                /* get new mean vector and number of points used to calculate it */
                npoints = meanvector(means_current + i * p, adata, p, n, radius2, mean);
            
                /* shift means_next in direction of mean (if npoints>0) */
                if (npoints) {
                    for (j = 0; j < p; j++)
                        means_next[i * p + j] = (1 - rate) * means_current[i * p + j] + rate*mean[j];
                } else {
                    for (j = 0; j < p; j++)
                        means_next[i * p + j] = means_current[i * p + j];
                }
            }
        }

        /* update means, after seeing if there was any change. */
        delta = 0;
        for (i = 0; i < n; i++) {
            if( deltas[i] && dist( means_next+i*p, means_current+i*p, p )>0.001 )
                delta=1;
            else
                deltas[i]=0;
        }
        means_current = (double*) memcpy (means_current, means_next, p * n * sizeof (double));
        
        /* update progress indicator */
        itercount++;
    }
    
    
    /* CONSOLIDATE */
    /* Assign all points that are within a squared distance of radius2 of each other to 
      the same cluster.  Also calculate their mean. */
    for (i = 0; i < n; i++) {
        consolidated[i] = 0;
        clusterlabels[i] = 0;
    }
    for (i = 0; i < n; i++)
        if (!consolidated[i]) {
        	for (j = 0; j < n; j++) {
                if (!consolidated[j])
                    if (dist(means_current + i * p, means_current + j * p, p) < radius2) {
                        clusterlabels[j] = nclusterlabels;
                        consolidated[j] = 1;
                    }
            }
        	nclusterlabels++;
    	}
    means_final = (double*) memcpy (means_final, means_current, p * n * sizeof (double));
    nclusterlabels = nclusterlabels-1;

    
    /* RELEASE MEMORY */
    free(means_next);
    free(means_current);
    free(mean);
    free(consolidated);
    free(deltas);
}

class MeanShift
{
public:
    numpy_vector<double> clusterlabels;
    numpy_vector<double> means_final;
    numpy_vector<int> clustercounts;
   
    void wrapper_py(pyublas::numpy_vector<double> pydata, double radius, double rate, int maxiter, int blur=0);

    double * getClusters() { 	
	numpy_array<double> raw_data(clusterlabels.to_python());
	return raw_data.data();
    }

    double * getMeans() { 	
	numpy_array<double> raw_data(means_final.to_python());
	return raw_data.data();
    }

    void setClusters(numpy_vector<double> pydata) { 	
	clusterlabels.resize(pydata.size());
	clusterlabels.assign(pydata);
    }

    void setMeans(numpy_vector<double> pydata) {
	means_final.resize(pydata.size());
	means_final.assign(pydata);
    }

 
};


// ******************************************************************************
void MeanShift::wrapper_py(pyublas::numpy_vector<double> pydata, double radius, double rate, int maxiter, int blur)
// void wrapper_py(double data*, int p, int n, double radius, double rate, int maxiter, int blur=0)
{
  double *pclusterlabels;
  double *pmeans_final;

  numpy_array<double> raw_data(pydata.to_python());

  const npy_intp *dims;
  npy_intp dims_clust[3] = {1, 1, 1};

  dims = pydata.dims();

  // Allocate memory for the return arguments
  clusterlabels.resize(dims[0]);
  dims_clust[0] = dims[0];
  clusterlabels.reshape(2, dims_clust);

  means_final.resize(dims[0]*dims[1]);
  means_final.reshape(pydata.ndim(), dims);
  

  pclusterlabels = getClusters();
  pmeans_final = getMeans();

  // Do the actual computations in a subroutine
  meanshift (raw_data.data(), dims[1], dims[0], radius, rate, maxiter, blur, pclusterlabels, pmeans_final);

  // Get number of clusters 
  numpy_vector<double>::iterator cl;

  int max_cl = 0;
  for (cl = clusterlabels.begin(); cl != clusterlabels.end(); ++cl)
    max_cl = ((*cl) > max_cl) ? (*cl) : max_cl;

  // Compute number of points per cluster
  clustercounts.resize(max_cl, 0);
  for (cl = clusterlabels.begin(); cl != clusterlabels.end(); ++cl)
    clustercounts[(*cl)-1]++; // Careful, clusters are 1-based
 
  return;
}


BOOST_PYTHON_MODULE(libmeanshift)
{
    using namespace boost::python;

    class_<MeanShift>("MeanShift")
    	.def(by_value_rw_member("labels", &MeanShift::clusterlabels))
    	.def(by_value_rw_member("means", &MeanShift::means_final))
	.def(by_value_rw_member("ccounts", &MeanShift::clustercounts))
	.def("cluster", &MeanShift::wrapper_py)
    ;
}

