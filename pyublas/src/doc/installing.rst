.. highlight:: sh

Installation
============

This tutorial will walk you through the process of building PyUblas. To follow,
you really only need three basic things:

* A UNIX-like machine with web access. (Running Windows? See
  :ref:`boost-build-install`.)
* A C++ compiler, preferably a Version 4.x gcc.
* A working `Python <http://www.python.org>`_ installation, Version 2.4 or newer.

Step 1: Install Boost
---------------------

You may already have a working copy of the `Boost C++
libraries <http://www.boost.org>`_. If so, make sure that it's version 1.35.0 or
newer. If not, no problem, we'll build it now. Before you start, make sure you
have the Python headers (i.e. development information) installed. Your
operating system may call this package something like `python-dev` or
`python-devel`. Next, `download <http://boost.org/users/download>`_ the Boost
release tar.bz2 file. Then, do this::

    $ tar xfj ~/download/boost_1_35_0.tar.bz2
    $ cd boost_1_35_0
    $ ./configure.py --prefix=$HOME/pool
    $ make
    $ make install

(Whenever you see the "`$`" dollar sign, this means you should enter this at
your shell prompt. You don't have to be `root`. A few spots are marked with "su
-c" to show that these *do* require root privileges if you are using a Python
interpreter that is install globally.)

You may adapt the file and directory names to suit your liking, however the
rest of this tutorial will assume that you use these paths.


.. warning::

  Please make sure that the Boost.Python configuration process finds
  the version of Python you intend to use. It is output during the configure/make
  stage.

If you see something like::

    ...failed updating 30 targets...
    ...skipped 2 targets...

at the end of the build process, please double-check that you have the Python
headers installed. If you failed fewer targets (up to 5), you're probably ok
for hedge, but you might still want to install `libz-dev` and `libbz2-dev` for
that "perfect score".

Tell the Dynamic Linker about Boost
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you use a bash or /bin/sh or another POSIX-compliant shell, use this command::

    export LD_LIBRARY_PATH=$HOME/pool/lib:${LD_LIBRARY_PATH}

or, if you are still using a C Shell, use this::

    setenv LD_LIBRARY_PATH $HOME/pool/lib:${LD_LIBRARY_PATH}

You might want to put this command in your startup script, so you don't have to
type this over and over. If you forget this step, you will see errors like this
one later on::

    ...gibberish...
    ImportError: libboost_python-gcc42-mt-1_35.so.1.35.0: 
    cannot open shared object file: No such file or directory
    ...gibberish...

Step 2: Download and unpack PyUblas
-----------------------------------

`Download PyUblas <http://pypi.python.org/pypi/PyUblas>`_ and unpack it::

    $ tar xfz PyUblas-VERSION.tar.gz

Step 3: Install Numpy
---------------------

If you don't already have `numpy <http://numpy.org>`_ installed, this is how to get
it::

    $ cd PyUblas-VERSION
    $ su -c "python ez_setup.py" # this will install setuptools
    $ su -c "easy_install numpy" # this will install numpy using setuptools

(If you're not sure, repeating these commands will not hurt.)

Step 4: Create and Customize a Configuration File
-------------------------------------------------

Copy and paste the following text into a file called
:file:`.aksetup-defaults.py` (Make sure not to miss
the initial dot, it's important.) in your home directory::

    BOOST_BINDINGS_INC_DIR = ['/home/andreas/pool/include/boost-bindings']
    BOOST_INC_DIR = ['/home/andreas/pool/include/boost-1_35']
    BOOST_LIB_DIR = ['/home/andreas/pool/lib']
    BOOST_PYTHON_LIBNAME = ['boost_python-gcc42-mt']

You will need to adapt the path names in this file to your personal
situation, of course.

Step 5: Build PyUblas
---------------------

Just type::

    $ cd PyUblas-VERSION # if you're not there already
    $ sudo python setup.py install

Note that ``gcc42`` is a compiler tag that depends on the compiler
with which you built boost. Check the contents of your boost 
library directory to find out what the correct tag is.

Once that works, congratulations! You've successfully built PyUblas.

Step 6: Test PyUblas
--------------------

If you'd like to be extra-careful, you can run PyUblas's unit tests::

    $ cd PyUblas-VERSION/test
    $ python test.py

If it says "OK" at the end, you're golden.

.. _boost-build-install:

Alternate Installation using Boost.Build
========================================

MSVC users can build PyUblas using boost bjam. Unpack Boost in a parallel
directory to pyublas and then run bjam in the :file:`pyublas` directory. A
subdirectory named "pyublas" will be created with the files required for the
Python extension module.
