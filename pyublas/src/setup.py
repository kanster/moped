#!/usr/bin/env python
# -*- coding: latin-1 -*-




def get_config_schema():
    from aksetup_helper import ConfigSchema, Option, \
            IncludeDir, LibraryDir, Libraries, BoostLibraries, \
            Switch, StringListOption, make_boost_base_options

    return ConfigSchema(make_boost_base_options() + [
        BoostLibraries("python"),

        Switch("WITH_SPARSE_WRAPPERS", False, "Whether to build sparse wrappers"),
        Switch("USE_ITERATORS", False, "Whether to use iterators (faster, requires new Boost)"),

        StringListOption("CXXFLAGS", ["-Wno-sign-compare"], 
            help="Any extra C++ compiler options to include"),
        StringListOption("LDFLAGS", [], 
            help="Any extra linker options to include"),
        ])




def main():
    import glob
    from aksetup_helper import hack_distutils, get_config, setup, \
            NumpyExtension

    hack_distutils()
    conf = get_config(get_config_schema())

    INCLUDE_DIRS = ["src/cpp"] + conf["BOOST_INC_DIR"] 
    LIBRARY_DIRS = conf["BOOST_LIB_DIR"]
    LIBRARIES = conf["BOOST_PYTHON_LIBNAME"]

    EXTRA_DEFINES = { }

    if conf["USE_ITERATORS"]:
        EXTRA_DEFINES["BOOST_UBLAS_USE_ITERATING"] = 1

    ext_src = [
            "src/wrapper/main.cpp",
            "src/wrapper/converters.cpp",
            ] 

    if conf["WITH_SPARSE_WRAPPERS"]:
        ext_src += [
                "src/wrapper/sparse_build.cpp",
                "src/wrapper/sparse_execute.cpp",
                ]
        EXTRA_DEFINES["HAVE_SPARSE_WRAPPERS"] = 1

    try:
        from distutils.command.build_py import build_py_2to3 as build_py
    except ImportError:
        # 2.x
        from distutils.command.build_py import build_py

    setup(
            name="PyUblas",
            version="2011.1",
            description="Seamless Numpy-UBlas interoperability",
            long_description="""
            PyUblas provides a seamless glue layer between
            `Numpy <http://www.numpy.org>`_ and
            `Boost.Ublas <http://www.boost.org/doc/libs/1_35_0/libs/numeric/ublas/doc/index.htm>`_
            for use with
            `Boost.Python <http://www.boost.org/doc/libs/1_35_0/libs/python/doc/index.html>`_.

            What does that mean? When writing 
            `hybrid scientific code <http://mathema.tician.de/node/455>`_, 
            one of the main problems is that abstractions that
            exist in the high-level language go away or become unwieldy in the
            low-level language. Sometimes libraries exist in both languages for
            these abstractions, but they refuse to talk to each other. PyUblas is
            a bridge between two such libraries, for some of the main
            abstractions used in scientific codes, namely vectors and matrices.

            Documentation
            =============

            See the 
            `PyUblas Documentation <http://tiker.net/doc/pyublas>`_
            page.

            PyUblasExt
            ==========

            PyUblasExt is a companion to PyUblas and exposes a variety of useful
            additions to PyUblas, such as an "operator" class, matrix-free linear
            system solvers and eigensolvers. Interested? Head over to the
            `PyUblasExt <http://mathema.tician.de/software/pyublas/pyublasext>`_ 
            page.
            """,
            author="Andreas Kloeckner",
            author_email="inform@tiker.net",
            license = "BSD",
            url="http://mathema.tician.de/software/pyublas",
            classifiers=[
              'Development Status :: 4 - Beta',
              'Environment :: Console',
              'Intended Audience :: Developers',
              'Intended Audience :: Science/Research',
              'License :: OSI Approved :: BSD License',
              'Operating System :: MacOS :: MacOS X',
              'Operating System :: POSIX',
              'Programming Language :: Python',
              'Programming Language :: Python :: 3',
              'Programming Language :: C++',
              'Topic :: Scientific/Engineering',
              'Topic :: Scientific/Engineering :: Mathematics',
              'Topic :: Office/Business',
              'Topic :: Utilities',
              ],

            # numpy is often under the setuptools radar.
            #setup_requires=[
                    #"numpy>=1.0.4",
                    #],
            #install_requires=[
                    #"numpy>=1.0.4",
                    #],

            packages=["pyublas"],
            ext_package="pyublas",
            ext_modules=[ 
                    NumpyExtension("_internal", 
                        ext_src,
                        include_dirs=INCLUDE_DIRS,
                        library_dirs=LIBRARY_DIRS,
                        libraries=LIBRARIES,
                        define_macros=list(EXTRA_DEFINES.items()),
                        extra_compile_args=conf["CXXFLAGS"],
                        extra_link_args=conf["LDFLAGS"],
                        ),
                    NumpyExtension("testhelp_ext", 
                        ["src/test/testhelp_ext.cpp"],
                        include_dirs=INCLUDE_DIRS,
                        library_dirs=LIBRARY_DIRS,
                        libraries=LIBRARIES,
                        define_macros=list(EXTRA_DEFINES.items()),
                        extra_compile_args=conf["CXXFLAGS"],
                        extra_link_args=conf["LDFLAGS"],
                        )
                    ],
            data_files=[("include/pyublas", glob.glob("src/cpp/pyublas/*.hpp"))],

            # 2to3 invocation
            cmdclass={'build_py': build_py})




if __name__ == '__main__':
    main()
