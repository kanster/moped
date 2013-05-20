import numpy
import sample_ext
import pyublas # not explicitly used--but makes converters available

vec = numpy.ones((5,), dtype=float)
print vec
print sample_ext.doublify(vec)
