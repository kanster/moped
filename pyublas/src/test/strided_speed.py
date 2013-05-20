
def print_extime(f):
    from time import time
    start = time()
    f()
    print "%s: %fs" % (f.__name__, time() - start)

import pyublas
import pyublas.testhelp_ext as t
print_extime(t.test_ublas_speed)
print_extime(t.test_unstrided_speed)
print_extime(t.test_strided_speed)

print_extime(t.test_ublas_speed2)
print_extime(t.test_unstrided_speed2)
print_extime(t.test_strided_speed2)
