# distutils: language=c++
from libs cimport sum

cdef public class Foo[object Foo, type fooType]:
    cdef sum.Sum s 

    def __cinit__(self, double a, double b):
        self.s = sum.Sum(a,b)
        print(a,"+",b," = ",self.s.compute_sum())

    cdef double bar(self, double c):
        print(self.s.compute_sum(),"+",c," = ")
        return self.s.compute_sum()+c  

cdef api Foo buildFoo(double a, double b):
    return Foo(a,b)

cdef api double foobar(Foo foo, double d):
    return foo.bar(d)