from __future__ import print_function

import numpy as np

from dynamic_graph.sot_talos_balance.test_cython import TestCython

# --- nmpcample ---
print("--- Entity test ---")

test = TestCython("test_cython")

print("\nSignals (at creation):")
test.displaySignals()

test.a.value = 1.
test.b.value = 3.

print("\nSignals (after pugging):")
test.displaySignals()

test.init()

test.c.recompute(1)

print("\nInputs:" ,test.a.value, test.b.value)
print("Output: ", test.c.value)

