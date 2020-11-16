import numpy as np
from dynamic_graph.sot_talos_balance.dummy_dcm_estimator import DummyDcmEstimator
from numpy.testing import assert_almost_equal

dummy = DummyDcmEstimator('dummy')

dummy.mass.value = 1.0
dummy.omega.value = 1.0
dummy.com.value = np.array([1.] * 3)
dummy.momenta.value = np.array([2.] * 3)

dummy.init()

dummy.dcm.recompute(0)

print(dummy.dcm.value)
assert_almost_equal(dummy.dcm.value, np.array([3.] * 3))

dummy.momenta.value = np.array([3.] * 6)
dummy.dcm.recompute(1)
print(dummy.dcm.value)
assert_almost_equal(dummy.dcm.value, np.array([4.] * 3))
