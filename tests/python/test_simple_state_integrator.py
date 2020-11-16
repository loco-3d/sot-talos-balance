from __future__ import print_function

import numpy as np
from dynamic_graph.sot_talos_balance.simple_state_integrator import SimpleStateIntegrator
from numpy.testing import assert_almost_equal

dt = 1e-3
initstate = np.array([0.] * 9)

integrator = SimpleStateIntegrator("integrator")
integrator.init(dt)
integrator.setState(initstate)
integrator.setVelocity(np.array([0.] * 9))
t = 0

integrator.control.value = np.array([0.] * 9)
integrator.state.recompute(0)
assert_almost_equal(integrator.state.value, initstate)

t += 1
integrator.control.value = np.array([1., 0., 0.] + [0.] * 6)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([dt, 0., 0.] + [0.] * 6))

t += 1
integrator.control.value = np.array([0., 1., 0.] + [0.] * 6)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([dt, dt, 0.] + [0.] * 6))

t += 1
integrator.control.value = np.array([0., 0., 1.] + [0.] * 6)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([dt, dt, dt] + [0.] * 6))

t += 1
integrator.control.value = np.array([0.] * 3 + [1., 0., 0.] + [0.] * 3)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([dt] * 3 + [dt, 0., 0.] + [0.] * 3))

t += 1
integrator.setState(initstate)
integrator.control.value = np.array([0.] * 3 + [0., 1., 0.] + [0.] * 3)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([0.] * 3 + [0., dt, 0.] + [0.] * 3))

t += 1
integrator.setState(initstate)
integrator.control.value = np.array([0.] * 3 + [0., 0., 1.] + [0.] * 3)
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([0.] * 3 + [0., 0., dt] + [0.] * 3))

t += 1
integrator.setState(initstate)
integrator.control.value = np.array([0.] * 6 + [1., 0., 0.])
integrator.state.recompute(t)
assert_almost_equal(integrator.state.value, np.array([0.] * 6 + [dt, 0., 0.]))

integrator.setState(initstate)
N = 10
for k in range(N):
    t += 1
    integrator.control.value = np.array([0.] * 3 + [1., 0., 0.] + [0.] * 3)
    integrator.state.recompute(t)
    assert_almost_equal(integrator.state.value, np.array([0.] * 3 + [dt * (k + 1), 0., 0.] + [0.] * 3))

t += 1
integrator.setState(np.array(initstate + [0.]))
integrator.control.value = np.array([0.] * 3 + [1., 0., 0.] + [0.] * 3)
# TODO: this should raise, and this is not raising...
# try:
# integrator.state.recompute(t)
# except:
# pass
# else:
# raise AssertionError("No exception raised for mismatching state and control size")
