from __future__ import print_function

import numpy as np

import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
from dynamic_graph.sot_talos_balance.create_entities_utils import NmpcOnline, create_parameter_server


# --- Parameter server ---
print("--- Parameter server ---")

dt = 1e-3
robot_name = 'robot'
param_server = create_parameter_server(param_server_conf, dt)

# --- Example ---
print("--- Nmpc entity test ---")

ex = NmpcOnline("nmpc_test")

print("\nSignals (at creation):")
ex.displaySignals()

ex.velocityref.value = np.array([0.,0.1,0.])
ex.trigger.value = True

print("\nSignals (after pugging):")
ex.displaySignals()

ex.init(0.2,4)

print("Output: ", ex.comref.value)
ex.comref.recompute(1)

print("\nInputs:" ,ex.velocityref.value, ex.trigger.value)
print("Output: ", ex.comref.value)
