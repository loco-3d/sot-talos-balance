from __future__ import print_function

import numpy as np

import dynamic_graph.sot_talos_balance.talos.parameter_server_conf as param_server_conf
from dynamic_graph.sot_talos_balance.create_entities_utils import NmpcOnline, create_parameter_server


# --- Parameter server ---
print("--- Parameter server ---")

dt = 1e-3
robot_name = 'robot'
param_server = create_parameter_server(param_server_conf, dt)

# --- nmpcample ---
print("--- Nmpc entity test ---")

nmpc = NmpcOnline("nmpc_test")

print("\nSignals (at creation):")
nmpc.displaySignals()

nmpc.velocityref.value = np.array([0.,0.,0.])
nmpc.trigger.value = True

print("\nSignals (after pugging):")
nmpc.displaySignals()

com_init = np.array([-3.16e-3,1.237384291203724555e-03,8.786810585901939641e-01])
footx_init = 1.86e-4
footy_init = 0.085
footq_init = 0.

nmpc.init(com_init,footx_init,footy_init,footq_init,"left","D")

nmpc.comref.recompute(1)
nmpc.dcomref.recompute(1)
nmpc.ddcomref.recompute(1)
nmpc.rightfootref.recompute(1)
nmpc.leftfootref.recompute(1)
nmpc.waistref.recompute(1)

print("\nInputs:" ,nmpc.velocityref.value, nmpc.trigger.value)
print("Outputs: ", nmpc.comref.value,nmpc.dcomref.value,nmpc.ddcomref.value,"\n",\
	nmpc.rightfootref.value,"\n",nmpc.leftfootref.value,"\n",nmpc.waistref.value)
