from time import sleep

from dynamic_graph.sot_talos_balance.utils.run_test_utils import run_test, runCommandClient

try:
    # Python 2
    input = raw_input  # noqa
except NameError:
    pass

run_test('appli_posture.py')

input("Wait before leaving the simulation")

