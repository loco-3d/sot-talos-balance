#include <dynamic-graph/python/module.hh>
#include <sot/talos_balance/talos-control-manager.hh>

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");

  dg::python::exposeEntity<dg::sot::talos_balance::TalosControlManager,
                           bp::bases<dg::Entity>,
                           dg::python::AddSignals & dg::python::AddCommands>();
}
