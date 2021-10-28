/*
 * Copyright 2018, Gepetto team, LAAS-CNRS
 *
 * This file is part of sot-talos-balance.
 * sot-talos-balance is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-talos-balance is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-talos-balance.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sot/talos_balance/nmpc-online.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                         "-------------------------------------------------------"
#define PROFILE_NMPC_SUM_COMPUTATION "NmpcOnline: sum computation                               "
#define PROFILE_NMPC_NBJOINTS_COMPUTATION "NmpcOnline: nbJoints extraction                           "

#define INPUT_SIGNALS m_velocityrefSIN << m_triggerSIN

#define OUTPUT_SIGNALS m_comrefSOUT //<<  m_dcomrefSOUT  m_ddcomrefSOUT <<
  //m_rightfootrefSOUT << m_leftfootrefSOUT << m_waistrefSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef NmpcOnline EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NmpcOnline, "NmpcOnline");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
NmpcOnline::NmpcOnline(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(velocityref, dynamicgraph::Vector),
      CONSTRUCT_SIGNAL_IN(trigger, bool),      
      CONSTRUCT_SIGNAL_OUT(comref, dynamicgraph::Vector, INPUT_SIGNALS),
/*      CONSTRUCT_SIGNAL_OUT(dcomref, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(ddcomref, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(rightfootref, MatrixHomogeneous, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(leftfootref, MatrixHomogeneous, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(waistref, MatrixHomogeneous, INPUT_SIGNALS),
*/    m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid2(*this, &NmpcOnline::init, docCommandVoid2("Initialize the entity.", "com","size")));

  //addCommand("set_comref", makeCommandVoid1(*this, &NmpcOnline::set_comref, docCommandVoid1("Set the com ref.", "comx")));

}

void NmpcOnline::init(const double& com_ref, const int& N) {
  if (!m_velocityrefSIN.isPlugged()) return SEND_MSG("Init failed: signal velocityref is not plugged", MSG_TYPE_ERROR);
  if (!m_triggerSIN.isPlugged())
    return SEND_MSG("Init failed: signal trigger is not plugged", MSG_TYPE_ERROR);

  m_com_ref.setZero(N);
  m_com_ref(0) = com_ref;
  m_initSucceeded = true;
}

void NmpcOnline::set_comref(double comx){
  m_com_ref(1) = comx;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(comref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }

  if (m_triggerSIN(iter)) {
    set_comref(0.1);
  } else {
    set_comref(0.2);
  }
  s = m_com_ref;

  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void NmpcOnline::display(std::ostream& os) const {
  os << "NmpcOnline " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph
