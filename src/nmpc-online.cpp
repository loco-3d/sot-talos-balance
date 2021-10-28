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

#define OUTPUT_SIGNALS m_comrefSOUT << m_dcomrefSOUT << m_ddcomrefSOUT << m_rightfootrefSOUT << m_leftfootrefSOUT << m_waistrefSOUT

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
      CONSTRUCT_SIGNAL_OUT(dcomref, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(ddcomref, dynamicgraph::Vector, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(rightfootref, MatrixHomogeneous, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(leftfootref, MatrixHomogeneous, INPUT_SIGNALS),
      CONSTRUCT_SIGNAL_OUT(waistref, MatrixHomogeneous, INPUT_SIGNALS),
    m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid6(*this, &NmpcOnline::init, docCommandVoid6(
    "initialize the entity.","Initial com","Initial foot (x)","Initial foot (y)",
    "Initial foot (q)","Initial foot (name)","Initial state (D,L,R)")));

}

void NmpcOnline::init(const dynamicgraph::Vector& com,const float& footx,
    const float& footy,const float& footq,const std::string& foot,
    const std::string& state) {
  if (!m_velocityrefSIN.isPlugged()) return SEND_MSG("init failed: signal velocityref is not plugged", MSG_TYPE_ERROR);
  if (!m_triggerSIN.isPlugged())
    return SEND_MSG("init failed: signal trigger is not plugged", MSG_TYPE_ERROR);

  m_comref = com;
  m_dcomref.setZero(3);
  m_ddcomref.setZero(3); 
  m_rightfootref.setIdentity();
  m_leftfootref.setIdentity(); 
  m_waistref.setIdentity(); 

  m_footx = footx;
  m_footy = footy;  
  m_footq = footq;  
  m_foot = foot;
  m_state = state;

  m_initSucceeded = true;
}


/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(comref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
/*  if (m_triggerSIN(iter)) {
    set_comref(0.1);
  } else {
    set_comref(0.2);
  }*/
  s = m_comref;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(dcomref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  s = m_dcomref;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(ddcomref, dynamicgraph::Vector) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  s = m_ddcomref;
  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(rightfootref, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  s = m_rightfootref;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(leftfootref, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  s = m_leftfootref;

  return s;
}

DEFINE_SIGNAL_OUT_FUNCTION(waistref, MatrixHomogeneous) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  s = m_waistref;

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
