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

#ifndef __sot_talos_balance_NmpcOnline_H__
#define __sot_talos_balance_NmpcOnline_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(position_controller_EXPORTS)
#define NMPC_EXPORT __declspec(dllexport)
#else
#define NMPC_EXPORT __declspec(dllimport)
#endif
#else
#define NMPC_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/robot-utils.hh>
#include <dynamic-graph/signal-helper.h>

#include <map>
#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class NMPC_EXPORT NmpcOnline : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  NmpcOnline(const std::string& name);

  void init(const dynamicgraph::Vector& com,const float& footx,const float& footy,
    const float& footq,const std::string& foot,const std::string& state);

/*  void set_comref(double comx);*/

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(velocityref, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(trigger, bool);

  DECLARE_SIGNAL_OUT(comref, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dcomref, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ddcomref, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(rightfootref,MatrixHomogeneous);
  DECLARE_SIGNAL_OUT(leftfootref,MatrixHomogeneous);  
  DECLARE_SIGNAL_OUT(waistref,MatrixHomogeneous);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool m_initSucceeded;  /// true if the entity has been successfully initialized

  dynamicgraph::Vector m_comref;
  dynamicgraph::Vector m_dcomref;
  dynamicgraph::Vector m_ddcomref;
  MatrixHomogeneous m_rightfootref;
  MatrixHomogeneous m_leftfootref;
  MatrixHomogeneous m_waistref;
  float m_footx;
  float m_footy;  
  float m_footq;  
  std::string m_foot;
  std::string m_state;

};  // class NmpcOnline

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_NmpcOnline_H__
