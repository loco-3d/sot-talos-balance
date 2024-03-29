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

#ifndef __sot_talos_balance_simple_controller_6d_H__
#define __sot_talos_balance_simple_controller_6d_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(simple_controller_6d_EXPORTS)
#define SIMPLE_CONTROLLER_6D_EXPORT __declspec(dllexport)
#else
#define SIMPLE_CONTROLLER_6D_EXPORT __declspec(dllimport)
#endif
#else
#define SIMPLE_CONTROLLER_6D_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-helper.h>

#include <map>
#include <sot/core/matrix-geometry.hh>

#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SIMPLE_CONTROLLER_6D_EXPORT SimpleController6d
    : public ::dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* --- CONSTRUCTOR ---- */
  SimpleController6d(const std::string& name);

  void init();

  template <typename Derived>
  Eigen::Matrix3d skew(const Eigen::MatrixBase<Derived>& v);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);

  DECLARE_SIGNAL_IN(x, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(x_des, MatrixHomogeneous);
  DECLARE_SIGNAL_IN(v_des, dynamicgraph::Vector);

  DECLARE_SIGNAL_OUT(v_ref, dynamicgraph::Vector);

  /* --- COMMANDS --- */
  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  bool
      m_initSucceeded;  /// true if the entity has been successfully initialized

};  // class SimpleController6d

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_simple_controller_6d_H__
