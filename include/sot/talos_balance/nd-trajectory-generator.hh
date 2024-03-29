/*
 * Copyright 2017,Thomas Flayols, LAAS-CNRS
 * File derived from sot-torque-control package available on
 * https://github.com/stack-of-tasks/sot-torque-control
 *
 * This file is part of sot-talos-balance.
 * sot-talos-balance is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-talos-balance.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_talos_balance_nd_trajectory_generator_H__
#define __sot_talos_balance_nd_trajectory_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(nd_position_controller_EXPORTS)
#define SOTNDTRAJECTORYGENERATOR_EXPORT __declspec(dllexport)
#else
#define SOTNDTRAJECTORYGENERATOR_EXPORT __declspec(dllimport)
#endif
#else
#define SOTNDTRAJECTORYGENERATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>

#include <map>
#include <parametric-curves/constant.hpp>
#include <parametric-curves/infinite-const-acc.hpp>
#include <parametric-curves/infinite-sinusoid.hpp>
#include <parametric-curves/linear-chirp.hpp>
#include <parametric-curves/minimum-jerk.hpp>
#include <parametric-curves/spline.hpp>
#include <parametric-curves/text-file.hpp>
#include <sot/core/matrix-geometry.hh>

#include "boost/assign.hpp"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTNDTRAJECTORYGENERATOR_EXPORT NdTrajectoryGenerator
    : public ::dynamicgraph::Entity {
  typedef NdTrajectoryGenerator EntityClassName;
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  /* --- CONSTRUCTOR ---- */
  NdTrajectoryGenerator(const std::string& name);

  void init(const double& dt, const unsigned int& n);

  /* --- SIGNALS --- */
  DECLARE_SIGNAL_IN(initial_value, dynamicgraph::Vector);
  DECLARE_SIGNAL_IN(trigger, bool);
  DECLARE_SIGNAL(x, OUT, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(dx, dynamicgraph::Vector);
  DECLARE_SIGNAL_OUT(ddx, dynamicgraph::Vector);

 protected:
  DECLARE_SIGNAL_OUT_FUNCTION(x, dynamicgraph::Vector);

 public:
  /* --- COMMANDS --- */

  void playTrajectoryFile(const std::string& fileName);

  void playSpline(const std::string& fileName);

  void setSpline(const std::string& filename, const double& timeToInitConf);
  void startSpline();

  /** Print the current value of the specified component. */
  void getValue(const int& id);

  /** Move a component to a position with a minimum-jerk trajectory.
   * @param id integer index.
   * @param xFinal The desired final position of the component.
   * @param time The time to go from the current position to xFinal [sec].
   */
  void move(const int& id, const double& xFinal, const double& time);

  /** Instantaneously set a component to a given position.
   * @param id integer index.
   * @param xVal The desired position of the component.
   */
  void set(const int& id, const double& xVal);

  /** Start an infinite sinusoidal trajectory.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max
   * amplitude of the sinusoid.
   * @param time The time to go from the current position to xFinal [sec].
   */
  void startSinusoid(const int& id, const double& xFinal, const double& time);

  /** Start an infinite triangle trajectory.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max
   * amplitude of the trajectory.
   * @param time The time to go from the current position to xFinal [sec].
   */
  // void startTriangle(const int& id, const double& xFinal, const double& time,
  // const double& Tacc);

  /** Start an infinite trajectory with piece-wise constant acceleration.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max
   * amplitude of the trajectory.
   * @param time The time to go from the current position to xFinal [sec].
   * @param Tacc The time during witch acceleration is keept constant [sec].
   */
  void startConstAcc(const int& id, const double& xFinal, const double& time);

  /** Start a linear-chirp trajectory, that is a sinusoidal trajectory with
   * frequency being a linear function of time.
   * @param id integer index.
   * @param xFinal The position of the component corresponding to the max
   * amplitude of the sinusoid [rad].
   * @param f0 The initial (min) frequency of the sinusoid [Hz]
   * @param f1 The final (max) frequency of the sinusoid [Hz]
   * @param time The time to get from f0 to f1 [sec]
   */
  void startLinearChirp(const int& id, const double& xFinal, const double& f0,
                        const double& f1, const double& time);

  /** Stop the motion of the specified component. If id is -1
   * it stops the trajectory of all the vector.
   * @param id integer index.
   * */
  void stop(const int& id);

  /* --- ENTITY INHERITANCE --- */
  virtual void display(std::ostream& os) const;

 protected:
  enum JTG_Status {
    JTG_STOP,
    JTG_SINUSOID,
    JTG_MIN_JERK,
    JTG_LIN_CHIRP,
    // JTG_TRIANGLE,
    JTG_CONST_ACC,
    JTG_TEXT_FILE,
    JTG_SPLINE
  };

  bool
      m_initSucceeded;  /// true if the entity has been successfully initialized
  bool m_firstIter;     /// true if it is the first iteration, false otherwise
  double m_dt;          /// control loop time step.
  double m_t;           /// current control loop time.
  unsigned int m_n;     /// size of ouput vector
  unsigned int m_iterLast;  /// last iter index
  bool m_splineReady;       /// true if the spline has been successfully loaded.

  std::vector<JTG_Status> m_status;  /// status of the component
  std::vector<
      parametriccurves::AbstractCurve<double, dynamicgraph::sot::Vector1d>*>
      m_currentTrajGen;
  std::vector<parametriccurves::Constant<double, 1>*> m_noTrajGen;
  std::vector<parametriccurves::MinimumJerk<double, 1>*> m_minJerkTrajGen;
  std::vector<parametriccurves::InfiniteSinusoid<double, 1>*> m_sinTrajGen;
  std::vector<parametriccurves::LinearChirp<double, 1>*> m_linChirpTrajGen;
  // std::vector<parametriccurves::InfiniteTriangular<double,1>* >
  // m_triangleTrajGen;
  std::vector<parametriccurves::InfiniteConstAcc<double, 1>*> m_constAccTrajGen;
  parametriccurves::TextFile<double, Eigen::Dynamic>* m_textFileTrajGen;
  parametriccurves::Spline<double, Eigen::Dynamic>* m_splineTrajGen;

};  // class NdTrajectoryGenerator

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // #ifndef __sot_talos_balance_nd_trajectory_generator_H__
