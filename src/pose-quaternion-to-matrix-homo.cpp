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

#include "sot/talos_balance/pose-quaternion-to-matrix-homo.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include "sot/talos_balance/utils/stop-watch.hh"

#include <Eigen/Core>

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

//Size to be aligned                                      "-------------------------------------------------------"
#define PROFILE_POSEQUATERNIONTOMATRIXHOMO_COMPUTATION    "PoseQuaternionToMatrixHomo computation                 "

#define INPUT_SIGNALS     m_sinSIN

#define OUTPUT_SIGNALS m_soutSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef PoseQuaternionToMatrixHomo EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PoseQuaternionToMatrixHomo,
                                         "PoseQuaternionToMatrixHomo");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      PoseQuaternionToMatrixHomo::PoseQuaternionToMatrixHomo(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(sin, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_OUT(sout, MatrixHomogeneous, INPUT_SIGNALS)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid0(*this, &PoseQuaternionToMatrixHomo::init, docCommandVoid0("Initialize the entity.")));
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(sout, MatrixHomogeneous)
      {
        getProfiler().start(PROFILE_POSEQUATERNIONTOMATRIXHOMO_COMPUTATION);

        const dynamicgraph::Vector & vect = m_sinSIN(iter);

        const Eigen::Map<const Eigen::Quaterniond> q(vect.segment<4>(3).data());

        s.translation() = vect.head<3>();
        s.linear() = q.toRotationMatrix();

        getProfiler().stop(PROFILE_POSEQUATERNIONTOMATRIXHOMO_COMPUTATION);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void PoseQuaternionToMatrixHomo::display(std::ostream& os) const
      {
        os << "PoseQuaternionToMatrixHomo " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph

