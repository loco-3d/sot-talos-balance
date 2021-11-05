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

#include "sot/talos_balance/test-cython.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>
#include <sot/core/stop-watch.hh>
#include "fooClass_api.h"

namespace dynamicgraph {
namespace sot {
namespace talos_balance {
namespace dg = ::dynamicgraph;
using namespace dg;
using namespace dg::command;

// Size to be aligned                         "-------------------------------------------------------"
#define PROFILE_NMPC_SUM_COMPUTATION "TestCython: sum computation                               "
#define PROFILE_NMPC_NBJOINTS_COMPUTATION "TestCython: nbJoints extraction                           "

#define INPUT_SIGNALS m_aSIN << m_bSIN

#define OUTPUT_SIGNALS m_cSOUT

/// Define EntityClassName here rather than in the header file
/// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
typedef TestCython EntityClassName;

/* --- DG FACTORY ---------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TestCython, "TestCython");

/* ------------------------------------------------------------------- */
/* --- CONSTRUCTION -------------------------------------------------- */
/* ------------------------------------------------------------------- */
TestCython::TestCython(const std::string& name)
    : Entity(name),
      CONSTRUCT_SIGNAL_IN(a, double),
      CONSTRUCT_SIGNAL_IN(b, double),
      CONSTRUCT_SIGNAL_OUT(c, double, INPUT_SIGNALS),
    m_initSucceeded(false) {
  Entity::signalRegistration(INPUT_SIGNALS << OUTPUT_SIGNALS);

  /* Commands. */
  addCommand("init", makeCommandVoid0(*this, &TestCython::init, docCommandVoid0(
    "initialize the entity.")));
}

void TestCython::init() {
  if (!m_aSIN.isPlugged()) return SEND_MSG("init failed: signal velocityref is not plugged", MSG_TYPE_ERROR);
  if (!m_bSIN.isPlugged())
    return SEND_MSG("init failed: signal trigger is not plugged", MSG_TYPE_ERROR);

  m_a = m_aSIN;
  m_b = m_bSIN;

  m_initSucceeded = true;
}

double TestCython::sum(double d){
  return m_a+m_b+d;
}

double TestCython::sumCython(int argc, char *argv[]){
    wchar_t *program;

    program = Py_DecodeLocale(argv[0], NULL);
    if (program == NULL) {
        fprintf(stderr, "Fatal error: cannot decode argv[0], got %d arguments\n", argc);
        exit(1);
    }

    // Add a built-in module, before Py_Initialize
    if (PyImport_AppendInittab("fooClass", PyInit_fooClass) == -1) {
        fprintf(stderr, "Error: could not extend in-built modules table\n");
        exit(1);
    }

    // Pass argv[0] to the Python interpreter
    Py_SetProgramName(program);

    // Initialize the Python interpreter.  Required.
    //   If this step fails, it will be a fatal error.
    Py_Initialize();

    // Optionally import the module; alternatively,
    //   import can be deferred until the b script
    //   imports it.
    PyRun_SimpleString(
       "import sys\n"
       "sys.path.append('')\n"
    );

    import_fooClass();

    Foo *foo = buildFoo(m_a,m_b);
    double d = foobar(foo,2.0);

    // Clean up after using CPython.
    PyMem_RawFree(program);
    Py_Finalize();

    return d;
}

/* ------------------------------------------------------------------- */
/* --- SIGNALS ------------------------------------------------------- */
/* ------------------------------------------------------------------- */

DEFINE_SIGNAL_OUT_FUNCTION(c, double) {
  if (!m_initSucceeded) {
    SEND_WARNING_STREAM_MSG("Cannot compute output signal before initialization!");
    return s;
  }
  //s = sumCython(1,NULL);
  s = sum(2);
  return s;
}

/* --- COMMANDS ---------------------------------------------------------- */

/* ------------------------------------------------------------------- */
/* --- ENTITY -------------------------------------------------------- */
/* ------------------------------------------------------------------- */

void TestCython::display(std::ostream& os) const {
  os << "TestCython " << getName();
  try {
    getProfiler().report_all(3, os);
  } catch (ExceptionSignal e) {
  }
}

}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph
