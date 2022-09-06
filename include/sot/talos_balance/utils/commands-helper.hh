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

#ifndef __sot_talos_balance_commands_helper_H__
#define __sot_talos_balance_commands_helper_H__

#include <boost/function.hpp>

/* --- COMMON INCLUDE -------------------------------------------------- */
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command.h>

/* --- HELPER ---------------------------------------------------------- */
namespace dynamicgraph {
namespace sot {
namespace talos_balance {
using ::dynamicgraph::command::docCommandVerbose;
using ::dynamicgraph::command::docCommandVoid0;
using ::dynamicgraph::command::docCommandVoid1;
using ::dynamicgraph::command::docCommandVoid2;
using ::dynamicgraph::command::docCommandVoid3;
using ::dynamicgraph::command::docCommandVoid4;
using ::dynamicgraph::command::docCommandVoid5;
using ::dynamicgraph::command::docCommandVoid6;
using ::dynamicgraph::command::docCommandVoid7;
using ::dynamicgraph::command::docCommandVoid8;
using ::dynamicgraph::command::docDirectGetter;
using ::dynamicgraph::command::docDirectSetter;
using ::dynamicgraph::command::makeCommandVerbose;
using ::dynamicgraph::command::makeCommandVoid0;
using ::dynamicgraph::command::makeCommandVoid1;
using ::dynamicgraph::command::makeCommandVoid2;
using ::dynamicgraph::command::makeCommandVoid3;
using ::dynamicgraph::command::makeCommandVoid4;
using ::dynamicgraph::command::makeCommandVoid5;
using ::dynamicgraph::command::makeCommandVoid6;
using ::dynamicgraph::command::makeCommandVoid7;
using ::dynamicgraph::command::makeCommandVoid8;
using ::dynamicgraph::command::makeDirectGetter;
using ::dynamicgraph::command::makeDirectSetter;
}  // namespace talos_balance
}  // namespace sot
}  // namespace dynamicgraph

#endif  // __sot_talos_balance_commands_helper_H__
