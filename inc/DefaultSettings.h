/*
	This file is part of the GeneratorOfDatasets program.

	GeneratorOfDatasets is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	GeneratorOfDatasets is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with GeneratorOfDatasets. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef HLIDAC_PES_DEFAULT_SETTINGS_H
#define HLIDAC_PES_DEFAULT_SETTINGS_H

/*!
 * \file DefaultSettings.h
 * \author Libor Bukata
 * \brief Default configuration of the generator.
 */

#include <string>
#include "GeneratorConfig.h"

//! Default number of instances to be generated.
#define DEFAULT_NUMBER_OF_PROJECTS 1
//! Path to the default parameters for the project instances.
#define DEFAULT_PROJECT_CONFIGURATION_FILE std::string(INSTALL_PATH)+"/share/generator-"+std::string(PROGRAM_VERSION)+"/project_configuration.cfg"

#endif

