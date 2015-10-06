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
#ifndef HLIDAC_PES_SETTINGS_H
#define HLIDAC_PES_SETTINGS_H

/*!
 * \file Settings.h
 * \author Libor Bukata
 * \brief It declares the namespace for program settings.
 */

#include <string>
#include "DefaultSettings.h"

namespace Settings {
	//! Number of instances to be generated.
	extern int NUMBER_OF_INSTANCES;
	//! Input file with the desired properties of instances.
	extern std::string PROJECT_CONFIGURATION_FILE;
	//! The name of the output xml file where the generated instances will be written.
	extern std::string OUTPUT_FILE;
	//! Title for the generated dataset.
	extern std::string DATASET_TITLE;
	//! Boolean value determining whether additional information (e.g. progress bar, total time) will be printed in the console.
	extern bool VERBOSE;
}

#endif

