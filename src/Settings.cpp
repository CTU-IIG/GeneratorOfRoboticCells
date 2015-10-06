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
#include <string>
#include "Settings.h"
#include "DefaultSettings.h"

using namespace std;

namespace Settings {
	int NUMBER_OF_INSTANCES = (DEFAULT_NUMBER_OF_PROJECTS < 0 ? 0 : DEFAULT_NUMBER_OF_PROJECTS);
	string PROJECT_CONFIGURATION_FILE = DEFAULT_PROJECT_CONFIGURATION_FILE;
	string OUTPUT_FILE = "";
	string DATASET_TITLE = "";
	bool VERBOSE = false;
}

