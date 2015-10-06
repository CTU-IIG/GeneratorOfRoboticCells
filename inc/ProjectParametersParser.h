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
#ifndef HLIDAC_PES_PROJECT_PARAMETERS_PARSER_H
#define HLIDAC_PES_PROJECT_PARAMETERS_PARSER_H

/*!
 * \file ProjectParametersParser.h
 * \author Libor Bukata
 * \brief It declares the class for parsing the desired properties of project instances.
 */

#include <string>
#include "Settings.h"
#include "ProjectParameters.h"

/*!
 * The class parses the project configuration text file
 * and fills the desired properties of instances to the ProjectParameters structure.
 * \brief It reads the desired properties of instances from the text file.
 */
class ProjectParametersParser	{
	public:
		/*!
		 * \param inputFile The project configuration file specifying the desired properties of instances.
		 * \brief It parses the configuration file and fills ProjectParameters structure.
		 * \exception runtime_error Input configuration file is badly formed.
		 */
		ProjectParametersParser(std::string inputFile = Settings::PROJECT_CONFIGURATION_FILE);
		ProjectParameters getParameters() const	{
			return mParsedParameters;
		}
	private:
		//! The structure determining the properties of the generated instances.
		ProjectParameters mParsedParameters;
};

#endif
