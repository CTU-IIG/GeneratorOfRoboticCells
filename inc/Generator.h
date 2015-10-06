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
#ifndef HLIDAC_PES_GENERATOR_H
#define HLIDAC_PES_GENERATOR_H

/*!
 * \file Generator.h
 * \author Libor Bukata
 * \brief The file contains the Generator class.
 */

#include <string>
#include <vector>
#include "Settings.h"
#include "RoboticLine.h"
#include "ProjectParameters.h"

/*!
 * Auxiliary class, encapsulating the RoboticLine and XmlWriter classes, helps to generate and write project instances.
 * It adds the features like text progress bar, time measurement, and so on.
 * \brief It encapsulates the RoboticLine and XmlWriter classes and adds the additional features.
 */
class Generator {
	public:
		Generator() { }
		void setParameters(const ProjectParameters& par)	{
			mParameters = par;
		}

		/*!
		 * \param numberOfProblems The number of instances to be generated.
		 * \brief It generates the specified number of instances and writes them to the xml file.
		 */
		void generateProblems(uint32_t numberOfProblems = Settings::NUMBER_OF_INSTANCES);
	private:
		//! Desired properties of the generated instances.
		ProjectParameters mParameters;
};

#endif
