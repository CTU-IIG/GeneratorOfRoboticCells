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
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include "ProjectParameters.h"

using namespace std;

bool checkProjectParameters(const ProjectParameters& par)       {

	function<bool(const Interval<double>&)> negativeInterval = [](const Interval<double>& i) { return i.from() < 0.0; };

	if (par.numberOfRobots < 1)
		return false;

	if (par.inputPowerOfModes.size() != par.minDelayOfModes.size() || par.inputPowerOfModes.empty())
		return false;

	if (any_of(par.inputPowerOfModes.cbegin(), par.inputPowerOfModes.cend(), negativeInterval))
		return false;

	if (any_of(par.minDelayOfModes.cbegin(), par.minDelayOfModes.cend(), negativeInterval))
		return false;

	if (par.minDurationOfMovement.from() < 0.0 || par.prolongationOfMovement.from() < 0.0)
		return false;

	if (par.energyFunctionCoefficients.size() != (uint32_t) (par.degreeOfCoefficients.to()-par.degreeOfCoefficients.from()+1))
		return false;

	if (par.numberOfPoints.from() < 1)
		return false;

	if (par.numberOfSequences.from() < 1)
		return false;

	if (par.sequenceLength.from() < 1)
		return false;

	if (par.minimalVertexDegree.from() < 0.0)
		return false;

	if (par.percentageOfTableHandover < 0.0 || par.percentageOfTableHandover > 100.0)
		return false;

	if (par.percentageOfRobotsHandover < 0.0 || par.percentageOfRobotsHandover > 100.0)
		return false;

	if (abs((par.percentageOfRobotsHandover+par.percentageOfTableHandover)-100.0) > 3.0*numeric_limits<double>::epsilon())
		return false;

	if (par.dilatationFactor.from() < 1.0)
		return false;

	return true;
}

