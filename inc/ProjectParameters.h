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
#ifndef HLIDAC_PES_PROJECT_PARAMETERS_H
#define HLIDAC_PES_PROJECT_PARAMETERS_H

/*!
 * \file ProjectParameters.h
 * \author Libor Bukata
 * \brief The file declares the structure for storing the properties of generated instances.
 */

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
#include <stdint.h>
#include "Interval.h"

/*!
 * \brief The structure with desired properties for the generated instances.
 */
struct ProjectParameters	{
	//! The number of robots in the robotic cell.
	uint32_t numberOfRobots;

	//! For each power saving mode of the robot the interval of input power is defined.
	std::vector<Interval<double> > inputPowerOfModes;
	//! The time interval of minimal duration for each power saving mode.
	std::vector<Interval<double> > minDelayOfModes;

	//! The interval of minimal duration of robot movements (defines the fastest movement).
	Interval<double> minDurationOfMovement;
	//! The interval of prolongation for the fastest robot movements.
	Interval<double> prolongationOfMovement;
	//! Intervals of energy function coefficients.
	std::vector<Interval<double> > energyFunctionCoefficients;
	//! The degree of coefficients, e.g. \f$[-1,1]\f$ results to \f$\mathrm{a}*d^{-1}+\mathrm{b}*d^0+\mathrm{c}*d^1\f$ where \f$a,b,c\f$ are energy function coefficients.
	Interval<int32_t> degreeOfCoefficients;

	//! The interval of minimal duration of the robot operation, e.g. welding, assembling, ...
	Interval<double> minDurationOfOperation;
	//! The time interval of the additive part to the minimal operation duration.
	Interval<double> prolongationOfOperation;
	//! The interval of the number of points (i.e. robot configurations) for each robot operation.
	Interval<uint32_t> numberOfPoints;

	//! The interval of the number of atomic sequences in each (dis)assembling/welding/cutting operation.
	Interval<uint32_t> numberOfSequences;
	//! The interval of the number of robot sub-operations for each atomic sequence.
	Interval<uint32_t> sequenceLength;
	//! The interval of the desired average node degree for the graph of robot interconnections.
	Interval<double> minimalVertexDegree;

	//! The probability that the workpiece/weldment is handed over to other robot by using a bench.
	double percentageOfTableHandover;
	//! The probability of the direct gripper-to-gripper handover.
	double percentageOfRobotsHandover;

	//! The interval of the number of generated collisions between robots.
	Interval<uint32_t> numberOfCollisions;

	//! Multiplicative factor by which the lower estimation of the cycle time will be adjusted.
	Interval<double> dilatationFactor;
};

/*!
 * \param par Characteristics of desired instances retrieved from the configuration file.
 * \return It returns true if the data are correct otherwise false.
 * \brief The function checks that the desired properties of instances are logically correct.
 */
bool checkProjectParameters(const ProjectParameters& par);

#endif
