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
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include "Interval.h"
#include "ProjectParameters.h"
#include "ProjectParametersParser.h"

using namespace std;

ProjectParametersParser::ProjectParametersParser(string inputFile)	{
	ifstream in(inputFile.c_str());
	if (!in)
		throw invalid_argument("ProjectParametersParser(const string&): Cannot open input file \""+inputFile+"\"!");

	string line;
	vector<string> fileLines;
	while (getline(in,line))	{
		bool onlySpaces = true;
		for (const char& c : line)	{
			if (c != ' ' && c != '\t')
				onlySpaces = false;
		}

		if (!onlySpaces && find(line.cbegin(), line.cend(), '#') == line.cend())
			fileLines.push_back(line);
	}
	in.close();

	int lineNumber = 1;
	bool fileParsedCorrectly = true;
	for (const string& line : fileLines)	{
		Interval<double> coeff;
		Interval<double> inputPowerOfMode;
		Interval<double> minDelayOfMode;
		istringstream istr(line, istringstream::in);
		switch (lineNumber)	{
			case 1:
				if (!(istr>>mParsedParameters.numberOfRobots))
					cerr<<"Cannot parse the number of robots!"<<endl;
				break;
			case 2:
				while (istr>>inputPowerOfMode)
					mParsedParameters.inputPowerOfModes.push_back(inputPowerOfMode);

				if (istr.eof())
				       	istr.clear();

				if (mParsedParameters.inputPowerOfModes.empty())
					cerr<<"Cannot read the input power of the power save mode!"<<endl;
				break;
			case 3:
				while (istr>>minDelayOfMode)
					mParsedParameters.minDelayOfModes.push_back(minDelayOfMode);

				if (istr.eof())
				       	istr.clear();

				if (mParsedParameters.minDelayOfModes.empty())
					cerr<<"Cannot read the minimal delays of the robot's static power save modes!"<<endl;
				break;
			case 4:
				if (!(istr>>mParsedParameters.minDurationOfMovement>>mParsedParameters.prolongationOfMovement))
					cerr<<"Cannot read the minimum and maximum time intervals of movements!"<<endl;
				break;
			case 5:
				while (istr>>coeff)
					mParsedParameters.energyFunctionCoefficients.push_back(coeff);

				if (istr.eof())
				       	istr.clear();

				if (mParsedParameters.energyFunctionCoefficients.empty())
					cerr<<"Coefficients of energy functions were not specified!"<<endl;
				break;
			case 6:
				if (!(istr>>mParsedParameters.degreeOfCoefficients))
					cerr<<"Cannot parse the range of powers for coefficients!"<<endl;
				break;
			case 7:
				if (!(istr>>mParsedParameters.minDurationOfOperation>>mParsedParameters.prolongationOfOperation))
					cerr<<"Cannot read the minimum and maximum time intervals of staying in the stationary position!"<<endl;
				break;
			case 8:
				if (!(istr>>mParsedParameters.numberOfPoints))
					cerr<<"Cannot read the number of points interval!"<<endl;
				break;
			case 9:
				if (!(istr>>mParsedParameters.numberOfSequences))
					cerr<<"The average number of sequences for assembling/disassembling operations was not specified!"<<endl;
				break;
			case 10:
				if (!(istr>>mParsedParameters.sequenceLength))
					cerr<<"The interval of the length of the assembling/disassembling sequences was not specified!"<<endl;
				break;
			case 11:
				if (!(istr>>mParsedParameters.minimalVertexDegree))
					cerr<<"The interval of the minimal degree of the vertex cannot be read!"<<endl;
				break;
			case 12:
				if (!(istr>>mParsedParameters.percentageOfTableHandover))
					cerr<<"Cannot read the probability of handovers using tables!"<<endl;
				break;
			case 13:
				if (!(istr>>mParsedParameters.percentageOfRobotsHandover))
					cerr<<"Cannot read the probability of the robot handovers!"<<endl;
				break;
			case 14:
				if (!(istr>>mParsedParameters.numberOfCollisions))
					cerr<<"Cannot read the interval specifying the number of collisions!"<<endl;
				break;
			case 15:
				if (!(istr>>mParsedParameters.dilatationFactor))
					cerr<<"Cannot read the dilatation factor!"<<endl;
				break;
			default:
				clog<<"Ignoring line "<<lineNumber<<"..."<<endl;
		}

		if (istr.fail())	{
			fileParsedCorrectly = false;
			break;
		}
		++lineNumber;
	}

	if (!fileParsedCorrectly || lineNumber <= 15)
		throw runtime_error("ProjectParametersParser(const string&): Error has occured during parsing of the configuration file!");

	if (!checkProjectParameters(mParsedParameters))
		throw runtime_error("ProjectParametersParser(const string&): Invalid value(s) in the config file!");
}

