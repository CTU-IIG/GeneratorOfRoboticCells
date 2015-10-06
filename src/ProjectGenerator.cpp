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
/*!
 * \file ProjectGenerator.cpp
 * \author Libor Bukata
 * \brief Entry point of the program, it parses the command line arguments, generates instances, and writes them to the xml file.
 */

#include <algorithm>
#include <iostream>
#include <string>
#include <stdexcept>
#include <sstream>
#include "Settings.h"
#include "Generator.h"
#include "GeneratorConfig.h"
#include "ProjectParameters.h"
#include "ProjectParametersParser.h"

using namespace std;

//! Return codes of the program.
enum ProgramReturnedCodes {
	EXIT_WITH_SUCCESS = 0,
	INVALID_PARAMETER = 1,
	INSUFFICIENT_PARAMETERS = 2,
	INPUT_OUTPUT_ERROR = 4,
	RUNTIME_ERROR = 8,
	UNKNOWN_ERROR = 16
};

//! It prints the program header to the standard output.
void printProgramHeader()	{
	cout<<"Generator of instances for energy optimisation of robotics lines."<<endl;
	cout<<"Author: Libor Bukata and Premysl Sucha"<<endl;
	cout<<"Licence: GNU General Public License"<<endl;
	cout<<"Version: "<<PROGRAM_VERSION<<endl<<endl;
}

/*!
 * \param progName The name of this executable.
 * \brief It prints the program header and help.
 */
void printProgramHelp(const string& progName)	{
	printProgramHeader();
	cout<<"Usage:"<<endl;
	cout<<"\t"<<progName<<" [options+parameters]"<<endl;
	cout<<"Options:"<<endl;
	cout<<"\t--project-configuration-file ARG, -pcf ARG, ARG=FILE"<<endl;
	cout<<"\t\tIt reads the desired parameters of the project from the file."<<endl;
	cout<<"\t--number-of-instances ARG, -noi ARG, ARG=POSITIVE_INTEGER"<<endl;
	cout<<"\t\tThe number of generated instances for the project."<<endl;
	cout<<"\t--output-file ARG, -of ARG, ARG=FILE"<<endl;
	cout<<"\t\tOutput xml file to which the generated instances will be written."<<endl;
	cout<<"\t--set-dataset-title ARG, -sdt ARG, ARG=STRING"<<endl;
	cout<<"\t\tSet short title (<name>ARG</name>) of the dataset."<<endl;
	cout<<"\t--help, -h"<<endl;
	cout<<"\t\tIt prints this program help."<<endl;
	cout<<"\t--verbose"<<endl;
	cout<<"\t\tIt also prints additional information (program header, progress bar, and runtime)."<<endl<<endl;
	cout<<"Default settings can be modified at \"DefaultSettings.h\" file."<<endl;
}

/*!
 * \param argc, argv Arguments given to the program.
 * \return A return code corresponding to the exit status, see \ref ProgramReturnedCodes enum.
 * \brief It handles the program command line arguments, reads desired properties of instances, generates and writes them to the xml file.
 */
int main(int argc, char* argv[])	{

	for (int i = 1; i < argc; ++i)	{

		string arg = argv[i];

		if (arg == "--project-configuration-file" || arg == "-pcf")	{
			if (i+1 < argc)	{
				Settings::PROJECT_CONFIGURATION_FILE = argv[++i];
			} else {
				cerr<<"Project configuration file was not specified!"<<endl;
				return INVALID_PARAMETER;
			}

			continue;
		}

		if (arg == "--number-of-instances" || arg == "-noi")	{
			if (i+1 < argc)	{
				string number = argv[++i];
				istringstream istr(number, istringstream::in);
				istr>>Settings::NUMBER_OF_INSTANCES;
				if (any_of(number.cbegin(), number.cend(), [](char c) { return (c < '0' || c > '9') ? true : false; }) || istr.fail())	{
					cerr<<"Cannot parse '"<<number<<"' to number!"<<endl;
					cout<<"\"--number-of-instances\" invalid parameter!"<<endl;
					return INVALID_PARAMETER;
				}
			} else	{
				cout<<"\"--number-of-instances\" requires the parameter!"<<endl;
			}

			continue;
		}

		if (arg == "--output-file" || arg == "-of")	{
			if (i+1 < argc)	{
				Settings::OUTPUT_FILE = argv[++i];
			} else {
				cerr<<"\"--output-file\" requires the parameter!"<<endl;
				return INVALID_PARAMETER;
			}

			continue;
		}

		if (arg == "--set-dataset-title" || arg == "-sdt")	{
			if (i+1 < argc)	{
				Settings::DATASET_TITLE = argv[++i];
			} else {
				cerr<<"\"--set-dataset-title\" requires the parameter!"<<endl;
				return INVALID_PARAMETER;
			}

			continue;
		}

		if (arg == "--verbose" || arg == "-v")	{
			Settings::VERBOSE = true;
			continue;
		}

		if (arg == "--help" || arg == "-h")	{
			printProgramHelp(argv[0]);
			return EXIT_WITH_SUCCESS;
		}

		cerr<<"Unknown argument \""<<arg<<"\"!"<<endl;
		cerr<<"Check help by using \"-h\" or \"--help\" arguments."<<endl;
		return INVALID_PARAMETER;
	}

	if (Settings::OUTPUT_FILE.empty())	{
		cerr<<"Insufficient parameters, at least the output file has to be specified!"<<endl;
		cerr<<"See the following program help..."<<endl<<endl;
		printProgramHelp(argv[0]);
		return INSUFFICIENT_PARAMETERS;
	}

	try {
		if (Settings::VERBOSE == true)
			printProgramHeader();

		ProjectParametersParser parser;
		ProjectParameters parameters = parser.getParameters();
		Generator generator;
		generator.setParameters(parameters);
		generator.generateProblems();
	} catch (invalid_argument e)	{
		cerr<<e.what()<<endl;
		return INPUT_OUTPUT_ERROR;
	} catch (runtime_error e)	{
		cerr<<e.what()<<endl;
		return RUNTIME_ERROR;
	} catch (exception& e)	{
		cerr<<e.what()<<endl;
		return UNKNOWN_ERROR;
	}

	return EXIT_WITH_SUCCESS;
}
