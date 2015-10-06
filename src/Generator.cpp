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
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
#include <stdint.h>
#include "Generator.h"
#include "XmlWriter.h"

using namespace std;
using namespace std::chrono;

void Generator::generateProblems(uint32_t numberOfProblems)	{

	high_resolution_clock::time_point start = high_resolution_clock::now();

	vector<RoboticLine*> lines;
	if (Settings::VERBOSE && numberOfProblems > 0)
		cout<<"Generation of instances: "<<endl;

	for (uint32_t pn = 0; pn < numberOfProblems; ++pn)	{
		lines.push_back(new RoboticLine(mParameters));
		if (Settings::VERBOSE)	{
			cout<<" 0 % ["<<setfill('#')<<setw(50)<<string(50*(numberOfProblems-pn-1)/numberOfProblems,' ')<<"] ";
			cout<<setprecision(2)<<fixed<<100.0*(pn+1)/numberOfProblems<<" %"<<'\r';
			cout.flush();
		}
	}

	if (Settings::VERBOSE && numberOfProblems > 0)
		cout<<endl<<endl;

	XmlWriter writer(lines);
	writer.writeXmlFile();

	for (uint32_t pn = 0; pn < numberOfProblems; ++pn)	{
		if (!lines[pn]->warnings().empty())	{
			if (Settings::VERBOSE)
				clog<<endl;
			clog<<"Warnings for instance "<<pn<<":"<<endl;
			for (const string& warn : lines[pn]->warnings())
				clog<<warn<<endl;
		}
	}

	for (RoboticLine *r : lines)
		delete r;

	duration<double> runtime = duration_cast<duration<double>>(high_resolution_clock::now()-start);
	if (Settings::VERBOSE)
		cout<<endl<<setprecision(10)<<"Total running time: "<<runtime.count()<<" s"<<endl;
}

