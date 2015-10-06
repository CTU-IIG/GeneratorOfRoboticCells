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
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include "Settings.h"
#include "XmlWriter.h"

using namespace std;

ostream& operator<<(ostream& out, const Indent& indent)	{
	out<<string(4*indent.mIndent, ' ');
	return out;
}

void XmlWriter::writeXmlFile(string filename) {
	ofstream ofile(filename.c_str());
	if (!ofile)	{
		throw invalid_argument("bool XmlWriter::writeXmlFile(...): Cannot open the output file!");
	}
	ofile.setf(ios::fixed, ios::floatfield);

	if (Settings::VERBOSE && !mLines.empty())
		cout<<"Writing instances to xml file:"<<endl;

	uint32_t lineId = 0;
	ofile<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<<endl;
	ofile<<"<dataset xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"dataset.xsd\">"<<endl;
	if (!Settings::DATASET_TITLE.empty())
		ofile<<Indent(1)<<"<name>"<<Settings::DATASET_TITLE<<"</name>"<<endl;

	for (RoboticLine *line : mLines)	{
		writeInstanceXmlPart(ofile, line, lineId++, 1);
		if (Settings::VERBOSE)	{
			cout<<" 0 % ["<<setfill('#')<<setw(50)<<string(50*(mLines.size()-lineId)/mLines.size(),' ')<<"] ";
			cout<<setprecision(2)<<fixed<<100.0*lineId/mLines.size()<<" %"<<'\r';
			cout.flush();
		}
	}
	ofile<<"</dataset>"<<endl;

	if (Settings::VERBOSE && !mLines.empty())
		cout<<endl;

	ofile.close();
}

void XmlWriter::writeInstanceXmlPart(ostream& out, RoboticLine* line, const uint32_t& lineId, uint32_t length) {
	out<<Indent(length++)<<"<instance>\n";

	out<<Indent(length)<<"<name>Robotic line "<<lineId<<"</name>\n";

	out<<Indent(length++)<<"<robots>\n";
	for (uint32_t r = 0; r < line->robots().size(); ++r)
		writeRobotXmlPart(out, line->robots()[r], r, length);
	out<<Indent(--length)<<"</robots>\n";

	writeInterRobotOperationsXmlPart(out, line, length);
	writeCollisionsXmlPart(out, line, length);

	out<<Indent(length)<<"<production-cycle-time>"<<line->productionCycleTime()<<"</production-cycle-time>\n";

	out<<Indent(--length)<<"</instance>"<<endl;
}

void XmlWriter::writeRobotXmlPart(ostream& out, Robot* robot, const uint32_t& robotId, uint32_t length) {
	out<<Indent(length++)<<"<robot>\n";

	out<<Indent(length)<<"<name>Robot "<<robotId<<"</name>\n";

	out<<Indent(length++)<<"<activities>\n";
	for (Activity *a : robot->activities())	{
		if (a->type() != MOVEMENT)	{
			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)
				writeStaticActivityXmlPart(out, sa, length);
			else
				throw runtime_error("void XmlWriter::writeRobotXmlPart(...): Invalid cast to the static activity!");
		}
	}

	for (Activity *a : robot->activities())	{
		if (a->type() == MOVEMENT)	{
			DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)
				writeDynamicActivityXmlPart(out, da, length);
			else
				throw runtime_error("void XmlWriter::writeRobotXmlPart(...): Invalid cast to the dynamic activity!");
		}
	}
	out<<Indent(--length)<<"</activities>\n";

	out<<Indent(length++)<<"<power-saving-modes>\n";
	const vector<RobotPowerMode*>& rpm = robot->robotModes();
	for (uint32_t p = 0; p < rpm.size(); ++p)	{
		out<<Indent(length++)<<"<power-mode pid=\""<<p<<"\">\n";
		out<<Indent(length)<<"<name>"<<rpm[p]->powerSaveModeName()<<"</name>\n";
		out<<Indent(length)<<"<minimal-idle-time>"<<rpm[p]->minimalDelay()<<"</minimal-idle-time>\n";
		if (rpm[p]->expectedInputPower() != -1.0)
			out<<Indent(length)<<"<expected-input-power>"<<rpm[p]->expectedInputPower()<<"</expected-input-power>\n";
		out<<Indent(--length)<<"</power-mode>\n";
	}
	out<<Indent(--length)<<"</power-saving-modes>\n";


	out<<Indent(--length)<<"</robot>\n";
}

void XmlWriter::writeStaticActivityXmlPart(ostream& out, StaticActivity* a, uint32_t length) {
	out<<Indent(length++)<<"<static-activity aid=\""<<a->aid()<<"\""<<(a->type() == WAIT ? " last_in_cycle=\"true\"" : "")<<">\n";

	out<<Indent(length)<<"<min-duration>"<<a->minAbsDuration()<<"</min-duration>\n";
	out<<Indent(length)<<"<max-duration>"<<a->maxAbsDuration()<<"</max-duration>\n";

	uint32_t modeId = 0;
	out<<Indent(length++)<<"<locations>\n";
	for (Location* pt : a->locations())	{
		out<<Indent(length++)<<"<location lid=\""<<modeId++<<"\">\n";
		out<<Indent(length)<<"<point>"<<pt->point()<<"</point>\n";

		if (!pt->locationDependentPowerConsumption().empty())	{
			out<<Indent(length++)<<"<location-dependent-power-consumption>\n";
			for (const LocationDependentPowerConsumption& ld : pt->locationDependentPowerConsumption())
				out<<Indent(length)<<"<consumption pid=\""<<ld.robotModeRef()->pid()<<"\" input_power=\""<<ld.inputPower()<<"\" />\n";
			out<<Indent(--length)<<"</location-dependent-power-consumption>\n";
		}

		out<<Indent(--length)<<"</location>\n";
	}
	out<<Indent(--length)<<"</locations>\n";

	out<<Indent(--length)<<"</static-activity>\n";
}

void XmlWriter::writeDynamicActivityXmlPart(ostream& out, DynamicActivity* a, uint32_t length) {
	out<<Indent(length++)<<"<dynamic-activity aid=\""<<a->aid()<<"\">\n";

	uint32_t modeId = 0;
	Activity *from = nullptr, *to = nullptr;

	if (!a->predecessors().empty() && !a->successors().empty())	{
		from = a->predecessors().front();
		to = a->successors().front();
	} else {
		throw runtime_error("void XmlWriter::writeDynamicActivityXmlPart(...): Predecessors and successors of the activity are unset!");
	}

	out<<Indent(length++)<<"<movements from_aid=\""<<from->aid()<<"\" to_aid=\""<<to->aid()<<"\">\n";
	for (Movement* mv : a->movements())	{
		out<<Indent(length++)<<"<movement mid=\""<<modeId++<<"\">\n";

		out<<Indent(length)<<"<from-point>"<<mv->from()<<"</from-point>\n";
		out<<Indent(length)<<"<to-point>"<<mv->to()<<"</to-point>\n";
		out<<Indent(length)<<"<min-duration>"<<mv->minDuration()<<"</min-duration>\n";
		out<<Indent(length)<<"<max-duration>"<<mv->maxDuration()<<"</max-duration>\n";

		out<<Indent(length++)<<"<energy-function>\n";
		for (const Monomial& m : mv->energyFunction())
			out<<Indent(length)<<"<monomial degree=\""<<m.degree<<"\" coeff=\""<<m.coeff<<"\"/>\n";
		out<<Indent(--length)<<"</energy-function>\n";

		out<<Indent(--length)<<"</movement>\n";
	}
	out<<Indent(--length)<<"</movements>\n";

	out<<Indent(--length)<<"</dynamic-activity>\n";
}

void XmlWriter::writeInterRobotOperationsXmlPart(ostream& out, RoboticLine* line, uint32_t length) {

	if (!line->interRobotOperations().empty())	{
		out<<Indent(length++)<<"<inter-robot-operations>\n";
		for (const InterRobotOperation& op : line->interRobotOperations())	{

			out<<Indent(length++)<<"<operation oid=\""<<op.oid()<<"\">\n";
			out<<Indent(length)<<"<name>"<<op.name()<<"</name>\n";

			out<<Indent(length++)<<"<time-compatibility>\n";
			for (const TimeLag& e : op.timeLags())	{
				out<<Indent(length++)<<"<time-lag>\n";
				out<<Indent(length)<<"<from-activity>"<<e.from()->aid()<<"</from-activity>\n";
				out<<Indent(length)<<"<to-activity>"<<e.to()->aid()<<"</to-activity>\n";
				out<<Indent(length)<<"<length>"<<e.length()<<"</length>\n";
				out<<Indent(length)<<"<height>"<<e.height()<<"</height>\n";
				out<<Indent(--length)<<"</time-lag>\n";
			}
			out<<Indent(--length)<<"</time-compatibility>\n";

			if (!op.spatialCompatibility().empty())	{
				out<<Indent(length++)<<"<spatial-compatibility>\n";
				for (const Pair& spc : op.spatialCompatibility())	{
					out<<Indent(length++)<<"<compatible-pair>\n";
					out<<Indent(length)<<"<location aid=\""<<spc.activity1()->aid()<<"\" lid=\""<<spc.mode1()<<"\" />\n";
					out<<Indent(length)<<"<location aid=\""<<spc.activity2()->aid()<<"\" lid=\""<<spc.mode2()<<"\" />\n";
					out<<Indent(--length)<<"</compatible-pair>\n";
				}
				out<<Indent(--length)<<"</spatial-compatibility>\n";
			}

			out<<Indent(--length)<<"</operation>\n";
		}
		out<<Indent(--length)<<"</inter-robot-operations>\n";
	}
}

void XmlWriter::writeCollisionsXmlPart(std::ostream& out, RoboticLine* line, uint32_t length)	{
	if (!line->collisions().empty())	{
		out<<Indent(length++)<<"<collision-zones>\n";
		for (const Pair& c : line->collisions())	{
			out<<Indent(length++)<<"<collision-pair>\n";
			vector<pair<Activity*, uint32_t> > pairs = {{c.activity1(), c.mode1()}, {c.activity2(), c.mode2()}};

			for (const auto& p : pairs)	{
				if (p.first->type() == MOVEMENT)
					out<<Indent(length)<<"<movement aid=\""<<p.first->aid()<<"\" mid=\""<<p.second<<"\" />\n";
				else
					out<<Indent(length)<<"<location aid=\""<<p.first->aid()<<"\" lid=\""<<p.second<<"\" />\n";
			}

			out<<Indent(--length)<<"</collision-pair>\n";
		}
		out<<Indent(--length)<<"</collision-zones>\n";
	}
}

