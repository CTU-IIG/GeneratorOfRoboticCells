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
#ifndef HLIDAC_PES_XML_WRITER_H
#define HLIDAC_PES_XML_WRITER_H

/*!
 * \file XmlWriter.h
 * \author Libor Bukata
 * \brief It primarily defines XmlWriter class for writing the generated instances to the file.
 */

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <utility>
#include "Settings.h"
#include "RoboticLine.h"

/*!
 * \struct Indent
 * \brief Auxiliary structure corresponding to the tabulator in an xml file.
 */
struct Indent	{
	Indent(const uint32_t& indent) : mIndent(indent) { }
	uint32_t mIndent;
};

/*!
 * \param out Output stream to which the white space of the specified length is written.
 * \param indent The length of the white space is represented by this structure.
 * \brief It writes the white space to the output. It is an auxiliary method for XmlWriter class.
 */
std::ostream& operator<<(std::ostream& out, const Indent& indent);

/*!
 * Instance of XmlWriter class reads dataset of robotic cells
 * and writes them in the form of a well-structured xml file.
 * \brief It writes problem instances to the xml file.
 */
class XmlWriter {
	public:
		/*!
		 * \param lines Robotic cells to be written to the xml file.
		 * \brief It initialises the XmlWriter instance by the given robotic cells.
		 */
		XmlWriter(const std::vector<RoboticLine*>& lines) : mLines(lines) { }

		/*!
		 * \param filename The filename of the output xml file.
		 * \brief It transforms the instances of the robotic cells to the form of the xml file.
		 */
		void writeXmlFile(std::string filename = Settings::OUTPUT_FILE);

	private:

		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param line Robotic cell to be transformed to the text stream.
		 * \param lineId Identification of the robotic cell according to the order.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to one robotic cell.
		 */
		void writeInstanceXmlPart(std::ostream& out, RoboticLine* line, const uint32_t& lineId, uint32_t length);
		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param robot The robot to be transformed to the text stream.
		 * \param robotId Integer identification of the robot.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to one robot.
		 */
		void writeRobotXmlPart(std::ostream& out, Robot* robot, const uint32_t& robotId, uint32_t length);
		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param a The static activity to be transformed to the text stream.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to one static activity.
		 */
		void writeStaticActivityXmlPart(std::ostream& out, StaticActivity* a, uint32_t length);
		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param a The dynamic activity to be transformed to the text stream.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to one dynamic activity.
		 */
		void writeDynamicActivityXmlPart(std::ostream& out, DynamicActivity* a, uint32_t length);
		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param line Inter-robot operations of this robotic cell will be transformed to the text stream.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to the inter-robot operations.
		 */
		void writeInterRobotOperationsXmlPart(std::ostream& out, RoboticLine* line, uint32_t length);
		/*!
		 * \param out Output stream to which the text stream will be written.
		 * \param line Spatial collisions of this robotic cell will be transformed to the text stream.
		 * \param length Initial indentation for each new line in this xml part.
		 * \brief It writes a part of the xml file corresponding to the spatial collisions.
		 */
		void writeCollisionsXmlPart(std::ostream& out, RoboticLine* line, uint32_t length);

		//! Robotic cells to be written to the xml file.
		std::vector<RoboticLine*> mLines;
};

#endif

