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
#ifndef HLIDAC_PES_INTERVAL_H
#define HLIDAC_PES_INTERVAL_H

/*!
 * \file Interval.h
 * \author Libor Bukata
 * \brief The file declares the Interval class and its iostream operators.
 */

#include <iostream>
#include <stdexcept>


template <class T>
class Interval;

template <class T>
std::istream& operator>>(std::istream&, Interval<T>&);


/*!
 * \brief The template class for intervals.
 * \tparam T Numeric type.
 */
template <class T>
class Interval {
	public:
		//! Default constructor creates an empty interval.
		Interval() : mFrom(T()), mTo(T()) { }
		/*!
		 * \param f The left endpoint.
		 * \param t The right endpoint.
		 * \brief It constructs the interval [f,t] from two endpoint values.
		 */
		Interval(T f, T t) : mFrom(f), mTo(t) {
			if (mFrom > mTo)
				throw std::invalid_argument("Interval<T>::Interval(T f, T t): Invalid range!");
		}

		T from() const { return mFrom; }
		T to() const { return mTo; }

	private:

		//! The left endpoint of the interval.
		T mFrom;
		//! The right endpoint of the interval.
		T mTo;

		//! The input operator needs to have the access to the private members of the class.
		friend std::istream& operator>> <>(std::istream&, Interval<T>&);
};

/*!
 * \param in Input stream object from which data will be extracted.
 * \param t Instance of the Interval class to be filled from the input stream.
 * \brief Overloading of the input operator for the Interval class.
 */
template <class T>
std::istream& operator>>(std::istream& in, Interval<T>& t)      {
	in>>t.mFrom>>t.mTo;
	if ((in.fail() && !in.eof()) || t.mFrom > t.mTo)
		throw std::invalid_argument("istream& operator>>(istream&, Interval<T>&): Invalid range!");
	return in;
}

/*!
 * \param out Output stream object to which the textual representation of the Interval class will be written.
 * \param t Instance of the Interval class to be written to the output stream.
 * \brief Overloading of the output operator for the Interval class.
 */
template <class T>
std::ostream& operator<<(std::ostream& out, const Interval<T>& t) {
	out<<"<"<<t.from()<<","<<t.to()<<">";
	return out;
}

#endif
