/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *                                                                  *                                                                              *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Web site: http://cgogn.unistra.fr/                                           *
* Contact information: cgogn@unistra.fr                                        *
*                                                                              *
*******************************************************************************/

#ifndef __CORE_BASIC_DART_H__
#define __CORE_BASIC_DART_H__



namespace cgogn
{

struct Dart
{
	unsigned int index;

	Dart(): index(0xffffffff) {}

	static Dart nil() { Dart d; d.index = 0xffffffff; return d; }

	static Dart create(unsigned int i) { Dart d; d.index = i; return d; }

	static std::string CGoGNnameOfType() { return "Dart"; }

	explicit Dart(unsigned int v): index(v) {}

	bool isNil() const { return index == 0xffffffff ; }

	Dart operator=(Dart d) { index = d.index; return *this; }

	bool operator==(Dart d) const { return d.index == index; }

	bool operator!=(Dart d) const { return d.index != index; }

	bool operator<(Dart d) const { return index < d.index; }


	friend std::ostream& operator<<( std::ostream &out, const Dart& fa );
	friend std::istream& operator>>( std::istream &in, Dart& fa );


};


std::ostream& operator<<( std::ostream &out, const Dart& fa ) { return out << fa.index; }
std::istream& operator>>( std::istream &in, Dart& fa ) { in >> fa.index; return in; }


const Dart NIL = Dart::nil();
const unsigned int EMBNULL = 0xffffffff;


}

#endif
