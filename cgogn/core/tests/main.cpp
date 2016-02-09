/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
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

#include <iostream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// using ::testing::AtLeast;


class Carte {
public:
	void begin() {
		std::cout << "Carte() " << val << std::endl;
	}
	Carte(int v) {
		val = v;
		std::cout << "constructor Carte() " << val << std::endl;
	}
	Carte(const Carte& c) {
		val = c.val;
		std::cout << "constructor Carte() " << val << std::endl;
	}

protected:
	int val;
};

class Browser1 : Carte {
public:
	void begin() {
		Carte::begin();
		std::cout << "Browser1()" << val << std::endl;
	}
	Browser1(const Carte& c) : Carte(c) {
	}
};

class Browser2 : Carte {
public:
	void begin() {
		Carte::begin();
		std::cout << "Browser2()" << val << std::endl;
	}
	Browser2(const Carte& c) : Carte(c) {
	}
};

int main(int argc, char **argv)
{
	Carte carte(3);

	carte.begin();
	static_cast<Browser1>(carte).begin();
	static_cast<Browser2>(carte).begin();

	testing::InitGoogleTest(&argc, argv);
	// testing::InitGoogleMock(&argc, argv);
	// Set LC_CTYPE according to the environnement variable.
	setlocale(LC_CTYPE, "");

	return 0; //RUN_ALL_TESTS();
}
