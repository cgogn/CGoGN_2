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

#include <gtest/gtest.h>
#include <cgogn/core/utils/type_traits.h>
#include <vector>
#include <string>

namespace // unnamed namespace for internal linkage
{

struct A1
{
public:
	int operator()(int x) const { return x;}
	static std::string cgogn_name_of_type() { return "A1"; }
};

auto lambda = [](double) -> float { return 0.0f; };

struct Vec
{
	const int& operator[](std::size_t i) const { return data[i]; }
	int& operator[](std::size_t i) { return data[i]; }
	int data[4];
	int size() const { return 4; }
};

struct Mat
{
	const int& operator()(std::size_t i, std::size_t j) const { return data[i][j]; }
	int& operator()(std::size_t i, std::size_t j) { return data[i][j]; }
	int rows() const { return 4; }
	int cols() const { return 4; }
	int data[4][4];
};

TEST(TypeTraitsTest, function_traits)
{
	{
		const auto arity = cgogn::func_arity<decltype (lambda)>::value;
		EXPECT_EQ(arity, 1u);
	}

	{
		const auto arity = cgogn::func_arity<A1>::value;
		EXPECT_EQ(arity, 1u);
	}

	{
		const bool expected_true = cgogn::is_func_parameter_same<A1, int>::value;
		const bool expected_false = cgogn::is_func_parameter_same<A1, void>::value;
		const bool expected_true_return = cgogn::is_func_return_same<A1,int>::value;
		const bool expected_false_return = cgogn::is_func_return_same<A1,float>::value;
		EXPECT_TRUE(expected_true);
		EXPECT_FALSE(expected_false);
		EXPECT_TRUE(expected_true_return);
		EXPECT_FALSE(expected_false_return);
	}

	{
		const bool expected_true = cgogn::is_func_parameter_same<decltype (lambda), double>::value;
		const bool expected_false = cgogn::is_func_parameter_same<decltype (lambda), void>::value;
		const bool expected_true_return = cgogn::is_func_return_same<decltype (lambda),float>::value;
		const bool expected_false_return = cgogn::is_func_return_same<decltype (lambda),double>::value;
		EXPECT_TRUE(expected_true);
		EXPECT_FALSE(expected_false);
		EXPECT_TRUE(expected_true_return);
		EXPECT_FALSE(expected_false_return);
	}
}

TEST(TypeTraitsTest, operator_parenthesis)
{
	{
		const bool expected_false0 = cgogn::has_operator_parenthesis_0<A1>::value;
		const bool expected_true = cgogn::has_operator_parenthesis_1<A1>::value;
		const bool expected_false2 = cgogn::has_operator_parenthesis_2<A1>::value;
		EXPECT_FALSE(expected_false0);
		EXPECT_TRUE(expected_true);
		EXPECT_FALSE(expected_false2);
	}

	{
		const bool expected_false0 = cgogn::has_operator_parenthesis_0<decltype (lambda)>::value;
		const bool expected_true = cgogn::has_operator_parenthesis_1<decltype (lambda)>::value;
		const bool expected_false2 = cgogn::has_operator_parenthesis_2<decltype (lambda)>::value;
		EXPECT_FALSE(expected_false0);
		EXPECT_TRUE(expected_true);
		EXPECT_FALSE(expected_false2);
	}

	{
		const bool expected_true = cgogn::has_operator_parenthesis_2<Mat>::value;
		EXPECT_TRUE(expected_true);
	}
}


TEST(TypeTraitsTest, operator_brackets)
{
	{
		const bool expected_false = cgogn::has_operator_brackets<A1>::value;
		EXPECT_FALSE(expected_false);
	}

	{
		const bool expected_false = cgogn::has_operator_brackets<decltype (lambda)>::value;
		EXPECT_FALSE(expected_false);
	}

	{
		const bool expected_true = cgogn::has_operator_brackets<Vec>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_false = cgogn::has_operator_brackets<Mat>::value;
		EXPECT_FALSE(expected_false);
	}
}

TEST(TypeTraitsTest, size_method)
{
	{
		const bool expected_true = cgogn::has_size_method<Vec>::value;
		EXPECT_TRUE(expected_true);
	}
	{
		const bool expected_true = cgogn::has_size_method<std::vector<int>>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_false = cgogn::has_size_method<Mat>::value;
		EXPECT_FALSE(expected_false);
	}
}

TEST(TypeTraitsTest, rows_cols_methods)
{
	{
		const bool expected_true = cgogn::has_rows_method<Mat>::value;
		EXPECT_TRUE(expected_true);
	}
	{
		const bool expected_true = cgogn::has_cols_method<Mat>::value;
		EXPECT_TRUE(expected_true);
	}
}

TEST(TypeTraitsTest, is_iterable)
{
	{
		const bool expected_true = cgogn::is_iterable<std::vector<float>>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_false = cgogn::is_iterable<Mat>::value;
		EXPECT_FALSE(expected_false);
	}
}

TEST(TypeTraitsTest, nb_components)
{
	std::vector<int> v;
	Mat mat;
	Vec vec;
	EXPECT_EQ(cgogn::nb_components(v), 0u);
	v.resize(4);
	EXPECT_EQ(cgogn::nb_components(v), 4u);
	EXPECT_EQ(cgogn::nb_components(mat), 16u);
}

TEST(TypeTraitsTest, nested_type)
{
	std::vector<int> v;
	Mat mat;
	Vec vec;
	{
		const bool expected_true = std::is_same<int, cgogn::nested_type<std::vector<int>>>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_true = std::is_same<int, cgogn::nested_type<Vec>>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_true = std::is_same<int, cgogn::nested_type<Mat>>::value;
		EXPECT_TRUE(expected_true);
	}


}


TEST(TypeTraitsTest, has_cgogn_name_of_type)
{
	{
		const bool expected_true = cgogn::has_cgogn_name_of_type<A1>::value;
		EXPECT_TRUE(expected_true);
	}

	{
		const bool expected_false = cgogn::has_cgogn_name_of_type<std::vector<float>>::value;
		EXPECT_FALSE(expected_false);
	}
}

}
