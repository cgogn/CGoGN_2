#include <iostream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// using ::testing::AtLeast;

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	// testing::InitGoogleMock(&argc, argv);
	// Set LC_CTYPE according to the environnement variable.
	setlocale(LC_CTYPE, "");

	return RUN_ALL_TESTS();
}
