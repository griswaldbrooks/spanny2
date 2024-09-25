// C++ Standard Library
#include <vector>
#include <tuple>

// Gtest
#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Code to test
#include "geometry/raytrace.hpp"

using ::testing::ElementsAre;
using ::testing::ElementsAreArray;
using ::testing::UnorderedElementsAreArray;

namespace geometry {

/**
* @brief Overloads the << operator to print out the Cells in a vector of cells
* @param os The ostream object
* @param pixels The vector of cells to be printed
* @returns reference to the ostream object
*/	
std::ostream& operator<<(std::ostream& os, const std::vector<Cell>& pixels){
	os << "\n";
	for (const auto& pixel:pixels) {
		os << "(" << pixel.x << "," << pixel.y << ")\n";
	}
	os << "\n";
	return os;
}

/**
* @brief Overloads the == operator to allow equality check between two Cell objects
* @param lhs The cell on the left side of the == operator
* @param rhs The cell on the right side of the == operator
* @returns boolean on if the x and y values of the Cell are equal
*/	
bool operator==(const Cell &lhs,const Cell &rhs) {
	return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}


/**
* @brief Overloads the == operator to allow equality check between pixel return of the overloaded bresenham function
* @param lhs The "cell" on the left side of the == operator
* @param rhs The cell on the right side of the == operator
* @returns boolean on if the x and y values of the pixels are equal
*/	
bool operator==(const std::vector<int> &lhs,const Cell &rhs) {
	return (lhs[0] == rhs.x) && (lhs[1] == rhs.y);
}


TEST(bresenham_conversion, overloaded_function_check){
	// GIVEN two endpoints
	const Cell start {1, -2};
	const Cell end {6, 8};

	// WHEN the pixels are produced via the two bresenham functions
	const auto pixels_1 = bresenham_conversion(start.x, start.y, end.x, end.y);
	const auto pixels_2 = bresenham_conversion(start, end);

	// THEN the two vectors should be the same size and be equal to each other
	EXPECT_EQ(pixels_1.size(), pixels_2.size());
	EXPECT_THAT(pixels_1, ElementsAreArray(pixels_2));

}

TEST(bresenham_conversion, front_and_back_cells) {
	// GIVEN horizontal line endpoints
	const Cell start {0,0};
	const Cell end {5,0};

	// WHEN the interpolated pixels are produced
	const auto pixels = bresenham_conversion(start, end);

	// THEN the resulting line should have 6 pixels and begin at (0,0) and end at (5,0)
	EXPECT_EQ(pixels.size(), 6);
	EXPECT_EQ(pixels.front(), Cell(0,0));
	EXPECT_EQ(pixels.back(), Cell(5,0));
}


TEST(bresenham_conversion, horz_line_going_right) {
	// GIVEN a horizontal line endpoints that go from left to right
	const Cell start {0,0};
	const Cell end {5,0};	

	// WHEN the interpolated pixels are produced
	const auto pixels = bresenham_conversion(start, end);
		
	// THEN the resulting line has 6 elements, which should be equal to the expected vector
	EXPECT_EQ(pixels.size(),6);
	const std::vector<Cell> expected{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, horz_line_going_left) {
	// GIVEN a horizontal line endpoints that goes from right to left
	const Cell start {4,5};
	const Cell end {-1,5};	

	// WHEN the interpolated pixels are produced
	const auto pixels = bresenham_conversion(start, end);
		
	// THEN the resulting line has 6 elements, which should be equal to the expected vector
	EXPECT_EQ(pixels.size(),6);
	const std::vector<Cell> expected{{4, 5}, {3, 5}, {2, 5}, {1, 5}, {0, 5}, {-1, 5}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, vert_line_going_up) {
	// GIVEN vertical line endpoints that go from bottom to top
	const Cell start {-2,-3};
	const Cell end {-2,2};	

	// WHEN the interpolated pixels are produced
	const auto pixels = bresenham_conversion(start, end);	

	// THEN the resulting line has 6 elements, which should be equal to the expected vector
	EXPECT_EQ(pixels.size(), 6);
	const std::vector<Cell> expected{{-2, -3}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, vert_line_going_down) {
	// GIVEN vertical line endpoints that go from top to bottom
	const Cell start {1,2};
	const Cell end {1,-3};	

	// WHEN the pixels are interpolated from the endpoints
	const auto pixels = bresenham_conversion(start, end);	

	// THEN the resulting line has 6 elements, which should equal to the expected vector
	EXPECT_EQ(pixels.size(), 6);
	const std::vector<Cell> expected{{1, 2}, {1, 1}, {1, 0}, {1, -1}, {1, -2}, {1, -3}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, point_check) {
	// GIVEN the endpoints of a point
	const Cell start {0,0};
	const Cell end {0,0};

	// WHEN the pixels are interpolated from the endpoints
	const auto pixels = bresenham_conversion(start, end);

	// THEN there should be one pixel produced, which should equal to the expected vector
	EXPECT_THAT(pixels.size(), 1);
	const std::vector<Cell> expected{{0,0}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, line_going_up_and_right) {
	// GIVEN the endpoints of a line that are going to the up and right
	const Cell start {0,0};
	const Cell end {5,3};

	// WHEN the pixels are interpolated from the endpoints
	const auto pixels = bresenham_conversion(start, end);

	// THEN the resulting line has 6 elements and should equal the expected vector
	EXPECT_EQ(pixels.size(), 6);
	const std::vector<Cell> expected {{0, 0}, {1, 1}, {2, 1}, {3, 2}, {4, 2}, {5, 3}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));	
}

TEST(bresenham_conversion, line_going_down_and_left) {
	// GIVEN the endpoints of a line that is going down and to the left
	const Cell start {5,3};
	const Cell end {0,0};

	// WHEN the pixels are interpolated from the endpoints
	const auto pixels = bresenham_conversion(start, end);

	// THEN the resulting line has 6 elements and should equal the expected vector
	EXPECT_EQ(pixels.size(), 6);
	const std::vector<Cell> expected {{5, 3}, {4, 2}, {3, 2}, {2, 1}, {1, 1}, {0, 0}};
	EXPECT_THAT(pixels, ElementsAreArray(expected));
}

TEST(raytrace, point_check) {
	// GIVEN the endpoints of a point and a max length
	const Cell startPix = {0, 0};
	const Cell endPix = {0, 0};
	const int max_ray_length = 10; // This is an arbitrary number

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(startPix, endPix, max_ray_length);

	// THEN the resulting line should have one element equal to the expected vector
	EXPECT_EQ(pixels.size(), 1);	
	const std::vector<Cell> expected{{0, 0}};	
	EXPECT_THAT(pixels,ElementsAreArray(expected));
}

TEST(raytrace, horz_line_going_right) {
	// GIVEN the endpoints that go from of left to right and a max length
	const Cell start = {0, 0};
	const Cell end = {5, 0};
	const int max_length = 10; // This is an arbitrary number

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);	

	// THEN the resulting line should have 6 elements equal to the expected vector
	const std::vector<Cell> expected{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};
	EXPECT_EQ(pixels.size(),(6));	
	EXPECT_THAT(pixels,ElementsAreArray(expected));
}

TEST(raytrace, horz_line_going_left) {
	// GIVEN the endpoints that go from right to left and a max length
	const Cell start = {4, 5};
	const Cell end = {-1, 5};
	const int max_length = 10; // This is an arbitrary number

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);	

	// THEN the resulting line should have 6 elements equal to the expected vector
	const std::vector<Cell> expected{{4, 5}, {3, 5}, {2, 5}, {1, 5}, {0, 5}, {-1, 5}};
	EXPECT_EQ(pixels.size(),(6));	
	EXPECT_THAT(pixels,ElementsAreArray(expected));
}

TEST(raytrace, vert_line_going_up) {
	// GIVEN the endpoints that go from bottom to top and a max length
	const Cell start {-2, -3};
	const Cell end {-2, 2};
	const int max_length = 10; // This is an arbitrary number

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);

	// THEN the resulting line should have 6 elements equal to the expected vector
	const std::vector<Cell> expected{{-2, -3}, {-2, -2}, {-2, -1}, {-2, 0}, {-2, 1}, {-2, 2}};
	EXPECT_EQ(pixels.size(),(6));	
	EXPECT_THAT(pixels,ElementsAreArray(expected));
}

TEST(raytrace, vert_line_going_down) {
	// GIVEN the endpoints that go from top to bottom and a max length
	const Cell start = {1, 2};
	const Cell end = {1,-3};
	const int max_length = 10; // This is an arbitrary number

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);

	// THEN the resulting line should have 6 elements equal to the expected vector
	const std::vector<Cell> expected{{1, 2}, {1, 1}, {1, 0}, {1, -1}, {1, -2}, {1, -3}};
	EXPECT_EQ(pixels.size(),(6));	
	EXPECT_THAT(pixels,ElementsAreArray(expected));
}

TEST(raytrace, max_ray_length_test) {
	// GIVEN the endpoints that produce a horizontal line and a max length of 3
	const Cell start = {0, 0};
	const Cell end = {5, 0};
	const int max_length = 3;

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);

	// THEN the resulting line should have 3 elements equal to the expected vector
	const std::vector<Cell> expected{{0, 0}, {1, 0}, {2, 0}};
	EXPECT_EQ(pixels.size(), 3);	
	EXPECT_THAT(pixels, ElementsAreArray(expected));
}

TEST(raytrace, zero_length) {
	// GIVEN the endpoints and a max length of 0, produce an empty container
	const Cell start = {0, 0};
	const Cell end = {5, 0};
	const int max_length = 0;

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);

	// THEN the resulting line should have 3 elements equal to the expected vector
	EXPECT_EQ(pixels.size(), 0);	
}

TEST(raytrace, negative_length) {
	// GIVEN the endpoints and a max length of 0, produce an empty container
	const Cell start = {0, 0};
	const Cell end = {5, 0};
	const int max_length = -5;

	// WHEN the interpolated pixels are produced
	const auto pixels = raytrace(start, end, max_length);

	// THEN the resulting line should have 3 elements equal to the expected vector
	EXPECT_EQ(pixels.size(), 0);	
}

/**
* @brief Creates struct for testing polygonOutline
*/	
struct PolygonScenario{
	std::vector<Cell> vertices;          /**< The vertices of the polygon to create*/
	int size_x;							 /**< Maximum length of a polygon side*/
	int expected_size; 					 /**< Expected number of cells for the outline*/
	std::vector<Cell> expected_elements; /**< Expected cells for the outline*/
};

class PolygonOutlineTests : public :: testing::TestWithParam<PolygonScenario> {};

TEST_P(PolygonOutlineTests, outline_tests){

	const auto curr_scenario = GetParam();

	// GIVEN the vertices of a polygon
	const std::vector<Cell> vertices = curr_scenario.vertices;
	const auto size_x = curr_scenario.size_x;

	// WHEN the outline is created
	const auto outline = polygonOutline(vertices, size_x);

	// THEN the resulting outline should be of expected size and contain the expected elements
	EXPECT_EQ(outline.size(), curr_scenario.expected_size);
	EXPECT_THAT(outline, ElementsAreArray(curr_scenario.expected_elements));		

}

INSTANTIATE_TEST_CASE_P(
	PolygonOutlineGroup,
	PolygonOutlineTests,
	::testing::Values(
		// singular point
		PolygonScenario{{{0,0}}, std::numeric_limits<int>::max(), 1, {{0,0}}},
		// small square
		PolygonScenario{{{0,0},{1,0},{1,1},{0,1}}, std::numeric_limits<int>::max(), 4, {{0,0},{1,0},{1,1},{0,1}}},
		// big square
		PolygonScenario{{{0,0},{5,0},{5,5},{0,5}}, std::numeric_limits<int>::max(), 20, {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{5,1},{5,2},{5,3},{5,4},{5,5},{4,5},{3,5},{2,5},{1,5},{0,5},{0,4},{0,3},{0,2},{0,1}}},
		// tilted square
		PolygonScenario{{{0,0},{4,1},{3,5},{-1,4}}, std::numeric_limits<int>::max(), 16, {{0,0},{1,0},{2,0},{3,1},{4,1},{4,2},{4,3},{3,4},{3,5},{2,5},{1,5},{0,4},{-1,4},{-1,3},{-1,2},{0,1}}}, 
		// weird polygon
		PolygonScenario{{{0,0},{3,2},{6,1},{3,5}}, std::numeric_limits<int>::max(), 15, {{0,0},{1,1},{2,1},{3,2},{4,2},{5,1},{6,1},{5,2},{5,3},{4,4},{3,5},{2,4},{2,3},{1,2},{1,1}}}
		)
	);

/**
* @brief Creates struct for testing convexFill
*/	
struct ConvexScenario
{
  std::string description;
	std::vector<Cell> outline; 			 /**< outline The outline to fill*/
	int expected_size; 					 /**< Expected number of cells for filled polygon*/
	std::vector<Cell> expected_elements; /**< Expected cells for filled polygon*/	
};

std::ostream& operator<<(std::ostream& os, ConvexScenario const& s){
	os << s.description;
	return os;
}

class ConvexFillTests : public ::testing::TestWithParam<ConvexScenario> {};

TEST_P(ConvexFillTests, fill_tests){
	auto const curr_scenario = GetParam();

	// GIVEN a the outline of the polygon, and a size
	auto const& outline = curr_scenario.outline;

	// WHEN the outline is filled in
	auto const filled_cells = convexFill(outline);

	// THEN the resulting area should be xx elements and equal to expected 
	EXPECT_EQ(filled_cells.size(), curr_scenario.expected_size);
	EXPECT_THAT(filled_cells, UnorderedElementsAreArray(curr_scenario.expected_elements));		

}

INSTANTIATE_TEST_CASE_P(
	ConvexFillGroup,
	ConvexFillTests,
	::testing::Values(
		ConvexScenario{"singular point", polygonOutline({{0,0}}, std::numeric_limits<int>::max()), 1, {{0,0}}},
		ConvexScenario{"small square", polygonOutline({{0,0},{1,0},{1,1},{0,1}}, std::numeric_limits<int>::max()), 4, {{0,0},{1,0},{1,1},{0,1}}},
		ConvexScenario{"big square", polygonOutline({{0,0},{5,0},{5,5},{0,5}}, std::numeric_limits<int>::max()), 36, {{0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{0,1},{1,1},{2,1},{3,1},{4,1},{5,1},{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{0,3},{1,3},{2,3},{3,3},{4,3},{5,3},{0,4},{1,4},{2,4},{3,4},{4,4},{5,4},{0,5},{1,5},{2,5},{3,5},{4,5},{5,5}}},
		ConvexScenario{"tilted square", polygonOutline({{0,0},{4,1},{3,5},{-1,4}}, std::numeric_limits<int>::max()), 28, {{0,0},{1,0},{2,0},{0,1},{1,1},{2,1},{3,1},{4,1},{-1,2},{0,2},{1,2},{2,2},{3,2},{4,2},{-1,3},{0,3},{1,3},{2,3},{3,3},{4,3},{-1,4},{0,4},{1,4},{2,4},{3,4},{1,5},{2,5},{3,5}}},
		ConvexScenario{"weird polygon", polygonOutline({{0,0},{3,2},{6,1},{3,5}}, std::numeric_limits<int>::max()), 18, {{0,0},{1,1},{2,1},{5,1},{6,1},{1,2},{2,2},{3,2},{4,2},{5,2},{2,3},{3,3},{4,3},{5,3},{2,4},{3,4},{4,4},{3,5}}}
		)
	);

}  // namespace geometry
