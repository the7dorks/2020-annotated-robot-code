/**
 * SimplePath.hpp
 *
 * SimplePath is a simple struct that has a list of points
 * on a path represented by ExtendedPoints in a std::vector.
 * This is used for path following by the Drivetrain class.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to objects declared elsewhere (std::vector and ExtendedPoint)
struct SimplePath
{
    std::vector<ExtendedPoint> mpoints;

    ExtendedPoint at(size_t iindex); // gets the point at iindex
    ExtendedPoint last(); // gets the last point
    int size(); // number of points
};