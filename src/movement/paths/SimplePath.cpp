/**
 * SimplePath.cpp
 *
 * SimplePath is a simple struct that has a list of points
 * on a path represented by ExtendedPoints in a std::vector.
 * This is used for path following by the Drivetrain class.
 */
#include "main.h" // gives access to SimplePath and other dependencies

ExtendedPoint SimplePath::at(size_t iindex) // returns the point at the index
{
    return mpoints.at(iindex);
}
ExtendedPoint SimplePath::last() // returns the point at the end
{
    return mpoints.back();
}
int SimplePath::size() // returns the length of the path
{
    return mpoints.size();
}