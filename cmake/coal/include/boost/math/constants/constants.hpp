// The one piece of Boost.Math coal's collision code uses: `boost::math::constants::pi<T>()`.
//
// For adacpp's in-tree build of coal (cmake/deps_coal.cmake), so the build needs no Boost. coal's
// other Boost users (serialization, the assimp mesh loader, the broadphase managers, logging) are not
// compiled. This directory is on coal's include path BEFORE anything else, so a Boost install that
// happens to be on the path (CGAL's, in the native build) is not what coal picks up.
#ifndef ADACPP_COAL_BOOST_MATH_CONSTANTS_HPP
#define ADACPP_COAL_BOOST_MATH_CONSTANTS_HPP

namespace boost::math::constants {

template <class T> constexpr T pi() {
    return static_cast<T>(3.141592653589793238462643383279502884L);
}

} // namespace boost::math::constants

#endif // ADACPP_COAL_BOOST_MATH_CONSTANTS_HPP
