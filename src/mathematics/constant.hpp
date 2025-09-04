#pragma once

#include <numbers>

// pi
constexpr std::float_t pi = static_cast<std::float_t>( std::numbers::pi );

// 1 / pi
constexpr std::float_t inv_pi = static_cast<std::float_t>( std::numbers::inv_pi );

// 1 / ( 2 * pi )
constexpr std::float_t inv_2pi = static_cast<std::float_t>( std::numbers::inv_pi * 0.5 );

// 2 * pi
constexpr std::float_t two_pi = static_cast<std::float_t>( std::numbers::pi * 2. );

// square root of 2
constexpr std::float_t sqrt2 = std::numbers::sqrt2;

// Convert degree to radian
constexpr std::float_t deg_to_rad = static_cast<std::float_t>( std::numbers::pi / 180. );
