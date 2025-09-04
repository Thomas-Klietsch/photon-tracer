#pragma once

#include <tuple>

#include "../colour/colour.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"

namespace Emitter
{

	enum class Type
	{
		Area,
		Directional,
		Environment,
		Point
	};

	class Polymorphic
	{

	public:

		// Returns:
		// Energy, point on emitter, direction from emitter,
		// pdf_W, pdf_A
		virtual std::tuple <Colour, Double3, Double3, std::float_t, std::float_t> emit(
			Random::Mersenne& prng
		) const = 0;

		virtual Emitter::Type type() const = 0;

		// True for emitters than can not be intersected (point/directional)
		virtual bool is_dirac() const = 0;

	};

};
