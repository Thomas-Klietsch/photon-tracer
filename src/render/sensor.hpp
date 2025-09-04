#pragma once

#include <memory>
#include <mutex>

#include "../colour/colour.hpp"
#include "../render/config.hpp"

namespace Render
{

	class Sensor final
	{

	private:

		// Data buffers
		std::shared_ptr<Colour[]> p_pixel{ nullptr };
		std::shared_ptr<Colour[]> p_splash{ nullptr };

		std::uint16_t const image_width{ 0 };
		std::uint16_t const image_height{ 0 };

		std::double_t const scalar{ 1. };

		// A terrible, terrible way to make this thread safe. Simple, but terrible.
		// The number of zone locks. If image height is not an integer multiplier of N_ZONE, latency is unbalanced
		// A higher value reduces write queue latency (e.g. the probability that a splash() has to wait).
		// Using at least 2-4 times the amount of CPU threads is suggested
#define N_ZONE 128u

		// Splash data is randomly accessed, when written to
		// Note: mutex is not copyable, i.e. it is a referenced class in BDPT
		std::mutex thread_lock_splash[N_ZONE];

	public:

		Sensor() {};

		Sensor(
			Render::Config const& config
		)
			: image_width( config.image_width )
			, image_height( config.image_height )
			, scalar( 1. / std::max<std::uint16_t>( 1, config.max_samples ) )
		{
			if ( config.max_samples < 1 )
				throw std::invalid_argument( "Invalid config. Needs at least one (1) sample" );
			// Setup sensor data with zeroes (black)
			p_pixel = std::make_shared<Colour[]>( image_width * image_height, Colour::Black );
			p_splash = std::make_shared<Colour[]>( image_width * image_height, Colour::Black );
			// Nullptr if unable to construct
			if ( !p_pixel || !p_splash )
				throw std::invalid_argument( "Out of memory sensor data!" );
		};

		void pixel(
			std::uint16_t const px,
			std::uint16_t const py,
			Colour const& colour
		)
		{
			// Assumes that the method is used in a thread safe manner
			if ( ( px >= image_width ) || ( py >= image_height ) )
				return;
			p_pixel[px + py * image_width] = colour;
		};

		void splash(
			std::uint16_t const px,
			std::uint16_t const py,
			Colour const& colour
		)
		{ // <- start of scope
			if ( ( px >= image_width ) || ( py >= image_height ) )
				return;
			// Make random memory write thread safe, but increases render time,
			// auto unlocks at end of scope
			std::unique_lock<std::mutex> lock_guard( thread_lock_splash[py % N_ZONE] ); // Select lock for zone
			p_splash[px + py * image_width] += colour;
		}; // <- end of scope

		Colour get_colour(
			std::uint16_t const px,
			std::uint16_t const py
		) const
		{
			// Assumes that the method is used in a thread safe manner
			if ( ( px >= image_width ) || ( py >= image_height ) )
				return Colour::Black;
			return ( p_pixel[px + py * image_width] + p_splash[px + py * image_width] ) * scalar;
		};

	};

};
