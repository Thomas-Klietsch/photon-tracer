#pragma once

#include <cstdint>

namespace Render
{

	struct Config
	{
		// Image resolution
		std::uint16_t image_width{ 32 };
		std::uint16_t image_height{ 32 };
		// Samples per pixels
		std::uint16_t max_samples{ 1 };
		// Number of path vertices
		std::uint8_t max_path_length{ 5 };

		Config() = default;

		Config(
			std::uint16_t const& image_width,
			std::uint16_t const& image_height,
			std::uint16_t const& max_samples,
			std::uint8_t const& max_path_length
		)
			: image_width( image_width )
			, image_height( image_height )
			, max_samples( std::max<std::uint16_t>( 1, max_samples ) )
			, max_path_length( std::max<std::uint16_t>( 3, max_path_length ) )
		{};

	};

};
