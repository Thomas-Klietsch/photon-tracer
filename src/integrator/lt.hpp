// Copyright (c) 2025 Thomas Klietsch, all rights reserved.
//
// Licensed under the GNU Lesser General Public License, version 3.0 or later
//
// This program is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or ( at your option ) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General
// Public License along with this program.If not, see < https://www.gnu.org/licenses/>. 

#pragma once

#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

#include "../bxdf/polymorphic.hpp"
#include "../bxdf/shading_correction.hpp"
#include "../colour/colour.hpp"
#include "../epsilon.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"
#include "../ray/section.hpp"
#include "../render/camera.hpp"
#include "../render/config.hpp"
#include "../render/scene.hpp"
#include "../render/sensor.hpp"

namespace Integrator
{

	class LT
	{

	private:

		std::uint8_t const max_path_length{ 3 };
		std::uint16_t const max_samples{ 1 };

		Render::Camera const camera;
		Render::Scene const scene;
		Render::Sensor& sensor;

		Random::Mersenne prng;

	public:

		LT() = delete;

		LT(
			Render::Camera const& camera,
			Render::Sensor& sensor,
			Render::Scene const& scene,
			Render::Config const& config
		)
			: camera( camera )
			, sensor( sensor )
			, scene( scene )
			, max_path_length( config.max_path_length )
			, max_samples( config.max_samples )
		{};

		void process(
			std::uint16_t const& x,
			std::uint16_t const& y
		)
		{
			prng = Random::Mersenne( ( ( x + 1 ) * 0x1337 ) + ( ( y + 1 ) * 0xbeef ) );

			for ( std::uint16_t sample{ 0 }; sample < max_samples; ++sample )
				trace_emission_path();
		};

	private:

		void trace_emission_path()
		{
			std::uint32_t const emitter_id = scene.random_emitter( prng );
			auto const [p_emitter, emitter_select_probability]
				= scene.emitter( emitter_id );

			auto const [emitter_factor, emitter_point, emitter_direction, emitter_pdf_W, emitter_pdf_A]
				= p_emitter->emit( prng );

			Colour throughput = emitter_factor / ( emitter_select_probability * emitter_pdf_W * emitter_pdf_A );

			Ray::Section ray( emitter_point, emitter_direction, EPSILON_RAY );
			std::uint8_t depth{ 1 };

			// Random lens point for path connections
			Double3 const lens_point = camera.sample_lens( prng );

			while ( 1 )
			{
				auto [f_hit, hit_distance, idata] = scene.intersect( ray );
				if ( !f_hit )
					return;

				std::shared_ptr<BxDF::Polymorphic> const& p_material = scene.material( idata.material_id );
				auto [bxdf_colour, bxdf_direction, bxdf_event, bxdf_pdf_W, bxdf_cos_theta]
					= p_material->sample( idata, BxDF::TraceMode::Importance, prng );

				switch ( bxdf_event )
				{
					default:
					case BxDF::Event::None:
					case BxDF::Event::Emission:
					{
						return;
					}
					case BxDF::Event::Diffuse:
					{
						// Connect to Camera
						auto [x, y, f_valid]
						 = camera.sensor( idata.point, lens_point );
						if ( f_valid )
						{
							Double3 const delta = idata.point - lens_point;
							Double3 const evaluate_direction = delta.normalise();
							std::double_t const evaluate_distance = delta.magnitude();
							Ray::Section const ray( lens_point, evaluate_direction, EPSILON_RAY );
							if ( !scene.occluded( ray, evaluate_distance - 2. * EPSILON_RAY ) )
							{
								auto const [evaluate_colour, evaluate_pdf_W, evaluate_cos_theta]
									= p_material->evaluate( -evaluate_direction, idata.from_direction, idata, BxDF::TraceMode::Importance );

								sensor.splash( x, y,
									throughput
									* ShadingCorrection( evaluate_direction, idata.from_direction, idata, BxDF::TraceMode::Importance )
									* evaluate_colour
									// Image to surface factor, see Veach
									* evaluate_cos_theta
									* camera.pdf( idata.point, lens_point )
									/ ( evaluate_distance * evaluate_distance )
								);
							}
						}

						// Update path trace
						throughput *= bxdf_colour * ( bxdf_cos_theta / bxdf_pdf_W * ShadingCorrection( bxdf_direction, idata.from_direction, idata, BxDF::TraceMode::Importance ) );
						break;
					}
					case BxDF::Event::Reflect:
					{
						throughput *= bxdf_colour * ShadingCorrection( bxdf_direction, idata.from_direction, idata, BxDF::TraceMode::Importance );
						break;
					}
				} // end switch

				if ( ++depth > max_path_length )
					break;

				ray = Ray::Section( idata.point, bxdf_direction, EPSILON_RAY );
			} // end trace loop

			return;
		};


	}; // end lt class

};
