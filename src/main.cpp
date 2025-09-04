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

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <omp.h>

#include "./integrator/lt.hpp"
#include "./random/mersenne.hpp"
#include "./render/camera.hpp"
#include "./render/config.hpp"
#include "./render/save_image.hpp"
#include "./render/scene.hpp"
#include "./render/sensor.hpp"

int main( int argc, char* argv[] )
{
	Render::Config const config(
		400, // image width
		400, // image height
		200, // samples per pixel
		5 // max path trace depth
	);

	Render::Sensor sensor( config );

	// Cornell camera, coordinates for world up using the z axis
	Render::Camera const camera(
		Double3( -278, -800, 273 ), // Camera (lens) position
		Double3( -278, 0, 273 ), // camera look at (target)
		50., // Camera lens focal length, in mm
		config
	);

	Render::Scene const scene;
	if ( !scene.is_valid() )
	{
		std::cout << "Nothing to render, no light and/or object(s)." << std::endl;
		return EXIT_FAILURE;
	}

	// Create an integrator for each thread
	std::vector<std::unique_ptr<Integrator::LT>> integrator;
	for ( std::uint8_t i{ 0 }; i < omp_get_max_threads(); ++i )
		integrator.emplace_back( std::make_unique<Integrator::LT>( camera, sensor, scene, config ) );

	std::cout << "\033[32mRender start\033[0m" << std::endl; // Green text, such luxury. XD
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

	// Lazy arse parallel processing. This is terrible. xD
#pragma omp parallel for
	for ( int y= 0 ; y < config.image_height; ++y )
		for ( int x= 0; x < config.image_width; ++x )
			// Execute thread
			integrator[omp_get_thread_num()]->process( x, y );

	std::chrono::steady_clock::time_point stop_time = std::chrono::steady_clock::now();
	std::chrono::milliseconds total_time = std::chrono::duration_cast<std::chrono::milliseconds>( stop_time - start_time );
	std::cout << "Render time: " << total_time.count() << " millie seconds." << std::endl;

	std::cout << "Saving image." << std::endl;
	if ( !Render::SaveImage( "result", sensor, config ) )
	{
		std::cout << "PANIC! Could not save image." << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Work complete." << std::endl;
	return EXIT_SUCCESS;
};
