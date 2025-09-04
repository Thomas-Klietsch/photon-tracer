#pragma once

#include <cstdint>
#include <tuple>

#include "../bxdf/polymorphic.hpp"

#include "../epsilon.hpp"
#include "../colour/colour.hpp"
#include "../mathematics/constant.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"
#include "../ray/intersection.hpp"
#include "../sample/hemisphere.hpp"

namespace BxDF
{

	// One sided material, diffuse reflector
	class Lambert final : public BxDF::Polymorphic
	{

	private:

		Colour const albedo;

	public:

		Lambert() = delete;

		Lambert(
			Colour const& albedo
		)
			: albedo( albedo )
		{};

		std::tuple<Colour, Double3, BxDF::Event, std::float_t, std::float_t> sample(
			Ray::Intersection const& idata,
			BxDF::TraceMode const trace_mode,
			Random::Mersenne& prng
		) const override
		{
			std::double_t const cos_theta = idata.from_direction.dot( idata.normal_shading );
			if ( cos_theta < EPSILON_COS_THETA )
				return { Colour::Black, Double3::Zero, BxDF::Event::None, 0.f, 0.f };
			Double3 const sample_direction = Sample::HemiSphere( prng ); // Local space
			Double3 const evaluate_direction = idata.orthogonal.to_world( sample_direction );
			return { albedo * inv_pi, evaluate_direction, BxDF::Event::Diffuse, sample_direction.z * inv_pi, sample_direction.z };
		};

		std::tuple<Colour, std::float_t, std::float_t> evaluate(
			Double3 const& evaluate_direction,
			Double3 const& from_direction,
			Ray::Intersection const& idata,
			BxDF::TraceMode const trace_mode
		) const override
		{
			std::double_t const cos_theta = evaluate_direction.dot( idata.orthogonal.normal() );
			std::double_t const from_cos_theta = from_direction.dot( idata.orthogonal.normal() );
			if ( ( cos_theta < EPSILON_COS_THETA ) || ( from_cos_theta < EPSILON_COS_THETA ) )
				return std::tuple( Colour::Black, 0.f, 0.f );
			return std::tuple( albedo * inv_pi, cos_theta * inv_pi, cos_theta );
		};

		Colour factor(
			Double3 const& evaluate_direction,
			Double3 const& from_direction,
			Ray::Intersection const& idata,
			BxDF::TraceMode const trace_mode
		) const override
		{
			std::double_t const cos_theta = evaluate_direction.dot( idata.orthogonal.normal() );
			std::double_t const from_cos_theta = from_direction.dot( idata.orthogonal.normal() );
			if ( ( cos_theta < EPSILON_COS_THETA ) || ( from_cos_theta < EPSILON_COS_THETA ) )
				return Colour::Black;
			return albedo * inv_pi;
		};

		std::float_t pdf(
			Double3 const& evaluate_direction,
			Double3 const& from_direction,
			Ray::Intersection const& idata
		) const override
		{
			std::double_t const eval_cos_theta = evaluate_direction.dot( idata.orthogonal.normal() );
			std::double_t const ray_cos_theta = from_direction.dot( idata.orthogonal.normal() );
			if ( ( eval_cos_theta < EPSILON_COS_THETA ) || ( ray_cos_theta < EPSILON_COS_THETA ) )
				return 0.f;
			return eval_cos_theta * inv_pi;
		};

		std::uint32_t emitter_id() const override
		{
			return UINT32_MAX;
		};

	};

};
