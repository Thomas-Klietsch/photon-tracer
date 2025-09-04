#pragma once

#include <cmath>
#include <sstream>

// 3D vector, using double
class Double3 final
{

public:

	std::double_t x{ 0.0 };
	std::double_t y{ 0.0 };
	std::double_t z{ 0.0 };

	Double3() {};

	Double3( std::double_t const x, std::double_t const y, std::double_t const z ) : x( x ), y( y ), z( z ) {};

	// Unary minus
	Double3 operator - () const { return Double3( -x, -y, -z ); };

	Double3 operator + ( Double3 const& value ) const { return Double3( x + value.x, y + value.y, z + value.z ); };
	Double3 operator - ( Double3 const& value ) const { return Double3( x - value.x, y - value.y, z - value.z ); };
	Double3 operator * ( std::double_t const value ) const { return Double3( x * value, y * value, z * value ); };
	Double3 operator / ( std::double_t const value ) const { return Double3( x / value, y / value, z / value ); };

	Double3 operator = ( Double3 const& value ) { x = value.x; y = value.y; z = value.z; return *this; };

	Double3 normalise() const { return Double3( x, y, z ) / std::sqrt( x * x + y * y + z * z ); };

	std::double_t absdot( Double3 const& value ) const { return std::abs( x * value.x + y * value.y + z * value.z ); };
	std::double_t dot( Double3 const& value ) const { return x * value.x + y * value.y + z * value.z; };

	Double3 cross( Double3 const& value ) const { return Double3( y * value.z - z * value.y, z * value.x - x * value.z, x * value.y - y * value.x ); };

	std::double_t magnitude() const { return std::sqrt( x * x + y * y + z * z ); };

	friend std::ostream& operator <<( std::ostream& os, Double3 const& value )
	{
		os << "( " << value.x << " , " << value.y << " , " << value.z << " )";
		return os;
	};

	Double3 static const Zero;
	Double3 static const One;
	// Unit vectors
	Double3 static const X;
	Double3 static const Y;
	Double3 static const Z;

};

// ( 0,0,0 )
Double3 const Double3::Zero( 0.0, 0.0, 0.0 );
// ( 1,1,1 )
Double3 const Double3::One( 1.0, 1.0, 1.0 );
// ( 1,0,0 )
Double3 const Double3::X( 1.0, 0.0, 0.0 );
// ( 0,1,0 ) 
Double3 const Double3::Y( 0.0, 1.0, 0.0 );
// ( 0,0,1 )
Double3 const Double3::Z( 0.0, 0.0, 1.0 );
