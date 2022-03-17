#pragma once

/**
 * @brief Camera distortian enumerator.
 */
enum Distortion
{
	DISTORTION_NONE,
	DISTORTION_MODIFIED_BROWN_CONRADY,
	DISTORTION_INVERSE_BROWN_CONRADY,
	DISTORTION_FTHETA,
	DISTORTION_BROWN_CONRADY,
	DISTORTION_KANNALA_BRANDT4,
	DISTORTION_COUNT
};

/**
 * @brief Camera intrinsics structure.
 */
struct Intrinsics
{
	int width;
	int height;
	float ppx;
	float ppy;
	float fx;
	float fy;
	Distortion model;
	float coeffs[5];
};

/**
 * @brief Camera depth sensor to color sensor extrinsics.
 */
struct Extrinsics
{
	float rotation[9];
	float translation[3];
};
