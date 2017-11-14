#pragma once
#include <stdio.h>
#include <cstdint>
#include <stdlib.h>
#include <array>
#include <vector>

namespace msl_ptgrey_camera
{
	class CNImage
	{
	public:
        CNImage();
        CNImage(CNImage& origin);
		CNImage(int height, int width, int channels);

		CNImage conv2d(std::array<std::array<int, 3>, 3> kernel);
		void conv1dH(CNImage& input, std::array<int, 3> kernel);
		void conv1dV(CNImage& input, std::array<int, 3> kernel);

		int height;
		int width;
        int channels;
        int step;

		unsigned char* imgData;
		virtual ~CNImage();

	private:

	};

} /* namespace msl_ptgrey_camera */
