#ifndef DISTANCE3DHELPER_H
#define DISTANCE3DHELPER_H

#include <stdint.h>

#include "../global/Types.h"


class Distance3DHelper
{
public:
	
	Distance3DHelper();
	~Distance3DHelper();
	
	/*
	 * Output Point
	 * x := Radius
	 * y := Height
	 */
	Point getDistance		(const double rInner, const double rOuter) const;
	Point getDistancePanno	(const uint16_t yPannoInner, const uint16_t yPannoOuter) const;
	Point getInnerDistance	(const double rInner) const;
	Point getOuterDistance	(const double rOuter) const;
	
private:
	
	uint16_t	pannoWidth;
	
	double	* Lookup3D;
	
	double	scan_step;
	double	sensor_radius;
	double	pixel_size;

	float cameraHeight;
	
	uint16_t width;
	uint16_t height;
	
	uint16_t	iRadiusStart;
	uint16_t	iRadiusEnd;
	uint16_t	oRadiusStart;
	uint16_t	oRadiusEnd;
};

#endif
