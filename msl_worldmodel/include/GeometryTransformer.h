/*
 * GeometryTransformer.h
 *
 *  Created on: 15.11.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_

#include <tuple>

#include "SystemConfig.h"

using namespace std;

namespace msl {

	class GeometryTransformer {
	public:

		virtual ~GeometryTransformer(){};
		static pair<double, double> allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos);

	private:
		GeometryTransformer();
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_ */
