#include "MapHelper.h"

using namespace std;

MapHelper* MapHelper::instance = NULL;

MapHelper* MapHelper::getInstance() {
	if (instance == NULL)
		instance = new MapHelper();
	return instance;
}

MapHelper::MapHelper() {
	derivation=true;
}

void MapHelper::initializeMap() {
	RosMsgReceiver* rmr = RosMsgReceiver::getInstance();
	WIDTH = rmr->getMapInfo()->width;
	HEIGHT = rmr->getMapInfo()->height;
	IWIDTH = rmr->getMapInfo()->width;
	IHEIGHT = rmr->getMapInfo()->height;
	minXLocation = WIDTH;
	maxXLocation = 0;
	minYLocation = HEIGHT;
	maxYLocation = 0;
	
	for(int i=0; i<rmr->getMapInfo()->width*rmr->getMapInfo()->height; i++) {
		if(rmr->getMap()[i]==100) rmr->getMap()[i] = 0;
		else if(rmr->getMap()[i]==0) rmr->getMap()[i] = 254;
		else rmr->getMap()[i] = 255;
	}
	
	newMap = new unsigned char[rmr->getMapInfo()->width*rmr->getMapInfo()->height];
	memset(newMap, 255, rmr->getMapInfo()->width*rmr->getMapInfo()->height*sizeof(unsigned char));

	double MAXDIST=3.0;
	RESOLUTION=rmr->getMapInfo()->resolution;
	int localArea=(int)(MAXDIST/RESOLUTION);	
	
	distMap = new unsigned char[(2*localArea+1)*(2*localArea+1)];
	memset(distMap, 255, (2*localArea+1)*(2*localArea+1)*sizeof(unsigned char));
	for(int n=-localArea; n<=localArea; n++) {
		for(int m=-localArea; m<=localArea; m++) {
			double v = sqrt((double)(m*m+n*n))*RESOLUTION*1000.0;
			int val = lrint(254.0*(1.0 - (250.0*250.0/(v*v + 250.0*250.0))));
			distMap[id(n+localArea, m+localArea, 2*localArea+1)] = val;
		}
	}

	unsigned char v;
	for(int x=0; x<rmr->getMapInfo()->width; x++) {
		for(int y=0; y<rmr->getMapInfo()->height; y++) {
			v = rmr->getMap()[id(x, y, rmr->getMapInfo()->width)];
			if(v==0) {
				for(int m=max(x-localArea,0); m<=min(x+localArea, (int)rmr->getMapInfo()->width-1); m++) {
					for(int n=max(y-localArea,0); n<=min(y+localArea, (int)rmr->getMapInfo()->height-1); n++) {
						int g = distMap[id(x-m+localArea, y-n+localArea, 2*localArea+1)];
						if(newMap[id(rmr->getMapInfo()->width-m, n, rmr->getMapInfo()->width)]>g)
							newMap[id(rmr->getMapInfo()->width-m, n, rmr->getMapInfo()->width)]=g;
					}
				}
			} else if(v==254) {
				minYLocation = min(minYLocation, x);
				maxYLocation = max(maxYLocation, x);
				minXLocation = min(minXLocation, y);
				maxXLocation = max(maxXLocation, y);
			}
		}
	}
	
	if(derivation) {
		wallDistanceMap = new float[rmr->getMapInfo()->width*rmr->getMapInfo()->height];
		for(int i=0; i<rmr->getMapInfo()->width*rmr->getMapInfo()->height; i++) {
			wallDistanceMap[i]=5.0;
		}
		
		MAXDIST = 5.0;
		localArea = (int)(MAXDIST/RESOLUTION);
		fdistMap = new float[(2*localArea+1)*(2*localArea+1)];
		
		float v;
		for(int n=-localArea; n<=localArea; n++) {
			for(int m=-localArea; m<=localArea; m++) {
				v = sqrt((double)(m*m+n*n))*RESOLUTION;
				fdistMap[id(n+localArea, m+localArea, 2*localArea+1)] = v;
			}
		}
		
		for(int x=0; x<rmr->getMapInfo()->width; x++) {
			for(int y=0; y<rmr->getMapInfo()->height; y++) {
				v = rmr->getMap()[id(x, y, rmr->getMapInfo()->width)];
				if(v==0) {
					for(int m=max(x-localArea,0); m<=min(x+localArea, (int)rmr->getMapInfo()->width-1); m++) {
						for(int n=max(y-localArea,0); n<=min(y+localArea, (int)rmr->getMapInfo()->height-1); n++) {
							
							v = fdistMap[id(x-m+localArea, y-n+localArea, 2*localArea+1)];
							if(wallDistanceMap[id(rmr->getMapInfo()->width-m, n, rmr->getMapInfo()->width)]>v)
								wallDistanceMap[id(rmr->getMapInfo()->width-m, n, rmr->getMapInfo()->width)]=v;
								
						}
					}
				}
			}
		}
		initializeGradientMap();
	}
	
	ofstream ofs("Test3.txt");
	ofstream ofs2("Test2.txt");
	float distance, ef, derrddist, csquare=0.25*0.25;
	for(int i=0; i<rmr->getMapInfo()->width*rmr->getMapInfo()->height; i++) {
		ofs2 << (int)rmr->getMap()[i] << " ";
		//ofs2 << (int)ySobelMap[i] << " ";
		
		distance = wallDistanceMap[i];
		ef = csquare + distance * distance;
		derrddist = (2 * csquare * distance) /(ef * ef);
		
		ofs << derrddist*fYSobel[i] << " ";
		if(i%rmr->getMapInfo()->width==rmr->getMapInfo()->width-1) {
			ofs << endl;		
			ofs2 << endl;		
		}
	}
	ofs.close();
	ofs2.close();
	cout << "Map Initialization Finished" << endl;
}


void MapHelper::initializeGradientMap() {
	RosMsgReceiver* rmr = RosMsgReceiver::getInstance();
	int WIDTH=rmr->getMapInfo()->width;
	int HEIGHT=rmr->getMapInfo()->height;
	
	xSobelMap = new signed char[WIDTH*HEIGHT*sizeof(signed char)];
	memset(xSobelMap, 0, WIDTH*HEIGHT*sizeof(signed char));
	
	ySobelMap = new signed char[WIDTH*HEIGHT*sizeof(signed char)];
	memset(ySobelMap, 0, WIDTH*HEIGHT*sizeof(signed char));
	
	{
		int slt, slc, slb, sct, scc, scb, srt, src, srb;
		for(int x=1; x<WIDTH-1; x++) {
			for(int y=1; y<HEIGHT-1; y++) {
				slt = newMap[id(x-1, y-1, WIDTH)];
				slc = newMap[id(x-1, y, WIDTH)];
				slb = newMap[id(x-1, y+1, WIDTH)];
				sct = newMap[id(x, y-1, WIDTH)];
				scc = newMap[id(x, y, WIDTH)];
				scb = newMap[id(x, y+1, WIDTH)];
				srt = newMap[id(x+1, y+1, WIDTH)];
				src = newMap[id(x+1, y, WIDTH)];
				srb = newMap[id(x+1, y+1, WIDTH)];
				
				if(scc==0) {
					xSobelMap[id(x, y, WIDTH)] = 0;
					ySobelMap[id(x, y, WIDTH)] = 0;
				}
				else {
					ySobelMap[id(x, y, WIDTH)] = (slt+2*slc+slb-srt-2*src-srb)/8;
					xSobelMap[id(x, y, WIDTH)] = (slt+2*sct+srt-slb-2*scb-srb)/8;
				}
			}
		}
	}
	
	fXSobel = new float[WIDTH*HEIGHT];
	fYSobel = new float[WIDTH*HEIGHT];
	float RES = RESOLUTION;
	{
		float slt, slc, slb, sct, scc, scb, srt, src, srb;
		for(int x=1; x<WIDTH-1; x++) {
			for(int y=1; y<HEIGHT-1; y++) {
				slt = wallDistanceMap[id(x-1, y-1, WIDTH)];
				slc = wallDistanceMap[id(x-1, y, WIDTH)];
				slb = wallDistanceMap[id(x-1, y+1, WIDTH)];
				sct = wallDistanceMap[id(x, y-1, WIDTH)];
				scc = wallDistanceMap[id(x, y, WIDTH)];
				scb = wallDistanceMap[id(x, y+1, WIDTH)];
				srt = wallDistanceMap[id(x+1, y+1, WIDTH)];
				src = wallDistanceMap[id(x+1, y, WIDTH)];
				srb = wallDistanceMap[id(x+1, y+1, WIDTH)];
				
				if(scc==0) {
					fXSobel[id(x, y, WIDTH)] = 0;
					fYSobel[id(x, y, WIDTH)] = 0;
				}
				else {
					fYSobel[id(x, y, WIDTH)] = ((slt+2*slc+slb-srt-2*src-srb)/8.0)/RES;
					fXSobel[id(x, y, WIDTH)] = ((slt+2*sct+srt-slb-2*scb-srb)/8.0)/RES;
				}
			}
		}
	}

}


double MapHelper::xGradient(double x, double y) {
	// -????? X und Y vertauscht?????
	int indX = lrint(y/RESOLUTION) + IHEIGHT;
	int indY = lrint(x/RESOLUTION) + IWIDTH;
	
	return (double) xSobelMap[id(indX, indY, IWIDTH)];
}


double MapHelper::xGradient(int indX, int indY) {
	return (double) xSobelMap[id(indX, indY, IWIDTH)];
}


double MapHelper::yGradient(int indX, int indY) {
	return (double) ySobelMap[id(indX, indY, IWIDTH)];
}


double MapHelper::fxGradient(int indX, int indY) {
	return (double) fXSobel[id(indX, indY, IWIDTH)];
}


double MapHelper::fyGradient(int indX, int indY) {
	return (double) fYSobel[id(indX, indY, IWIDTH)];
}


double MapHelper::yGradient(double x, double y) {
	// -????? X und Y vertauscht?????
	int indX = lrint(y/RESOLUTION) + IHEIGHT;
	int indY = lrint(x/RESOLUTION) + IWIDTH;
	
	return (double) ySobelMap[id(indX, indY, IWIDTH)];
}



double MapHelper::angleGradient(int px, int py, double pangle, double lx, double ly) {
	double sinangle = sin(pangle);
	double cosangle = sin(pangle);
	return (-sinangle*lx-cosangle*ly)*xGradient(px,py)+(cosangle*lx -sinangle*ly)*yGradient(px,py);
}



double MapHelper::angleGradient(double px, double py, double pangle, double lx, double ly) {
	double sinangle = sin(pangle);
	double cosangle = sin(pangle);
	return (-sinangle*lx-cosangle*ly)*xGradient(px,py)+(cosangle*lx -sinangle*ly)*yGradient(px,py);
}


double MapHelper::fangleGradient(int px, int py, double pangle, double lx, double ly) {
	double sinangle = sin(pangle);
	double cosangle = sin(pangle);
	return (-sinangle*lx-cosangle*ly)*fxGradient(px,py)+(cosangle*lx -sinangle*ly)*fyGradient(px,py);
}



double MapHelper::fangleGradient(double px, double py, double pangle, double lx, double ly) {
	double sinangle = sin(pangle);
	double cosangle = sin(pangle);
	return (-sinangle*lx-cosangle*ly)*fxGradient(px,py)+(cosangle*lx -sinangle*ly)*fyGradient(px,py);
}
