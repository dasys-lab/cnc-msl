#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <SystemConfig.h>

#define NSECTORS 360
#define LUFILENAME "/DistanceLookup.dat"
#define LUFILENAMETXT "/DistanceLookup.txt"

using namespace std;
using namespace supplementary;

double angleDiff(double a, double b) {
	return min(abs(a-b)*1.0, 2.0*3.141592-abs(a-b));
}


int nearestAngleIndes(vector< vector<double> > &dangles, double angle) {
	double minDist = 100000000;
	int minIndex=-1;
	for(int i=0; i<dangles.size(); i++) {
		if(dangles[i][0]==0) continue;
		bool hasValues=false;
		for(int j=1; j<dangles[i].size(); j++) if(dangles[i][j]!=0) hasValues = true;
		if(!hasValues) continue;

		double dang = angleDiff(dangles[i][0], angle);
		if(dang < minDist) {
			minIndex = i;
			minDist  = dang;

		}
	}
	return minIndex;
}




int main(int argc, char * argv[]){
	SystemConfig* sc = SystemConfig::getInstance();
        std::string confPath = sc->getConfigPath();
	Configuration *vision = (*sc)["Vision"];
	Configuration *carpetCalib = (*sc)["CarpetCalibrator"];
        int dcHEIGHT = vision->get<int>("Vision", "ImageArea", NULL);
        int dcWIDTH = vision->get<int>("Vision", "ImageArea", NULL);

	vector< vector<double> > distances;
	vector<double> tmp;
	char line[4096];
	double val;
	stringstream ss(stringstream::in | stringstream::out);

	vector<double> realDist;

//    int counter = 0;


    for(int i = 0; i < 10; i++) {
    	realDist.push_back(((*sc)["CarpetCalibrator"])->get<int>("Dist", ("dist"+to_string(i)).c_str() , NULL));
    }

	for(int i=1; i<20; i++) realDist.push_back(5000+i*1000);


	ifstream* ifs;
	ifs = new ifstream(sc->getConfigPath() + sc->getHostname() + "/clearedLP.txt");
	if(!ifs->is_open()) {
		ifs = new ifstream(sc->getConfigPath() + "/clearedLP.txt");
	}
	while(!ifs->eof()) {
		tmp.clear();
		ifs->getline(line, 4096);
		if(ifs->eof()) break;

		ss << line;

		int i=0;
		while(!ss.eof()) {
			i++;
			ss >> val;
			if(i<=6) tmp.push_back(val);
		}
		distances.push_back(tmp);
		ss.clear();
	}

	for(int i=0; i<8; i++)	cout << distances[44][i] << " ";
	cout << endl;

	string dbgPath = confPath + sc->getHostname() + "/distancelookup.txt";
	ofstream* ofs;
	ofs = new ofstream(dbgPath.c_str());
	if(!ofs->is_open()) {
		ofs = new ofstream((confPath + "/distancelookup.txt").c_str());
	}

	double *lookup = new double[dcHEIGHT*dcWIDTH];
	memset(lookup, 0, dcHEIGHT*dcWIDTH*sizeof(double));

        double LookupTable[dcHEIGHT][dcWIDTH];
        int LookupTableInt[dcHEIGHT][dcWIDTH][2];

	double maxM=0;

	for(int x=0; x<dcWIDTH; x++) {
		for(int y=0; y<dcHEIGHT; y++) {
			int realx = x-dcWIDTH/2;
			int realy = y-dcHEIGHT/2;
			double angle = atan2(realy, realx);
			double dist = sqrt(realx*realx + realy*realy);
			int minIndex = nearestAngleIndes(distances, angle);
			//cout << angle << " " << minIndex << " " << distances[minIndex][0] << endl;

			double d=0, m=0;
			int limit = distances[minIndex].size();
			int nextInd=0;
			int lastInd=distances[minIndex].size()-1;
			while(nextInd<distances[minIndex].size() && distances[minIndex][nextInd] < dist) nextInd++;
			while(lastInd>0 && (distances[minIndex][lastInd] > dist || distances[minIndex][lastInd]==0)) lastInd--;
			if(lastInd==0) {
				d = (realDist[1]/distances[minIndex][nextInd])*(dist);
			} else if (lastInd>0 && nextInd<=distances[minIndex].size()-1) {
				m = (realDist[nextInd]-realDist[lastInd])/(distances[minIndex][nextInd]-distances[minIndex][lastInd]);
				d = m*(dist-distances[minIndex][lastInd]) + realDist[lastInd];
			} else if (nextInd>=distances[minIndex].size()) {
				m = (realDist[lastInd]-realDist[lastInd-1])/(distances[minIndex][lastInd]-distances[minIndex][lastInd-1]);
				d = 2*m*(dist-distances[minIndex][lastInd])/*(dist-distances[minIndex][lastInd])*/ + realDist[lastInd];
			}
			//if(d>100000000000000) cout << d << " " << minIndex << " " << dist << " " << nextInd << " " << limit << endl;
			lookup[y*dcHEIGHT+x] = d;
                        if(d > 0.0){
                                if(d > 0.0 && d < 150000.0){
                                        LookupTable[x][y] = d; //newDist;
                                	LookupTableInt[x][y][0] = (int) rint(d);
                                }
                                else{
                                        LookupTable[x][y] = d;
                                	LookupTableInt[x][y][0] = -1;
                        	}
                        }
                        else{
                                LookupTable[x][y] = d;
                        	LookupTableInt[x][y][0] = -1;
                        }

	                LookupTableInt[x][y][1] = (int) floor((-atan2(realy, realx) + M_PI)*NSECTORS/(2.0*M_PI));
                        if(LookupTableInt[x][y][1] >= NSECTORS)
                                LookupTableInt[x][y][1] = NSECTORS - 1;


			/*for(int i=1; i<=limit; i++) {
				if(distances[minIndex][0]==0 //|| distances[minIndex][i]==0) continue;
				//double ix = sin(distances[minIndex][0])*distances[minIndex][i];
				//double iy = cos(distances[minIndex][0])*distances[minIndex][i];
				if(!(distances[minIndex][i+1]==0 ||  distances[minIndex][i]>dist && distances[minIndex][i+1]<dist) && i!=limit) continue;
				double m=0;
				if(i==1) {
					if(distances[minIndex][1]!=0) {
						d = (1000/distances[minIndex][1])*dist;
						m = 1000/distances[minIndex][1];
					}
					maxM = max(maxM, m);
				} else if(i<limit) {
                                        int inda=i;
                                        while(inda<limit && distances[minIndex][inda]==0) inda++;
					if(inda>=limit) continue;

                                        int indb=i-1;
                                        while(indb>0 && distances[minIndex][indb]==0) indb--;
					if(indb<=0) continue;

                                        m = (1000*(inda-indb))/(distances[minIndex][inda]-distances[minIndex][indb]);
					//double m = 1000/(distances[minIndex][i]-distances[minIndex][i-1]);
					d = m*dist + distances[minIndex][indb];
					maxM = max(maxM, m);
				} else {
					int inda=i-1;
					while(inda>0 && distances[minIndex][inda]==0) inda--;
					int indb=inda-1;
					while(indb>0 && distances[minIndex][indb]==0) indb--;
					if(indb<=0) continue;
					if(distances[minIndex][indb]==0 || distances[minIndex][inda]==0) continue;
					m = (1000*(inda-indb))/(distances[minIndex][inda]-distances[minIndex][indb]);
					d = m*dist*dist + distances[minIndex][inda];
					maxM = max(maxM, m);
				}
				//if(d>100000000000000) cout << d << " " << minIndex << " " << dist << " " << i << " " << limit << endl;
				//if(m>696) cout << d << " " << minIndex << " " << dist << " " << i << " " << limit << endl;
				lookup[y*area+x] = d;
			}*/

			*ofs << lookup[y*dcHEIGHT+x] << " ";
		}
		*ofs << endl;
	}
	cout << maxM << endl;

        std::string filePath = confPath + sc->getHostname() + "/" + LUFILENAME;
        std::cout << std::endl << "Building " << filePath << std::endl;


        FILE * fd = fopen(filePath.c_str(), "w");

        if(fd == NULL) {
        	fd = fopen((confPath + LUFILENAME).c_str(), "w");
        }

        fwrite(&(LookupTable[0]), sizeof(double), dcWIDTH*dcHEIGHT, fd);
        fwrite(&(LookupTableInt[0][0]), sizeof(int), dcWIDTH*dcHEIGHT*2, fd);
        fclose(fd);

        ofstream *ofs2;
        ofs2 = new ofstream(sc->getConfigPath() + sc->getHostname() + LUFILENAMETXT);
        if(!ofs2->is_open()) {
        	ofs2 = new ofstream(sc->getConfigPath() + LUFILENAMETXT);
        }


        for(int i=0; i<dcWIDTH; i++) {
                for(int j=0; j<dcHEIGHT; j++) {
                        *ofs2 << LookupTable[i][j] << " ";
                }
                *ofs2 << endl;
        }


	return 0;
}

