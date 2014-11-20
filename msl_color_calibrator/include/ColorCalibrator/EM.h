#ifndef EM_H
#define EM_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <list>
#include <vector>

#include <QRect>

#include <math.h>       // sqrt
#include <stdlib.h>     // abs

using namespace std;
using namespace Eigen;

namespace em {
    typedef Vector2d dataType;

    class Cluster;

    class EM {
    public:
        EM(unsigned int numberOfClusters, QRect rect);
        ~EM();

        void perform();

        void addData(dataType data);

        double p(dataType data);
        double p(dataType data, list<Cluster*> clusterList);
        double pXC(dataType data, Cluster* cluster);
        double pCX(Cluster* cluster, dataType data);

        list <Cluster*> m_cluster;
    private:
        std::list<dataType> m_data;

        void e();
        void m();

        void recalcCovariance(Cluster* cluster);

        double expect();
    };

    class Cluster {
    public:
        Cluster(QRect rect);
        double w;

        dataType mu;
        Matrix2d covariance;

        double coeff;
        list<double> storedPCX;
    };
}

#endif // EM_H
