#include "EM.h"

#include <iostream>

using namespace em;

EM::EM(unsigned int numberOfClusters, QRect rect) {
    for (int i = 0; i < numberOfClusters; i++) {
        m_cluster.push_back(new Cluster(rect));
    }
}

EM::~EM() {
    for (list<Cluster*>::iterator it = m_cluster.begin(); it != m_cluster.end(); it++) {
        delete *it;
    }
}

void EM::perform() {
    vector<int> preclusterDataIndex;
    int preclusterCount = std::min((int) m_data.size(), (int) m_cluster.size() * 25);
    for (int i = 0; i < preclusterCount; ++i) {
        int index;
        do {
            index = rand() % m_data.size();
        } while (find(preclusterDataIndex.begin(), preclusterDataIndex.end(), index) != preclusterDataIndex.end());
        preclusterDataIndex.push_back(index);
    }

    Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distances(preclusterCount, preclusterCount);

    for (int i = 0; i < preclusterCount; i++) {
        list<dataType>::iterator it_data_i = m_data.begin();
        advance(it_data_i, preclusterDataIndex[i]);
        dataType data_i = *it_data_i;
        for (int j = 0; j < preclusterCount; j++) {
            if (i == j) {
                distances(i, j) = 0;
            } else {
                list<dataType>::iterator it_data_j = m_data.begin();
                advance(it_data_j, preclusterDataIndex[j]);
                dataType data_j = *it_data_j;

                double distance = sqrt(pow(data_i(0) - data_j(0), 2) + pow(data_i(1) - data_j(1), 2));
                distances(i, j) = distance;
                distances(j, i) = distance;
            }
        }
    }

    vector<int> preclusters;
    for (int i = 0; i < m_cluster.size(); ++i) {
        preclusters.push_back(-1);
    }
    if (m_cluster.size() > 1) {
        double distance = 0;
        for (int i = 0; i < preclusterCount; i++) {
            for (int j = 0; j < preclusterCount; j++) {
                if (i != j && distance < distances(i , j)) {
                    distance = distances(i , j);
                    preclusters[0] = i;
                    preclusters[1] = j;
                }
            }
        }

        int clustered = 2;
        while (clustered < m_cluster.size()) {
            int dev = INT_MAX;
            int distance = 0;

            for (int i = 0; i < preclusterCount; i++) {
                if (find(preclusters.begin(), preclusters.end(), i) == preclusters.end()) {
                    int tmp_distance = 0;
                    int count = 0;
                    for (int j = 0; j < preclusters.size(); j++) {
                        if (preclusters[j] > -1) {
                            tmp_distance += distances(i , preclusters[j]);
                            count++;
                        }
                    }
                    tmp_distance = tmp_distance / count;

                    int tmp_dev = 0;
                    for (int j = 0; j < preclusters.size(); j++) {
                        if (preclusters[j] > -1) {
                            tmp_dev += abs(distances(i , preclusters[j]) - tmp_distance);
                        }
                    }

                    if (distance < tmp_distance && dev > tmp_dev) {
                        preclusters[clustered] = i;
                        dev = tmp_dev;
                        distance = tmp_distance;
                    }
                }
            }

            clustered++;
        }
    } else {
        preclusters[0] = rand() % preclusterDataIndex.size();
    }

    int i = 0;
    for (list<Cluster*>::iterator it = m_cluster.begin(); it != m_cluster.end(); it++, i++) {
        Cluster *cluster = *it;

        list<dataType>::iterator it_data = m_data.begin();
        advance(it_data, preclusterDataIndex[preclusters[i]]);
        dataType data = *it_data;

        cluster->mu = data;
    }

    int iter = 0;
    double oldExpect;
    do {
        oldExpect = expect();
        e();
        m();
        cout << "oldExpect: " << oldExpect << " new: " << expect() << "   " << abs(oldExpect - expect()) << endl;
        iter++;
    } while (abs(oldExpect - expect()) > pow(10, -5));

    cout << "number of iterations: " << iter << endl;
}

void EM::addData(dataType data) {
    m_data.push_back(data);

    for (list<Cluster*>::iterator it = m_cluster.begin(); it != m_cluster.end(); it++) {
        Cluster *cluster = *it;
        cluster->storedPCX.push_back(0);
    }
}

void EM::e() {
    list<dataType>::iterator itData = m_data.begin();
    for (int i = 0; i < m_data.size(); i++, itData++) {
        dataType data = *itData;

        for (list<Cluster*>::iterator itCluster = m_cluster.begin(); itCluster != m_cluster.end(); itCluster++) {
            Cluster *cluster = *itCluster;

            list<double>::iterator itPCX = cluster->storedPCX.begin();
            advance(itPCX, i);
            double & pcx ( *itPCX );

            pcx = pCX(cluster, data);
        }
    }
}

void EM::m() {
    for (list<Cluster*>::iterator it = m_cluster.begin(); it != m_cluster.end(); it++) {
        Cluster *cluster = *it;

        double w = 0;
        dataType my_enumerator;
        my_enumerator(0) = my_enumerator(1) = 0;
        double my_denominator = 0;

        int i = 0;
        for (list<dataType>::iterator it = m_data.begin(); it != m_data.end(); it++, i++) {
            dataType data = *it;

            list<double>::iterator itPCX = cluster->storedPCX.begin();
            advance(itPCX, i);
            double pCX = *itPCX;

            w = w + pCX;

            my_enumerator = my_enumerator + data * pCX;
            my_denominator = my_denominator + pCX;
        }

        cluster->w = w / m_data.size();

        cluster->mu = my_enumerator / my_denominator;

        recalcCovariance(cluster);
    }
}

double EM::p(dataType data) {
    return p(data, m_cluster);
}

double EM::p(dataType data, list<Cluster*> clusterList) {
    double p = 0;
    for (list<Cluster*>::iterator it = clusterList.begin(); it != clusterList.end(); it++) {
        Cluster *cluster = *it;
        p += cluster->w * pXC(data, cluster);
    }
    return p;
}

double EM::pXC(dataType data, Cluster* cluster) {
    dataType diff = data - cluster->mu;
    Matrix<double, 1, 2> transpose = diff.transpose();
    Matrix2d inverse = cluster->covariance.inverse();
    Matrix<double, 1, 2> intermediateResult = - 0.5 * transpose;
    Matrix<double, 1, 2> intermediateResult1 = intermediateResult * inverse;
    Matrix<double, 1, 1> intermediateResult2 = intermediateResult1 * diff;
    double exponent = intermediateResult2(0);
    double exp_result = exp(exponent);
    double result = cluster->coeff * exp_result;
    return result;
}

double EM::pCX(Cluster* cluster, dataType data) {
    return cluster->w * pXC(data, cluster) / p(data);
}

void EM::recalcCovariance(Cluster* cluster) {
    Matrix2d enumerator;
    enumerator(0, 0) = 0;
    enumerator(0, 1) = 0;
    enumerator(1, 0) = 0;
    enumerator(1, 1) = 0;
    double denominator = 0;


    int i = 0;
    for (list<dataType>::iterator it = m_data.begin(); it != m_data.end(); it++, i++) {
        dataType data = *it;

        list<double>::iterator itPCX = cluster->storedPCX.begin();
        advance(itPCX, i);
        double pCX = *itPCX;

        enumerator = enumerator + pCX * (data - cluster->mu) * (data - cluster->mu).transpose();
        denominator = denominator + pCX;
    }
    cluster->covariance = enumerator / denominator;

    cluster->coeff = 1 / sqrt(pow(2 * M_PI, cluster->covariance.rows()) * cluster->covariance.determinant());
}

double EM::expect() {
    double e = 0;
    for (list<dataType>::iterator it = m_data.begin(); it != m_data.end(); it++) {
        dataType data = *it;
        e = e + p(data, m_cluster);
    }
    return e;
}

Cluster::Cluster(QRect rect) {
    double top = rect.top() / 10.0;
    double left = rect.left() / 10.0;
    double bottom = rect.bottom() / 10.0;
    double right = rect.right() / 10.0;

    w = 0.5;//0.3*(rand()/(double)RAND_MAX) + 0.2;

    mu(0) = 0;//25.5*(rand()/(double)RAND_MAX);//
    mu(1) = 0;//25.5*(rand()/(double)RAND_MAX);//

    // zwischen 10 und 20 prozente von der laenge/breite des rechtecks
    covariance(0, 0) = (right - left) * 0.1;//5*(rand()/(double)RAND_MAX);//
    covariance(0, 1) = 0;
    covariance(1, 0) = 0;
    covariance(1, 1) = (bottom - top) * 0.1;//5*(rand()/(double)RAND_MAX);//

    coeff = 1 / sqrt(pow(2 * M_PI, covariance.rows()) * covariance.determinant());
}
