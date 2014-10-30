#include <Eigen/Core>
#include <Eigen/Array>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LeastSquares>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/QtAlignedMalloc>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/SVD>

#include <stdio.h>


// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN


extern "C" void Eigen_InvertMatrix(double * matrix, double * invMatrix, int dim){
	
	Eigen::Map<MatrixXd> m(matrix, dim, dim);
	Eigen::Map<MatrixXd> m2(invMatrix, dim, dim);
	
	m2 = m.inverse();
	
}

extern "C" double Eigen_MahalanobisDistance(double * mean1, double * cov1, double * mean2, double * cov2, int dim){
	
	
	Eigen::Map<VectorXd> m1(mean1, dim);
	Eigen::Map<VectorXd> m2(mean2, dim);

	Eigen::Map<MatrixXd> c1(cov1, dim, dim);
	Eigen::Map<MatrixXd> c2(cov2, dim, dim);
	
	VectorXd m = m1 - m2;
	
	//std::cout << "M\n" << m << std::endl;
	
	MatrixXd covInv = (c1 + c2).inverse();
	
	MatrixXd mMD = m.transpose() * covInv * m;
	
	double dist = sqrt(mMD(0,0));
	
	return dist;
	
}


extern "C" double Eigen_BhattacharyyaDistance(double * mean1, double * cov1, double * mean2, double * cov2, int dim){
	
	
	Eigen::Map<VectorXd> m1(mean1, dim);
	Eigen::Map<VectorXd> m2(mean2, dim);

	Eigen::Map<MatrixXd> c1(cov1, dim, dim);
	Eigen::Map<MatrixXd> c2(cov2, dim, dim);
	
	double mahalanobis_dist = Eigen_MahalanobisDistance(mean1, cov1, mean2, cov2, dim);
	
	double bhatt_dist = 0.125 * mahalanobis_dist*mahalanobis_dist + 0.5*log( (c1 + c2).determinant() / (2 * sqrt(c1.determinant() * c2.determinant())) );
	
	return bhatt_dist;
	
}



extern "C" double Eigen_MahalanobisDistanceCovAndValueRange(double * mean1, double * cov1, double * mean2, double * range2, int dim){
	
	
	Eigen::Map<VectorXd> m1(mean1, dim);
	Eigen::Map<VectorXd> m2(mean2, dim);

	Eigen::Map<MatrixXd> c1(cov1, dim, dim);
	MatrixXd c2(dim, dim);
	
	for(int i = 0; i < dim; i++){
	
		for(int j = 0; j < dim; j++){
			
			if(i == j){
				c2(i,j) = (range2[i]*range2[i])/3.0;
			}
			else {
			
				c2(i,j) = 0;
				
			}
			
		}
		
	}
	
	
	VectorXd m = m1 - m2;
	
	MatrixXd covInv = (c1 + c2).inverse();
	
	MatrixXd mMD = m.transpose() * covInv * m;
	
	double dist = sqrt(mMD(0,0));
	
	return dist;
	
}



extern "C" void Eigen_CombineCovAndCov(double * mean1, double * cov1, double * mean2, double * cov2, double * mean3, double * cov3, int dim){
	
	
	Eigen::Map<VectorXd> m1(mean1, dim);
	Eigen::Map<VectorXd> m2(mean2, dim);
	Eigen::Map<VectorXd> m3(mean3, dim);	

	Eigen::Map<MatrixXd> c1(cov1, dim, dim);
	Eigen::Map<MatrixXd> c2(cov2, dim, dim);
	Eigen::Map<MatrixXd> c3(cov3, dim, dim);	
	
	MatrixXd covInv = (c1 + c2).inverse();
	
	m3 = (c2 * covInv * m1) + (c1 * covInv * m2);
	c3 = c1 * covInv * c2;
	
	
}


extern "C" void Eigen_NMatrixSquareRoot(double * cov, double * squareRoot, int n, int dim){
	
	for(int i = 0; i < dim*dim; i++){
	
		cov[i] *= n;
		
	}	
	
	Eigen::Map<MatrixXd> c(cov, dim, dim);
	Eigen::Map<MatrixXd> c2(squareRoot, dim, dim);

	Eigen::LLT<MatrixXd> lltOfC(c);
	c2 = lltOfC.matrixL().transpose();
	

}


void calculateNMatrixSquareRoot(double * cov, double * covRes, int dim){
	
	for(int i = 0; i < dim*dim; i++){
		covRes[i] = dim*cov[i];
		
	}
	

	for(int i = 0; i < dim; i++){
		
		double sum = 0;
		
		for(int j = 0; j <= i-1; j++){							
					
			sum = covRes[i*dim + j];
						
			for(int k = 0; k <= j-1; k++){
			
				sum -= covRes[i*dim + k]*covRes[j*dim + k];
					
			}
						
			covRes[i*dim + j] = sum / covRes[j*dim + j];
			
			//Console.WriteLine("G(" + i + "," + j + ") = " + matrix[i*totalLength + j]);
			
		}
					
		sum = covRes[i*dim + i];
					
		for(int k = 0; k <= i - 1; k++){

			sum -= covRes[i*dim + k] * covRes[i*dim + k];
					
		}
		
		if(sum < 0){
			printf("Problem calculating Matrix Square Root!\n");
			return;
		}
		
		covRes[i*dim + i] = sqrt(sum);
		
		//Console.WriteLine("G(" + i + "," + i + ") = " + matrix[i*totalLength + i]);
		
		
	}
	
	for(int i = 0; i < dim; i++){
	
		for(int j = 0; j < dim; j++){
		
			if(i < j)
				covRes[i*dim + j] = 0;
			
		}
		
	}

	
}





extern "C" void Eigen_CombineCovAdd(double * mean1, double * cov1, double * mean2, double * cov2, double * mean3, double * cov3, double * beliefs, double * meanRes, double * covRes, int dim){
	
		double covRes1[dim*dim];		
		double covRes2[dim*dim];		
		double covRes3[dim*dim];
		
		calculateNMatrixSquareRoot(cov1, covRes1, dim);
		calculateNMatrixSquareRoot(cov2, covRes2, dim);
		calculateNMatrixSquareRoot(cov3, covRes3, dim);
		
		double sumBeliefs = beliefs[0] + beliefs[1] + beliefs[2];
		
		double factor = 1.0/(2.0*dim);
				
		for(int i = 0; i < dim; i++){
		
			meanRes[i] = 0.0;
			
			for(int j = 0; j < dim; j++){
				
				meanRes[i] += beliefs[0]/sumBeliefs * factor * (mean1[i] + covRes1[i*dim + j]);
				meanRes[i] += beliefs[0]/sumBeliefs * factor * (mean1[i] - covRes1[i*dim + j]);
				
				meanRes[i] += beliefs[1]/sumBeliefs * factor * (mean2[i] + covRes2[i*dim + j]);
				meanRes[i] += beliefs[1]/sumBeliefs * factor * (mean2[i] - covRes2[i*dim + j]);
				
				meanRes[i] += beliefs[2]/sumBeliefs * factor * (mean3[i] + covRes3[i*dim + j]);
				meanRes[i] += beliefs[2]/sumBeliefs * factor * (mean3[i] - covRes3[i*dim + j]);
				
			}

		}
		
		
		for(int a = 0; a < dim; a++){
		
			for(int b = a; b < dim; b++){
				
				double sum = 0.0;	
			
				for(int i = 0; i < dim; i++){
					
					sum += beliefs[0]/sumBeliefs * factor * (mean1[a] + covRes1[a*dim + i] - meanRes[a])*(mean1[b] + covRes1[b*dim + i] - meanRes[b]);			
					sum += beliefs[0]/sumBeliefs * factor * (mean1[a] - covRes1[a*dim + i] - meanRes[a])*(mean1[b] - covRes1[b*dim + i] - meanRes[b]);			

					sum += beliefs[1]/sumBeliefs * factor * (mean2[a] + covRes2[a*dim + i] - meanRes[a])*(mean2[b] + covRes2[b*dim + i] - meanRes[b]);			
					sum += beliefs[1]/sumBeliefs * factor * (mean2[a] - covRes2[a*dim + i] - meanRes[a])*(mean2[b] - covRes2[b*dim + i] - meanRes[b]);			

					sum += beliefs[2]/sumBeliefs * factor * (mean3[a] + covRes3[a*dim + i] - meanRes[a])*(mean3[b] + covRes3[b*dim + i] - meanRes[b]);			
					sum += beliefs[2]/sumBeliefs * factor * (mean3[a] - covRes3[a*dim + i] - meanRes[a])*(mean3[b] - covRes3[b*dim + i] - meanRes[b]);			

				}
		
				covRes[a*dim + b] = sum;
				covRes[b*dim + a] = sum;
		
			}
		
		}
	
}


extern "C" void Eigen_TransformCNEgo2CNEgoArt(double * mean1, double * cov1, double * meanRes, double * covRes, int dim){
	
		double covRes1[dim*dim];		
		double Points[2*dim*dim];		

/*		printf("Mean\n");
		
		for(int i = 0; i < dim; i++){
			
			printf("%f ", mean1[i]);
		}
*/		
		
				
		calculateNMatrixSquareRoot(cov1, covRes1, dim);
		
		for(int i = 0; i < dim; i++){
		
			for(int j = 0; j < dim; j++){
				
				Points[i*2*dim + j] = mean1[i] + covRes1[i*dim + j];
				
			}
			
			int a = 0; 
			for(int j = dim; j < 2*dim; j++){
				
				Points[i*2*dim + j] = mean1[i] - covRes1[i*dim + a];
				a++;
			}
			
		}
				
		
/*		printf("\nSamplePoints\n");
		
		for(int j = 0; j < 2*dim; j++){
			
			for(int i = 0; i < dim; i++){
	
				printf("%f ", Points[i*2*dim + j]);			
				
			}	
			
			printf("\n");
			
		}
		
		printf("\n\n");*/

		//Transform Points
		
		int numberNeg = 0;
		
		for(int i = 0; i < 2*dim; i++){
			
			double x = Points[i];
			double y = Points[2*dim + i];
			
			Points[i] = atan2(y,x);
			if(Points[i] < 0.0)
				numberNeg++;
			Points[2*dim + i] = sqrt(x*x + y*y); 	
			
			
		}
		
		bool negNormalize = false;
		if(numberNeg > dim)
			negNormalize = true;
		
		
		for(int i = 0; i < 2*dim; i++){
			
			if(negNormalize){
				if(Points[i] > M_PI/2)
					Points[i] -= 2*M_PI;
				
			}
			else{
				if(Points[i] < -M_PI/2)
					Points[i] += 2*M_PI;			
			}	
			
		}
		
		
		double factor = 1.0/(2*dim);		
				
		for(int i = 0; i < dim; i++){
		
			meanRes[i] = 0.0;
			
			for(int j = 0; j < 2*dim; j++){
				
				meanRes[i] += factor * Points[i*2*dim + j];
				
			}

		}
		
		
		for(int a = 0; a < dim; a++){
		
			for(int b = a; b < dim; b++){
				
				double sum = 0.0;	
			
				for(int i = 0; i < 2*dim; i++){
					
					sum += factor * (Points[a*2*dim + i] - meanRes[a])*(mean1[b] + Points[b*2*dim + i] - meanRes[b]);			

				}
		
				covRes[a*dim + b] = sum;
				covRes[b*dim + a] = sum;
		
			}
		
		}
		
		
		//Normalize Angle

		if(meanRes[0] < -M_PI)
			meanRes[0] += 2*M_PI;
			
		if(meanRes[0] > M_PI)
			meanRes[0] -= 2*M_PI;
			
			
/*		printf("MeanRes\n");
		
		for(int i = 0; i < dim; i++){
			
			printf("%f ", meanRes[i]);
		}
*/			
		
	
}


extern "C" void Eigen_TransformCNAlloLoc2CNAlloLocArt(double * mean1, double * cov1, double * meanRes, double * covRes, int dim){
	
		double covRes1[dim*dim];		
		double Points[2*dim*dim];		
		
		calculateNMatrixSquareRoot(cov1, covRes1, dim);
		
		for(int i = 0; i < dim; i++){
		
			for(int j = 0; j < dim; j++){
				
				Points[i*2*dim + j] = mean1[i] + covRes1[i*dim + j];
				
			}
			
			int a = 0;
			for(int j = dim; j < 2*dim; j++){
				
				Points[i*2*dim + j] = mean1[a] - covRes1[i*dim + a];
				a++;
			}
			
		}
		

		//Transform Points
		
		int numberNeg = 0;
		
		for(int i = 0; i < 2*dim; i++){
			
			Points[i] = Points[i] + M_PI/2.0;
			double x = Points[2*dim + i];
			double y = Points[4*dim + i];

			Points[2*dim + i] = -y/1000.0;
			Points[4*dim + i] = x/1000.0;			
			
		}
		
		
		
		double factor = 1.0/(2*dim);		
				
		for(int i = 0; i < dim; i++){
		
			meanRes[i] = 0.0;
			
			for(int j = 0; j < 2*dim; j++){
				
				meanRes[i] += factor * Points[i*2*dim + j];
				
			}

		}
		
		
		for(int a = 0; a < dim; a++){
		
			for(int b = a; b < dim; b++){
				
				double sum = 0.0;	
			
				for(int i = 0; i < 2*dim; i++){
					
					sum += factor * (Points[a*2*dim + i] - meanRes[a])*(mean1[b] + Points[b*2*dim + i] - meanRes[b]);			

				}
		
				covRes[a*dim + b] = sum;
				covRes[b*dim + a] = sum;
		
			}
		
		}
		
		
		//Normalize Angle

		if(meanRes[0] < -M_PI)
			meanRes[0] += 2*M_PI;
			
		if(meanRes[0] > M_PI)
			meanRes[0] -= 2*M_PI;
		
	
}
