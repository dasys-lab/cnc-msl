/*
 * $Id: CovarianceHelper.cpp 2032 2007-04-11 20:19:07Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "CovarianceHelper.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <sys/time.h>
//#include <GoalMessage.h>
//#include <FreeAreaMessage.h>
//#include <YellowGoalInfo.h>
//#include <BlueGoalInfo.h>
//#include <YellowFreeAreaInfo.h>
//#include <BlueFreeAreaInfo.h>
#include "SpicaHelper.h"
#include "FootballField.h"

//#include "../global/Packets.h"
//#include "PacketHelper.h"


void CovarianceHelper::calculateNMatrixSquareRoot(double * cov, double * covRes, int dim){
	
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



void CovarianceHelper::TransformCovMatrixPolar2Ego(double * mean1, double * cov1, double * meanRes, double * covRes, int dim){
	
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
		
		for(int i = 0; i < 2*dim; i++){
			
			double alpha = Points[i];
			double dist = Points[2*dim + i];
			
			Points[i] = cos(alpha)*dist;
			Points[2*dim + i] = sin(alpha*dist);
			
		}
		
		//End Transform Points
		
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
	
	
}
