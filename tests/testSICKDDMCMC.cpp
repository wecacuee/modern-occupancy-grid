#include "../OccupancyGrid/include/OccupancyGrid.h"
#include "../OccupancyGrid/include/MCMC.h"


int main(int argc, char *argv[]){

	if(argc != 4){
		printf("ERROR [USAGE]: executable width height resolution");
		exit(1);
	}
	double width 		=	atof(argv[1]); 		//meters
	double height 		= 	atof(argv[2]); 		//meters
	double resolution 	= 	atof(argv[3]); 	//meters
	OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle

	//======== load odometry ==========//
		vector<Pose2> pose;
		FILE *fptr = fopen("Data/SICK_Odometry.txt","r");
		while(!feof(fptr)){
			double x, y, theta;
			fscanf(fptr, "%lf %lf %lf", &x, &y, &theta);
			theta *= 3.174159/180.0;

			pose.push_back( *(new Pose2(x,y,theta)));
		}
		fclose(fptr);

	//======= load laser scans ===============//
		fptr = fopen("Data/SICK_Snapshot.txt","r");
		if(fptr == NULL){
			printf("Cannot open file...exiting\n");
			exit(1);
		}

		double min_angle, max_angle, step;
		double max_dist, min_dist;
		int n_readings;

		fscanf(fptr, "%lf %lf %lf %lf %lf %d", &min_angle, &max_angle, &step, &max_dist, &min_dist, &n_readings);
		double range;
		Index it = 0;
		while(!feof(fptr)){
			double theta = 3.14159/2 + pose[it].theta() + min_angle; //compute angle of first laser
			for(int i = 0; i < n_readings && !feof(fptr); i++){
				fscanf(fptr, "%lf", &range);	//load range
				if(range < max_dist){
					occupancyGrid.addLaser(Pose2(pose[it].y()/1000.0, pose[it].x()/1000.0, theta), range); //add laser to grid
				}
				theta += step;	//increment angle for next laser
			}
			it++;
		}


		fclose(fptr);

		occupancyGrid.saveLaser("Data/lasers.lsr");
		occupancyGrid.saveHeatMap("Data/HeatMap.ppm");

	//run metropolis
	OccupancyGrid::Marginals occupancyMarginals = runDDMCMC(occupancyGrid, 1000000);

	char marginalsOutput[1000];
			sprintf(marginalsOutput, "Data/DDMCMC_Marginals.txt");
	fptr = fopen(marginalsOutput, "w");
	fprintf(fptr, "%lu %lu\n", occupancyGrid.width(), occupancyGrid.height());

	for(int i = 0; i < occupancyMarginals.size(); i++){
		fprintf(fptr, "%lf ", occupancyMarginals[i]);
	}

	fclose(fptr);
}


