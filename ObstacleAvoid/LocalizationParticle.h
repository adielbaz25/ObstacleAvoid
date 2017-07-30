
#ifndef PARTICLE_H_
#define PARTICLE_H_

//this class represent a particale in the map window
class LocalizationParticle {
public:
	int row, col; //the row/col index in the map
	double x, y; //the exact place of the robot; x- place in column, y- place in row
	double yaw; //the heading angle of the robot
	double belief;

	//a deafult constructor
	LocalizationParticle();
	//constructor
	LocalizationParticle(int i, int j, double x, double y, double yaw, double belief);
	//disconstructor
	~LocalizationParticle();
};

#endif /* PARTICLE_H_ */
