#include <planning/pure_pursuit.hpp>

PurePursuit::PurePursuit(){}

PurePursuit::~PurePursuit(){}

void PurePursuit::init(Control control, Erod erod){
	//Set parameter.
	control_ = control;
	erod_ = erod;
	//Init controls.
	should_controls_.steering_angle = 0.0;
	should_controls_.velocity = 0.0;
	//Init current state.
	pose_.position = Eigen::Vector2d(0,0);
	pose_.orientation = Eigen::Vector4d(0,0,0,1);
	velocity_ = 0.0;
}

AckermannControl PurePursuit::calculateControls(std::vector<Eigen::Vector2d> path){
	//Update path.
	path_ = path;
	//Calculate controls.
	should_controls_.steering_angle = calculateSteering();
	should_controls_.velocity = 2.0; //calculateVel();
	return should_controls_;
}

double PurePursuit::calculateSteering(){
	//Empirical linear function to determine the look-ahead-distance.
	double lad = control_.k2_lad_s + control_.k1_lad_s*velocity_;
	lad = std::max(lad,control_.lowerbound_lad_s);
	lad = std::min(lad,control_.upperbound_lad_s);
	//Calculate reference steering index.
	int ref_index = path::indexOfDistanceFront(lad, path_);
	//Calculate distance to reference point.
	double distance = path::distanceToIndex(ref_index, path_);
	//Calculate slope.
	Eigen::Vector2d local_vector = pose_.globalToLocal(path_[ref_index]);
	float dy = local_vector(1);
	float dx = local_vector(0);
	float alpha = atan2(dy,dx);
	//Calculate steering angle using Pure Pursuit.
	std::cout << "Distance wheel axis: " << erod_.distance_wheel_axis << std::endl;
	double steering_angle = atan2(2*erod_.distance_wheel_axis*sin(alpha),distance);
	std::cout << "LAD: " << lad << ", distance: " << distance << ", dx: " 
			  << dx << ", dy: " << dy << " and angle: " << steering_angle << std::endl;
	return steering_angle;
}

double PurePursuit::calculateVel(){
	//First calculate optimal velocity
	//for the moment take curvature at fix distance lad_v
	double lad = control_.k2_lad_v + control_.k1_lad_s*velocity_;
	//Calculate reference curvature index.
	int ref_index = path::indexOfDistanceFront(lad, path_);
	//Find upper velocity limits (physical, safety and teach).	
	double ref_velocity = sqrt(control_.max_lateral_acceleration*curveRadius());
	ref_velocity= std::min(ref_velocity, control_.max_absolute_velocity);
	return ref_velocity;
}

std_msgs::Float32MultiArray PurePursuit::getControlsMsg(){
	std_msgs::Float32MultiArray control_msg;
	control_msg.data.push_back(should_controls_.steering_angle);
	control_msg.data.push_back(should_controls_.velocity);
	return control_msg;
}

double PurePursuit::curveRadius(){
	int count=0;
	double radius_sum = 0.0;
	for(int t=1;t<=3;t++){	
		count++;
		double inter_dis = control_.distance_interpolation/t;
		int n_front = path::indexOfDistanceFront(inter_dis,path_);
		int n_back = 0;
		Eigen::Vector2d i_back = path_[n_back];
		Eigen::Vector2d i_front = path_[n_front];
		Eigen::Vector2d back_front = i_front - i_back;
		//Angle between i back and i front.
		double argument = i_back.dot(-i_front)/i_back.squaredNorm();
		if(argument > 1.0) argument = 1.0;
		if(argument < -1.0) argument = -1.0;
		float gamma = acos(argument);
		if(fabs(sin(gamma))<=0.01) radius_sum += 9999999;
		else radius_sum += back_front.norm()/(2*sin(gamma));	
	}
	double radius = radius_sum/count;
	//Preventing radius from nan.
	if(radius > 2000) radius = 2000;
	return radius;
}

void PurePursuit::setPose(Pose pose){pose_ = pose;}

void PurePursuit::setVelocity(double velocity){
	if(velocity >= 0.0) velocity_ = velocity;
}