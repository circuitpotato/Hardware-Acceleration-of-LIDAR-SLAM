#include "tools.h"
#include<Eigen>
#include<iostream>
#include <random>
#include<cmath>
#include <stdio.h>
#include <stdlib.h>
#define row 3480
#define column 1079
#define PARTICLE_NUMBER 16
#define MAX_LANDMARKS 100
float test_input_memory[row][column];
using namespace std;
typedef struct {
    float* x;
    float* y;
    int size;
} ScanData;

void openFileValidity(FILE* fp){
    if (fp == NULL) {
        printf("Failed to open the file.\n");
        return;
    }
}

void readDatasetFromFile(const char* filename){
    FILE *fp;

    // open dataset to read
    fp = fopen(filename, "r");
    openFileValidity(fp);     // check if file can be open

    // Read float values from file
    for (int i = 0; i < row; i++){
        float value;
        if (fscanf(fp, "%f,", &value) != 1) {
            printf("Failed to read float value %d from the file.\n", i + 1);
            fclose(fp);
            exit(1);
        }
        test_input_memory[i][0] = value;
        for (int k = 1; k < column; k++){
            if (fscanf(fp, "%f,", &value) != 1) {
                printf("Failed to read float value %d from the file.\n", k + 1);
                fclose(fp);
                exit(1);
            }
            test_input_memory[i][k] = value;
        }
    }

    fclose(fp);
}

typedef struct {
    float angle_min;
    float angle_max;
    float angle_increment;
    int npoints;
    float range_min;
    float range_max;
    float scan_time;
    float time_increment;
    float angles[column]; // Array of angles (dynamic allocation)
} LidarParameters;


typedef struct {
    float x;
    float y;
} CartesianPoint;

CartesianPoint polarToCartesian(float r, float theta) {
    CartesianPoint cartesian;

    cartesian.x = r * cosf(theta);
    cartesian.y = r * sinf(theta);

    return cartesian;
}

LidarParameters SetLidarParameters() {
    LidarParameters lidar;

    lidar.angle_min = (float)-2.351831;
    lidar.angle_max = (float)2.351831;
    lidar.angle_increment = (float)0.004363;
    lidar.npoints = column;
    lidar.range_min = (float)0.023;
    lidar.range_max = 60;
    lidar.scan_time = (float)0.025;
    lidar.time_increment = (float)1.736112e-05;


    float angle = lidar.angle_min;
    for (int i = 0; i < lidar.npoints; i++) {
        lidar.angles[i] = angle;
        angle += lidar.angle_increment;
    }
    return lidar;
}

ScanData  readAScan_2(const LidarParameters lidar, const float scan[row][column], const int idx, const int usableRange){
    float maxRange = (lidar.range_max < (float)usableRange) ? lidar.range_max : (float)usableRange;

    float xs[row];
    float ys[row];

    int valid_points = 0;
    for (int i = 0; i < column; i++){
        if ((scan[idx][i] < lidar.range_min) | (scan[idx][i] > maxRange)){
            continue;   // skip if range is bad
        }
        else{
            CartesianPoint point = polarToCartesian(scan[idx][i], lidar.angles[i]);
            xs[valid_points] = point.x;
            ys[valid_points] = point.y;
            valid_points++;
        }
    }


    // Create the ScanData structure to store the final scan data
    ScanData clean_scan_range;
    clean_scan_range.x = xs;
    clean_scan_range.y = ys;
    clean_scan_range.size = valid_points;

    return clean_scan_range;

}

struct pose_data{
    Eigen::Vector3d true_pose; //location with noise included
    Eigen::Vector3d ideal_pose; //location without included noise 
};

struct landmark{
    int id;
    Eigen::Vector2d location_mean; //mean of a landmark location
    Eigen::Matrix2d location_variance; //variance of a landmark location, in an array
};
struct sensor_input{
    float distance;
    float angle;
};

Eigen::Vector3d motion_command;
struct robot{
    Eigen::Matrix3d motion_uncertainity;
    Eigen::Matrix2d sensor_uncertainity;
    pose_data Location;
    landmark map[MAX_LANDMARKS];


};
struct Particle{
    float weight;
    Eigen::Vector3d pose;
    int num_landmarks;
    landmark map[MAX_LANDMARKS];
    int id;
};
Eigen::MatrixXd jacobian(const VectorXd& x_state){
      /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);
    
    //check division by zero
    if(fabs(c1) < 0.0001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }
    
    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;

}
Eigen::Vector2d simplifiy_pose(Eigen::Vector3d Pose){ //removes angle data from pose
    Eigen::Vector2d simp_pose;
    simp_pose << Pose[0],Pose[1];
    return simp_pose;
}
int normal_distribution_sample(float mu, float sigma){
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution d{mu, sigma};
    return d(gen);
}//returns a sample from a certain normal distribution
Eigen::Vector3d motion_model(Eigen::Vector3d u, Eigen::Vector3d pose){
    Eigen::Vector3d sigma;
    sigma << (0.04), (0.04) , (0.01); //inputting the sigma variations
    Eigen::Vector3d noisy_u;
    noisy_u << normal_distribution_sample(0,sigma[0]),normal_distribution_sample(0,sigma[1]),normal_distribution_sample(0,sigma[2]); //creates noise with regards to the motion
    return pose + u + noisy_u;  
}
Eigen::Vector2d senosr_model(Eigen::Vector2d read){
    Eigen::Vector2d sigma;
    sigma << (0.08), (0.08); //inputting the sigma variations
    Eigen::Vector2d noisy_read;
    noisy_read << normal_distribution_sample(0,sigma[0]),normal_distribution_sample(0,sigma[1]); //creates noise with regards to the motion
    return read + noisy_read;
}
//motion model random distribution
//adjusts the sensor input from distance and angle, to global pose
Eigen::Vector2d sensor_adjustment(sensor_input input,Eigen::Vector3d pose){
    Eigen::Vector2d dif_distance;
    dif_distance << (cos(input.angle)*input.distance) + pose[0],(sin(input.angle)*input.distance) + pose[1];
    return dif_distance;
}

// Define motion model, sensor model, and other parameters 
void weight_update(Particle* particle,Eigen::Vector2d Measurment, int lm_id){
    Eigen::Vector3d pose = particle -> pose;
    Eigen::Vector2d simp_pose;
    simp_pose =simplifiy_pose(pose);
    landmark target = particle ->map[lm_id];
    Eigen::Matrix2d Q_cov;
    Q_cov << (0.08),(0.08);
    Eigen::Vector2d Measurment_error = Measurment - (target.location_mean - simp_pose); //get's the difference between the real and the predicted error
    double det = Q_cov.determinant();
    double exponentTerm = -0.5 * Measurment_error.transpose() * Q_cov * Measurment_error;
    Eigen::Matrix2d inv_cov = Q_cov.inverse();
    double likelihood = 1.0 / (sqrt(2.0 * M_PI) * sqrt(det)) * exp(exponentTerm);
    particle -> weight = likelihood;
};
void new_landmark_map(Particle* particle, Eigen::Vector2d Measurment){
    //adds new landmark after being detected by sensor with dafualt covariance
    const Eigen::Vector2d Sensor_covariance;
    landmark new_landmark;
    Eigen::Vector2d location;
    new_landmark.location_mean = Measurment;
    new_landmark.location_variance << 0.08,0.00,0.00,0.08;
    int new_lm_id = (particle ->num_landmarks) + 1 ;
    particle -> map[new_lm_id] = new_landmark;
    particle -> num_landmarks += 1;
}
void Kalman_Filter(Eigen::Vector2d sensor_reading, landmark* lm,Eigen::Vector2d pose){
    //H is ignored here because the measrument is preprocessed into the same format
    Eigen::Matrix2d mov_cov;
    mov_cov << (0.04), (0.04);
    Eigen::Vector2d init_guess = lm ->location_mean;
    Eigen::Matrix2d lm_cov = lm -> location_variance;
    //adjust the existing landmark measurment for the current pose
    Eigen::Vector2d predicted_loc_mean = init_guess - pose;
    Eigen::Matrix2d predicted_loc_cov  =  lm_cov + mov_cov; //no need for the A * P * A.transpose() because those vectosrs are just [1 0; 0 1]
    //get the sensor covariance
    Eigen::Matrix2d sens_cov;
    sens_cov << (0.01),(0.01);
    //get the difference between the measured result and the sensor reading
    Eigen::Vector2d Y_residual = predicted_loc_mean - predicted_loc_mean;
    Eigen::Matrix2d S_cov = predicted_loc_cov + sens_cov;
    Eigen::Matrix2d Kalman_Gain = predicted_loc_cov*S_cov.inverse();
    //include line for extending the kalman filter later
    lm ->location_mean = init_guess + Kalman_Gain*Y_residual;
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    lm -> location_variance = (I - Kalman_Gain)*predicted_loc_cov;
};

float distance_finder(Eigen::Vector2d measurment){
    //finds the distance via pythagores assuming it's centered on the origin
    return sqrt(measurment[0]*measurment[0] + measurment[1]*measurment[1]);
} 

int new_landmark_detector(Particle particle, Eigen::Vector2d measurment){
    //Returns negative if no good match, otherwise returns the ID
    int output_ident = -1;
     for(int lm_id=0; lm_id <= particle.num_landmarks; lm_id++){
        Eigen::Vector2d error_vector;
        Eigen::Vector2d lm_mean = particle.map[lm_id].location_mean;
        Eigen::Vector2d pose = simplifiy_pose(particle.pose);
        float error_amount,existing_error = 100000;
        lm_mean = lm_mean - pose;
        error_vector = lm_mean - measurment;
        error_amount = distance_finder(error_vector);
        if(error_amount < existing_error && error_amount < 0.1){
            int output_ident = lm_id;
            error_amount = existing_error;
        }
        /// float dist = ((lm_mean - Sensor_reading ) + ()) ** (0.5);
     }
     return output_ident;
}
/*
Eigen::Vector2d get_new_sensor_input(){
    Eigen::Vector2d placeholder;
    return placeholder;
}
*/
int main() { 
    Particle particle_array[PARTICLE_NUMBER];
    Eigen::Vector3d starting_pose;
    starting_pose << (0),(0),(0);
    Particle New_Particle_array[PARTICLE_NUMBER];
    ScanData scan;
    for(int particle_num=0; particle_num < PARTICLE_NUMBER; particle_num++ ){
        Particle init_particle;
        init_particle.weight = 1.0/PARTICLE_NUMBER;
        init_particle.pose = starting_pose;
        init_particle.id = particle_num;
        particle_array[particle_num] = init_particle;
    }

    // Call the function to set Lidar parameters
    LidarParameters lidar = SetLidarParameters();

    // save LIDAR dataset into test_input_memory[row][column]
    readDatasetFromFile("A.csv");
    for (size_t i = 0; i < 3000; i++)
    {
        scan = readAScan_2(lidar, test_input_memory, i, 24);
        Eigen::Vector2d z;
        z << *scan.x,*scan.y;
        cout << "x_cords:" << *scan.x << " ycords:" << *scan.y << "\n";
        std::vector<float> weight_list;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::cout << "output: ";
        float total_weight = 0;

        // ----- Prediction Step -----
        // Update motion model based on control inputs
        Eigen::Vector3d control_input; //placeholder for later
        control_input << 0,0,0;
        for (int particle_num=0; particle_num < PARTICLE_NUMBER; particle_num++){
             Particle *particle_mov ; 
             particle_mov = &particle_array[particle_num];
             particle_mov->pose = motion_model(control_input,particle_mov->pose) ;

        }

        for (int particle_num=0; particle_num < PARTICLE_NUMBER; particle_num++){
             Particle particle_mov ; 
             particle_mov = particle_array[particle_num];
                  int lm_id_sensor = new_landmark_detector(particle_mov,z);
             if(lm_id_sensor == -1){
                //add new landmark
                //cout << "new landmark" << "\n";
                new_landmark_map(&particle_mov,z);
             }
             else{
                cout << "detected Previous landmark" << "/n";
                //also update wieght of the particles based on how well the measurment matches
                weight_update(&particle_mov,z,lm_id_sensor); //possible pointer memoery issues
                //uses the sensor update to improve the existing slam 
                landmark *landmark_updated = &particle_mov.map[lm_id_sensor];
                Kalman_Filter(z,landmark_updated,simplifiy_pose(particle_mov.pose));
             }
             total_weight = particle_mov.weight + total_weight;
             weight_list.push_back(particle_mov.weight);
        }
        //resample the particles to ensure the total weight is equal to 1
        for (int particle_num=0; particle_num < PARTICLE_NUMBER; particle_num++){
            std::discrete_distribution<int> distribution(weight_list.begin(), weight_list.end());
            Particle* particle_mov;
            int new_particle_id = distribution(gen);
            New_Particle_array[particle_num] = particle_array[new_particle_id];
        }
        //copy over the array into the mainstream and adds some variation into the copied over particles
        for (int particle_num=0; particle_num < PARTICLE_NUMBER; particle_num++){
            particle_array[particle_num] = New_Particle_array[particle_num];
            Particle* particle =  &particle_array[particle_num];
            particle -> pose = (particle -> pose); 
    } 
        }
     /*
    

    // Initialize state vector (robot pose + map landmarks)
    /*Particle New_Particle_array[PARTICLE_NUMBER]; 
        for(int particle_num=0; particle_num <= PARTICLE_NUMBER; particle_num++ ){
        Particle init_particle;
        init_particle.weight = 1.0/PARTICLE_NUMBER;
        init_particle.pose = starting_pose;
        init_particle.id = particle_num;
        particle_array[particle_num] = init_particle;
    }

    // Initialize covariance matrix
    bool cont; // Activate the code so long as there

    while (cont) {
        std::vector<float> weight_list;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::cout << "help";
        float total_weight = 0;

        // ----- Prediction Step -----
        // Update motion model based on control inputs
        Eigen::Vector2d control_input; //placeholder for later
        for (int particle_num=0; particle_num <= PARTICLE_NUMBER; particle_num++){
             Particle *particle_mov ; 
             particle_mov = &particle_array[particle_num];
             particle_mov->pose = motion_model(control_input,particle_mov->pose) ;

        }
        Eigen::Vector2d z = get_new_sensor_input();
        for (int particle_num=0; particle_num <= PARTICLE_NUMBER; particle_num++){
             Particle particle_mov ; 
             particle_mov = particle_array[particle_num];
             int lm_id_sensor = new_landmark_detector(particle_mov,z);
             if(lm_id_sensor == -1){
                //add new landmark
             }
             else{
                //also update wieght of the particles based on how well the measurment matches
                weight_update(&particle_mov,z,lm_id_sensor); //possible pointer memoery issues
                //uses the sensor update to improve the existing slam 
                landmark *landmark_updated = &particle_mov.map[lm_id_sensor];
                Kalman_Filter(z,landmark_updated,particle_mov.pose);
             }
             total_weight = particle_mov.weight + total_weight;
             weight_list.push_back(particle_mov.weight);
        }
        //resample the particles to ensure the total weight is equal to 1
        for (int particle_num=0; particle_num <= PARTICLE_NUMBER; particle_num++){
            std::discrete_distribution<int> distribution(weight_list.begin(), weight_list.end());
            Particle* particle_mov;
            int new_particle_id = distribution(gen);
            New_Particle_array[particle_num] = particle_array[new_particle_id];
        }
        //copy over the array into the mainstream and adds some variation into the copied over particles
        for (int particle_num=0; particle_num <= PARTICLE_NUMBER; particle_num++){
            particle_array[particle_num] = New_Particle_array[particle_num];
            Particle* particle =  &particle_array[particle_num];
            particle -> pose = (particle -> pose); 
    } */
    cout << "final";
    return 0;}

