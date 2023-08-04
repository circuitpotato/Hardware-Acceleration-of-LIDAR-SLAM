// Code by: CW Lynn
// Purpose: This C code takes in raw scan values from LIDAR and converts it to its appropriate values in ARM
// Implementation: Vitis ARM processor
// Output: global scan, pose, score

#include <stdio.h>
#include <stdlib.h> // For exit()
#define row 5522
#define column 1079
#include <math.h>

float test_input_memory[row][column];

typedef struct {
    double angle_min;
    double angle_max;
    double angle_increment;
    int npoints;
    double range_min;
    double range_max;
    double scan_time;
    double time_increment;
    double* angles; // Array of angles (dynamic allocation)
} LidarParameters;

LidarParameters SetLidarParameters() {
    LidarParameters lidar;

    lidar.angle_min = -2.351831;
    lidar.angle_max = 2.351831;
    lidar.angle_increment = 0.004363;
    lidar.npoints = 1079;
    lidar.range_min = 0.023;
    lidar.range_max = 60;
    lidar.scan_time = 0.025;
    lidar.time_increment = 1.736112e-05;

    // Allocate memory for angles array and populate it
    lidar.angles = (double*)malloc((lidar.npoints) * sizeof(double));
    if (lidar.angles != NULL) {
        double angle = lidar.angle_min;
        for (int i = 0; i < lidar.npoints; i++) {
            lidar.angles[i] = angle;
            angle += lidar.angle_increment;
        }
    } else {
        printf("Failed to allocate memory for angles array.\n");
        // Handle error or exit the program
    }

    return lidar;
}

// check if file can be open
void openFileValidity(FILE* fp){
    if (fp == NULL) {
        printf("Failed to open the file.\n");
        return;
    }
}

// Convert matrix to CSV file
// Used to test data on MATLAB
void saveMatrixToCSV(const char* filename, float matrix[row][column]) {
    FILE* fp = fopen(filename, "w");
    openFileValidity(fp);

    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            if (j > 0) {
                fprintf(fp, ",");
            }
            fprintf(fp, "%.5f", matrix[i][j]); // Save with 5 decimal places
        }
        fprintf(fp, "\n"); // Move to the next row
    }
    fclose(fp);
}


//// save LIDAR dataset
//// clean up LIDAR dataset into scans
//void readDatasetFromFile(const char* filename){
//    FILE *fp;
//
//    // open dataset to read
//    fp = fopen(filename, "r");
//    openFileValidity(fp);     // check if file can be open
//
//    // Read float values from file
//    for (int i = 0; i < row; i++){
//        float value;
//        if (fscanf(fp, "%f,", &value) != 1) {
//            printf("Failed to read float value %d from the file.\n", i + 1);
//            fclose(fp);
//            exit(1);
//        }
//        test_input_memory[i][0] = value;
//        for (int k = 1; k < column; k++){
//            if (fscanf(fp, "%f,", &value) != 1) {
//                printf("Failed to read float value %d from the file.\n", k + 1);
//                fclose(fp);
//                exit(1);
//            }
//            test_input_memory[i][k] = value;
//        }
//    }
//
//    fclose(fp);
//}

// save LIDAR dataset
// clean up LIDAR dataset into scans
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
    double x;
    double y;
} CartesianPoint;

CartesianPoint polarToCartesian(double r, double theta) {
    CartesianPoint cartesian;

    cartesian.x = r * cos(theta);
    cartesian.y = r * sin(theta);

    return cartesian;
}

// Data structure for readAScan function
typedef struct {
    double* x;
    double* y;
    int size;
} ScanData;

// Read a clean scan range
ScanData readAScan(LidarParameters lidar, float scan[row][column], int idx, int usableRange){
    double maxRange = (lidar.range_max < usableRange) ? lidar.range_max : usableRange;

    // Allocate memory for Cartesian coordinates (x, y) of the scan points
    double* xs = (double*)malloc(row * sizeof(double));
    double* ys = (double*)malloc(row * sizeof(double));

    int valid_points = 0;
    for (int i = 0; i < column; i++){
        if ((scan[idx][i] >= lidar.range_max) | (scan[idx][i] <= maxRange)){
            CartesianPoint point = polarToCartesian(scan[idx][i], lidar.angles[i]);
            xs[valid_points] = point.x;
            ys[valid_points] = point.y;
            valid_points++;
        }
    }

    // Allocate memory for final scan points
    double* final_xs = (double*)malloc(valid_points * sizeof(double));
    double* final_ys = (double*)malloc(valid_points * sizeof(double));

    // Copy the valid points to the final scan data
    for (int i = 0; i < valid_points; i++) {
        final_xs[i] = xs[i];
        final_ys[i] = ys[i];
    }

    // Free memory for temporary arrays
    free(xs);
    free(ys);

    // Create the ScanData structure to store the final scan data
    ScanData clean_scan_range;
    clean_scan_range.x = final_xs;
    clean_scan_range.y = final_ys;
    clean_scan_range.size = valid_points;

    return clean_scan_range;

}

// Transform scan range & pose into world points in world frame
double * Transform (ScanData scan, const double pose[3], double tscan [][2]) {
    double tx = pose[0];
    double ty = pose[1];
    double theta = pose[2];

    double ct = cos(theta);
    double st = sin(theta);
    double R[2][2] = {{ct, -st}, {st,ct}};
    for (int i = 0; i < scan.size; i++){

        // multiply scan(x,y) by transformed R
        // scan is (N,2) matrix and R is (2,2) matrix
        double transformed_x = R[0][0] * scan.x[i] + R[1][0] * scan.y[i];
        double transformed_y = R[0][1] * scan.x[i] + R[1][1] * scan.y[i];

        // Translate to points on world frame
        tscan[i][0] = transformed_x + tx;
        tscan[i][1] = transformed_y + ty;
    }
    return (double *) tscan;
}


typedef struct {
    double pose[3];
    int iBegin;
    int iEnd;
    int loopClosed;
    int loopTried;
} my_keyscans;

typedef struct {
    my_keyscans* keyscans;
    double* points;
    int numKeyScans; // Added a field to keep track of the number of keyscans
} my_map;

my_map Initialise(my_map map, ScanData scan, const double pose[3]) {
    double tscan[scan.size][2];

    // Dynamically allocate memory for the keyscans array if it's not already allocated
    if (map.keyscans == NULL) {
        map.keyscans = (my_keyscans*)malloc(sizeof(my_keyscans));
        map.numKeyScans = 1; // Update the number of keyscans
    } else {
        // Update the size of the keyscans array based on the number of keyscans you want to add
        int numKeyScansToAdd = 1;
        my_keyscans* tmp = (my_keyscans*)realloc(map.keyscans, (map.numKeyScans + numKeyScansToAdd) * sizeof(my_keyscans));

        if (tmp != NULL) {
            map.keyscans = tmp;
            map.numKeyScans += numKeyScansToAdd; // Update the number of keyscans
        } else {
            // Handle the case when realloc fails (memory allocation failed)
            // You might want to print an error message or take appropriate actions
            // In this case, the original map.keyscans remains valid, and you should not lose the original buffer
            // The map.numKeyScans will not be updated to reflect the failure to add the new keyscan
        }
    }

    // Perform the transformation and set the points field
    map.points = Transform(scan, pose, tscan);

    // Set the values for the new keyscan
    int newKeyScanIndex = map.numKeyScans - 1; // index of the newly added keyscan
    map.keyscans[newKeyScanIndex].pose[0] = pose[0];
    map.keyscans[newKeyScanIndex].pose[1] = pose[1];
    map.keyscans[newKeyScanIndex].pose[2] = pose[2];
    map.keyscans[newKeyScanIndex].iBegin = 1;
    map.keyscans[newKeyScanIndex].iEnd = scan.size;
    map.keyscans[newKeyScanIndex].loopClosed = 1;
    map.keyscans[newKeyScanIndex].loopTried = 0;

    return map;
}



/***********Main Code************/
int main() {
    double pose[3] = {0,0,0};   // initialise pose

    // Initialise map values
    my_map map;
    map.points = NULL;
    map.keyscans = NULL;


    // Call the function to set Lidar parameters
    LidarParameters lidar = SetLidarParameters();

    // save LIDAR dataset into test_input_memory[row][column]
    readDatasetFromFile("C:/Lynn_SSD/NUS_Tingzz/EE4002D/Research/code_dump/my_CSM_C_test/A.csv");

    /*******Software Calculation Process*****/
    /*******Timing Starts here***************/
    for (int i = 0; i < row; i++) {
        ScanData scan = readAScan(lidar, test_input_memory, i, 24);     // read a clean scan range

        if (i == 0) {
            Initialise(map, scan, pose);
            for(int a = 0; a < scan.size; a++){
                printf("%f %f\n", map.points[0], map.points[1]);
            }
        }

        // convert to transformeed scan range
        // use "theoretical pose" to test

//        if (i == 0){
//            for (int a = 0; a < scan.size; a++){
//                printf("point %d: %f %f\n", (a+1), tscan[a][0], tscan[a][1]);
//            }
//        }

    }


    /*******Timing Ends here*****************/


    // To test on MATLAB
    saveMatrixToCSV("C:/Users/lynth/Desktop/output.csv", test_input_memory);

    return 0;
}

