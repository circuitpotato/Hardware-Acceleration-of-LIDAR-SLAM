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

int saveMatrix_row = 130;
int saveMatrix_column = 80;
void saveMatrixToCSV(const char* filename, double matrix[saveMatrix_row][saveMatrix_column]) {
    FILE* fp = fopen(filename, "w");
    openFileValidity(fp);

    for (int i = 0; i < saveMatrix_row; i++) {
        for (int j = 0; j < saveMatrix_column; j++) {
            if (j > 0) {
                fprintf(fp, ",");
            }
            fprintf(fp, "%.5f", matrix[i][j]); // Save with 5 decimal places
        }
        fprintf(fp, "\n"); // Move to the next row
    }
    fclose(fp);
}

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

    // Allocate memory for final scan points
    double* final_xs = (double*)malloc(valid_points * sizeof(double));
    double* final_ys = (double*)malloc(valid_points * sizeof(double));

    // Copy the valid points to the final scan data
    for (int i = 0; i < valid_points; i++) {
        final_xs[i] = xs[i];
        final_ys[i] = ys[i];

//        if (idx == 0){
//            printf("%f %f\n", xs[i], ys[i]);
//        }
    }

    // Free memory for temporary arrays
    free(xs);
    free(ys);

    // Create the ScanData structure to store the final scan data
    ScanData clean_scan_range;
    clean_scan_range.x = final_xs;
    clean_scan_range.x = final_xs;
    clean_scan_range.y = final_ys;
    clean_scan_range.size = valid_points;

    return clean_scan_range;

}

// Transform scan range & pose into world points in world frame
double * Transform (ScanData scan, const double pose[3], double tscan[][2]) {
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

    // test
    //printf("%f %f %f\n", pose[0], pose[1], pose[3]);

    return *tscan;
}

typedef struct {
    double * pose;
    int iBegin;
    int iEnd;
    int loopClosed;
    int loopTried;
} my_keyscans;

typedef struct {
    double (*points)[2];
    my_keyscans * keyscans;
} my_map;

// Initialise map
void Initialise (my_map *map, double pose[3], ScanData scan){
    double tscan[scan.size][2];

    // Allocate memory for points array
    map->points = (double (*)[2])malloc(scan.size * sizeof(double[2]));

    // First keyscan information
    Transform(scan, pose, tscan); // Populate tscan

//    for (int a = 0; a<scan.size; a++){
//        printf("%f %f\n", tscan[a][0], tscan[a][1]);
//    }


    // Copy tscan to map.points
    for (int i = 0; i < scan.size; i++) {
        map->points[i][0] = tscan[i][0];   // x-coordinate
        map->points[i][1] = tscan[i][1]; // y-coordinate
    }

    map->keyscans[0].pose = pose;
    map->keyscans[0].iBegin = 1;
    map->keyscans[0].iEnd = scan.size;
    map->keyscans[0].loopClosed = 1;
    map->keyscans[0].loopTried = 0;
}

typedef struct {
    double* x;
    double* y;
    int size;
} myLocalMap;

myLocalMap ExtractLocalMap(my_map map, double pose[3], ScanData scan, double borderSize) {
//    //test map values
//    for (int a = 0; a < map.keyscans->iEnd; a++){
//        printf("map point [%d] = %f %f\n", a, map.points[a][0], map.points[a][1]);
//    }

    double tscan[scan.size][2];

    // Allocate memory for points array
    double *xs = (double *) malloc(map.keyscans->iEnd * sizeof(double));
    double *ys = (double *) malloc(map.keyscans->iEnd * sizeof(double));

    Transform(scan, pose, tscan); // Populate tscan

//    //print values
//    printf("scan size %d\n", scan.size);
//    for (int a = 0; a < scan.size; a++){
//        printf("%f %f\n", tscan[a][0], tscan[a][1]);
//
//    }

    double minX = tscan[0][0];
    double minY = tscan[0][1];
    double maxX = tscan[0][0];
    double maxY = tscan[0][1];

    for (int i = 1; i < scan.size; i++) {
        if (tscan[i][0] < minX) {
            minX = tscan[i][0];
        }

        if (tscan[i][0] > maxX) {
            maxX = tscan[i][0];
        }

        if (tscan[i][1] < minY) {
            minY = tscan[i][1];
        }

        if (tscan[i][1] > maxY) {
            maxY = tscan[i][1];
        }
    }

    // Set top-left & bottom-right corner
    minX = minX - borderSize;
    minY = minY - borderSize;
    maxX = maxX + borderSize;
    maxY = maxY + borderSize;

//    printf("minX = %f\n", minX);
//    printf("minY = %f\n", minY);
//    printf("maxX = %f\n", maxX);
//    printf("maxY = %f\n", maxY);

    // Extract x and y-axis of localMap
    int valid_points = 0;
    for (int i = 0; i < map.keyscans->iEnd; i++) {
        if ((map.points[i][0] > minX) && (map.points[i][0] < maxX) && (map.points[i][1] > minY) &&
            (map.points[i][1] < maxY)) {

            xs[i] = map.points[i][0];
            ys[i] = map.points[i][1];
            valid_points++;
            //printf("%d\n",valid_points);
        }
    }


    // dump values into localMap struct array
    myLocalMap localMap_data;

    // Allocate memory for points array
    double *final_xs = (double *) malloc((valid_points)* sizeof(double));
    double *final_ys = (double *) malloc((valid_points)* sizeof(double));

    for (int i = 0; i < valid_points; i++) {
        final_xs[i] = xs[i];
        final_ys[i] = ys[i];
        //printf("localMap[%d] = %f %f\n", i, final_xs[i], final_ys[i]);
    }
//    printf("localMap size = %d\n", valid_points);

    // Free memory for temporary arrays
    free(xs);
    free(ys);

    localMap_data.x = final_xs;
    localMap_data.y = final_ys;
    localMap_data.size = valid_points;

    return localMap_data;
}

// Helper function to calculate the Euclidean distance
float euclidean_distance(int x1, int y1, int x2, int y2, int width) {
    return (float)sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Function to compute Euclidean distance transform
void euclidean_distance_transform(const int input[], float output[], int width, int height) {
    float MAX_DIST = 10;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x; // Calculate the 1D index from 2D coordinates
            if (input[index] != 0) {
                output[index] = 0;
            } else {
                float min_dist = MAX_DIST;
                for (int j = 0; j < height; ++j) {
                    for (int i = 0; i < width; ++i) {
                        int idx = j * width + i; // Calculate the 1D index from 2D coordinates
                        if (input[idx] != 0) {
                            float dist = euclidean_distance(x, y, i, j, width);
                            if (dist < min_dist) {
                                min_dist = dist;
                            }
                        }
                    }
                }
                output[index] = min_dist;
            }
        }
    }
}



typedef struct {
    int * occGrid;
    int sizeGridrow;
    int sizeGridcolumn;
    float * metricMap;
    double pixelSize;
    double * topLeftCorner;
} my_grid;


my_grid OccuGrid(myLocalMap localMap, double pixelSize){
    double minXY[2] = {localMap.x[0], localMap.y[0]};
    double maxXY[2] = {localMap.x[0], localMap.y[0]};
    double Sgrid[2];

    for(int a = 0; a < localMap.size; a++){
        if (localMap.x[a] < minXY[0]){
            minXY[0] = localMap.x[a];
        }
        if (localMap.x[a] > maxXY[0]){
            maxXY[0] = localMap.x[a];
        }
        if (localMap.y[a] < minXY[1]){
            minXY[1] = localMap.y[a];
        }
        if (localMap.y[a] > maxXY[1]){
            maxXY[1] = localMap.y[a];
        }
    }



    // Grid Size
    for (int a = 0; a < 2; a++) {
        minXY[a] = minXY[a] - (3 * pixelSize);
        maxXY[a] = maxXY[a] + (3 * pixelSize);
        Sgrid[a] = round((maxXY[a] - minXY[a]) / pixelSize) + 1;
    }

    int temp_grid[(int)Sgrid[1] * (int)Sgrid[0]];
    for (int i = 0; i < ((int)Sgrid[1] * (int)Sgrid[0]); i++){
        temp_grid[i] = 0;
    }

    double hits[2];
    double idx[localMap.size];
    //printf("Local Map size = %d\n", (int)localMap.size);
    //int grid[(int) Sgrid[1]*(int)Sgrid[0]];
    int *grid = (int *)malloc(sizeof(int) * (int)Sgrid[0] * (int)Sgrid[1]);

    for (int a = 0; a < localMap.size; a++){
        hits[0] = round((localMap.x[a] - minXY[0]) / pixelSize) + 1;
        hits[1] = round((localMap.y[a] - minXY[1]) / pixelSize) + 1;
        //printf("hits = %d %d\n", (int)hits[0], (int)hits[1]);

        idx[a] = ((hits[0] - 1) * Sgrid[1]) + hits[1];
        idx[a]--;   // index start from zero (minus 1)
        //printf("idx %d = %d\n", a, (int) idx[a]);
        temp_grid[(int) idx[a]] = 1;
    }

    int temp_grid_index = 0;
    for (int b = 0; b < (int)Sgrid[0]; b++) {
        for (int a = 0; a < (int)Sgrid[1]; a++) {
            int grid_index = a * (int)Sgrid[0] + b; // Calculate the 1D index for the 'grid' array
            grid[grid_index] = temp_grid[temp_grid_index];
            temp_grid_index++;
        }
    }

    //float metric_map[(int) Sgrid[1]*(int)Sgrid[0]];
    float *metric_map = (float *)malloc(sizeof(float) * (int)Sgrid[0] * (int)Sgrid[1]);

    euclidean_distance_transform(grid, metric_map, (int) Sgrid[0], (int) Sgrid[1]);

    my_grid gridmap;

    gridmap.occGrid = (int *)grid;
    gridmap.sizeGridrow = (int) Sgrid[1];
    gridmap.sizeGridcolumn = (int) Sgrid[0];
    gridmap.metricMap = (float *)metric_map;
    gridmap.pixelSize = pixelSize;
    gridmap.topLeftCorner = minXY;
    return gridmap;
}

double angdiff(double alpha, double beta) {
    double delta = fmod((beta - alpha + M_PI), 2 * M_PI) - M_PI;
    return delta;
}

void DiffPose(double pose1[3], double pose2[3], double dp[3]){
    for (int i = 0; i < 3; i++){
        dp[i] = pose2[i] - pose1[i];
    }
    dp[2] = angdiff(pose1[2], pose2[2]);
}

/***********Main Code************/
int main() {
    double pose[3] = {0,0,0};
    my_map map;

    int miniUpdated = 0;

    // Map parameters
    double borderSize = 1;
    double pixelSize = 0.2;

    // Initialize map values
    map.points = NULL; // Initialize to NULL for safety
    map.keyscans = malloc(sizeof(my_keyscans)); // Allocate memory for a single my_keyscans element

    // Call the function to set Lidar parameters
    LidarParameters lidar = SetLidarParameters();

    // save LIDAR dataset into test_input_memory[row][column]
    readDatasetFromFile("C:/Lynn_SSD/NUS_Tingzz/EE4002D/Research/code_dump/my_CSM_C_test/A.csv");

    /*******Software Calculation Process*****/
    /*******Timing Starts here***************/

    // initiate first scan
    for (int i = 0; i < row; i++) {
        ScanData scan = readAScan(lidar, test_input_memory, i, 24);     // read a clean scan range
        printf("%d\n", i);


        // convert to transformeed scan range
        // use "theoretical pose" to test
        if (i == 0){
            Initialise(&map, pose, scan);
            miniUpdated = 1;

            // To test on MATLAB
            //saveMatrixToCSV("C:/Users/lynth/Desktop/output.csv", map.points );
            continue;
        }

        /*==================================Matching current scan to local map =======================================*/
        if (miniUpdated == 1){
            //ExtractLocalMap(my_map map, double pose[3], ScanData scan, double borderSize, double localMap[][2])
            myLocalMap localMap = ExtractLocalMap(map, pose, scan, borderSize);
//                printf("size of localMap = %d\n", localMap.size);
//                printf("pose = %f %f %f\n", pose[0], pose[1], pose[2]);   // pose
//                for (int a = 0; a < localMap.size; a++) {
//                    printf("%f %f\n", localMap.x[a], localMap.y[a]);    // localMap values
//                }

            my_grid gridmap1 = OccuGrid(localMap,pixelSize);
            my_grid gridmap2 = OccuGrid(localMap,pixelSize);

            free(localMap.x);
            free(localMap.y);

//                //printf("gridmap1 pixelSize = %f\n", gridmap1.pixelSize);

//                printf("gridmap2 pixelSize = %f\n", gridmap2.pixelSize);


//            for (int a = 0; a < gridmap.sizeGridrow; a++){
//                for (int b = 0; b < gridmap.sizeGridcolumn; b++){
//                    if ((int)gridmap.metricMap[a * gridmap.sizeGridcolumn + b] == 0){
//                        printf("grid[%d][%d]\n", a, b);
//                        //printf("%d\n", a);
//                        //printf("%d\n", b);
//                    }
//                }
//            }


//                for (int a = 0; a < gridmap2.sizeGridrow; a++){
//                    for (int b = 0; b < gridmap2.sizeGridcolumn; b++){
//                        if ((int)gridmap2.metricMap[a * gridmap2.sizeGridcolumn + b] == 0){
//                            printf("grid[%d][%d]\n", a, b);
//                            //printf("%d\n", a);
//                            //printf("%d\n", b);
//                        }
//                    }
//                }

            free(gridmap1.occGrid);
            free(gridmap2.occGrid);
            free(gridmap1.metricMap);
            free(gridmap2.metricMap);
        }

        // free scan values if no need
        free(scan.x);
        free(scan.y);
    }


    /*******Timing Ends here*****************/

    free(lidar.angles);     // free dynamically allocated memory
//    free(map.points);
//    free(map.keyscans);
    return 0;
}
