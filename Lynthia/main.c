// Code by: CW Lynn
// Purpose: This C code takes in raw scan values from LIDAR and converts it to its appropriate values in ARM
// Implementation: Vitis ARM processor
// Output: global scan, pose, score


#include <string.h>
#include <stdio.h>
#include <stdlib.h> // For exit()

#define row 3480
#define column 1079
#include <math.h>

double test_input_memory[row][column];

typedef struct {
    double angle_min;
    double angle_max;
    double angle_increment;
    int npoints;
    double range_min;
    double range_max;
    double scan_time;
    double time_increment;
    double angles[column]; // Array of angles (dynamic allocation)
} LidarParameters;

LidarParameters SetLidarParameters() {
    LidarParameters lidar;

    lidar.angle_min = (double)-2.351831;
    lidar.angle_max = (double)2.351831;
    lidar.angle_increment = (double)0.004363;
    lidar.npoints = column;
    lidar.range_min = (double)0.023;
    lidar.range_max = 60;
    lidar.scan_time = (double)0.025;
    lidar.time_increment = (double)1.736112e-05;


    double angle = lidar.angle_min;
    for (int i = 0; i < lidar.npoints; i++) {
        lidar.angles[i] = angle;
        angle += lidar.angle_increment;
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

// save LIDAR dataset
// clean up LIDAR dataset into scans
void readDatasetFromFile(const char* filename){
    FILE *fp;

    // open dataset to read
    fp = fopen(filename, "r");
    openFileValidity(fp);     // check if file can be open

    // Read double values from file
    for (int i = 0; i < row; i++){
        float value;
        if (fscanf(fp, "%f,", &value) != 1) {
            printf("Failed to read double value %d from the file.\n", i + 1);
            fclose(fp);
            exit(1);
        }
        test_input_memory[i][0] = value;
        for (int k = 1; k < column; k++){
            if (fscanf(fp, "%f,", &value) != 1) {
                printf("Failed to read double value %d from the file.\n", k + 1);
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
ScanData readAScan(LidarParameters lidar, double scan[row][column], int idx, int usableRange){
    double maxRange = (lidar.range_max < (double)usableRange) ? lidar.range_max : (double)usableRange;

    double xs[row];
    double ys[row];

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
        double transformed_x = (double)(R[0][0] * scan.x[i] + R[1][0] * scan.y[i]);
        double transformed_y = (double)(R[0][1] * scan.x[i] + R[1][1] * scan.y[i]);



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
    int points_size;
    double (*points)[2];

    my_keyscans * keyscans;
} my_map;

// Initialise map
my_map Initialise (const double pose[3], const ScanData scan) {
    double tscan[scan.size][2];
    Transform(scan, pose, tscan); // Populate tscan

    my_map map;

    // Allocate memory for points array and copy values from tscan
    map.points = malloc(scan.size * sizeof(double[2]));
    for (int i = 0; i < scan.size; i++) {
        map.points[i][0] = tscan[i][0];
        map.points[i][1] = tscan[i][1];
    }

    map.points_size = scan.size;

    map.keyscans[0].pose = (double *)pose;
    map.keyscans[0].iBegin = 1;
    map.keyscans[0].loopClosed = 1;
    map.keyscans[0].loopTried = 0;

    return map;
}


typedef struct {
    double* x;
    double* y;
    int size;
} myLocalMap;

myLocalMap ExtractLocalMap(double map_points[][2], int points_size,const double pose[3], ScanData scan, const double borderSize) {
    double tscan[scan.size][2];

//    for (int p = 0; p < points_size; p++){
//        printf("%f %f\n", map_points[p][0], map_points[p][1]);
//    }

    // Allocate memory for points array
    double *xs = (double *) malloc(points_size * sizeof(double));
    double *ys = (double *) malloc(points_size* sizeof(double));

//    double xs[map.keyscans->iEnd];
//    double ys[map.keyscans->iEnd];
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
    for (int i = 0; i < points_size; i++) {
        //printf("%f %f\n", map.points[i][0], map.points[i][1]);
        if ((map_points[i][0] > minX) && (map_points[i][0] < maxX) && (map_points[i][1] > minY) &&
            (map_points[i][1] < maxY)) {

            xs[valid_points] = map_points[i][0];
            ys[valid_points] = map_points[i][1];
            //printf("%f %f\n", map_points[i][0], map_points[i][1]);
            valid_points++;
            //printf("%d\n",valid_points);
        }
    }


    // dump values into localMap struct array
    myLocalMap localMap_data;

    localMap_data.x = xs;
    localMap_data.y = ys;
    localMap_data.size = valid_points;

//    for (int o = 0; o < localMap_data.size; o++){
//        printf("local data x = %f\n", localMap_data.x[o]);
//    }
//
//    printf("local map size  = %d\n", valid_points);
    return localMap_data;
}


void euclidean_distance_transform(const int input[], double output[], const int width, const int height) {
    double MAX_DIST = 10;
    double precomputedDistances[width * height];

    // Precompute squared distances
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            precomputedDistances[j * width + i] = (double)(i * i) + (double)(j * j);
        }
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            if (input[index] != 0) {
                output[index] = 0;
            } else {
                double min_dist_squared = MAX_DIST * MAX_DIST;
                for (int j = 0; j < height; ++j) {
                    for (int i = 0; i < width; ++i) {
                        int idx = j * width + i;
                        if (input[idx] != 0) {
                            double dist_squared = precomputedDistances[idx] + precomputedDistances[index];
                            if (dist_squared < min_dist_squared) {
                                min_dist_squared = dist_squared;
                            }
                        }
                    }
                }
                output[index] = sqrt(min_dist_squared); // Calculate square root only once
            }
        }
    }
}




typedef struct {
    int * occGrid;
    int sizeGridrow;
    int sizeGridcolumn;
    double * metricMap;
    double pixelSize;
    double * topLeftCorner;
} my_grid;


my_grid OccuGrid(const myLocalMap localMap, const double pixelSize){
    double minXY[2] = {localMap.x[0], localMap.y[0]};
    double maxXY[2] = {localMap.x[0], localMap.y[0]};

    for(size_t a = 0; a < localMap.size; a++){
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

    size_t Sgrid[2];

    for (int a = 0; a < 2; a++) {
        minXY[a] -= (3 * pixelSize);
        maxXY[a] += (3 * pixelSize);
        Sgrid[a] = (int)round((maxXY[a] - minXY[a]) / pixelSize) + 1;
    }

    size_t gridSize = (int)Sgrid[0] * (int)Sgrid[1];

    int temp_grid[gridSize];
    memset(temp_grid, 0, sizeof(int) * gridSize);

    double hits[2];
    int idx[localMap.size];
    int *grid = (int *)malloc(sizeof(int) * gridSize);
    //int grid[gridSize];

    for (int a = 0; a < localMap.size; a++){
        hits[0] = round((localMap.x[a] - minXY[0]) / pixelSize) + 1;
        hits[1] = round((localMap.y[a] - minXY[1]) / pixelSize) + 1;
        //printf("hits = %d %d\n", (int)hits[0], (int)hits[1]);

        idx[a] = (int)( ((hits[0] - 1) * (double)Sgrid[1]) + hits[1] ) - 1;
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

    //double metric_map[gridSize];
    double *metric_map = (double *)malloc(sizeof(double) * gridSize);
    //double metric_map[gridSize];
    euclidean_distance_transform(grid, metric_map, (int) Sgrid[0], (int) Sgrid[1]);

    my_grid gridmap;

    gridmap.occGrid = (int *)grid;
    gridmap.sizeGridrow = (int) Sgrid[1];
    gridmap.sizeGridcolumn = (int) Sgrid[0];
    gridmap.metricMap = (double *)metric_map;
    gridmap.pixelSize = pixelSize;
    gridmap.topLeftCorner = minXY;
    return gridmap;
}

double angdiff(const double alpha, const double beta) {
    double delta = fmod((beta - alpha + M_PI), 2 * M_PI) - M_PI;
    return (double)delta;
}

//void DiffPose(const double pose1[3], const double pose2[3], double dp[3]){
//    for (int i = 0; i < 3; i++){
//        dp[i] = pose2[i] - pose1[i];
//    }
//    dp[2] = angdiff(pose1[2], pose2[2]);
//}

/***********Main Code************/
int main() {
    ScanData scan;
    my_map map;
    my_grid gridmap1;
    my_grid gridmap2;

    int miniUpdated = 0;

    // Map parameters
    double borderSize = 1;
    double pixelSize = 0.2f;

    // Initialize map values
    map.points = NULL; // Initialize to NULL for safety
    //map.keyscans = malloc(sizeof(my_keyscans)); // Allocate memory for a single my_keyscans element

    // Call the function to set Lidar parameters
    LidarParameters lidar = SetLidarParameters();

    // save LIDAR dataset into test_input_memory[row][column]
    readDatasetFromFile("C:/Lynn_SSD/NUS_Tingzz/EE4002D/Research/code_dump/my_CSM_C_test/A.csv");

    /*******Software Calculation Process*****/
    /*******Timing Starts here***************/

    // Initialise scan
    double pose[3] = {0.0,0.0,0.0};
    scan = readAScan(lidar, test_input_memory, 0, 24);     // read a clean scan range

    map = Initialise(pose, scan);

//    for (int p = 0; p < map.points_size; p++){
//        printf("%f %f\n", map.points[p][0], map.points[p][1]);
//    }

//    printf("%d \n", map.points_size);
//    printf("pose = %f %f %f\n", map.keyscans[0].pose[0], map.keyscans[0].pose[1], map.keyscans[0].pose[2]);
//    printf("iBegin = %d\n", map.keyscans[0].iBegin);
//    printf("loopClosed = %d\n", map.keyscans[0].loopClosed);
//    printf("loopTried = %d\n", map.keyscans[0].loopTried);

    miniUpdated = 1;

    // initiate first scan
    for (int i = 1; i < row; i++) {
        scan = readAScan(lidar, test_input_memory, i, 24);     // read a clean scan range
        //printf("%d\n", i);

        /*==================================Matching current scan to local map =======================================*/
        if (miniUpdated == 1 && i == 1){
            //ExtractLocalMap(my_map map, double pose[3], ScanData scan, double borderSize, double localMap[][2])
            myLocalMap localMap = ExtractLocalMap(map.points, map.points_size, pose, scan, borderSize);
//                printf("size of localMap = %d\n", localMap.size);
//                printf("pose = %f %f %f\n", pose[0], pose[1], pose[2]);   // pose
//
//                for (int a = 0; a < localMap.size; a++) {
//                    printf("%f %f\n", localMap.x[a], localMap.y[a]);    // localMap values
//                }

            gridmap1 = OccuGrid(localMap,pixelSize/2);
            //gridmap2 = OccuGrid(localMap,pixelSize/2);

            free(localMap.x);
            free(localMap.y);

//                //printf("gridmap1 pixelSize = %f\n", gridmap1.pixelSize);

//                printf("gridmap2 pixelSize = %f\n", gridmap2.pixelSize);

                for (int a = 0; a < gridmap1.sizeGridrow; a++){
                    for (int b = 0; b < gridmap1.sizeGridcolumn; b++){
                        if ((int)gridmap1.metricMap[a * gridmap1.sizeGridcolumn + b] == 0){
                            printf("grid[%d][%d]\n", a, b);
                            //printf("%d\n", a);
                            //printf("%d\n", b);
                        }
                    }
                }

//
//
//            printf("\n");
//
//                for (int a = 0; a < gridmap2.sizeGridrow; a++){
//                    for (int b = 0; b < gridmap2.sizeGridcolumn; b++){
//                        if ((int)gridmap2.metricMap[a * gridmap2.sizeGridcolumn + b] == 0){
//                            printf("grid[%d][%d]\n", a, b);
//                            //printf("%d\n", a);
//                            //printf("%d\n", b);
//                        }
//                    }
//                }

        }
        free(gridmap1.metricMap);
        free(gridmap1.occGrid);
//        free(gridmap2.metricMap);
//        free(gridmap2.occGrid);
    }

    free(map.points);
    /*******Timing Ends here*****************/
    return 0;
}
