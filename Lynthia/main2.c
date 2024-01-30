#include <string.h>
#include <stdio.h>
#include <stdlib.h> // For exit()
#include <math.h>
#include <time.h>
#define row 3408
#define column 1079

// reads raw data
float test_input_memory[column];
int miniUpdated = 0;
int scan_iter;

// check if file can be open
void openFileValidity(FILE* fp){
    if (fp == NULL) {
        printf("Failed to open the file.\n");
        return;
    }
}

void readDatasetLineByLine(FILE *filename){
    // Read float values from file
    float value;

    if (fscanf(filename, "%f,", &value) != 1) {
        printf("Failed to read float value %d from the file.\n", 1);
        fclose(filename);
        exit(1);
    }
    test_input_memory[0] = value;
    for (int k = 1; k < column; k++){
        if (fscanf(filename, "%f,", &value) != 1) {
            printf("Failed to read float value %d from the file.\n", k + 1);
            fclose(filename);
            exit(1);
        }
        test_input_memory[k] = value;
    }
}

typedef struct {
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    float angles[column]; // Array of angles
} LidarParameters;

LidarParameters lidar;

void SetLidarParameters() {

    lidar.angle_min = (float)-2.351831;
    lidar.angle_max = (float)2.351831;
    lidar.angle_increment = (float)0.004363;
    lidar.range_min = (float)0.023;
    lidar.range_max = 60;

    float angle = lidar.angle_min;
    for (int i = 0; i < column; i++) {
        lidar.angles[i] = angle;
        angle += lidar.angle_increment;
    }
}

typedef struct {
    float x[row];
    float y[row];
    float tx[row];
    float ty[row];
    int size;
} ScanData;


ScanData scan;

void readAScan(const int usableRange){
    float maxRange = (lidar.range_max < (float)usableRange) ? lidar.range_max : (float)usableRange;
    int valid_points = 0;

    for (int i = 0; i < column; i++){
        if ((test_input_memory[i] < lidar.range_min) | (test_input_memory[i] > maxRange)){
            continue;   // skip if range is bad
        }
        else{
            float cartesian_x, cartesian_y;
            cartesian_x = test_input_memory[i] * cosf(lidar.angles[i]);
            cartesian_y = test_input_memory[i] * sinf(lidar.angles[i]);
            scan.x[valid_points] = cartesian_x;
            scan.y[valid_points] = cartesian_y;
            valid_points++;
        }
    }
    scan.size = valid_points;
}


void Transform(const float POSE[3]){
    float tx = POSE[0];
    float ty = POSE[1];
    float theta = POSE[2];
    float ct = cosf(theta);
    float st = sinf(theta);
    float R[2][2] = {{ct, -st}, {st,ct}};

    for (int i = 0; i < scan.size; i++){
        // multiply scan(x,y) by transformed R
        // scan is (N,2) matrix and R is (2,2) matrix
        float transformed_x = (float)(R[0][0] * scan.x[i] + R[1][0] * scan.y[i]);
        float transformed_y = (float)(R[0][1] * scan.x[i] + R[1][1] * scan.y[i]);

        // Translate to points on world frame
        scan.tx[i] = transformed_x + tx;
        scan.ty[i] = transformed_y + ty;
    }
}



// updated points in world frame
typedef struct {
    float x[30000];
    float y[30000];
    int size;

    float newPoints_x[row*4];
    float newPoints_y[row*4];
    int newPointsSize;
    float pose[3];
} MapPoints;

MapPoints map;

void Initialise(const float POSE[3]){
    for (int i = 0; i < scan.size; i++){
        map.x[i] = scan.tx[i];
        map.y[i] = scan.ty[i];
    }
    map.size = scan.size;
    map.pose[0] = POSE[0];
    map.pose[1] = POSE[1];
    map.pose[2] = POSE[2];
}

typedef struct{
    float x[25000];
    float y[25000];
    int size;
} myLocalMap;

myLocalMap local_map;

void ExtractLocalMap(const float BORDERSIZE){
    float minX = scan.tx[0];
    float minY = scan.ty[0];
    float maxX = scan.tx[0];
    float maxY = scan.ty[0];

    for (int i = 1; i < scan.size; i++) {
        if (scan.tx[i] < minX) {
            minX = scan.tx[i];
        }
        if (scan.tx[i] > maxX) {
            maxX = scan.tx[i];
        }
        if (scan.ty[i] < minY) {
            minY = scan.ty[i];
        }
        if (scan.ty[i] > maxY) {
            maxY = scan.ty[i];
        }
    }

    // Set top-left & bottom-right corner
    minX = minX - BORDERSIZE;
    minY = minY - BORDERSIZE;
    maxX = maxX + BORDERSIZE;
    maxY = maxY + BORDERSIZE;

    // Extract x and y-axis of localMap
    int valid_points = 0;
    for (int i = 0; i < map.size; i++) {
        if ((map.x[i] > minX) && (map.x[i] < maxX) && (map.y[i] > minY) &&
            (map.y[i] < maxY)) {

            local_map.x[valid_points] = map.x[i];
            local_map.y[valid_points] = map.y[i];

            valid_points++;
        }
    }
    local_map.size = valid_points;}

typedef struct {
    int grid[200][200];
    int grid_size[2]; // grid_size[0] is row, grid_size[1] is column
    float metric_grid[200][200];
    float pixel_size;
    float top_left_corner[2];

    int grid2[400][400];
    int grid_size2[2];
    float metric_grid2[400][400];
    float pixel_size2;
    float top_left_corner2[2];
} MyGrid;
MyGrid occ_grid;

// Helper function to calculate the Euclidean distance
float euclidean_distance(int x1, int y1, int x2, int y2) {
    return (float)sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Function to compute Euclidean distance transform
void euclidean_distance_transform(const int width, const int height) {
    float MAX_DIST = 10;
    for (int y = 0; y < height; ++y) {
        int y_square = y*y;
        for (int x = 0; x < width; ++x) {
            int x_square = x*x;
            if (occ_grid.grid[y][x] != 0) {
                occ_grid.metric_grid[y][x] = 0;
            }
            else {
                float min_dist = MAX_DIST;
                for (int j = 0; j < height; ++j) {
                    int j_square = j*j;
                    int two_jy = j*y << 1;
                    for (int i = 0; i < width; ++i) {
                        if (occ_grid.grid[j][i] != 0) {
                            int dist_square = x_square - (i*x << 1) + (i*i) + y_square - two_jy + j_square;
                            if ((float)dist_square < min_dist*min_dist) {
                                min_dist = sqrtf((float)dist_square);
                            }
                        }
                    }
                }
                occ_grid.metric_grid[y][x] = min_dist;
            }
        }
    }
}

// Function to compute Euclidean distance transform
void euclidean_distance_transform2(const int width, const int height) {
    float MAX_DIST = 10;
    for (int y = 0; y < height; ++y) {
        int y_square = y*y;
        int two_y = y << 1;
        for (int x = 0; x < width; ++x) {
            int x_square = x*x;
            int two_x = x << 1;
            if (occ_grid.grid2[y][x] != 0) {
                occ_grid.metric_grid2[y][x] = 0;
            }
            else {
                float min_dist = MAX_DIST;
                for (int j = 0; j < height; ++j) {
                    int j_square = j*j;
                    int two_jy = j*two_y;
                    for (int i = 0; i < width; ++i) {
                        if (occ_grid.grid2[j][i] != 0) {
                            int dist_square = x_square - (i*two_x) + (i*i) + y_square - two_jy + j_square;
                            if ((float)dist_square < min_dist*min_dist) {
                                min_dist = sqrtf((float)dist_square);
                            }
                        }
                    }
                }
                occ_grid.metric_grid2[y][x] = min_dist;
            }
        }
    }
}
void OccupationalGrid(const float PIXELSIZE, const float PIXELSIZE2){
    float minXY[2] = {local_map.x[0], local_map.y[0]};
    float maxXY[2] = {local_map.x[0], local_map.y[0]};

    for(int a = 0; a < local_map.size; a++){
        if (local_map.x[a] < minXY[0]){
            minXY[0] = local_map.x[a];
        }
        if (local_map.x[a] > maxXY[0]){
            maxXY[0] = local_map.x[a];
        }
        if (local_map.y[a] < minXY[1]){
            minXY[1] = local_map.y[a];
        }
        if (local_map.y[a] > maxXY[1]){
            maxXY[1] = local_map.y[a];
        }
    }

    float minXY2[2] = {minXY[0], minXY[1]};
    float maxXY2[2] = {maxXY[0], maxXY[1]};
    int Sgrid[2];
    int Sgrid2[2];

    for (int a = 0; a < 2; a++) {
        minXY[a] -= (3 * PIXELSIZE);
        maxXY[a] += (3 * PIXELSIZE);

        minXY2[a] -= (3 * PIXELSIZE2);
        maxXY2[a] += (3 * PIXELSIZE2);

        Sgrid[a] = (int)roundf((maxXY[a] - minXY[a]) / PIXELSIZE) + 1;
        Sgrid2[a] = (int)roundf((maxXY2[a] - minXY2[a]) / PIXELSIZE2) + 1;
    }

    int hits[2];
    int hits2[2];
    int idx_occGrid[local_map.size];
    int idx_occGrid2[local_map.size];

    occ_grid.grid_size[0] = Sgrid[1];
    occ_grid.grid_size[1] = Sgrid[0];

    occ_grid.grid_size2[0] = Sgrid2[1];
    occ_grid.grid_size2[1] = Sgrid2[0];

    memset(occ_grid.grid, 0, sizeof occ_grid.grid);
    memset(occ_grid.grid2, 0, sizeof occ_grid.grid2);

    float x_minus_minX;
    float y_minus_minY;
    float x_minus_minX2;
    float y_minus_minY2;
    int idx_row;
    int idx_col;
    int idx_row2;
    int idx_col2;

    for (int a = 0; a < local_map.size; a++){
        x_minus_minX = local_map.x[a] - minXY[0];
        y_minus_minY = local_map.y[a] - minXY[1];

        x_minus_minX2 = local_map.x[a] - minXY2[0];
        y_minus_minY2 = local_map.y[a] - minXY2[1];

        hits[0] = (int)roundf(x_minus_minX / PIXELSIZE) + 1;
        hits[1] = (int)roundf(y_minus_minY / PIXELSIZE) + 1;

        hits2[0] = (int)roundf(x_minus_minX2 / PIXELSIZE2) + 1;
        hits2[1] = (int)roundf(y_minus_minY2 / PIXELSIZE2) + 1;

        idx_occGrid[a] = (int)( ((float)(hits[1] - 1) * (float)Sgrid[0]) + (float)hits[0] ) - 1;
        idx_occGrid2[a] = (int)( ((float)(hits2[1] - 1) * (float)Sgrid2[0]) + (float)hits2[0] ) - 1;

        idx_row = idx_occGrid[a] / Sgrid[0];
        idx_col = idx_occGrid[a] % Sgrid[0];

        idx_row2 = idx_occGrid2[a] / Sgrid2[0];
        idx_col2 = idx_occGrid2[a] % Sgrid2[0];

        occ_grid.grid[idx_row][idx_col] = 1;
        occ_grid.grid2[idx_row2][idx_col2] = 1;
    }
    euclidean_distance_transform(occ_grid.grid_size[1], occ_grid.grid_size[0]);
    euclidean_distance_transform2(occ_grid.grid_size2[1], occ_grid.grid_size2[0]);
    occ_grid.pixel_size = PIXELSIZE;
    occ_grid.pixel_size2 = PIXELSIZE2;
    occ_grid.top_left_corner[0] = minXY[0];
    occ_grid.top_left_corner[1] = minXY[1];
    occ_grid.top_left_corner2[0] = minXY2[0];
    occ_grid.top_left_corner2[1] = minXY2[1];
}

// more Scan Matching stuff
float pixelScan_x[row];
float pixelScan_y[row];
float S_x[row];
float S_y[row];
float Sx[row];
float Sy[row];
float ix[row];
float iy[row];


typedef struct {
    float pose[3];
    float bestHits[2500];
    int bestHits_size;
} MyFastMatchParameters;
MyFastMatchParameters FastMatchParameters;

void FastMatch(const float POSE[3], const float searchResolution[3]){
    // Grid Map Information
    float ipixel = 1/occ_grid.pixel_size;
    float minX = occ_grid.top_left_corner[0];
    float minY = occ_grid.top_left_corner[1];
    float t = searchResolution[0];
    float r = searchResolution[2];
    int nRows = occ_grid.grid_size[0];
    int nCols = occ_grid.grid_size[1];

    // Go down the hill
    int maxIter = 50;
    int maxDepth = 3;
    int iter = 0;
    int depth = 0;

    int theta_index;
    int tx_index;
    int ty_index;
//    float theta[3];
//    float tx[3];
//    float ty[3];
    float bestScore = INFINITY;
    int noChange;

    float ct;
    float st;


    float theta_temp;
    float tx_temp;
    float ty_temp;

    float score;

    // get pixelscan
    for (int a = 0; a < scan.size; a++){
        pixelScan_x[a] = scan.x[a] * ipixel;
        pixelScan_y[a] = scan.y[a] * ipixel;
        //printf("scan[%d] %f %f\n", a, pixelScan_x[a], pixelScan_y[a]);
    }
    float bestPose[3] = {POSE[0], POSE[1], POSE[2]};


    while (iter < maxIter){
        noChange = 1;
        // Rotation
        for (theta_index = 0; theta_index < 3; theta_index++){
//            printf("FM_pose[2] = %f\n", FM_pose[2]);
//            theta[theta_index] = POSE[2] + (theta_index == 0 ? -r : (theta_index == 2 ? r : 0));
            if (theta_index == 0){
                theta_temp = POSE[2] - r;
            }
            else if (theta_index == 1){
                theta_temp = POSE[2];
            }
            else {
                theta_temp = POSE[2] + r;
            }
//            printf("theta[%d] = %f\n", theta_index, theta[theta_index]);
            ct = cosf(theta_temp);
            st = sinf(theta_temp);

            for (int q = 0; q < scan.size; q++){
//                S_x[q] = (pixelScan_x[q]*ct) + (pixelScan_y[q]*(-st));
//                S_y[q] = (pixelScan_x[q]*(st)) + (pixelScan_y[q]*ct);
                S_x[q] = (pixelScan_x[q]*ct) + (pixelScan_y[q]*(st));
                S_y[q] = (pixelScan_x[q]*(-st)) + (pixelScan_y[q]*ct);
                //printf("S[%d] = %f %f\n", q, S_x[q], S_y[q]);
            }

            // Translation
            for (tx_index = 0; tx_index < 3; tx_index++){
//                tx[tx_index] = POSE[0] + (tx_index == 0 ? -t : (tx_index == 2 ? t : 0));
                if (tx_index == 0){
                    tx_temp = POSE[0] - t;
                }
                else if (tx_index == 1){
                    tx_temp = POSE[0];
                }
                else {
                    tx_temp = POSE[0] + t;
                }
//                printf("tx[%d] = %f\n", tx_index, tx[tx_index]);

                // get Sx
                float Sx_temp = (tx_temp - minX)*ipixel;
                for (int i = 0; i < scan.size; i++){
                    Sx[i] = roundf(S_x[i] + Sx_temp) + 1;
                    //printf("Sx[%d] = %d\n", i, (int)Sx[i]);
                }

                for (ty_index = 0; ty_index < 3; ty_index++){
//                    ty[ty_index] = POSE[1] + (ty_index == 0 ? -t : (ty_index == 2 ? t : 0));
                    if (ty_index == 0){
                        ty_temp = POSE[1] - t;
                    }
                    else if (ty_index == 1){
                        ty_temp = POSE[1];
                    }
                    else {
                        ty_temp = POSE[1] + t;
                    }
//                    printf("ty[%d] = %f\n", ty_index, ty[ty_index]);
                    // get Sy
                    float Sy_temp = (ty_temp - minY)*ipixel;
                    for (int i2 = 0; i2 < scan.size; i2++){
                        Sy[i2] = roundf(S_y[i2] + Sy_temp) + 1;
                        //printf("Sy[%d] = %d\n", i2, (int)Sy[i2]);
                    }

                    int ixy_index =0;
                    //printf("ixy %d\n", ixy_index);
                    for (int i3 = 0; i3 < scan.size; i3 ++){
                        // IsIn = 1
                        if ((Sx[i3] > 1.0) && (Sy[i3] > 1.0) && (Sx[i3] < (float)nCols) && (Sy[i3] < (float)nRows)) {
                            ix[ixy_index] = Sx[i3];   // ix
                            iy[ixy_index] = Sy[i3];   // iy

                            //printf("iy[%d] = %f\n", ixy_index, S_y[ixy_index]);
                            ixy_index++;
                        }
                        //printf("ix[%d] = %f\n", i3, S_x[i3]);
                    }

//                    for(int w = 0; w < ixy_index; w++){
//                        printf("ix[%d] = %d\n", w, (int)ix[w]);
//                    }

//                    for(int w = 0; w < ixy_index; w++){
//                        printf("iy[%d] = %d\n", w, (int)iy[w]);
//                    }


                    //printf("ixy %d\n", ixy_index);
                    //float sum = 0;
                    for (int i4 = 0; i4 < ixy_index; i4++){
                        FastMatchParameters.bestHits[i4] = occ_grid.metric_grid[(int)iy[i4] - 1][(int)ix[i4] - 1];
                        //printf("hits[%d] = %d\n", i4, (int)hits[i4]);
                    }

                    score = 0;
                    for (int i5 = 0; i5 < ixy_index; i5++){
                        score = score + FastMatchParameters.bestHits[i5];
                    }
                    FastMatchParameters.bestHits_size = ixy_index;
//                    printf("score = %f\n", score);

                    // update
//                    printf("tx = %f\n", tx_temp);

                    if (score < bestScore){
//                        printf("tx = %f\n", tx_temp);
//                        printf("tx[%d] = %f\n", tx_index, tx[tx_index]);
//                        printf("score = %f\n", score);
                        noChange = 0;
                        bestPose[0] = tx_temp;
                        bestPose[1] = ty_temp;
                        bestPose[2] = theta_temp;

//                        if (scan_iter + 1 == 2)
//                            printf("bestPose FastMatch = %f  %f  %f\n", tx_temp, ty_temp, theta_temp);



                        bestScore = score;
//                        for (int q = 0; q < ixy_index; q++){
//                            FastMatchParameters.bestHits[q] = hits_fastmatch[q];
//                        }
                        FastMatchParameters.bestHits_size = ixy_index;
//                        printf("ixy index = %d\n", ixy_index);
                    }

                }
            }
        }

        // No better match was found, increase resolution
        if (noChange == 1){
//            r/=2;
//            printf("r = %f\n", r);
//            t/=2;
//            printf("t = %f\n", t);
            depth = depth + 1;
//            printf("depth = %d\n", depth);
            if (depth > maxDepth){
                break;
            }

        }
//        printf("bestPose = %f  %f  %f\n", bestPose[0], bestPose[1], bestPose[2]);

        iter = iter + 1;
    }
    FastMatchParameters.pose[0] = bestPose[0];
    FastMatchParameters.pose[1] = bestPose[1];
    FastMatchParameters.pose[2] = bestPose[2];

}

void FastMatch2(const float POSE[3], const float searchResolution[3]){
    // Grid Map Information
    float ipixel = 1/occ_grid.pixel_size2;
    float minX = occ_grid.top_left_corner2[0];
    float minY = occ_grid.top_left_corner2[1];
    float t = searchResolution[0];
    float r = searchResolution[2];
    int nRows = occ_grid.grid_size2[0];
    int nCols = occ_grid.grid_size2[1];

    // Go down the hill
    int maxIter = 50;
    int maxDepth = 3;
    int iter = 0;
    int depth = 0;

    int theta_index;
    int tx_index;
    int ty_index;
//    float theta[3];
//    float tx[3];
//    float ty[3];
    float bestScore = INFINITY;
    int noChange;

    float ct;
    float st;

    float theta_temp;
    float tx_temp;
    float ty_temp;
    float score;

    // get pixelscan
    for (int a = 0; a < scan.size; a++){
        pixelScan_x[a] = scan.x[a] * ipixel;
        pixelScan_y[a] = scan.y[a] * ipixel;
        //printf("scan[%d] %f %f\n", a, pixelScan_x[a], pixelScan_y[a]);
    }
    float bestPose[3] = {POSE[0], POSE[1], POSE[2]};


    while (iter < maxIter){
        noChange = 1;
        // Rotation
        for (theta_index = 0; theta_index < 3; theta_index++){
//            printf("FM_pose[2] = %f\n", FM_pose[2]);
//            theta[theta_index] = POSE[2] + (theta_index == 0 ? -r : (theta_index == 2 ? r : 0));
            if (theta_index == 0){
                theta_temp = POSE[2] - r;
            }
            else if (theta_index == 1){
                theta_temp = POSE[2];
            }
            else {
                theta_temp = POSE[2] + r;
            }
//            printf("theta[%d] = %f\n", theta_index, theta[theta_index]);
            ct = cosf(theta_temp);
            st = sinf(theta_temp);

            for (int q = 0; q < scan.size; q++){
                S_x[q] = (pixelScan_x[q]*ct) + (pixelScan_y[q]*(st));
                S_y[q] = (pixelScan_x[q]*(-st)) + (pixelScan_y[q]*ct);
                //printf("S[%d] = %f %f\n", q, S_x[q], S_y[q]);
            }

            // Translation
            for (tx_index = 0; tx_index < 3; tx_index++){
//                tx[tx_index] = POSE[0] + (tx_index == 0 ? -t : (tx_index == 2 ? t : 0));
                if (tx_index == 0){
                    tx_temp = POSE[0] - t;
                }
                else if (tx_index == 1){
                    tx_temp = POSE[0];
                }
                else {
                    tx_temp = POSE[0] + t;
                }
//                printf("tx[%d] = %f\n", tx_index, tx[tx_index]);

                // get Sx
                float Sx_temp = (tx_temp - minX)*ipixel;
                for (int i = 0; i < scan.size; i++){
                    Sx[i] = roundf(S_x[i] + Sx_temp) + 1;
                    //printf("Sx[%d] = %d\n", i, (int)Sx[i]);
                }

                for (ty_index = 0; ty_index < 3; ty_index++){
//                    ty[ty_index] = POSE[1] + (ty_index == 0 ? -t : (ty_index == 2 ? t : 0));
                    if (ty_index == 0){
                        ty_temp = POSE[1] - t;
                    }
                    else if (ty_index == 1){
                        ty_temp = POSE[1];
                    }
                    else {
                        ty_temp = POSE[1] + t;
                    }
//                    printf("ty[%d] = %f\n", ty_index, ty[ty_index]);
                    // get Sy
                    float Sy_temp = (ty_temp - minY)*ipixel;
                    for (int i2 = 0; i2 < scan.size; i2++){
                        Sy[i2] = roundf(S_y[i2] + Sy_temp) + 1;
                        //printf("Sy[%d] = %d\n", i2, (int)Sy[i2]);
                    }

                    int ixy_index =0;
                    //printf("ixy %d\n", ixy_index);
                    for (int i3 = 0; i3 < scan.size; i3 ++){
                        // IsIn = 1
                        if ((Sx[i3] > 1) && (Sy[i3] > 1) && (Sx[i3] < (float)nCols) && (Sy[i3] < (float)nRows)) {
                            ix[ixy_index] = Sx[i3];   // ix
                            iy[ixy_index] = Sy[i3];   // iy

                            //printf("iy[%d] = %f\n", ixy_index, S_y[ixy_index]);
                            ixy_index++;
                        }
                        //printf("ix[%d] = %f\n", i3, S_x[i3]);
                    }

//                    for(int w = 0; w < ixy_index; w++){
//                        printf("ix[%d] = %d\n", w, (int)ix[w]);
//                    }

//                    for(int w = 0; w < ixy_index; w++){
//                        printf("iy[%d] = %d\n", w, (int)iy[w]);
//                    }


                    //printf("ixy %d\n", ixy_index);
                    for (int i4 = 0; i4 < ixy_index; i4++){
                        FastMatchParameters.bestHits[i4] = occ_grid.metric_grid2[(int)iy[i4] - 1][(int)ix[i4] - 1];
                        //printf("hits[%d] = %d\n", i4, (int)hits[i4]);
                    }

                    score = 0;
                    for (int i5 = 0; i5 < ixy_index; i5++){
                        score = score + FastMatchParameters.bestHits[i5];
                    }
                    FastMatchParameters.bestHits_size = ixy_index;
//                    printf("score = %f\n", score);

                    // update
//                    printf("tx = %f\n", tx_temp);

                    if (score < bestScore){
//                        printf("tx = %f\n", tx_temp);
//                        printf("tx[%d] = %f\n", tx_index, tx[tx_index]);
//                        printf("score = %f\n", score);
                        noChange = 0;
                        bestPose[0] = tx_temp;
                        bestPose[1] = ty_temp;
                        bestPose[2] = theta_temp;

//                        if (scan_iter + 1 == 21 && searchResolution[0] == 0.05f) {
////                            printf("bestPose FastMatch = %f  %f  %f\n", tx_temp, ty_temp, theta_temp);
//                            printf("score = %f\n", score);
//                        }

                        bestScore = score;
//                        for (int q = 0; q < ixy_index; q++){
//                            FastMatchParameters.bestHits[q] = hits_fastmatch[q];
//                        }
//                        FastMatchParameters.bestHits_size = ixy_index;
//                        printf("ixy index = %d\n", ixy_index);
                    }

                }
            }
        }

        // No better match was found, increase resolution
        if (noChange == 1){
//            r/=2;
//            printf("r = %f\n", r);
//            t/=2;
//            printf("t = %f\n", t);
            depth = depth + 1;
//            printf("depth = %d\n", depth);
            if (depth > maxDepth){
                break;
            }

        }
//        printf("bestPose = %f  %f  %f\n", bestPose[0], bestPose[1], bestPose[2]);

        iter = iter + 1;
    }
    FastMatchParameters.pose[0] = bestPose[0];
    FastMatchParameters.pose[1] = bestPose[1];
    FastMatchParameters.pose[2] = bestPose[2];

}


void DiffPose(const float pose1[3], const float pose2[3], float dp[3]){
    for (int i = 0; i < 3; i++){
        dp[i] = pose2[i] - pose1[i];
    }
    dp[2] = pose2[2] - pose1[2];
}



float path[3][row];
FILE *fp;
FILE *fp1;

int main(){
    double start,end;
    start=clock();//predefined  function in c


    float pose[3] = {0,0,0};
    // Scan matching parameters
    float fastResolution[3] = {(float)0.05, (float)0.05, M_PI * 0.5 / 180}; // [m; m; rad]
    float fastResolution2[3] = {(float)0.05/2, (float)0.05/2, (float)(M_PI * 0.5 / 180)/2}; // [m; m; rad]
    float borderSize = 1;
    float pixelSize = 0.2f;
    float pixelSize2 = 0.1f;

    float miniUpdateDT = 0.3f;   // m
    float miniUpdateDR = 0.0872665f;    // deg2rad(5) in MATLAB
    int scan_transform_flag;

    fp = fopen("C:/Lynn_SSD/NUS_Tingzz/EE4002D/Research/code_dump/my_CSM_C_test/A.csv", "r");
    openFileValidity(fp);     // check if file can be open
    readDatasetLineByLine(fp);  // read 1st line of code (scan 0)
    SetLidarParameters();   // declare lidar parameters since there is no actual lidar
    readAScan(24);

    Transform(pose);
    Initialise(pose);  // initialise map
    path[0][0] = pose[0];
    path[1][0] = pose[1];
    path[2][0] = pose[2];

    float pose_guess[3];
    float dp[3];

    miniUpdated = 1;
    int path_iter = 1;
    for (scan_iter = 1; scan_iter < 2500; scan_iter++){
        printf("scan %d\n", scan_iter+1);

        readDatasetLineByLine(fp);  // read current line of code starting from scan 1
        readAScan(24);
        scan_transform_flag = 0;
        if (miniUpdated == 1) {
//            printf("update\n");
            Transform(pose);
//            scan_transform_flag = 1;
            ExtractLocalMap(borderSize);
            OccupationalGrid(pixelSize, pixelSize2);
        }

        // Predict current pose using constant velocity motion model
        if (scan_iter > 1) {
            float previous_pose[3];
//            printf("previous pose = %f  %f  %f\n", previous_pose[0], previous_pose[1], previous_pose[2]);
            previous_pose[0] = path[0][path_iter-2];
            previous_pose[1] = path[1][path_iter-2];
            previous_pose[2] = path[2][path_iter-2];
//            printf("previous pose = %f  %f  %f\n", previous_pose[0], previous_pose[1], previous_pose[2]);
//            printf("pose = %f  %f  %f\n", pose[0], pose[1], pose[2]);
//            float my_dp[3];
            DiffPose(previous_pose, pose, dp);

//            printf("dp = %f  %f  %f\n", my_dp[0], my_dp[1], my_dp[2]);
//            printf("pose = %f  %f  %f\n", pose[0], pose[1], pose[2]);

            for (int i = 0; i < 3; i++){
                pose_guess[i] = pose[i] + dp[i];
            }
//            printf("pose_guess = %f  %f  %f\n", pose_guess[0], pose_guess[1], pose_guess[2]);
        }
        else {
            pose_guess[0] = pose[0];
            pose_guess[1] = pose[1];
            pose_guess[2] = pose[2];
        }

        // Fast Matching
        if (miniUpdated) {
            FastMatch(pose_guess, fastResolution);
//            for (int i = 0; i < 3; i++){
//                pose[i] = FastMatchParameters.pose[i];
//            }
        }
        else {

            FastMatch2(pose_guess, fastResolution);

//            for (int i = 0; i < 3; i++){
//                pose[i] = FastMatchParameters.pose[i];
//            }

        }

        // Refine the pose using smaller pixels
        FastMatch2(FastMatchParameters.pose, fastResolution2);

        for (int i = 0; i < 3; i++){
            pose[i] = FastMatchParameters.pose[i];
        }

        // Execute a mini update, if robot has moved a certain distance
        float forced_pose[3] = {0,0,0};     // self insert 0,0,0 until scan 44 (edit later)
//        float dp[3];    // output pose of mini update

        DiffPose(map.pose, pose, dp);
        for (int i = 0; i < 3; i++){
            dp[i]=fabsf(dp[i]);
        }

        if (dp[0] > miniUpdateDT || dp[1] > miniUpdateDT || dp[2] > miniUpdateDR){
            miniUpdated = 1;
            Transform(pose);


            int newPointSize = 0;
            for (int j = 0; j < FastMatchParameters.bestHits_size; j++){
                if (FastMatchParameters.bestHits[j] > 1.5){
                    map.x[map.size + newPointSize] = scan.tx[j];
                    map.y[map.size + newPointSize] = scan.ty[j];
                    newPointSize++;
                }
            }

//            if (newPointSize == 0){
//                break;
//            }
            map.size = map.size + newPointSize;
            map.pose[0] = pose[0];
            map.pose[1] = pose[1];
            map.pose[2] = pose[2];
//            printf("%d\n", map.size);
        }
        else {
            miniUpdated = 0;
        }


//        if (scan_iter + 1 == 21)
        printf("pose = %f  %f  %f\n", pose[0], pose[1], pose[2]);
        path[0][path_iter] = pose[0];
        path[1][path_iter] = pose[1];
        path[2][path_iter] = pose[2];
        path_iter++;
    }
    end=clock();
    double t=(end-start)/CLOCKS_PER_SEC;
    printf("time taken = %f\n", t);

    fclose(fp);

//    printf("\n\n\n\n");
//    for (int i = 0; i < path_iter; i++){
//        printf("path[%d] = %f  %f  %f\n" ,i+1, path[0][i], path[1][i], path[2][i]);
//    }

    fp1 = fopen("C:/Users/lynth/Desktop/map_output.csv", "w");//create a file
    for (int j =0; j < map.size; j++){
        fprintf(fp1, "%f,%f\n", map.x[j], map.y[j]);
    }


    fclose(fp1);

}
