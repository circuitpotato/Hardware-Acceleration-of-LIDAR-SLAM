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

//    float ct;
//    float st;


//    float theta_temp;
//    float tx_temp;
//    float ty_temp;

    float score;
    float theta[3] = {POSE[2] - r, POSE[2], POSE[2] + r};
    float tx[3] = {POSE[0] - t, POSE[0], POSE[0] + t};
    float ty[3] = {POSE[1] - t, POSE[1], POSE[1] + t};

    float ct[3] = {cosf(theta[0]), cosf(theta[1]), cosf(theta[2])};
    float st[3] = {sinf(theta[0]), sinf(theta[1]),sinf(theta[2])};

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
//            if (theta_index == 0){
//                theta_temp = POSE[2] - r;
//            }
//            else if (theta_index == 1){
//                theta_temp = POSE[2];
//            }
//            else {
//                theta_temp = POSE[2] + r;
//            }
//            printf("theta[%d] = %f\n", theta_index, theta[theta_index]);
//            ct = cosf(theta[theta_index]);
//            st = sinf(theta[theta_index]);

            for (int q = 0; q < scan.size; q++){
//                S_x[q] = (pixelScan_x[q]*ct) + (pixelScan_y[q]*(-st));
//                S_y[q] = (pixelScan_x[q]*(st)) + (pixelScan_y[q]*ct);
                S_x[q] = (pixelScan_x[q]*ct[theta_index]) + (pixelScan_y[q]*(st[theta_index]));
                S_y[q] = -(pixelScan_x[q]*(st[theta_index])) + (pixelScan_y[q]*ct[theta_index]);
                //printf("S[%d] = %f %f\n", q, S_x[q], S_y[q]);
            }

            // Translation
            for (tx_index = 0; tx_index < 3; tx_index++){
//                tx[tx_index] = POSE[0] + (tx_index == 0 ? -t : (tx_index == 2 ? t : 0));
//                if (tx_index == 0){
//                    tx_temp = POSE[0] - t;
//                }
//                else if (tx_index == 1){
//                    tx_temp = POSE[0];
//                }
//                else {
//                    tx_temp = POSE[0] + t;
//                }
//                printf("tx[%d] = %f\n", tx_index, tx[tx_index]);

                // get Sx
                float Sx_temp = (tx[tx_index] - minX)*ipixel;
                for (int i = 0; i < scan.size; i++){
                    Sx[i] = (int)roundf(S_x[i] + Sx_temp) + 1;
                    //printf("Sx[%d] = %d\n", i, (int)Sx[i]);
                }

                for (ty_index = 0; ty_index < 3; ty_index++){
//                    ty[ty_index] = POSE[1] + (ty_index == 0 ? -t : (ty_index == 2 ? t : 0));
//                    if (ty_index == 0){
//                        ty_temp = POSE[1] - t;
//                    }
//                    else if (ty_index == 1){
//                        ty_temp = POSE[1];
//                    }
//                    else {
//                        ty_temp = POSE[1] + t;
//                    }
//                    printf("ty[%d] = %f\n", ty_index, ty[ty_index]);
                    // get Sy
                    float Sy_temp = (ty[ty_index] - minY)*ipixel;
                    for (int i2 = 0; i2 < scan.size; i2++){
                        Sy[i2] = (int)roundf(S_y[i2] + Sy_temp) + 1;
                        //printf("Sy[%d] = %d\n", i2, (int)Sy[i2]);
                    }

                    int ixy_index =0;
                    //printf("ixy %d\n", ixy_index);
                    score = 0;
                    for (int i3 = 0; i3 < scan.size; i3 ++){
                        // IsIn = 1
                        if ((Sx[i3] > 1.0) && (Sy[i3] > 1.0) && (Sx[i3] < nCols) && (Sy[i3] < nRows)) {
//                            ix[ixy_index] = Sx[i3];   // ix
//                            iy[ixy_index] = Sy[i3];   // iy
                            FastMatchParameters.bestHits[ixy_index] = occ_grid.metric_grid[Sy[i3] - 1][Sx[i3] - 1];
                            score = score + occ_grid.metric_grid[Sy[i3] - 1][Sx[i3] - 1];
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
//                    for (int i4 = 0; i4 < ixy_index; i4++){
//                        FastMatchParameters.bestHits[i4] = occ_grid.metric_grid[iy[i4] - 1][ix[i4] - 1];
//                        //printf("hits[%d] = %d\n", i4, (int)hits[i4]);
//                    }

//                    score = 0;
//                    for (int i5 = 0; i5 < ixy_index; i5++){
//                        score = score + FastMatchParameters.bestHits[i5];
//                    }
                    FastMatchParameters.bestHits_size = ixy_index;
//                    printf("score = %f\n", score);

                    // update
//                    printf("tx = %f\n", tx_temp);

                    if (score < bestScore){
//                        printf("tx = %f\n", tx_temp);
//                        printf("tx[%d] = %f\n", tx_index, tx[tx_index]);
//                        printf("score = %f\n", score);
                        noChange = 0;
                        bestPose[0] = tx[tx_index];
                        bestPose[1] = ty[ty_index];
                        bestPose[2] = theta[theta_index];

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
        if (noChange){
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

        iter++;
    }
    FastMatchParameters.pose[0] = bestPose[0];
    FastMatchParameters.pose[1] = bestPose[1];
    FastMatchParameters.pose[2] = bestPose[2];

}
