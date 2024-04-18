void euclidean_distance_transform(int input_map[400][400], float output_distance_map[400][400], int width, int height) {
    // Compute the Euclidean distance transform using a Voronoi algorithm

    // Initialize distance grid
    float distance[width][height];
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            distance[i][j] = INFINITY;
        }
    }

    // Compute Voronoi diagram and update distance grid
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            if (input_map[x][y] == 1) { // Object cell
                for (int i = 0; i < width; i++) {
                    for (int j = 0; j < height; j++) {
                        double dist = sqrt((x - i) * (x - i) + (y - j) * (y - j));
                        if (dist < distance[i][j]) {
                            distance[i][j] = (float)dist;
                        }
                    }
                }
            }
        }
    }

    // Copy the distance values to the output map
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            output_distance_map[i][j] = distance[i][j];
        }
    }
}
