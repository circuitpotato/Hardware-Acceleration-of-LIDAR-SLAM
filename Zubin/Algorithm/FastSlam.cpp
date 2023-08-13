#include<stdio.h>
#include<iostream>
using namespace std;
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

// Forward declaration of Particle class


class ParticleFilter {
private:
    int numParticles;
    std::vector<Particle> particles;
    int step;
    double prevMatchedReading;
    double prevRawReading;
    std::vector<std::vector<double>> particlesTrajectory;

public:
    ParticleFilter(int numParticles, const std::vector<double>& ogParameters, const std::vector<double>& smParameters) :
        numParticles(numParticles), step(0), prevMatchedReading(0.0), prevRawReading(0.0) {
        initParticles(ogParameters, smParameters);
    }

    void initParticles(const std::vector<double>& ogParameters, const std::vector<double>& smParameters) {
        for (int i = 0; i < numParticles; ++i) {
            Particle p(ogParameters, smParameters);
            particles.push_back(p);
        }
    }

    void updateParticles(double reading, int count) {
        for (int i = 0; i < numParticles; ++i) {
            particles[i].update(reading, count);
        }
    }

    bool weightUnbalanced() {
        normalizeWeights();
        double variance = 0;
        for (int i = 0; i < numParticles; ++i) {
            variance += std::pow(particles[i].weight - 1.0 / numParticles, 2);
        }
        std::cout << variance << std::endl;
        if (variance > std::pow((numParticles - 1) / static_cast<double>(numParticles), 2) + (numParticles - 1.000000000000001) * std::pow(1 / static_cast<double>(numParticles), 2)) {
            return true;
        }
        else {
            return false;
        }
    }

    void normalizeWeights() {
        double weightSum = 0;
        for (int i = 0; i < numParticles; ++i) {
            weightSum += particles[i].weight;
        }
        for (int i = 0; i < numParticles; ++i) {
            particles[i].weight = particles[i].weight / weightSum;
        }
    }

    void resample() {
        std::vector<double> weights(numParticles, 0.0);
        std::vector<Particle> tempParticles;
        for (int i = 0; i < numParticles; ++i) {
            weights[i] = particles[i].weight;
            tempParticles.push_back(particles[i]);
        }
        std::discrete_distribution<int> distribution(weights.begin(), weights.end());
        for (int i = 0; i < numParticles; ++i) {
            int resampledIdx = distribution(generator);
            particles[i] = tempParticles[resampledIdx];
            particles[i].weight = 1.0 / numParticles;
        }
    }

    // You may need to implement a random number generator (generator) in your ParticleFilter class.
    std::default_random_engine generator;
};