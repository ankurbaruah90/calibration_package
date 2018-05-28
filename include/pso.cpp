#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <gsl/gsl_rng.h>
#include <cstring>
#include <time.h>
#include <deque>

#define FINETUNE 0

using namespace std;

#define inertia 0.729                                                   // inertia parameter
#define c1 1.494                                                        // PSO parameter c1
#define c2 1.494                                                        // PSO parameter c2
#define SPAN 10                                                         // Span of the swarm
#define SWARM_SIZE 20                                                   // Size of the swarm
#define DIM 12                                                          // Dimension
#define LOG 0

/*---------------------------------*/
        /*---Add VNS---*/
/*---------------------------------*/

class PSO
{
public:
    struct tm tstruct;
    char log[100], termination[100];
    time_t now;
    const gsl_rng_type *T;
    gsl_rng * r;
    int iteration, optimalSolution, epoch, solutionCount;
    float penalty, globalBestFitnessValue;
    float random1, random2;
    deque <float> bestGlobalFitnessQueue;
    float terminationThreshold;
    float bestFitness;
    int terminationWindow;

    vector<float> getOptimal();
    vector <float> currentFitness, localBestFitness, optimalValues;
    vector < vector <float> > currentPosition, velocity, localBestPosition, globalBestPosition;

    void getFitness();
    void updateVelocityAndPosition();
    void initialize();
    bool computeTerminationCondition();
    PSO();
    ~PSO();
};

PSO::PSO()
{
    now = time(0);
    tstruct = *localtime(&now);
    strftime(log, sizeof(log), "/home/ankur/log-%Y-%m-%d-%H-%M-%S", &tstruct);
    strcat(log, ".csv");
    strftime(termination, sizeof(termination), "/home/ankur/term-%Y-%m-%d-%H-%M-%S", &tstruct);
    strcat(termination, ".csv");
    T = gsl_rng_default;
    r = gsl_rng_alloc (T);
    random1 = 1.0;
    random2 = 1.0;
    iteration = 10;                                                             // number of iterations
    epoch = 10;
    penalty = 1000.0;
    globalBestFitnessValue = 10000.0;

    //Termination related parameters
    terminationWindow = 3;
    terminationThreshold = 0.02;
    bestFitness = 0.05;
}

PSO::~PSO()
{

}

bool PSO::computeTerminationCondition()
{
    bestGlobalFitnessQueue.push_back(globalBestFitnessValue);
    if(bestGlobalFitnessQueue.size() > terminationWindow)
        bestGlobalFitnessQueue.pop_front();

    float stddev = 10000.0, devParticles = 1000.0;
    float mean = 0.0, meanParticles = 0.0;
    float sumofsquares = 0.0, sumSquareParticles;

    if(bestGlobalFitnessQueue.size() == terminationWindow)
    {
        for(int i = 0; i < bestGlobalFitnessQueue.size(); i++)
        {
            mean += bestGlobalFitnessQueue[i];
        }
        mean = mean/bestGlobalFitnessQueue.size();
        for(int i = 0; i < bestGlobalFitnessQueue.size(); i++)
            sumofsquares += pow(bestGlobalFitnessQueue[i] - mean, 2);
        stddev = sqrt(sumofsquares/bestGlobalFitnessQueue.size());
    }

    ///// Size of Particle Cloud /////

    for (int j = 0; j < currentFitness.size(); j++)
        meanParticles += currentFitness[j];
    meanParticles = meanParticles/currentFitness.size();

    for (int j = 0; j < currentFitness.size(); j++)
        sumSquareParticles += pow((currentFitness[j] - meanParticles), 2);

    devParticles = sqrt(sumSquareParticles/currentFitness.size());

#if LOG
    std::ofstream dataLog;
    dataLog.open(termination, std::ofstream::out | std::ofstream::app);
    for(int i = 0; i < bestGlobalFitnessQueue.size(); i++)
    {
        dataLog << bestGlobalFitnessQueue[i] << ",";
    }
    dataLog << stddev << "," << meanParticles << "," << devParticles;
    dataLog << "\n";
    dataLog.close();
#endif

    if((stddev <= terminationThreshold) && (mean <= bestFitness))
        return true;
    else
        return false;
}

void PSO::initialize()
{
    /* ------------ Initialize Swarm  -------------  */
    localBestFitness.resize(SWARM_SIZE);
    currentFitness.resize(SWARM_SIZE);
    localBestPosition.resize(DIM);
    globalBestPosition.resize(DIM);
    currentPosition.resize(DIM);
    velocity.resize(DIM);
    for (int i = 0 ; i < DIM; i++)
    {
        currentPosition[i].resize(SWARM_SIZE);
        localBestPosition[i].resize(SWARM_SIZE);
        globalBestPosition[i].resize(SWARM_SIZE);
        velocity[i].resize(SWARM_SIZE);
        for (int j = 0; j < SWARM_SIZE; j++)
        {
#if FINETUNE
            std::ifstream file("/home/ankur/catkin_ws/src/navigation/log/examples/parameters.yaml");
            std::string str, value;
            std::vector <std::string> file_contents;
            while (std::getline(std::getline(file, str, ':'), value, '\n'))
                file_contents.push_back(value);
            double generateRandom = 0.2 * (gsl_rng_uniform(r) - 0.5);
            currentPosition[i][j] = generateRandom + atof(file_contents[i].c_str());
            if(currentPosition[i][j] < 0.0)
                currentPosition[i][j] = 0.001;
//            cout << "currentValue: " << currentPosition[i][j] << " originalValue: " << atof(file_contents[i].c_str()) << " difference: " << (currentPosition[i][j] - atof(file_contents[i].c_str())) << "\n";
            file.close();
#else
            currentPosition[i][j] = SPAN * (gsl_rng_uniform(r) - 0.5);         // random initial swarm positions, positive values
#endif
            velocity[i][j] = 0.05 * (gsl_rng_uniform(r));             // random initial swarm velocities
        }
    }
    localBestPosition = currentPosition;
}

vector <float> PSO::getOptimal()
{
    std::ofstream textLog, dataLog;
    initialize();                                                               // Initialize Swarm
    getFitness();                                                               // pass parameters and get fitness
    bool terminationConditionAchieved = false;
    localBestFitness = currentFitness;
    vector <float>::iterator globalBestFitness = min_element(localBestFitness.begin(), localBestFitness.end());
    globalBestFitnessValue = localBestFitness[distance(localBestFitness.begin(), globalBestFitness)];

    for (int a = 0; a < SWARM_SIZE; a++)
        for (int b = 0; b < DIM; b++)
            globalBestPosition[b][a] = localBestPosition[b][distance(localBestFitness.begin(), globalBestFitness)];

    updateVelocityAndPosition();

    /* --------------- Update Swarm -----------------  */
    for (int loop = 0; loop < epoch; loop++)
    {
#if LOG
        dataLog.open(log, std::ofstream::out | std::ofstream::app);
        dataLog << "\nepoch: " << loop << " rate: " << penalty << "\n";
        dataLog.close();
#endif
        for (int i = 0; i < iteration; i++)
        {
#if LOG
            dataLog.open(log, std::ofstream::out | std::ofstream::app);
            dataLog << "\niteration: " << i << "\n";
            dataLog.close();
#endif
            getFitness();
            for (int j = 0; j < SWARM_SIZE; j++)
            {
                if (currentFitness[j] < localBestFitness[j])
                {
                    localBestFitness[j] = currentFitness[j];
                    for (int c = 0; c < DIM; c++)
                        localBestPosition[c][j] = currentPosition[c][j];
                }
            }

            vector <float>::iterator currentGlobalBestFitness = min_element(localBestFitness.begin(), localBestFitness.end());
            float currentGlobalBestFitnessValue = localBestFitness[distance(localBestFitness.begin(), currentGlobalBestFitness)];
            if (currentGlobalBestFitnessValue < globalBestFitnessValue)
            {
                globalBestFitnessValue = currentGlobalBestFitnessValue;
                for (int a = 0; a < SWARM_SIZE; a++)
                    for (int b = 0; b < DIM; b++)
                        globalBestPosition[b][a] = localBestPosition[b][distance(localBestFitness.begin(), currentGlobalBestFitness)];
            }
#if LOG
            dataLog.open(log, std::ofstream::out | std::ofstream::app);
            for (int k = 0; k < globalBestPosition.size(); k++)
                dataLog << globalBestPosition[k][1] << ",";
            dataLog<< globalBestFitnessValue;
            dataLog.close();
#endif
            terminationConditionAchieved = computeTerminationCondition();
            if (terminationConditionAchieved == true)
                break;
            updateVelocityAndPosition();
        }
        if (terminationConditionAchieved == true)
            break;
        penalty *= 1;
    }
    for (int i = 0; i < DIM; i++)
        optimalValues.push_back(globalBestPosition[i][1]);
    return optimalValues;
}
