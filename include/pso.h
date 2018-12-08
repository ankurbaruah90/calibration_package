#include <vector>
#include <algorithm>
#include <math.h>
#include <gsl/gsl_rng.h>
#include <deque>
#include <sys/time.h>

#define V_MAX 2.0                                                               // max particle velocity
#define w 0.729                                                                 // inertia parameter w
#define c1 1.494                                                                // PSO parameter c1
#define c2 1.494                                                                // PSO parameter c2
#define SPAN 50                                                                 // Span of the swarm
#define SWARM_SIZE 20                                                           // Size of the swarm
#define DIM 2                                                                   // parameters
#define ITERATION 10                                                            // number of iterations

class PSO
{
public:
    const gsl_rng_type *T;
    gsl_rng *r;
    int optimalSolution, solutionCount;
    float globalBestFitnessValue, terminationThreshold;
    float random1, random2;

    std::vector <float> currentFitness, localBestFitness;
    std::vector < std::vector <float> > currentPosition, velocity, localBestPosition, globalBestPosition;

    float getScore(std::vector <float> values);
    void getFitness();
    void updateVelocityAndPosition();
    void initialize();
    std::vector<float> getOptimal();
    bool computeTerminationCondition();

    PSO();
    ~PSO();
};

PSO::PSO()
{
    gsl_rng_env_setup();
    struct timeval now;                                                 // Seed generation based on time
    gettimeofday(&now,0);
    unsigned long time_seed = now.tv_sec + now.tv_usec;
    T = gsl_rng_default;                                                // Random number generator setup
    r = gsl_rng_alloc(T);
    gsl_rng_set(r, time_seed);
    globalBestFitnessValue = 10000.0;
    terminationThreshold = 0.08;                                        // Termination related parameters
}

PSO::~PSO()
{
    gsl_rng_free(r);
}

//TODO: check for sanity of termination condition
bool PSO::computeTerminationCondition()
{
    float meanParticles[DIM], sumSquareParticles, devParticles = 1000.0;

    ///// Size of Particle Cloud /////
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < SWARM_SIZE; j++)
            meanParticles[i] += currentPosition[i][j];

    for (int k = 0; k < DIM; k++)
        meanParticles[k] = meanParticles[k]/SWARM_SIZE;

    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < SWARM_SIZE; j++)
            sumSquareParticles += pow((currentPosition[i][j] - meanParticles[i]), 2);

    devParticles = sqrt(sumSquareParticles)/SWARM_SIZE;

    if (devParticles <= terminationThreshold)
        return true;
    else
        return false;
}

void PSO::getFitness()
{
    for (int i = 0; i < SWARM_SIZE; i++)
    {
        std::vector <float> candidate_parameters;
        for (int j = 0; j < currentPosition.size(); j++)
            candidate_parameters.push_back(currentPosition[j][i]);

        // Storing fitness score corresponding to each particle
        currentFitness[i] = getScore(candidate_parameters);

        ///*---------Exterior Penalty - Quadratic Loss Function---------*///
        ///----- Add your custom penalty functions here------///

    }
}

void PSO::updateVelocityAndPosition()
{
    double variableVelocity = 0;
    /* ------- update velocity and position ------ */
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < SWARM_SIZE; j++)
        {
            // These random variables further stirs up the swarm
            // You might want to cross-verify the values here
            random1 = gsl_rng_uniform(r);
            random2 = gsl_rng_uniform(r);

            variableVelocity = w * velocity[i][j] + c1 * (random1 * (localBestPosition[i][j] - currentPosition[i][j])) + c2 * (random2 * (globalBestPosition[i][j] - currentPosition[i][j]));
            if (variableVelocity < 0)
                velocity[i][j] = fmax(variableVelocity, (-1.0 * V_MAX));
            else
                velocity[i][j] = fmin(variableVelocity, V_MAX);

            currentPosition[i][j] = currentPosition[i][j] + velocity[i][j];
        }
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
            currentPosition[i][j] = SPAN * (gsl_rng_uniform(r));      // random initial swarm positions, positive values
            velocity[i][j] = 0.05 * (gsl_rng_uniform(r));             // random initial swarm velocities
        }
    }
    localBestPosition = currentPosition;
}

std::vector <float> PSO::getOptimal()
{
    initialize();                                                               // Initialize Swarm
    getFitness();                                                               // pass parameters and get fitness
    bool terminationConditionAchieved = false;
    localBestFitness = currentFitness;
    std::vector <float>::iterator globalBestFitness = min_element(localBestFitness.begin(), localBestFitness.end());
    globalBestFitnessValue = localBestFitness[distance(localBestFitness.begin(), globalBestFitness)];

    for (int a = 0; a < SWARM_SIZE; a++)
        for (int b = 0; b < DIM; b++)
            globalBestPosition[b][a] = localBestPosition[b][distance(localBestFitness.begin(), globalBestFitness)];

    updateVelocityAndPosition();

    /* --------------- Update Swarm -----------------  */
    for (int i = 0; i < ITERATION; i++)
    {
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

        std::vector <float>::iterator currentGlobalBestFitness = min_element(localBestFitness.begin(), localBestFitness.end());
        float currentGlobalBestFitnessValue = localBestFitness[distance(localBestFitness.begin(), currentGlobalBestFitness)];
        if (currentGlobalBestFitnessValue < globalBestFitnessValue)
        {
            globalBestFitnessValue = currentGlobalBestFitnessValue;
            for (int a = 0; a < SWARM_SIZE; a++)
                for (int b = 0; b < DIM; b++)
                    globalBestPosition[b][a] = localBestPosition[b][distance(localBestFitness.begin(), currentGlobalBestFitness)];
        }

        terminationConditionAchieved = computeTerminationCondition();
        if (terminationConditionAchieved == true)
            break;

        updateVelocityAndPosition();
    }

    std::vector <float> optimal_particles;
    for (int j = 0; j < globalBestPosition.size(); j++)
        optimal_particles.push_back(globalBestPosition[j][0]);

    return optimal_particles;
}
