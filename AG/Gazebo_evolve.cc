#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <time.h>
#include <cstdlib>
#include <algorithm>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Publisher.hh>
#include <boost/shared_ptr.hpp>
#include "msgs/build/Evolve.pb.h"
#include "msgs/build/EvolveRequest.pb.h"

using namespace std;

#define numberOfCities 20
#define numberOfIndividuals 100
#define maxIt 300
#define mutationRate 0.015
#define tournamentSize 5
#define NumRobots 10
#define elitism true

typedef const boost::shared_ptr<const evolve_robots_msgs::msgs::EvolveRequest> evReqPtr;
typedef const boost::shared_ptr<const evolve_robots_msgs::msgs::Evolve> EvPtr;

int counter;

class Cities{
public:
	int x[numberOfCities];
	int y[numberOfCities];

	// Constructs a randomly placed city
	Cities(){
		for (int i = 0; i < numberOfCities; i++)
		{
			x[i] = (rand() %200);
			y[i] = (rand() %200);
		}
	}

	// Constructs cities at chosen x, y location
	Cities(int x[numberOfCities], int y[numberOfCities]){
		x = x;
		y = y;
	}

	// Gets city's x coordinate
	int getX(int i){
		return x[i];
	}

	// Gets city's y coordinate
	int getY(int i){
		return y[i];
	}
};

int myrandom (int i) { return std::rand()%i;};

class Tour{
private:
	// Holds our tour of cities
	vector<int> tour;
	vector<double> speed;
	double fitness;
	double time;
	vector<double> carrot;

public:
	// Constructs a blank tour
	Tour(){
		for (int i = 0; i <numberOfCities; i++) {
			tour.push_back(i);
			speed.push_back(1.0 + 2.0*(((double)rand())/RAND_MAX));
			carrot.push_back((double)rand())/RAND_MAX;
		}
		random_shuffle(tour.begin() + 1,tour.end(), myrandom);
		fitness = 0;
		time = 0;
	}

	Tour(vector<int> newTour, vector<double> newSpeed,vector<double> newCarrot){
		tour = newTour;
		speed = newSpeed;
		carrot = newCarrot;
		fitness = 0;
		time = 0;
	}

	// Gets the tours fitness
	double getFitness() {
		if (fitness == 0) {
			fitness = 1.0/time;
		}
		return fitness;
	}

	//gets the city of a given position on the tour
	int getCity(int i){
		return tour[i];
	}

	//gets the speed of a given position on the tour
	double getSpeed(int i){
		return speed[i];
	}

	//gets the carrot of the tour
	double getCarrot(int i){
		return carrot[i];
	}

	//swap two positions in the tour
	void swapCities(int i, int j){
		int aux = tour[i];
		tour[i] = tour[j];
		tour[j] = aux;
	}

	//mutates the tour speed for a set patch
	void changeSpeed(int i){
        speed[i] = speed[i] + (-0.5 + 1.0*(((double)rand())/RAND_MAX));
	}

	//mutates the tour carrot vector
	void changeCarrot(int  i){
        this->carrot[i] = this->carrot[i] + (-0.05 + 0.1*(((double)rand())/RAND_MAX));
	}

	// Gets the total travel time for the tour
	void setTime (double time){
        this->time = time;
	}
};

class Population {
private:
	// Holds population of tours
	Tour* tours;
	int popSize;

public:
    // Saves a tour
	void saveTour(int index, Tour tour) {
		tours[index] = tour;
	}

	// Construct a population
	Population(int populationSize, bool initialise) {
		tours = new Tour[populationSize];

        // If we need to initialize a population of tours do so
		if (initialise) {
			// Loop and create individuals
			for (int i = 0; i < populationSize; i++) {
				Tour newTour;
				saveTour( i, newTour);
			}
		}
		popSize = populationSize;
	}

	// Gets a tour from population
	Tour getTour(int index) {
		return tours[index];
	}

	// Gets the best tour in the population
	Tour getFittest() {
		Tour fittest = tours[0];
		// Loop through individuals to find fittest
		for (int i = 0; i < popSize; i++) {
			if (fittest.getFitness() <= getTour(i).getFitness()) {
				fittest = getTour(i);
			}
		}
		return fittest;
	}

	// Gets population size
	int populationSize() 
	{
		return popSize;
	}

	void cb(EvPtr &_msg)
    {
        counter++;
        this->tours[_msg->index()].setTime(_msg->time());
    }

    void SendGenes( gazebo::transport::PublisherPtr imagePub){
    	for(int i=0; i<NumRobots; i++){
    		evReqPtr request;
    		request->set_index(i);
    		for( int j=0; j<numberOfCities; j++){	
    			request->set_road(j,this->tours[i].getCity(j));
    			request->set_speeds(j,this->tours[i].getSpeed(j));
			request->set_carrot(this->tours[i].getCarrot(j));
    		}    		
    	imagePub->WaitForConnection();
        imagePub->Publish(*request);
    	}
    }
};

class AG{
public:
	AG(){
	}

	// Evolves a population over one generation
	Population evolvePopulation(Population pop) {
		Population newPopulation(pop.populationSize(), false);

		// Keep our best individual if elitism is enabled
		int elitismOffset = 0;
		if (elitism) {
			newPopulation.saveTour(0, pop.getFittest());
			elitismOffset = 1;
		}

		// Crossover population
		// Loop over the new population's size and create individuals from
		// Current population
		for (int i = elitismOffset; i < newPopulation.populationSize(); i++) {
			// Select parents
			Tour parent1 = tournamentSelection(pop);
			Tour parent2 = tournamentSelection(pop);
			// Crossover parents
			Tour child = crossover(parent1, parent2);
			// Add child to new population
			newPopulation.saveTour(i, child);
		}

		// Mutate the new population a bit to add some new genetic material
		for (int i = elitismOffset; i < newPopulation.populationSize(); i++) {
			mutate(newPopulation.getTour(i));
		}

		return newPopulation;
	}

	// Applies crossover to a set of parents and creates offspring
	Tour crossover(Tour parent1, Tour parent2) {
		//create vector of int for the cities and of double for the speeds, and add the starting position
		vector<int> vec;
		vector<double> speed;
		for (int i = 0; i < numberOfCities; i++)
		{
			vec.push_back(0);
			speed.push_back(0.0);
		}

		// Get start and end sub tour positions for parent1's tour
		int startPos = (rand() % (numberOfCities-1))+1;
		int endPos = (rand() % (numberOfCities-1))+1;

		// Loop and add the sub tour from parent1 to our vec
		for (int i = 1; i < numberOfCities; i++) {
			// If our start position is less than the end position
			if (startPos < endPos && i > startPos && i < endPos) {
				vec[i]=parent1.getCity(i);
			} // If our start position is larger
			else if (startPos > endPos) {
				if (!(i < startPos && i > endPos)) {
					vec[i]=parent1.getCity(i);
				}
			}
		}


		// Loop through parent2's city tour
		for (int i = 1; i < numberOfCities; i++) {
			//select the empty positions on the vector
			if(vec[i] == 0){
				for (int j = 1; j < numberOfCities; j++){
					bool isOnVec = false;
					//check if the city is already on the vector and if is not place at the empty place
					for(int k=1; k<numberOfCities; k++){
						if(vec[k] != 0 && vec[k] == parent2.getCity(j)){
							isOnVec = true;
							break;
						}
					}
					if(!isOnVec){
						vec[i] = parent2.getCity(j);
					}
				}
			}
		}

		double alpha = (((double)rand())/RAND_MAX);
		for (int i = 1; i < numberOfCities; i++) {
			speed.push_back(alpha*parent1.getSpeed(i) + (1-alpha)*parent2.getSpeed(i));
			carrot.push_back(alpha*parent1.getCarrot(i) + (1-alpha)*parent2.getCarrot(i));
		}

		Tour child = Tour(vec, speed, carrot);
		return child;
	}

	// Mutate a tour using swap mutation
	void mutate(Tour tour) {
		// Loop through tour cities
		for(int tourPos1=1; tourPos1 < numberOfCities; tourPos1++){
			// Apply mutation rate
			if(rand() < mutationRate){
				// Get a second random position in the tour
				int tourPos2 = (rand() % (numberOfCities-1))+1;

				// Swap them around
				tour.swapCities(tourPos2, tourPos1);
			}

			if(rand() < mutationRate){
                //Increases or decreases the speed for a set patch
                tour.changeSpeed(tourPos1);
			}

			if(rand() < mutationRate){
                //Changes carrot vector
                tour.changeCarrot(tourPos1);
			}
		}
	}

	// Selects candidate tour for crossover
	Tour tournamentSelection(Population pop) {
		// Create a tournament population
		Population tournament (tournamentSize, false);
		// For each place in the tournament get a random candidate tour and add it
		for (int i = 0; i < tournamentSize; i++) {
			int randomId = (rand() % pop.populationSize());
			tournament.saveTour(i, pop.getTour(randomId));
		}
		// Get the fittest tour
		Tour fittest = tournament.getFittest();
		return fittest;
	}
};

int main(int _argc, char **_argv){

    // Populate a std::vector with the command line arguments
    std::vector<std::string> v;
    for (size_t i = 0; i < _argc; ++i)
    v.push_back(std::string(_argv[i]));

    // Initialize gazebo.
    gazebo::setupServer(v);


    srand(time(NULL));
    FILE *dados,*dados1;

	dados = fopen("Dados.txt","w");
    dados1 = fopen("Dados1.txt","w");

	//Initializes genetic logic, population and waypoints
	AG ag;
	Population pop(numberOfIndividuals,true);
	Cities ct;

    // Load a world ajustar isso
    gazebo::physics::WorldPtr world = gazebo::loadWorld("worlds/empty.world");

    // Create our nodes for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/Evolve", &Population::cb, &pop);
    gazebo::transport::PublisherPtr sender = node->Advertise<evolve_robots_msgs::msgs::EvolveRequest>("~/EvolveRequest");

    Tour fittest;
    for(int loop=0; loop<=maxIt; loop++){
        //Send to world new parameters for robots
        pop.SendGenes(sender);

    	//Receive fitness for each robot simulated
        counter = 0;
        while(counter < NumRobots){
            gazebo::common::Time::MSleep(10);
        }
        fittest = pop.getFittest();
        pop = ag.evolvePopulation(pop);
    }
    fittest = pop.getFittest();
    cout<< fittest.getFitness() << endl;
    for(int j = 0; j < numberOfCities; j++){
        fprintf(dados,"Point: %d  %d ",ct.x[fittest.getCity(j)],ct.y[fittest.getCity(j)]);
        fprintf(dados," Speed: %f \n",fittest.getSpeed(j));
	fprintf(dados,"Carrot: %f\n",fittest.getCarrot(j));
    }

    fclose(dados);
    fclose(dados1);

    // Close everything.
    gazebo::shutdown();
}
