#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <time.h>
#include <cstdlib>
#include <algorithm>

using namespace std;

#define numberOfCities 20
#define numberOfIndividuals 100
#define maxIt 300
#define mutationRate 0.015
#define tournamentSize 5
#define elitism true

double **Dmatrix;

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

	// Gets the distance matrix
	double** Dmat(){
		double** distance;
		distance = new double *[numberOfCities];
		for(int i = 0; i < numberOfCities; i++){
			distance[i] = new double[numberOfCities];
		}
		for (int i = 0; i < numberOfCities; i++)
		{
			for (int j = i; j < numberOfCities; j++)
			{
				int xDistance = x[i] - x[j];
				int yDistance = y[i] - y[j];
				distance[i][j] = sqrt( (xDistance*xDistance) + (yDistance*yDistance) );
				distance[j][i] = distance[i][j];
			}
		}

		return distance;
	}
};

int myrandom (int i) { return std::rand()%i;}

class Tour{
private:
	// Holds our tour of cities
	vector<int> tour;
	double fitness;
	int distance;
public:
	// Constructs a blank tour
	Tour(){
		for (int i = 0; i <numberOfCities; i++) {
			tour.push_back(i);
		}
		random_shuffle(tour.begin() + 1,tour.end(), myrandom);
		fitness = 0;
		distance = 0;
	}

	Tour(vector<int> newTour){
		tour = newTour;
		fitness = 0;
		distance = 0;
	}

	// Gets the tours fitness
	double getFitness() {
		if (fitness == 0) {
			fitness = 1.0/(double)getDistance();
		}
		return fitness;
	}

	//gets the city of a given position on the tour
	int getCity(int i){
		return tour[i];
	}

	//swap two positions in the tour
	int swapCities(int i, int j){
		int aux = tour[i];
		tour[i] = tour[j];
		tour[j] = aux;
	}

	// Gets the total distance of the tour
	int getDistance(){
		if (distance == 0) {
			for(int i=1; i<numberOfCities; i++){
				distance += Dmatrix[tour[i-1]][tour[i]];
			}
			distance += Dmatrix[tour[0]][tour.back()];
		}
		return distance;
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
		// If we need to initialise a population of tours do so
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
	int populationSize() {
		return popSize;
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
		//criate vector of int for the cities and add the starting position
		vector<int> vec;
		for (int i = 0; i < numberOfCities; i++)
		{
			vec.push_back(0);
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
			//select the empty positions on the vec
			if(vec[i] == 0){
				for (int j = 1; j < numberOfCities; j++){
					bool isOnVec = false;
					//check if the city is alredy on the vec and if is not place at the empty place
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


		Tour child = Tour(vec);
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

int main(){
	srand(time(NULL));
	FILE *dados,*dados1;

	dados = fopen("Dados.txt","w");
    dados1 = fopen("Dados1.txt","w");

	AG ag;
	Population pop(numberOfIndividuals,true);
	Cities ct;
	Dmatrix = ct.Dmat();

	Tour t, tinit;
	tinit = pop.getFittest();
	for(int j = 0; j < numberOfCities; j++){
        fprintf(dados1,"%d  %d\n",ct.x[tinit.getCity(j)],ct.y[tinit.getCity(j)]);
    }
	for(int loop=0; loop< maxIt; loop++){
		t = pop.getFittest();
		for (int j = 0; j < numberOfCities; j++){
				cout << t.getCity(j) << " ";
		}
		cout<< t.getDistance() << " " << t.getFitness() << endl;
		pop = ag.evolvePopulation(pop);
	}
	t = pop.getFittest();
    for (int j = 0; j < numberOfCities; j++){
        cout << t.getCity(j) << " ";
				//fprintf(dados,"%d ",t.getCity(j));
    }
    cout<< t.getDistance() << " " << t.getFitness() << endl;
    for(int j = 0; j < numberOfCities; j++){
        fprintf(dados,"%d  %d\n",ct.x[t.getCity(j)],ct.y[t.getCity(j)]);
    }

    fclose(dados);
    fclose(dados1);

	return 0;
}
