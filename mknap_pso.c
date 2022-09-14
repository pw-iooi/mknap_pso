//
//  @author Patricia Wong
//  mknapsack PSO
//


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>

/* global parameters*/
int RAND_SEED[] = {1,20,30,40,50,60,70,80,90,100,110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
int NUM_OF_RUNS = 1;
int MAX_TIME = 300;  //max amount of time permited (in sec); set to 5 mins 
int num_of_problems;

/* parameters for PSO algorithm*/
int NUM_OF_PARTICLES = 100; // 100
double C1 = 5; // 5.0
double C2 = 5;  // 5.0
double w = 5.0; // 5.0 inertia weight
double vMax = 10.0; // 10.0 maximum particle velocity 
int penalty_coeff = 1; // 1
int PSO_MAX_ITER = 5000; // 5000
/*************************************************************************************************************************/

// item struct 
struct item_struct{
    int dim; //no. of dimensions
    int* size; //volume of item in all dimensions
    int p;
    int indx;
};

// problem struct; contains the items (item_struct) and swarm (swarm_struct)
struct problem_struct{
    int n; //number of items
    int dim; //number of dimensions
    struct item_struct* items;
    struct swarm_struct* swarm; 
    int* capacities;  //knapsack capacities
};


// swarm struct; contains the particles (particle_struct) 
struct swarm_struct{
    struct particle_struct* particles;
    int* gBestPosition; 
    int gBest; // scalar (fitness value)
    bool gBestInitialized;
    int objective; 
    int feasibility; // indicate the feasibility of the solution 
    int* cap_left; // capacity left in all dimensions 
    int* gFinalPosition; 
}; 

struct particle_struct{
    int pBest; // scalar (fitness value)
    int* pBestPosition; 
    int* position; // position vector; solution x(j) = 1 element is packed and x(j) = 0 element is not packed
    double* velocity; // velocity vector; velocity is restricted to stay [0,1] *probability of position
    int objective; 
    int feasibility; // indicate the feasibility of the solution 
    int* cap_left; // capacity left in all dimensions     
}; 

// FUNCTIONS prototypes 
void initializeParticles(struct problem_struct** my_prob);
int calculateProfit(struct problem_struct* prob, struct particle_struct* particle);
int calculateConstraint(struct problem_struct* prob, struct particle_struct* particle, int index_m);
int calculatePoslin(struct problem_struct* prob, struct particle_struct* particle, int index_m);
int calculatePenalty(struct problem_struct* prob, struct particle_struct* particle, int temp);
void mknap_pso(struct problem_struct* prob);
void setpBestAndpBestPosition(struct problem_struct* prob, struct particle_struct* particle, int fitness, int objective);
void setgBestAndgBestPosition(struct problem_struct* prob, struct particle_struct* particle, int fitness);
void setgFinalPosition(struct problem_struct* prob, struct particle_struct* particle, int objective, int feasibility);

/*************************************************************************************************************************/
// particle position vector - get random integer value between min and max
int rand_int(int min, int max)
{
    int div = max-min+1;
    int val =rand() % div + min;
    //printf("rand_range= %d \n", val);
    return val;
}

/*************************************************************************************************************************/
// particle velocity vector - get random float value between [0,1]
double rand_01()
{
    double number;
    number = (double) rand();
    number = number/RAND_MAX;
    //printf("rand01=%f\n", number);
    return number;
}

/*************************************************************************************************************************/
// particle velocity vector - get random float value between [0,1]
double rand_02(double min, double max)
{
    double number = (double) rand()/RAND_MAX;
    //printf("rand01=%f\n", number);
    return (min + number*(max-min));
}

/*************************************************************************************************************************/

// note: at the beginning, particles are randomly spread over the search space
// velocities randomly generated and the local best and global best positions initialised
void initializeSwarm(struct problem_struct** my_prob) 
{
	for(int k=0; k<num_of_problems; k++)
	{	
		struct swarm_struct *new_swarm = malloc(sizeof(struct swarm_struct));
		my_prob[k]->swarm = new_swarm;
		new_swarm->gBest = 0;
		new_swarm->gBestInitialized = false;
		new_swarm->gBestPosition = malloc(sizeof(int)*my_prob[k]->n); 
		new_swarm->gFinalPosition = malloc(sizeof(int)*my_prob[k]->n); 
		new_swarm->particles = malloc(sizeof(struct particle_struct)*NUM_OF_PARTICLES); 
		new_swarm->objective = 0;
		new_swarm->feasibility = 0;
		new_swarm->cap_left = malloc(sizeof(int)*my_prob[k]->dim); 
	
		for(int i=0; i<NUM_OF_PARTICLES; i++)
		{	
			// random initialize particle velocity
			new_swarm->particles[i].velocity = malloc(sizeof(double)*my_prob[k]->n); 
			new_swarm->particles[i].pBestPosition = malloc(sizeof(int)*my_prob[k]->n);	
			
			// random generated velocities [0,1]
			for(int j=0; j<my_prob[k]->n; j++)
			{
				new_swarm->particles[i].velocity[j] = rand_01(); 
			}
			
			// random initialize particle position {0,1}
			new_swarm->particles[i].position = malloc(sizeof(int)*my_prob[k]->n); 
			
			for(int m=0; m<my_prob[k]->n; m++)
			{
				new_swarm->particles[i].position[m] = rand_int(0,1); 				
			}						
			new_swarm->particles[i].pBest = 0; 				
			new_swarm->particles[i].objective = 0;
			new_swarm->particles[i].feasibility = 0;
			new_swarm->particles[i].cap_left = malloc(sizeof(int)*my_prob[k]->dim); 		
		}
	}
}
/*************************************************************************************************************************/

void initializeParticles(struct problem_struct** my_prob)
{	
	// calculate fitness sum{j=1,...,n} p(j)x(j)
	for(int k=0; k<num_of_problems; k++)
	{	
		for(int j=0; j<NUM_OF_PARTICLES; j++)
		{
			int profit = calculateProfit(my_prob[k], &my_prob[k]->swarm->particles[j]); 
			int fitness = profit - calculatePenalty(my_prob[k], &my_prob[k]->swarm->particles[j], fitness);
			
			setpBestAndpBestPosition(my_prob[k], &my_prob[k]->swarm->particles[j],fitness,0);
						
			// check particle position against swarm global best 
			if(my_prob[k]->swarm->gBestInitialized == false)
			{
				setgBestAndgBestPosition(my_prob[k], &my_prob[k]->swarm->particles[j],fitness);
				my_prob[k]->swarm->gBestInitialized = true;	 
			}
			else if(fitness > my_prob[k]->swarm->gBest)
			{
				setgBestAndgBestPosition(my_prob[k], &my_prob[k]->swarm->particles[j],fitness);
				//printf("myProb[%i]->Particle[%i]->Subsequest gBest fitness = %d\n\n", k, j, fitness);
			}
		}	
	}// endfor
}
/*************************************************************************************************************************/

// Evaluate the particle solution
void evaluate_solution(struct problem_struct* prob, struct particle_struct* particle)
{
	particle->feasibility = 1;
	
	struct item_struct* items_p = prob->items; 
	
	for(int d =0; d<prob->dim; d++)
	{
		particle->cap_left[d] = prob->capacities[d];
		
		//printf("start[%i]->cap_left[%i]\n\n", d,particle->cap_left[d]);
		
		for(int j=0; j<prob->n; j++)
		{
			//printf("dim[%i]->item[%i]->cap_left->before %i\n\n", d,j,particle->cap_left[d]);
			//printf("item[%i]->size[%i]->pos[%i]\n\n",j, items_p[j].size[d],particle->position[j]); 
		
			particle->cap_left[d] -= items_p[j].size[d]*particle->position[j];
			
			//printf("dim[%i]->item[%i]->cap_left->after %i\n\n", d,j,particle->cap_left[d]);
			
			if(particle->cap_left[d]<0)
			{
				particle->feasibility = -1*d; // exceeding capacity
				//return;
			}
		}
	}
}

/*************************************************************************************************************************/

// Discrete PSO algorithm eq(3), eq(4), eq(5), eq(6)
void mknap_pso(struct problem_struct* prob)
{
	clock_t time_start, time_fin; 
	time_start = clock(); 
	double time_spent = 0;
	int iter = 0;	
	while(iter<PSO_MAX_ITER || time_spent<MAX_TIME) // 
	{
		for(int j=0; j<NUM_OF_PARTICLES; j++)
		{	
			struct particle_struct* particle = &prob->swarm->particles[j];
			double* newVelocity = malloc(sizeof(double)*prob->n);  
			int* newPosition = malloc(sizeof(int)*prob->n); 
													
			// compute random variable for p1 and p2		
			double p1 = rand_01(); // personal
			double p2 = rand_01(); // neighbors
						
			// eq(3) v_i(t) = w.v_i(t-1) + p1C1.(p_i-x_i(t-1)) + p2C2.(p_g-x_i(t-1))
			for(int i=0; i<prob->n; i++)
			{	
				newVelocity[i] = (particle->velocity[i]*w) + (C1*p1*(particle->pBestPosition[i]-particle->position[i])) + (C2*p2*(prob->swarm->gBestPosition[i]-particle->position[i]));
			}
						
			// map continuous velocity to binary value {0,1} using the sigmoid transformation function eq(5)
			// particle velocity is restricted to stay within an interval of [0,1]
			for(int i=0; i<prob->n; i++)
			{
				newVelocity[i] = 1.0 / (1 + exp(-newVelocity[i]));
				//printf("newVelocity[%i]->%lf...-newVelocity[i]->%lf\n", i, newVelocity[i], exp(-newVelocity[i]));
			}
		
			// compute random variable for delta in decision func
			//double delta = rand_01();
			//
			double delta = rand_02(0.0,1.0);
			// printf("Delta: %f\n", delta);
			
			// update particle position using decision funcion x_ij = {1 x<=0, x otherwise) eq(8)	
			for(int i=0; i<prob->n; i++)
			{								
				if (delta <= newVelocity[i]) 
				{
					newPosition[i] = 1; 
				}
				else
				{
					newPosition[i] = 0; 
				}				
			}
			
			// replace particle current velocity and position with new velocity and position
			for(int i=0; i<prob->n; i++) 
			{
				particle->velocity[i] = newVelocity[i];
				particle->position[i] = newPosition[i];
			}
						
			// calculate fitness value of f(x_i)
			int objective = calculateProfit(prob, particle); 
			//printf("objective before %d\n", objective);
			
			int fitness_x_i = objective - calculatePenalty(prob, particle, fitness_x_i);
			//printf("fitness_pso after %d\n", fitness_x_i);
			
			evaluate_solution(prob, particle); 
			
			int particle_feasibility = particle->feasibility;
			//printf("particle_feasibility: %i, particle_objective, %i", particle_feasibility, objective);
			//printf("\n\n");
				
			//printf("Particle[%i]->pBest[%i] before check\n", j, particle->pBest);
			//printf("Swarm->gBest[%i] before check\n", prob->swarm->gBest);
							
			// check particle position against particle local best 
			if(fitness_x_i > particle->pBest) 
			{
				setpBestAndpBestPosition(prob, particle, fitness_x_i, objective);
				//printf("Particle[%i]->pBest[%i] \n", j, fitness_x_i);		
			
				// check particle position against swarm global best
				if(fitness_x_i > prob->swarm->gBest)
				{
					setgBestAndgBestPosition(prob, particle, fitness_x_i);			
					//printf("Particle[%i]->gBest[%i] \n", j, fitness_x_i);
					
				}	
			}
			
			if(particle_feasibility == 1 && objective != 0)					
			{
				setgFinalPosition(prob, particle, objective, particle_feasibility);
			}
	
		// free memory 
		free(newVelocity); 
		free(newPosition);
		
		}// endfor	
			iter++;
			time_fin=clock(); 
			time_spent = (double)(time_fin-time_start)/CLOCKS_PER_SEC;	
	
	}// end while 
}

/*************************************************************************************************************************/
// set swarm Objective, feasibility and pFinalPosition values
void setgFinalPosition (struct problem_struct* prob, struct particle_struct* particle, int objective, int feasibility)
{
	for(int i=0; i<prob->n; i++)
	{
		prob->swarm->gBestPosition[i] = particle->position[i]; 				
	}
	
	prob->swarm->objective = objective;
	prob->swarm->feasibility = feasibility;
	//printf("setpBestAndpBestPosition->%i \n", particle->objective);
}
/*************************************************************************************************************************/

// set particle pBest and pBestPosition values
void setpBestAndpBestPosition (struct problem_struct* prob, struct particle_struct* particle, int fitness, int objective)
{
	for(int i=0; i<prob->n; i++)
	{
		particle->pBestPosition[i] = particle->position[i]; 				
	}
	particle->pBest = fitness;
	particle->objective = objective;
	//printf("setpBestAndpBestPosition->%i \n", particle->objective);
}
/*************************************************************************************************************************/

// set swarm gBest and gBestPosition values
void setgBestAndgBestPosition (struct problem_struct* prob, struct particle_struct* particle, int fitness)
{
	for(int i=0; i<prob->n; i++)
	{
		prob->swarm->gBestPosition[i] = particle->position[i]; 				
	}
	prob->swarm->gBest = fitness;
	
}
/*************************************************************************************************************************/

// Profit of the position
// func called in initializeParticles()
int calculateProfit(struct problem_struct* prob, struct particle_struct* particle)
{
	int profit = 0;
	
	// calculate profit sum{j=1,...,n} p_jx_j
	for(int i=0; i<prob->n; i++)
	{
		profit += prob->items[i].p*particle->position[i];
	}
	//printf("Profit %d\n", profit);	
	
	return profit; 
}

/*************************************************************************************************************************/

// return penalty value of fitness function eq(7)
// func called in initializeParticles()
int calculatePenalty(struct problem_struct* prob, struct particle_struct* particle, int temp)
{
	int penalty_value = 0; 
	//int penalty = 0; 
	
	// loop through all dim
	for(int i=0; i<prob->dim; i++)
	{
		// check constraint i
		int dist = calculatePoslin(prob, particle, i);
		//printf("dist: %i\n", dist);
		
		penalty_value+=dist; 
		
	}	
	penalty_value = penalty_value*penalty_coeff; 
	return penalty_value;
}
/*************************************************************************************************************************/

// poslin(x) = {0 x<=0, x otherwise) eq(8)
// return x or 0 
// func called in calculatePenalty()
int calculatePoslin(struct problem_struct* prob, struct particle_struct* particle, int index_m)
{
	int x  = calculateConstraint(prob, particle, index_m);	
	int capacity = prob->capacities[index_m];
	
	//printf("calculatePoslin index_m: %d, x: %d, capacity: %d diff: %d\n", index_m, x, capacity, x-capacity);
	
	if(x > capacity)
		return x-capacity;
	return 0;
}

/*************************************************************************************************************************/

// return sum(w_ij.x_j)
// func called in calculatePoslin()
int calculateConstraint(struct problem_struct* prob, struct particle_struct* particle, int index_m)
{
	int sum = 0; 
	
	for(int i=0; i<prob->n; i++)
	{
		struct item_struct* item_i = &prob->items[i]; 
		for(int d=0; d<prob->dim; d++)
		{
			if(d==index_m)
			{
				sum += particle->position[i]*item_i->size[d];
				//printf("Dim[%i]->item[%i]->weight: %d\n", d,i, item_i->size[d]);
			}			
		}
	}
	//printf("calculateConstraint index_m: %d, constraint: %d \n", index_m, sum);
	return sum; 
}

/*************************************************************************************************************************/

// initialize problem (each problem will have n dimension)
// func called in load_problems()
void init_problem(int n, int dim, struct problem_struct** my_prob)
{
    struct problem_struct* new_prob = malloc(sizeof(struct problem_struct));
    new_prob->n=n; new_prob->dim=dim;
    new_prob->items=malloc(sizeof(struct item_struct)*n);
    for(int j=0; j<n; j++)
        new_prob->items[j].size= malloc(sizeof(int)*dim);
    new_prob->capacities = malloc(sizeof(int)*dim);
    *my_prob = new_prob;
}

/*************************************************************************************************************************/

// example to create problem instances, actual date should come from file
// func called in main()
struct problem_struct** load_problems(char* data_file)
{
    int i,j,k;
    //int num_of_probs;
    FILE* pfile = fopen(data_file, "r");
    if(pfile==NULL)
        {printf("Data file %s does not exist. Please check!\n", data_file); exit(2); }
    fscanf (pfile, "%d", &num_of_problems);
 
    struct problem_struct** my_problems = malloc(sizeof(struct problem_struct*)*num_of_problems);
    for(k=0; k<num_of_problems; k++)
    {
        int n, dim, obj_opt;
        fscanf (pfile, "%d", &n);
        fscanf (pfile, "%d", &dim); fscanf (pfile, "%d", &obj_opt);
        
        init_problem(n, dim, &my_problems[k]);  //allocate data memory
        for(j=0; j<n; j++)
        {
            my_problems[k]->items[j].dim=dim;
            my_problems[k]->items[j].indx=j;
            fscanf(pfile, "%d", &(my_problems[k]->items[j].p)); //read profit data
            //printf("item[%d].p=%d\n",j,my_problems[k]->items[j].p);
        }
        for(i=0; i<dim; i++)
        {
            for(j=0; j<n; j++)
            {
                fscanf(pfile, "%d", &(my_problems[k]->items[j].size[i])); //read size data
                //printf("my_problems[%i]->items[%i].size[%i]=%d\n",k,j,i,my_problems[k]->items[j].size[i]);
            }
        }
        for(i=0; i<dim; i++)
        {
            fscanf(pfile, "%d", &(my_problems[k]->capacities[i]));
            //printf("capacities[%d]=%d\n",i,my_problems[k]->capacities[i] );
        }
    }
    fclose(pfile); //close file
    return my_problems;
}

/*************************************************************************************************************************/

// output a given solution to a file
// func called in main()
void output_solution(struct problem_struct* prob, char* out_file)
{
    if(out_file !=NULL){
        FILE* pfile = fopen(out_file, "a"); //append solution data
        fprintf(pfile, "%i\n", prob->swarm->objective);
        for(int i=0; i<prob->n; i++)
        {
            //fprintf(pfile, "%i ", sln->x[i]);
            fprintf(pfile, "%i ", prob->swarm->gBestPosition[i]);
        }
        fprintf(pfile, "\n");
        /*for(int j=0; j<sln->prob->n; j++)
            fprintf(pfile, "%i ", sln->prob->items[j].p);
        fprintf(pfile, "\n");*/
        fclose(pfile);
    }
    else
        //printf("sln.feas=%d, sln.obj=%f\n", sln->feasibility, sln->objective);
        printf("output\n");
}

/*************************************************************************************************************************/

// housekeeping - free memory for struct problem
// func called in main()
void free_problem(struct problem_struct* prob)
{
    if(prob!=NULL)
    {
        if(prob->capacities !=NULL) 
        	free(prob->capacities);
        	
        if(prob->items!=NULL)
        {
            for(int j=0; j<prob->n; j++)
            {
                if(prob->items[j].size != NULL)
                    free(prob->items[j].size);
            }
            free(prob->items);
        }
        if(prob->swarm!=NULL)
        {
		    free(prob->swarm->gBestPosition);
		    free(prob->swarm->gFinalPosition);
		    free(prob->swarm->cap_left);
		    prob->swarm->feasibility=false;
		    prob->swarm->objective=0;
		    prob->swarm->gBest=0;
		    if(prob->swarm->particles!=NULL)
		    {
		    	for(int i=0; i<NUM_OF_PARTICLES; i++)
		    	{
		    		if(prob->swarm->particles[i].cap_left != NULL)
		    			free(prob->swarm->particles[i].cap_left);
		    		if(prob->swarm->particles[i].pBestPosition != NULL)
		    			free(prob->swarm->particles[i].pBestPosition);
		    		if(prob->swarm->particles[i].position != NULL)
		    			free(prob->swarm->particles[i].position);        
		    		if(prob->swarm->particles[i].velocity != NULL)
		    			free(prob->swarm->particles[i].velocity); 
		  			
		  			prob->swarm->particles[i].objective=0;
		    		prob->swarm->particles[i].feasibility=0;	          					
		    	}// endfor   
		    	free(prob->swarm->particles);
       		 }  
    	}
    }
    free(prob);
}

/*************************************************************************************************************************/

// main method 
int main(int argc, const char * argv[]) {
    
    printf("Starting the run!\n");
    char data_file[50]={"somefile"}, out_file[50]={}, solution_file[50]={};  //max 50 problem instances per run
    if(argc<3)
    {
        printf("Insufficient arguments. Please use the following options:\n   -s data_file (compulsory)\n   -o out_file (default my_solutions.txt)\n   -c solution_file_to_check\n   -t max_time (in sec)\n");
        return 1;
    }
    else if(argc>9)
    {
        printf("Too many arguments.\n");
        return 2;
    }
    else
    {
        for(int i=1; i<argc; i=i+2)
        {
            if(strcmp(argv[i],"-s")==0)
                strcpy(data_file, argv[i+1]);
            else if(strcmp(argv[i],"-o")==0)
                strcpy(out_file, argv[i+1]);
            else if(strcmp(argv[i],"-c")==0)
                strcpy(solution_file, argv[i+1]);
            else if(strcmp(argv[i],"-t")==0)
                MAX_TIME = atoi(argv[i+1]);
        }
        //printf("data_file= %s, output_file= %s, sln_file=%s, max_time=%d", data_file, out_file, solution_file, MAX_TIME);
    }
    struct problem_struct** my_problems = load_problems(data_file);
    
    initializeSwarm(my_problems);
    initializeParticles(my_problems);
         
    if(strlen(solution_file)<=0)
    {
        if(strcmp(out_file,"")==0) strcpy(out_file, "my_solutions.txt"); //default output
        FILE* pfile = fopen(out_file, "w"); //open a new file
        fprintf(pfile, "%d\n", num_of_problems); fclose(pfile);
        for(int k=0; k<num_of_problems; k++)
        {
        	for (int run=0; run<NUM_OF_RUNS; run++)
    		{
    			//srand(RAND_SEED[run]);
    			mknap_pso(my_problems[k]); // call mknap_pso method    			
    		}			
   			output_solution(my_problems[k],out_file);		
        }
    }
    
     
    for(int k=0; k<num_of_problems; k++)
    { 	
       free_problem(my_problems[k]); //free problem data memory
       
    }
    free(my_problems); //free problems array    
    
    return 0;
}

/************************************End of Program /************************************/








