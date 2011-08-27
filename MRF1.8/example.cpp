// example.cpp -- illustrates calling the MRF code

static char *usage = "usage: %s energyType (a number between 0 and 3)\n";

// Marshall's code does currently not compile under VC++:
#ifndef _MSC_VER
#define USE_MAXPROD
#endif

#include "mrf.h"
#include "ICM.h"
#include "GCoptimization.h"
#ifdef USE_MAXPROD
#include "MaxProdBP.h"
#endif
#include "TRW-S.h"
#include "BP-S.h"


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>


const int sizeX = 50;
const int sizeY = 50;
const int K = 10;

MRF::CostVal D[sizeX*sizeY*K];
MRF::CostVal V[K*K];
MRF::CostVal hCue[sizeX*sizeY];
MRF::CostVal vCue[sizeX*sizeY];

EnergyFunction* generate_DataARRAY_SmoothFIXED_FUNCTION()
{
    int i, j;


    // generate function
    for (i=0; i<K; i++)
	for (j=i; j<K; j++)
	    {
		V[i*K+j] = V[j*K+i] = (i == j) ? 0 : 2;
	    }
    MRF::CostVal* ptr;
    for (ptr=&D[0]; ptr<&D[sizeX*sizeY*K]; ptr++) *ptr = rand() % 10;
    for (ptr=&hCue[0]; ptr<&hCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3 - 1; // negative multiplier possible
    for (ptr=&vCue[0]; ptr<&vCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3;

    // allocate eng
    DataCost *data         = new DataCost(D);
    SmoothnessCost *smooth = new SmoothnessCost(V,hCue,vCue);
    EnergyFunction *eng    = new EnergyFunction(data,smooth);

    return eng;
}

EnergyFunction* generate_DataARRAY_SmoothTRUNCATED_LINEAR()
{
    // generate function
    MRF::CostVal* ptr;
    for (ptr=&D[0]; ptr<&D[sizeX*sizeY*K]; ptr++) *ptr = rand() % 10;
    for (ptr=&hCue[0]; ptr<&hCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3;
    for (ptr=&vCue[0]; ptr<&vCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3;
    MRF::CostVal smoothMax = 5, lambda = 2;

    // allocate eng

    DataCost *data         = new DataCost(D);
    SmoothnessCost *smooth = new SmoothnessCost(1,smoothMax,lambda,hCue,vCue);
    EnergyFunction *eng    = new EnergyFunction(data,smooth);

    return eng;
}


EnergyFunction* generate_DataARRAY_SmoothTRUNCATED_QUADRATIC()
{
    
    // generate function
    MRF::CostVal* ptr;
    for (ptr=&D[0]; ptr<&D[sizeX*sizeY*K]; ptr++) *ptr = rand() % 10;
    for (ptr=&hCue[0]; ptr<&hCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3;
    for (ptr=&vCue[0]; ptr<&vCue[sizeX*sizeY]; ptr++) *ptr = rand() % 3;
    MRF::CostVal smoothMax = 5,lambda=2;

    // allocate eng
    DataCost *data         = new DataCost(D);
    SmoothnessCost *smooth = new SmoothnessCost(2,smoothMax,lambda,hCue,vCue);
    EnergyFunction *eng    = new EnergyFunction(data,smooth);

    return eng;
}


MRF::CostVal dCost(int pix, int i)
{
    return (pix*i + i + pix) % 10;
}

MRF::CostVal fnCost(int pix1, int pix2, int i, int j)
{
    if (pix2 < pix1)
	{
	    int tmp;
	    tmp = pix1; pix1 = pix2; pix2 = tmp; 
	    tmp = i; i = j; j = tmp;
	}
    MRF::CostVal answer = (pix1*(i+1)*(j+2) + pix2*i*j*pix1 - 2*i*j*pix1) % 10;
    return answer;
}


EnergyFunction* generate_DataFUNCTION_SmoothGENERAL_FUNCTION()
{
    DataCost *data         = new DataCost(dCost);
    SmoothnessCost *smooth = new SmoothnessCost(fnCost);
    EnergyFunction *eng    = new EnergyFunction(data,smooth);

    return eng;
}

int main(int argc, char **argv)
{
    MRF* mrf;
    EnergyFunction *eng;
    MRF::EnergyVal E;
    double lowerBound;
    float t,tot_t;
    int iter;

    int seed = 1124285485;
    srand(seed);

    if (argc < 2) {
	fprintf(stderr, usage, argv[0]);
	exit(1);
    }
    int Etype = atoi(argv[1]);
    switch(Etype) {
    // Here are 4 sample energies to play with.
    case 0:
	eng = generate_DataARRAY_SmoothFIXED_FUNCTION();
	fprintf(stderr, "using fixed (array) smoothness cost\n");
	break;
    case 1:
	eng = generate_DataARRAY_SmoothTRUNCATED_LINEAR();
	fprintf(stderr, "using truncated linear smoothness cost\n");
	break;
    case 2:
	eng = generate_DataARRAY_SmoothTRUNCATED_QUADRATIC();
	fprintf(stderr, "using truncated quadratic smoothness cost\n");
	break;
    case 3:
	eng = generate_DataFUNCTION_SmoothGENERAL_FUNCTION();
	fprintf(stderr, "using general smoothness functions\n");
	break;
    default:
	fprintf(stderr, usage, argv[0]);
	exit(1);
    }

    bool runICM       = true;
    bool runExpansion = true;
    bool runSwap      = true;
#ifdef USE_MAXPROD
    bool runMaxProdBP = Etype != 3;
#endif
    bool runTRWS      = true;
    bool runBPS       = true;


    ////////////////////////////////////////////////
    //                     ICM                    //
    ////////////////////////////////////////////////
    if (runICM)
	{
	    printf("\n*******Started ICM *****\n");

	    mrf = new ICM(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter<6; iter++)
		{
		    mrf->optimize(10, t);

		    E = mrf->totalEnergy();
		    tot_t = tot_t + t ;
		    printf("energy = %d (%f secs)\n", E, tot_t);
		}

	    delete mrf;
	}

    ////////////////////////////////////////////////
    //          Graph-cuts expansion              //
    ////////////////////////////////////////////////
    if (runExpansion)
	{
	    printf("\n*******Started the graph-cuts expansion *****\n");
	    mrf = new Expansion(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter<6; iter++)
		{
		    mrf->optimize(1, t);

		    E = mrf->totalEnergy();
		    tot_t = tot_t + t ;
		    printf("energy = %d (%f secs)\n", E, tot_t);
		}

	    delete mrf;
	}

    ////////////////////////////////////////////////
    //          Graph-cuts swap                   //
    ////////////////////////////////////////////////
    if (runSwap)
	{
	    printf("\n*******Started the graph-cuts swap *****\n");
	    mrf = new Swap(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter<8; iter++)
		{
		    mrf->optimize(1, t);

		    E = mrf->totalEnergy();
		    tot_t = tot_t + t ;
		    printf("energy = %d (%f secs)\n", E, tot_t);
		}

   
	    delete mrf;
	}


#ifdef USE_MAXPROD
    ////////////////////////////////////////////////
    //          Belief Propagation                //
    ////////////////////////////////////////////////
    if (runMaxProdBP)
	{
	    printf("\n*******  Started MaxProd Belief Propagation *****\n");
	    mrf = new MaxProdBP(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter < 10; iter++)
		{
		    mrf->optimize(1, t);

		    E = mrf->totalEnergy();
		    tot_t = tot_t + t ;
		    printf("energy = %d (%f secs)\n", E, tot_t);
		}

	    
	    delete mrf;
	}
#endif

    ////////////////////////////////////////////////
    //                  TRW-S                     //
    ////////////////////////////////////////////////
    if (runTRWS)
	{
	    printf("\n*******Started TRW-S *****\n");
	    mrf = new TRWS(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter<10; iter++)
		{
		    mrf->optimize(10, t);

		    E = mrf->totalEnergy();
		    lowerBound = mrf->lowerBound();
		    tot_t = tot_t + t ;
		    printf("energy = %d, lower bound = %f (%f secs)\n", E, lowerBound, tot_t);
		}

	    delete mrf;
	}

    ////////////////////////////////////////////////
    //                  BP-S                     //
    ////////////////////////////////////////////////
    if (runBPS)
	{
	    printf("\n*******Started BP-S *****\n");
	    mrf = new BPS(sizeX,sizeY,K,eng);
	    mrf->initialize();
	    mrf->clearAnswer();
	    
	    E = mrf->totalEnergy();
	    printf("Energy at the Start= %d (%d,%d)\n", E,mrf->smoothnessEnergy(),mrf->dataEnergy());

	    tot_t = 0;
	    for (iter=0; iter<10; iter++)
		{
		    mrf->optimize(10, t);

		    E = mrf->totalEnergy();
		    tot_t = tot_t + t ;
		    printf("energy = %d (%f secs)\n", E, tot_t);
		}

	    delete mrf;
	}


    return 0;
}
