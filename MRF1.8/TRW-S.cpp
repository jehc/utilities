#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "TRW-S.h"


#define private public
#include "typeTruncatedQuadratic2D.h"
#undef private


#define m_D(pix,l)  m_D[(pix)*m_nLabels+(l)]
#define m_V(l1,l2)  m_V[(l1)*m_nLabels+(l2)]


#define MIN(a,b)  (((a) < (b)) ? (a) : (b))
#define MAX(a,b)  (((a) > (b)) ? (a) : (b))
#define TRUNCATE_MIN(a,b) { if ((a) > (b)) (a) = (b); }
#define TRUNCATE_MAX(a,b) { if ((a) < (b)) (a) = (b); }
#define TRUNCATE TRUNCATE_MIN

/////////////////////////////////////////////////////////////////////////////
//                  Operations on vectors (arrays of size K)               //
/////////////////////////////////////////////////////////////////////////////

inline void CopyVector(TRWS::REAL* to, MRF::CostVal* from, int K)
{
	TRWS::REAL* to_finish = to + K;
	do
	{
		*to ++ = *from ++;
	} while (to < to_finish);
}

inline void AddVector(TRWS::REAL* to, TRWS::REAL* from, int K)
{
	TRWS::REAL* to_finish = to + K;
	do
	{
		*to ++ += *from ++;
	} while (to < to_finish);
}

inline TRWS::REAL SubtractMin(TRWS::REAL *D, int K)
{
	int k;
	TRWS::REAL delta;
	
	delta = D[0];
	for (k=1; k<K; k++) TRUNCATE(delta, D[k]);
	for (k=0; k<K; k++) D[k] -= delta;

	return delta;
}

// Functions UpdateMessageTYPE (see the paper for details):
//
// - Set Di[ki] := gamma*Di_hat[ki] - M[ki]
// - Set M[kj] := min_{ki} (Di[ki] + V[ki,kj])
// - Normalize message: 
//        delta := min_{kj} M[kj]
//        M[kj] := M[kj] - delta
//        return delta
//
// If dir = 1, then the meaning of i and j is swapped.

///////////////////////////////////////////
//                  L1                   //
///////////////////////////////////////////

inline TRWS::REAL UpdateMessageL1(TRWS::REAL* M, TRWS::REAL* Di_hat, int K, TRWS::REAL gamma, MRF::CostVal lambda, MRF::CostVal smoothMax)
{
	int k;
	TRWS::REAL delta;

	delta = M[0] = gamma*Di_hat[0] - M[0];
	for (k=1; k<K; k++)
	{
		M[k] = gamma*Di_hat[k] - M[k];
		TRUNCATE(delta, M[k]);
		TRUNCATE(M[k], M[k-1] + lambda);
	}

	M[--k] -= delta;
	TRUNCATE(M[k], lambda*smoothMax);
	for (k--; k>=0; k--)
	{
		M[k] -= delta;
		TRUNCATE(M[k], M[k+1] + lambda);
		TRUNCATE(M[k], lambda*smoothMax);
	}

	return delta;
}

////////////////////////////////////////
//               L2                   //
////////////////////////////////////////

inline TRWS::REAL UpdateMessageL2(TRWS::REAL* M, TRWS::REAL* Di_hat, int K, TRWS::REAL gamma, MRF::CostVal lambda, MRF::CostVal smoothMax, void *buf)
{
	TRWS::REAL* Di = (TRWS::REAL*) buf;
	int* parabolas = (int*) ((char*)buf + K*sizeof(TRWS::REAL));
	int* intersections = parabolas + K;
	TypeTruncatedQuadratic2D::Edge* tmp = NULL;

	int k;
	TRWS::REAL delta;

	assert(lambda >= 0);

	Di[0] = gamma*Di_hat[0] - M[0];
	delta = Di[0];
	for (k=1; k<K; k++)
	{
		Di[k] = gamma*Di_hat[k] - M[k];
		TRUNCATE(delta, Di[k]);
	}

	if (lambda == 0)
	{
		for (k=0; k<K; k++) M[k] = 0;
		return delta;
	}

	tmp->DistanceTransformL2(K, 1, lambda, Di, M, parabolas, intersections);

	for (k=0; k<K; k++)
	{
		M[k] -= delta;
		TRUNCATE(M[k], lambda*smoothMax);
	}

	return delta;
}


//////////////////////////////////////////////////
//                FIXED_MATRIX                  //
//////////////////////////////////////////////////

inline TRWS::REAL UpdateMessageFIXED_MATRIX(TRWS::REAL* M, TRWS::REAL* Di_hat, int K, TRWS::REAL gamma, MRF::CostVal lambda, MRF::CostVal* V, void* buf)
{
	TRWS::REAL* Di = (TRWS::REAL*) buf;
	int ki, kj;
	TRWS::REAL delta;

	if (lambda == 0)
	{
		delta = gamma*Di_hat[0] - M[0];
		M[0] = 0;
		for (ki=1; ki<K; ki++) 
		{
			TRUNCATE(delta, gamma*Di_hat[ki] - M[ki]);
			M[ki] = 0;
		}
		return delta;
	}

	for (ki=0; ki<K; ki++)
	{
		Di[ki] = (gamma*Di_hat[ki] - M[ki]) * (1/(TRWS::REAL)lambda);
	}

	if (lambda > 0)
	{
		for (kj=0; kj<K; kj++)
		{
			M[kj] = Di[0] + V[0]; 
			V ++;
			for (ki=1; ki<K; ki++)
			{
				TRUNCATE(M[kj], Di[ki] + V[0]);
				V ++;
			}
			M[kj] *= lambda;
		}
	}
	else
	{
		for (kj=0; kj<K; kj++)
		{
			M[kj] = Di[0] + V[0]; 
			V ++;
			for (ki=1; ki<K; ki++)
			{
				TRUNCATE_MAX(M[kj], Di[ki] + V[0]);
				V ++;
			}
			M[kj] *= lambda;
		}
	}

	delta = M[0];
	for (kj=1; kj<K; kj++) TRUNCATE(delta, M[kj]);
	for (kj=0; kj<K; kj++) M[kj] -= delta;

	return delta;
}

/////////////////////////////////////////////
//                GENERAL                  //
/////////////////////////////////////////////

inline TRWS::REAL UpdateMessageGENERAL(TRWS::REAL* M, TRWS::REAL* Di_hat, int K, TRWS::REAL gamma, int dir, MRF::CostVal* V, void* buf)
{
	TRWS::REAL* Di = (TRWS::REAL*) buf;
	int ki, kj;
	TRWS::REAL delta;

	for (ki=0; ki<K; ki++)
	{
		Di[ki] = (gamma*Di_hat[ki] - M[ki]);
	}

	if (dir == 0)
	{
		for (kj=0; kj<K; kj++)
		{
			M[kj] = Di[0] + V[0]; 
			V ++;
			for (ki=1; ki<K; ki++)
			{
				TRUNCATE(M[kj], Di[ki] + V[0]);
				V ++;
			}
		}
	}
	else
	{
		for (kj=0; kj<K; kj++)
		{
			M[kj] = Di[0] + V[0];
			V += K;
			for (ki=1; ki<K; ki++)
			{
				TRUNCATE(M[kj], Di[ki] + V[0]);
				V += K;
			}
			V -= K*K - 1;
		}
	}

	delta = M[0];
	for (kj=1; kj<K; kj++) TRUNCATE(delta, M[kj]);
	for (kj=0; kj<K; kj++) M[kj] -= delta;

	return delta;
}
















TRWS::TRWS(int width, int height, int nLabels,EnergyFunction *eng):MRF(width,height,nLabels,eng)
{
    Allocate();
}
TRWS::TRWS(int nPixels, int nLabels,EnergyFunction *eng):MRF(nPixels,nLabels,eng)
{
    Allocate();
}

TRWS::~TRWS()
{ 
    delete[] m_answer;
    if ( m_needToFreeD ) delete [] m_D;
    if ( m_needToFreeV ) delete [] m_V;
	if ( m_messages ) delete [] m_messages;
	if ( m_DBinary ) delete [] m_DBinary;
	if ( m_horzWeightsBinary ) delete [] m_horzWeightsBinary;
	if ( m_vertWeightsBinary ) delete [] m_vertWeightsBinary;
}


void TRWS::Allocate()
{
	m_type = NONE;
    m_needToFreeV = false;
	m_needToFreeD = false;

	m_D = NULL;
	m_V = NULL;
	m_horzWeights = NULL;
	m_vertWeights = NULL;
	m_horzWeightsBinary = NULL;
	m_vertWeightsBinary = NULL;

	m_DBinary = NULL;
	m_messages = NULL;
	m_messageArraySizeInBytes = 0;

	m_answer = (Label *) new Label[m_nPixels];
    if ( !m_answer ){printf("\nNot enough memory, exiting");exit(0);}
}

void TRWS::clearAnswer()
{
    memset(m_answer, 0, m_nPixels*sizeof(Label));
	if (m_messages)
	{
		memset(m_messages, 0, m_messageArraySizeInBytes);
	}
}


MRF::EnergyVal TRWS::smoothnessEnergy()
{
    EnergyVal eng = (EnergyVal) 0;
    EnergyVal weight;
    int x,y,pix;

    if ( m_grid_graph )
    {
        if ( m_smoothType != FUNCTION  )
        {
            for ( y = 0; y < m_height; y++ )
                for ( x = 1; x < m_width; x++ )
                {
                    pix    = x+y*m_width;
                    weight = m_varWeights ? m_horzWeights[pix-1] :  1;
                    eng = eng + m_V(m_answer[pix],m_answer[pix-1])*weight;
                }

            for ( y = 1; y < m_height; y++ )
                for ( x = 0; x < m_width; x++ )
                {
                    pix = x+y*m_width;
                    weight = m_varWeights ? m_vertWeights[pix-m_width] :  1;
                    eng = eng + m_V(m_answer[pix],m_answer[pix-m_width])*weight;
                }
        }
        else
        {
            for ( y = 0; y < m_height; y++ )
                for ( x = 1; x < m_width; x++ )
                {
                    pix = x+y*m_width;
                    eng = eng + m_smoothFn(pix,pix-1,m_answer[pix],m_answer[pix-1]);
                }

            for ( y = 1; y < m_height; y++ )
                for ( x = 0; x < m_width; x++ )
                {
                    pix = x+y*m_width;
                    eng = eng + m_smoothFn(pix,pix-m_width,m_answer[pix],m_answer[pix-m_width]);
                }
        }
    }
    else
    {
        // not implemented
    }

    return(eng);
}



MRF::EnergyVal TRWS::dataEnergy()
{
    EnergyVal eng = (EnergyVal) 0;

    
    if ( m_dataType == ARRAY) 
    {
        for ( int i = 0; i < m_nPixels; i++ )
            eng = eng + m_D(i,m_answer[i]);
    }
    else
    {
        for ( int i = 0; i < m_nPixels; i++ )
            eng = eng + m_dataFn(i,m_answer[i]);
    }
    return(eng);

}


void TRWS::setData(DataCostFn dcost)
{
	int i, k;

	m_dataFn = dcost;
	CostVal* ptr;
	m_D = new CostVal[m_nPixels*m_nLabels];
	if ( !m_D ){printf("\nNot enough memory, exiting");exit(0);}
	for (ptr=m_D, i=0; i<m_nPixels; i++)
	for (k=0; k<m_nLabels; k++, ptr++)
	{
		*ptr = m_dataFn(i,k);
	}
	m_needToFreeD = true;
}

void TRWS::setData(CostVal* data)
{
	m_D = data;
	m_needToFreeD = false;
}


void TRWS::setSmoothness(SmoothCostGeneralFn cost)
{
	assert(m_horzWeights == NULL && m_vertWeights == NULL);

	int x, y, i, ki, kj;
	CostVal* ptr;

	m_smoothFn = cost;

	m_V = new CostVal[2*m_nPixels*m_nLabels*m_nLabels];
	if (!m_V) { fprintf(stderr, "Not enough memory!\n"); exit(1); }
	m_type = GENERAL;
	m_needToFreeV = true;

	for (ptr=m_V,i=0,y=0; y<m_height; y++)
	for (x=0; x<m_width; x++, i++)
	{
		if (x < m_width-1)
		{
			for (kj=0; kj<m_nLabels; kj++)
			for (ki=0; ki<m_nLabels; ki++)
			{
				*ptr++ = cost(i,i+1,ki,kj);
			}
		}
		else ptr += m_nLabels*m_nLabels;

		if (y < m_height-1)
		{
			for (kj=0; kj<m_nLabels; kj++)
			for (ki=0; ki<m_nLabels; ki++)
			{
				*ptr++ = cost(i,i+m_width,ki,kj);
			}
		}
		else ptr += m_nLabels*m_nLabels;
	}
}
void TRWS::setSmoothness(CostVal* V)
{
	m_type = FIXED_MATRIX;
    m_V = V;
}


void TRWS::setSmoothness(int smoothExp,CostVal smoothMax, CostVal lambda)
{
	assert(smoothExp == 1 || smoothExp == 2);
	assert(lambda >= 0);

	m_type = (smoothExp == 1) ? L1 : L2;

	int ki, kj;
	CostVal cost;

	m_needToFreeV = true;

	m_V = new CostVal[m_nLabels*m_nLabels];
	if (!m_V) { fprintf(stderr, "Not enough memory!\n"); exit(1); }

	for (ki=0; ki<m_nLabels; ki++)
	for (kj=ki; kj<m_nLabels; kj++)
	{
		cost = (CostVal) ((smoothExp == 1) ? kj - ki : (kj - ki)*(kj - ki));
		if (cost > smoothMax) cost = smoothMax;
		m_V[ki*m_nLabels + kj] = m_V[kj*m_nLabels + ki] = cost*lambda;
	}

	m_smoothMax = smoothMax;
	m_lambda = lambda;
}


void TRWS::setCues(CostVal* hCue, CostVal* vCue)
{
    m_horzWeights = hCue;
    m_vertWeights  = vCue;
}


void TRWS::initializeAlg()
{
	assert(m_type != NONE);

	int i;

	// determine type
	if (m_type == L1 && m_nLabels == 2)
	{
		m_type = BINARY;
	}

	// allocate messages
	int messageNum = (m_type == BINARY) ? 4*m_nPixels : 4*m_nPixels*m_nLabels;
	m_messageArraySizeInBytes = messageNum*sizeof(REAL);
	m_messages = new REAL[messageNum];
	if ( !m_messages ){printf("\nNot enough memory, exiting");exit(0);}
	memset(m_messages, 0, messageNum*sizeof(REAL));

	if (m_type == BINARY)
	{
		assert(m_DBinary == NULL && m_horzWeightsBinary == NULL && m_horzWeightsBinary == NULL);
		m_DBinary = new CostVal[m_nPixels];
		m_horzWeightsBinary = new CostVal[m_nPixels];
		m_vertWeightsBinary = new CostVal[m_nPixels];
		if ( !m_DBinary || !m_horzWeightsBinary || !m_vertWeightsBinary ){printf("\nNot enough memory, exiting");exit(0);}

	    if ( m_dataType == ARRAY)
		{
			for (i=0; i<m_nPixels; i++)
			{
				m_DBinary[i] = m_D[2*i+1] - m_D[2*i];
			}
		}
		else
		{
			for (i=0; i<m_nPixels; i++)
			{
				m_DBinary[i] = m_dataFn(i,1) - m_dataFn(i,0);
			}
		}

		assert(m_V[0] == 0 && m_V[1] == m_V[2] && m_V[3] == 0);
		for (i=0; i<m_nPixels; i++)
		{
			m_horzWeightsBinary[i] = (m_varWeights) ? m_V[1]*m_horzWeights[i] : m_V[1];
			m_vertWeightsBinary[i] = (m_varWeights) ? m_V[1]*m_vertWeights[i] : m_V[1];
		}
	}
}

void TRWS::optimizeAlg(int nIterations)
{
	assert(m_type != NONE);

	if (m_grid_graph)
	{
		switch (m_type)
		{
			case L1:            optimize_GRID_L1(nIterations);           break;
			case L2:            optimize_GRID_L2(nIterations);           break;
			case FIXED_MATRIX:  optimize_GRID_FIXED_MATRIX(nIterations); break;
			case GENERAL:       optimize_GRID_GENERAL(nIterations);      break;
			case BINARY:        optimize_GRID_BINARY(nIterations);       break;
			default: assert(0); exit(1);
		}
	}
	else {printf("\nNot implemented for general graphs yet, exiting!");exit(1);}

//	printf("lower bound = %f\n", m_lowerBound);

	////////////////////////////////////////////////
	//          computing solution                //
	////////////////////////////////////////////////

	if (m_type != BINARY)
	{
		int x, y, n, K = m_nLabels;
		CostVal* D_ptr;
		REAL* M_ptr;
		REAL* Di;
		REAL delta;
		int ki, kj;

		Di = new REAL[K];
		if ( !Di ){printf("\nNot enough memory, exiting");exit(0);}

		n = 0;
		D_ptr = m_D;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, D_ptr+=K, M_ptr+=2*K, n++)
		{
			CopyVector(Di, D_ptr, K);

			if (m_type == GENERAL)
			{
				CostVal* ptr = m_V + 2*(x+y*m_width-1)*K*K;
				if (x > 0)
				{
					kj = m_answer[n-1];
					for (ki=0; ki<K; ki++)
					{
						Di[ki] += ptr[kj + ki*K];
					}
				}
				ptr -= (2*m_width-3)*K*K;
				if (y > 0)
				{
					kj = m_answer[n-m_width];
					for (ki=0; ki<K; ki++)
					{
						Di[ki] += ptr[kj + ki*K];
					}
				}
			}
			else // m_type == L1, L2 or FIXED_MATRIX
			{
				if (x > 0)
				{
					kj = m_answer[n-1];
					CostVal lambda = (m_varWeights) ? m_horzWeights[n-1] : 1;
					for (ki=0; ki<K; ki++)
					{
						Di[ki] += lambda*m_V[kj*K + ki];
					}
				}
				if (y > 0)
				{
					kj = m_answer[n-m_width];
					CostVal lambda = (m_varWeights) ? m_vertWeights[n-m_width] : 1;
					for (ki=0; ki<K; ki++)
					{
						Di[ki] += lambda*m_V[kj*K + ki];
					}
				}
			}

			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			// compute min
			delta = Di[0];
			m_answer[n] = 0;
			for (ki=1; ki<K; ki++)
			{
				if (delta > Di[ki])
				{
					delta = Di[ki];
					m_answer[n] = ki;
				}
			}
		}

		delete [] Di;
	}
	else // m_type == BINARY
	{
		int x, y, n;
		REAL* M_ptr;
		REAL Di;

		n = 0;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, M_ptr+=2, n++)
		{
			Di = m_DBinary[n];
			if (x > 0) Di += (m_answer[n-1] == 0)       ? m_horzWeightsBinary[n-1]       : -m_horzWeightsBinary[n-1];
			if (y > 0) Di += (m_answer[n-m_width] == 0) ? m_vertWeightsBinary[n-m_width] : -m_vertWeightsBinary[n-m_width];

			if (x < m_width-1)  Di += M_ptr[0]; // message (x+1,y)->(x,y)
			if (y < m_height-1) Di += M_ptr[1]; // message (x,y+1)->(x,y)

			// compute min
			m_answer[n] = (Di >= 0) ? 0 : 1;
		}
	}
}

void TRWS::optimize_GRID_L1(int nIterations)
{
	int x, y, n, K = m_nLabels;
	CostVal* D_ptr;
	REAL* M_ptr;
	REAL* Di;

	Di = new REAL[K];
	if ( !Di ){printf("\nNot enough memory, exiting");exit(0);}

	for ( ; nIterations > 0; nIterations --)
	{
		// forward pass
		n = 0;
		D_ptr = m_D;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, D_ptr+=K, M_ptr+=2*K, n++)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			if (x < m_width-1) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_horzWeights[n] : m_lambda;
				UpdateMessageL1(M_ptr, Di, K, 0.5, lambda, m_smoothMax);
			}
			if (y < m_height-1) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_vertWeights[n] : m_lambda;
				UpdateMessageL1(M_ptr+K, Di, K, 0.5, lambda, m_smoothMax);
			}
		}

		// backward pass
		m_lowerBound = 0;

		n --;
		D_ptr -= K;
		M_ptr -= 2*K;

		for (y=m_height-1; y>=0; y--)
		for (x=m_width-1; x>=0; x--, D_ptr-=K, M_ptr-=2*K, n--)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			m_lowerBound += SubtractMin(Di, K);

			if (x > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_horzWeights[n-1] : m_lambda;
				m_lowerBound += UpdateMessageL1(M_ptr-2*K, Di, K, 0.5, lambda, m_smoothMax);
			}
			if (y > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_vertWeights[n-m_width] : m_lambda;
				m_lowerBound += UpdateMessageL1(M_ptr-(2*m_width-1)*K, Di, K, 0.5, lambda, m_smoothMax);
			}
		}
	}

	delete [] Di;
}

void TRWS::optimize_GRID_L2(int nIterations)
{
	int x, y, n, K = m_nLabels;
	CostVal* D_ptr;
	REAL* M_ptr;
	REAL* Di;
	void* buf;

	Di = new REAL[K];
	buf = new char[(2*K+1)*sizeof(int) + K*sizeof(REAL)];
	if ( !Di || !buf ){printf("\nNot enough memory, exiting");exit(0);}

	for ( ; nIterations > 0; nIterations --)
	{
		// forward pass
		n = 0;
		D_ptr = m_D;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, D_ptr+=K, M_ptr+=2*K, n++)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			if (x < m_width-1) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_horzWeights[n] : m_lambda;
				UpdateMessageL2(M_ptr, Di, K, 0.5, lambda, m_smoothMax, buf);
			}
			if (y < m_height-1) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_vertWeights[n] : m_lambda;
				UpdateMessageL2(M_ptr+K, Di, K, 0.5, lambda, m_smoothMax, buf);
			}
		}

		// backward pass
		m_lowerBound = 0;

		n --;
		D_ptr -= K;
		M_ptr -= 2*K;

		for (y=m_height-1; y>=0; y--)
		for (x=m_width-1; x>=0; x--, D_ptr-=K, M_ptr-=2*K, n--)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			m_lowerBound += SubtractMin(Di, K);

			if (x > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_horzWeights[n-1] : m_lambda;
				m_lowerBound += UpdateMessageL2(M_ptr-2*K, Di, K, 0.5, lambda, m_smoothMax, buf);
			}
			if (y > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_lambda*m_vertWeights[n-m_width] : m_lambda;
				m_lowerBound += UpdateMessageL2(M_ptr-(2*m_width-1)*K, Di, K, 0.5, lambda, m_smoothMax, buf);
			}
		}
	}

	delete [] Di;
	delete [] (char *)buf;
}


void TRWS::optimize_GRID_BINARY(int nIterations)
{
	int x, y, n;
	REAL* M_ptr;
	REAL Di;

	for ( ; nIterations > 0; nIterations --)
	{
		// forward pass
		n = 0;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, M_ptr+=2, n++)
		{
			Di = m_DBinary[n];
			if (x > 0) Di += M_ptr[-2]; // message (x-1,y)->(x,y)
			if (y > 0) Di += M_ptr[-2*m_width+1]; // message (x,y-1)->(x,y)
			if (x < m_width-1) Di += M_ptr[0]; // message (x+1,y)->(x,y)
			if (y < m_height-1) Di += M_ptr[1]; // message (x,y+1)->(x,y)

			REAL DiScaled = Di * 0.5;
			if (x < m_width-1) 
			{
				Di = DiScaled - M_ptr[0];
				CostVal lambda = m_horzWeightsBinary[n];
				if (lambda < 0) { Di = -Di; lambda = -lambda; }
				if (Di > lambda) M_ptr[0] = lambda;
				else             M_ptr[0] = (Di < -lambda) ? -lambda : Di;
			}
			if (y < m_height-1) 
			{
				Di = DiScaled - M_ptr[1];
				CostVal lambda = m_vertWeightsBinary[n];
				if (lambda < 0) { Di = -Di; lambda = -lambda; }
				if (Di > lambda) M_ptr[1] = lambda;
				else             M_ptr[1] = (Di < -lambda) ? -lambda : Di;
			}
		}

		// backward pass
		n --;
		M_ptr -= 2;

		for (y=m_height-1; y>=0; y--)
		for (x=m_width-1; x>=0; x--, M_ptr-=2, n--)
		{
			Di = m_DBinary[n];
			if (x > 0) Di += M_ptr[-2]; // message (x-1,y)->(x,y)
			if (y > 0) Di += M_ptr[-2*m_width+1]; // message (x,y-1)->(x,y)
			if (x < m_width-1) Di += M_ptr[0]; // message (x+1,y)->(x,y)
			if (y < m_height-1) Di += M_ptr[1]; // message (x,y+1)->(x,y)

			REAL DiScaled = Di * 0.5;
			if (x > 0) 
			{
				Di = DiScaled - M_ptr[-2];
				CostVal lambda = m_horzWeightsBinary[n-1];
				if (lambda < 0) { Di = -Di; lambda = -lambda; }
				if (Di > lambda) M_ptr[-2] = lambda;
				else             M_ptr[-2] = (Di < -lambda) ? -lambda : Di;
			}
			if (y > 0) 
			{
				Di = DiScaled - M_ptr[-2*m_width+1];
				CostVal lambda = m_vertWeightsBinary[n-m_width];
				if (lambda < 0) { Di = -Di; lambda = -lambda; }
				if (Di > lambda) M_ptr[-2*m_width+1] = lambda;
				else             M_ptr[-2*m_width+1] = (Di < -lambda) ? -lambda : Di;
			}
		}
	}

	m_lowerBound = 0;
}

void TRWS::optimize_GRID_FIXED_MATRIX(int nIterations)
{
	int x, y, n, K = m_nLabels;
	CostVal* D_ptr;
	REAL* M_ptr;
	REAL* Di;
	void* buf;

	Di = new REAL[K];
	buf = new REAL[K];
	if ( !Di || !buf ){printf("\nNot enough memory, exiting");exit(0);}

	for ( ; nIterations > 0; nIterations --)
	{
		// forward pass
		n = 0;
		D_ptr = m_D;
		M_ptr = m_messages;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, D_ptr+=K, M_ptr+=2*K, n++)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			if (x < m_width-1) 
			{
				CostVal lambda = (m_varWeights) ? m_horzWeights[n] : 1;
				UpdateMessageFIXED_MATRIX(M_ptr, Di, K, 0.5, lambda, m_V, buf);
			}
			if (y < m_height-1) 
			{
				CostVal lambda = (m_varWeights) ? m_vertWeights[n] : 1;
				UpdateMessageFIXED_MATRIX(M_ptr+K, Di, K, 0.5, lambda, m_V, buf);
			}
		}

		// backward pass
		m_lowerBound = 0;

		n --;
		D_ptr -= K;
		M_ptr -= 2*K;

		for (y=m_height-1; y>=0; y--)
		for (x=m_width-1; x>=0; x--, D_ptr-=K, M_ptr-=2*K, n--)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			m_lowerBound += SubtractMin(Di, K);

			if (x > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_horzWeights[n-1] : 1;
				m_lowerBound += UpdateMessageFIXED_MATRIX(M_ptr-2*K, Di, K, 0.5, lambda, m_V, buf);
			}
			if (y > 0) 
			{
				CostVal lambda = (m_varWeights) ? m_vertWeights[n-m_width] : 1;
				m_lowerBound += UpdateMessageFIXED_MATRIX(M_ptr-(2*m_width-1)*K, Di, K, 0.5, lambda, m_V, buf);
			}
		}
	}

	delete [] Di;
	delete [] (REAL *)buf;
}

void TRWS::optimize_GRID_GENERAL(int nIterations)
{
	int x, y, n, K = m_nLabels;
	CostVal* D_ptr;
	REAL* M_ptr;
	REAL* Di;
	void* buf;

	Di = new REAL[K];
	buf = new REAL[K];
	if ( !Di || !buf ){printf("\nNot enough memory, exiting");exit(0);}

	for ( ; nIterations > 0; nIterations --)
	{
		// forward pass
		n = 0;
		D_ptr = m_D;
		M_ptr = m_messages;
		CostVal* V_ptr = m_V;

		for (y=0; y<m_height; y++)
		for (x=0; x<m_width; x++, D_ptr+=K, M_ptr+=2*K, V_ptr+=2*K*K, n++)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			if (x < m_width-1) 
			{
				UpdateMessageGENERAL(M_ptr, Di, K, 0.5, /* forward dir*/ 0, V_ptr, buf);
			}
			if (y < m_height-1) 
			{
				UpdateMessageGENERAL(M_ptr+K, Di, K, 0.5, /* forward dir*/ 0, V_ptr+K*K, buf);
			}
		}

		// backward pass
		m_lowerBound = 0;

		n --;
		D_ptr -= K;
		M_ptr -= 2*K;
		V_ptr -= 2*K*K;

		for (y=m_height-1; y>=0; y--)
		for (x=m_width-1; x>=0; x--, D_ptr-=K, M_ptr-=2*K, V_ptr-=2*K*K, n--)
		{
			CopyVector(Di, D_ptr, K);
			if (x > 0) AddVector(Di, M_ptr-2*K, K); // message (x-1,y)->(x,y)
			if (y > 0) AddVector(Di, M_ptr-(2*m_width-1)*K, K); // message (x,y-1)->(x,y)
			if (x < m_width-1) AddVector(Di, M_ptr, K); // message (x+1,y)->(x,y)
			if (y < m_height-1) AddVector(Di, M_ptr+K, K); // message (x,y+1)->(x,y)

			// normalize Di, update lower bound
			m_lowerBound += SubtractMin(Di, K);

			if (x > 0) 
			{
				m_lowerBound += UpdateMessageGENERAL(M_ptr-2*K, Di, K, 0.5, /* backward dir */ 1, V_ptr-2*K*K, buf);
			}
			if (y > 0)
			{
				m_lowerBound += UpdateMessageGENERAL(M_ptr-(2*m_width-1)*K, Di, K, 0.5, /* backward dir */ 1, V_ptr-(2*m_width-1)*K*K, buf);
			}
		}
	}

	delete [] Di;
	delete [] (REAL *)buf;
}
