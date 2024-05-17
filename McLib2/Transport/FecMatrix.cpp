#include "StdAfx.h"
#include "FecMatrix.h"
#include "GaloisField256.h"

using namespace VideoTransmission;

FecMatrix::FecMatrix(int numberElements, int redundancyLevel)
{
    // Store coefficient
    m_NumberElements = numberElements;
    m_RedundancyLevel = redundancyLevel;
    // Allocate matrix
    m_matrix = new BYTE*[m_NumberElements + m_RedundancyLevel];
    // Allocate and initialize first rows as identity
    for (int i = 0; i < m_NumberElements; i++)
    {
        m_matrix[i] = new BYTE[numberElements];
        for (int j = 0; j < m_NumberElements; j++)
            m_matrix[i][j] = 0;
        m_matrix[i][i] = 1;
    }
    // Allocate and initialize redundancy rows as explained by MSR
    if (m_RedundancyLevel >= 1)
    {
        m_matrix[m_NumberElements] = new BYTE[m_NumberElements];
        for (int j = 0; j < m_NumberElements; j++)
        {
            m_matrix[m_NumberElements][j] = 1;
        }
    }
    for (int i = 1; i < m_RedundancyLevel; i++)
    {
        m_matrix[m_NumberElements + i] = new BYTE[m_NumberElements];

        for (int j = 0; j < m_NumberElements; j++)
        {
            m_matrix[m_NumberElements + i][j] = GaloisField256::Divide(
                (BYTE)((j + 1) ^ (m_NumberElements + 1)), (BYTE)((j + 1) ^ (m_NumberElements + i + 1)));
        }
    }
	// Inverted and transfer only used when actual inversion required.
	m_transfer = 0;
    m_inverted = 0;
}

FecMatrix::~FecMatrix(void)
{
	for (int i = 0; i < (m_NumberElements + m_RedundancyLevel); i++)
    {
        delete m_matrix[i];
		m_matrix[i] = 0;
	}
	delete m_matrix;
	
	if (m_transfer != 0)
	{
		for (int i = 0; i < m_NumberElements; i++)
		{
			delete m_transfer[i];
			m_transfer[i] = 0;
		}
		delete m_transfer;
	}

	if (m_inverted !=0)
	{
		for (int i = 0; i < m_NumberElements; i++)
		{
			delete m_inverted[i];
			m_inverted[i] = 0;
		}
		delete m_inverted;
	}
}

/// <summary>
/// Initialize the m_transfer and m_inverted matrices, as a function
/// of the list of elements present after transfer.
/// </summary>
/// <param name="present"></param>
bool FecMatrix::InvertMatrixInit(bool present[])
{
	if (m_transfer == 0)
	{
		m_transfer = new BYTE*[m_NumberElements ];
	}
	if (m_inverted == 0)
	{
		m_inverted = new BYTE*[m_NumberElements];
	}

    // Initialize both transfer and invert to unity.
    for (int i = 0; i < m_NumberElements; i++)
    {
        m_transfer[i] = new BYTE[m_NumberElements];
        m_inverted[i] = new BYTE[m_NumberElements];
        for (int j = 0; j < m_NumberElements; j++)
        {
            m_transfer[i][j] = 0;
            m_inverted[i][j] = 0;
        }
        m_transfer[i][i] = 1;
        m_inverted[i][i] = 1;
    }
    // Initialize the redondancy rows
    int first = 0;
    int row;
    for (row = 0; row < m_NumberElements; row++)
    {
        if (present[row])
            first++;
    }
    while (first < m_NumberElements && row < (m_NumberElements+m_RedundancyLevel))
    {
        if (present[row])
        {
            int column = 0;
            for (int i = 0; i < m_NumberElements; i++)
            {
                if (present[i])
                {
                    m_transfer[first][column] = m_matrix[row][i];
                    column++;
                }
            }
            for (int i = 0; i < m_NumberElements; i++)
            {
                if (!present[i])
                {
                    m_transfer[first][column] = m_matrix[row][i];
                    column++;
                }
            }
            first++;
        }
        row++;
    }
    return (first >= m_NumberElements);
}
/// <summary>
/// Using the Gauss Jordan algorithm, compute the invert matrix.
/// </summary>
bool FecMatrix::GaussJordanReduce()
{
    bool ret = true;
    // Proceed row by row, until the end.
    for (int i = 0; i < m_NumberElements; i++)
    {
        // Verify that the matrix is regular
        if (m_transfer[i][i] == 0)
        {
            ret = false;
            break;
        }
        // Make sure that pivot is set to unity.
        if (m_transfer[i][i] != 1)
        {
			// TODO: Could be optimized by going to logarithms explicitly.
            BYTE a = GaloisField256::Divide(1, m_transfer[i][i]);
            for (int j = 0; j < m_NumberElements; j++)
            {
                m_transfer[i][j] = GaloisField256::Multiply(a, m_transfer[i][j]);
                m_inverted[i][j] = GaloisField256::Multiply(a, m_inverted[i][j]);
            }
        }
        // Reduce the ith column in all the other rows.
        for (int j = 0; j < m_NumberElements; j++)
        {
            if (j == i)
                continue;
            // In the modulo group, the addition is XOR, the subtraction XOR as well.
			// TODO: get the log once in the loop.
			// TODO: optimize for Xbox.
            BYTE a = m_transfer[j][i];
            if (a != 0)
            {
                for (int k = 0; k < m_NumberElements; k++)
                {
                    m_transfer[j][k] ^=
                        GaloisField256::Multiply(a, m_transfer[i][k]);
                    m_inverted[j][k] ^=
                        GaloisField256::Multiply(a, m_inverted[i][k]);
                }
            }
        }
    }
    return (ret);
}

// Compute the values of the redundancy buffers
void FecMatrix::ComputeRedundancy(const BYTE * inputMessage,
								  int messageLength,
								  BYTE * redundancyBuffer,
								  int packetLength)
{
    // For each redundancy buffer
    for (int i = 0; i < m_RedundancyLevel; i++)
    {
        BYTE* cy = redundancyBuffer + i*packetLength;
		BYTE * matrixRow = m_matrix[m_NumberElements + i];
		// For each byte in the packet
		for (int j=0; j<packetLength; j++)
		{
			int r = 0;
			// for each input variable
			int offset;
			int k;
			for (offset=j, k=0; 
				offset < messageLength && k < m_NumberElements; 
				offset += packetLength, k++)
			{
				r ^= GaloisField256::Multiply(
					inputMessage[offset], 
					matrixRow[k]);
			}
			cy[j] = (BYTE)r;
		}
    }
}

void FecMatrix::ComputeRedundancy2(const BYTE * inputMessage1,
			                        int messageLength1,
			                        const BYTE * inputMessage2,
			                        int messageLength2,
			                        BYTE * redundancyBuffer,
			                        int packetLength)
{
    int messageLength = messageLength1 + messageLength2;
    // For each redundancy buffer
    for (int i = 0; i < m_RedundancyLevel; i++)
    {
        BYTE* cy = redundancyBuffer + i*packetLength;
		BYTE * matrixRow = m_matrix[m_NumberElements + i];
		// For each byte in the packet
		for (int j=0; j<packetLength; j++)
		{
			int r = 0;
			// for each input variable
			int offset;
			int k;
			for (offset=j, k=0; 
				offset < messageLength && k < m_NumberElements; 
				offset += packetLength, k++)
			{
                BYTE input = (offset < messageLength1)
                                ? inputMessage1[offset]
                                : inputMessage2[offset-messageLength1];
				r ^= GaloisField256::Multiply(
					input, 
					matrixRow[k]);
			}
			cy[j] = (BYTE)r;
		}
    }
}
		void ComputeRedundancy2(
			const BYTE * inputMessage1,
			int messageLength1,
			const BYTE * inputMessage2,
			int messageLength2,
			BYTE * redundancyBuffer,
			int packetLength);


// Reconstruct the missing rows in the output matrix
bool FecMatrix::ReconstructBuffer(BYTE * outputMessage,
								  int messageLength,
								  BYTE * redundancyBuffer,
								  int packetLength,
								  bool * present)
{
	int missingRank = 0;
	int missingTotal = 0;
	bool ret = true;
	for (int i=0; i < m_NumberElements; i++)
	{
		if (!present[i])
			missingTotal++;
	}
	if (missingTotal != 0)
	{
		// Need to do something!
		if (missingRank == 0)
		{
			if (!InvertMatrixInit(present))
			{
				ret = false;
			}
			else if (!GaussJordanReduce())
			{
				ret = false;
			}
		}
		if (ret)
		{
			// Prepare the received matrix
			BYTE ** rowMap = new BYTE*[m_NumberElements];
			int * lengthMap = new int[m_NumberElements];
			int rank=0;
			for (int i=0; i<m_NumberElements; i++)
			{
				if (present[i])
				{
					int offset = i*packetLength;
					rowMap[rank] = outputMessage + offset;
					int length = (messageLength - offset);
					if (length > packetLength)
						length = packetLength;
					lengthMap[rank] = length;
					rank++;
				}
			}
			for (int i=0; i<m_RedundancyLevel && rank < m_NumberElements; i++)
			{
				if (present[m_NumberElements+ i])
				{
					int offset = i*packetLength;
					rowMap[rank] = redundancyBuffer + offset;
					lengthMap[rank] = packetLength;
					rank++;
				}
			}
			// For each missing buffer, compute missing rank,
			// then multiple inverted matrix by received buffer
			rank = m_NumberElements - missingTotal;

			for (int i=0; i < m_NumberElements; i++)
			{
				if (present[i])
					continue;
				// Now, we need to fill the missing packet.
				// Vector[rank] = Inverted[rank, *]*rowMap[*];
				// .. for each element
				// First locate the write target and length
				int offset = i*packetLength;
				int length = messageLength - offset;
				if (length > packetLength)
				{
					length = packetLength;
				}
				BYTE * target = outputMessage+offset;
				// then perform the multiplication
				for (int l=0; l < length; l++)
				{
					int r = 0;
					for (int j=0; j< m_NumberElements; j++)
					{
						if (l < lengthMap[j])
						{
							r ^= GaloisField256::Multiply(
								m_inverted[rank][j], rowMap[j][l]);
						}
					}

					target[l] = (BYTE)r;
				}
				rank++;
			}
			delete lengthMap;
			delete rowMap;
		}
	}
	return(ret);
}

