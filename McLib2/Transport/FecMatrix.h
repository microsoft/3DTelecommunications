#pragma once
namespace VideoTransmission 
{
    /// <summary>
    /// We call it "packet FEC", but the procedure we use is actually "dealing with erasures".
    /// Erasure correction can be described as a linear operation. We treat each packet as a 
    /// "symbol", and we represent a set of N packets to be transmitted as a vector of N symbols,
    /// X[0]...X[N-1]. We will actually transmit a set of N+P symbols, Y[0]...Y[N+P-1], derived from
    /// X by the multiplication product: Y = M*X. M is a matrix of size N*(N+P).
    /// 
    /// Upon reception, if at least N symbols out of N+P have been received, we can write the received
    /// set as a vector Y' of N symbols, e.g. by erasing the missing symbols. We can also derive a matrix 
    /// M' of size N*N by erasing the rows of M corresponding to the missing symbols. We obtain the
    /// equation: Y' = M'*X. If M' is invertible, we can write: X = (1/M')*Y, allowing us to retrieve
    /// the original values.
    /// 
    /// The multiplication product operates on a specific number space, a finite field of 256 elements. 
    /// Each element represents a polynom of degree lower than 8 on GF(2). The addition operation is
    /// defined as XOR (as in GF(2)). The multiply operation is defined as polynomial multiply modulo
    /// a base polynom, an irreducible polynom of degree 8. 
    /// 
    /// Each of our "symbols" (X or Y) is in fact a packet of L bytes, each of which is considered an 
    /// element in the finite field. The matrix operations are performed on each of these bytes. For 
    /// example, if we note as Xk the vector of N symbols corresponding to the k-position BYTE in each of
    /// the N messages X[0..N-1], and Yk the similar message for Y[0..N+P-1], then we can write: Yk = M*Xk,
    /// and we can similarly derive Xk = (1/M')*Yk'.
    /// 
    /// The hard part of the program are the proper choice of the matrix M, and the efficient inversion
    /// of the matrix M'. 
    /// 
    /// Traditionally, the N first rows of M correspond to the identity matrix, so if the N packets are 
    /// received correctly they can be delivered without any additional computation. The next row is 
    /// traditionally set to unity, so the message Y[N] is effectively the XOR of all previous messages. 
    /// The following rows must be chosen so the matrix M' will be invertible.
    /// 
    /// We generate the set of rows using an algorithm proposed by MSR – Cheng Huang (chengh), 
    /// Sanjeev Mehrotra (sanjeevm), and Jin Li (jinl). The matrix is a variation of the Cauchy matrix. 
    /// In a Cauchy matrix, rows have the form 1/(1+N+K), 1/(2+N+K), ... 1/(N+N+K). The MSR scheme adapts
    /// the Cauchy form to ensure that row N is always set to unity (0xFF), setting rows N+1 to N+P-1
    /// to values of the form (1+N+1)/(N+(N+k)), (2+N+1)/(N+(N+k)), ..., (N+N+1)/(N+(N+k))
    /// 
    /// The advantage of the MSR scheme is to enable extensions to large number of redondancy levels, using
    /// pretty much the same formula throughout. The disadvantage is that the matrix coefficients depend on
    /// the number N, the number of input symbols. This means the coefficients have to be recomputed for
    /// each different image size.
    /// 
    /// The matrix M has a particular form: essentially, an NxN identity matrix complemented by a set of
    /// defined vectors. M' similarly starts by N-P "almost identity" rows, followed by the P redondancy 
    /// rows. It can be trivially rearranged as an identity matrix followed by P redondancy rows by 
    /// reordering the elements so the present symbols come first, and the P redondancy lines correspond
    /// to the missing symbols. After that, we can proceed with Gauss Jordan elimination:
    /// 1) use a set of row additions transforms to reduce the coefficients of rank less than N-P in the
    ///    last P rows;
    /// 2) for all remaining rows, use a row multiplication transform to reduce the diagonal term to 1,
    ///    and then use a set of row addition transforms to reduce the coefficents of that rank to zero in
    ///    the redondancy rows.
    /// Once we have the inverse matrix, we can calculate the missing symbols.
    /// </summary>
	class FecMatrix
	{
	public:
		FecMatrix(int numberElements, int redundancyLevel);
		~FecMatrix(void);

		const int NumberElements() { return((const int) m_NumberElements);};
		const int RedundancyLevel() { return((const int) m_RedundancyLevel);};

		const BYTE** Matrix() {return ((const BYTE **)m_matrix);};
		const BYTE** Transfer() { return ((const BYTE **)m_transfer);};
		const BYTE** Inverted() { return ((const BYTE **)m_inverted);};

		// Compute the values of the redundancy buffers
		void ComputeRedundancy(
			const BYTE * inputMessage,
			int messageLength,
			BYTE * redundancyBuffer,
			int packetLength);
		void ComputeRedundancy2(
			const BYTE * inputMessage1,
			int messageLength1,
			const BYTE * inputMessage2,
			int messageLength2,
			BYTE * redundancyBuffer,
			int packetLength);

		// Reconstruct the missing rows in the output matrix
		bool FecMatrix::ReconstructBuffer(
			BYTE * outputMessage,
			int messageLength,
			BYTE * redundancyBuffer,
			int packetLength,
			bool * present);
		
        /// <summary>
        /// Initialize the m_transfer and m_inverted matrices, as a function
        /// of the list of elements present after transfer.
        /// </summary>
        /// <param name="present"></param>
        bool InvertMatrixInit(bool present[]);

        /// <summary>
        /// Using the Gauss Jordan algorithm, compute the invert matrix.
        /// </summary>
        bool GaussJordanReduce();

    private:
        BYTE** m_matrix;
        int m_NumberElements;
        int m_RedundancyLevel;
        BYTE** m_transfer;
        BYTE** m_inverted;
	};
}
