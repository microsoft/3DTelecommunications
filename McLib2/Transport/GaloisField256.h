#pragma once

namespace VideoTransmission
{
	/*
	 * GaloisField256:
	 *	Implementation of the dimension 256 Galois field in which the elements
	 *  are defined as the binary coefficients of a polynomial 
	 *		a[0] + a[1]*x + .. + a[7]*x**7.
	 *  These coefficients are represented on a single byte (unsigned char).
	 *  The addition operation is defined as the sum of the binary coeeficents,
	 *  which is implemented as simple XOR of the bytes.
	 *  The multiplication operation is defined as the polynomial multiplication
	 *  modulo the base polynomial x8 + x7 + x3 + x2 + 1, AKA Crc8CCITT polynom.
	 *  The multiplication is implemented using logarithms and exponential 
	 *  tables, which are defined as:
	 *		ExpTable[N] = 2**N modulo x8 + x7 + x3 + x2 + 1;
	 *      LogTable[ExpTable[N]] = N (for N=1..254)
	 *  In theory, we only need 255 elements in the log table. Because the 
	 *  generating polynomial is prime, Fermat's little theorem holds, and x**255 = 1 for
	 *  all values of x. However, we fill the Exponential table for all values up to
	 *  512, so we do not have to perform modulo operations when computing products.
	 *  Also, there are another 513 zeros at the end of the exponential table so we can 
	 *  avoid 0 checks in the Multiply.
	 */
	extern const int g_GaloisField256LogTable[256];
	extern const int g_GaloisField256ExpTable[1025];

	class GaloisField256
	{

	public:
		GaloisField256(void);
		~GaloisField256(void) {}

		inline static unsigned char Add (unsigned char x, unsigned char y)
		{
			return (x^y);
		}

		inline static unsigned char Subtract (unsigned char x, unsigned char y)
		{
			return (x^y);
		}

		inline static unsigned char Multiply(unsigned char x, unsigned char y)
		{
			return Exp(Log(x) + Log(y));
		}

		inline static unsigned char Divide(unsigned char x, unsigned char y)
		{
			return ((x==0 || y==0)?0:Exp(Log(x) + 255 - Log(y)));
		}

		inline static int Log(unsigned char x)
		{
			return(g_GaloisField256LogTable[x]);
		}

		inline static unsigned char Exp(int x)
		{
			return((unsigned char)g_GaloisField256ExpTable[x]);
		}
	};
}
