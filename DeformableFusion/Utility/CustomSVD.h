//===============================================
//			CustomSVD.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================

#ifndef __CUMSTOMSVD_H__
#define __CUMSTOMSVD_H__

/*  svd.c -- Singular value decomposition. Translated to 'C' from the
 *           original Algol code in "Handbook for Automatic Computation,
 *           vol. II, Linear Algebra", Springer-Verlag.
 *
 *  (C) 2000, C. Bond. All rights reserved.
 *
 *  This is almost an exact translation from the original, except that
 *  an iteration counter is added to prevent stalls. This corresponds
 *  to similar changes in other translations.
 *
 *  Returns an error code = 0, if no errors and 'k' if a failure to
 *  converge at the 'kth' singular value.
 * 
 */

//e--in, work space, should be with size of n
int svd_la( int m,int n,int withu,int withv,double eps,double tol,
		    double a[][4], double *q, double u[][4],double v[][4], double *e);

//Given a matrix a[1..m][1..n], this routine computes its singular value decomposition, A =
//U¡¤W¡¤V T. Thematrix U replaces a on output. The diagonal matrix of singular values W is output
//as a vector w[1..n]. Thematrix V (not the transpose V T ) is output as v[1..n][1..n].
void svd_nr(float a[][4], int m, int n, float w[], float v[][4], float *rv1);

#endif