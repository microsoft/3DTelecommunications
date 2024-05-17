#include "stdafx.h"
#include "CustomSVD.h"
#include <math.h>    /* for 'fabs'           */

int svd_la( int m,int n,int withu,int withv,double eps,double tol,
			double a[][4], double *q, double u[][4],double v[][4], double *e)
{
	int i,j,k,l,l1,iter,retval;
	double c,f,g,h,s,x,y,z;

	retval = 0;

/* Copy 'a' to 'u' */    
	for (i=0;i<m;i++) {
		for (j=0;j<n;j++)
			u[i][j] = a[i][j];
	}
/* Householder's reduction to bidiagonal form. */
	g = x = 0.0;    
	for (i=0;i<n;i++) {
		e[i] = g;
		s = 0.0;
		l = i+1;
		for (j=i;j<m;j++)
			s += (u[j][i]*u[j][i]);
		if (s < tol)
			g = 0.0;
		else {
			f = u[i][i];
			g = (f < 0) ? sqrt(s) : -sqrt(s);
			h = f * g - s;
			u[i][i] = f - g;
			for (j=l;j<n;j++) {
				s = 0.0;
				for (k=i;k<m;k++)
					s += (u[k][i] * u[k][j]);
				f = s / h;
				for (k=i;k<m;k++)
					u[k][j] += (f * u[k][i]);
			} /* end j */
		} /* end s */
		q[i] = g;
		s = 0.0;
		for (j=l;j<n;j++)
			s += (u[i][j] * u[i][j]);
		if (s < tol)
			g = 0.0;
		else {
			f = u[i][i+1];
			g = (f < 0) ? sqrt(s) : -sqrt(s);
			h = f * g - s;
			u[i][i+1] = f - g;
			for (j=l;j<n;j++) 
				e[j] = u[i][j]/h;
			for (j=l;j<m;j++) {
				s = 0.0;
				for (k=l;k<n;k++) 
					s += (u[j][k] * u[i][k]);
				for (k=l;k<n;k++)
					u[j][k] += (s * e[k]);
			} /* end j */
		} /* end s */
		y = fabs(q[i]) + fabs(e[i]);                         
		if (y > x)
			x = y;
	} /* end i */

/* accumulation of right-hand transformations */
	if (withv) {
		for (i=n-1;i>=0;i--) {
			if (g != 0.0) {
				h = u[i][i+1] * g;
				for (j=l;j<n;j++)
					v[j][i] = u[i][j]/h;
				for (j=l;j<n;j++) {
					s = 0.0;
					for (k=l;k<n;k++) 
						s += (u[i][k] * v[k][j]);
					for (k=l;k<n;k++)
						v[k][j] += (s * v[k][i]);

				} /* end j */
			} /* end g */
			for (j=l;j<n;j++)
				v[i][j] = v[j][i] = 0.0;
			v[i][i] = 1.0;
			g = e[i];
			l = i;
		} /* end i */
 
	} /* end withv, parens added for clarity */

/* accumulation of left-hand transformations */
	if (withu) {
		for (i=n;i<m;i++) {
			for (j=n;j<m;j++)
				u[i][j] = 0.0;
			u[i][i] = 1.0;
		}
	}
	if (withu) {
		for (i=n-1;i>=0;i--) {
			l = i + 1;
			g = q[i];
			for (j=l;j<m;j++)  /* upper limit was 'n' */
				u[i][j] = 0.0;
			if (g != 0.0) {
				h = u[i][i] * g;
				for (j=l;j<m;j++) { /* upper limit was 'n' */
					s = 0.0;
					for (k=l;k<m;k++)
						s += (u[k][i] * u[k][j]);
					f = s / h;
					for (k=i;k<m;k++) 
						u[k][j] += (f * u[k][i]);
				} /* end j */
				for (j=i;j<m;j++) 
					u[j][i] /= g;
			} /* end g */
			else {
				for (j=i;j<m;j++)
					u[j][i] = 0.0;
			}
			u[i][i] += 1.0;
		} /* end i*/
	} /* end withu, parens added for clarity */

/* diagonalization of the bidiagonal form */
	eps *= x;
	for (k=n-1;k>=0;k--) {
		iter = 0;
test_f_splitting:
		for (l=k;l>=0;l--) {
			if (fabs(e[l]) <= eps) goto test_f_convergence;
			if (fabs(q[l-1]) <= eps) goto cancellation;
		} /* end l */

/* cancellation of e[l] if l > 0 */
cancellation:
		c = 0.0;
		s = 1.0;
		l1 = l - 1;
		for (i=l;i<=k;i++) {
			f = s * e[i];
			e[i] *= c;
			if (fabs(f) <= eps) goto test_f_convergence;
			g = q[i];
			h = q[i] = sqrt(f*f + g*g);
			c = g / h;
			s = -f / h;
			if (withu) {
				for (j=0;j<m;j++) {
					y = u[j][l1];
					z = u[j][i];
					u[j][l1] = y * c + z * s;
					u[j][i] = -y * s + z * c;
				} /* end j */
			} /* end withu, parens added for clarity */
		} /* end i */
test_f_convergence:
		z = q[k];
		if (l == k) goto convergence;

/* shift from bottom 2x2 minor */
		iter++;
		if (iter > 30) {
			retval = k;
			break;
		}
		x = q[l];
		y = q[k-1];
		g = e[k-1];
		h = e[k];
		f = ((y-z)*(y+z) + (g-h)*(g+h)) / (2*h*y);
		g = sqrt(f*f + 1.0);
		f = ((x-z)*(x+z) + h*(y/((f<0)?(f-g):(f+g))-h))/x;
/* next QR transformation */
		c = s = 1.0;
		for (i=l+1;i<=k;i++) {
			g = e[i];
			y = q[i];
			h = s * g;
			g *= c;
			e[i-1] = z = sqrt(f*f+h*h);
			c = f / z;
			s = h / z;
			f = x * c + g * s;
			g = -x * s + g * c;
			h = y * s;
			y *= c;
			if (withv) {
				for (j=0;j<n;j++) {
					x = v[j][i-1];
					z = v[j][i];
					v[j][i-1] = x * c + z * s;
					v[j][i] = -x * s + z * c;
				} /* end j */
			} /* end withv, parens added for clarity */
			q[i-1] = z = sqrt(f*f + h*h);
			c = f/z;
			s = h/z;
			f = c * g + s * y;
			x = -s * g + c * y;
			if (withu) {
				for (j=0;j<m;j++) {
					y = u[j][i-1];
					z = u[j][i];
					u[j][i-1] = y * c + z * s;
					u[j][i] = -y * s + z * c;
				} /* end j */
			} /* end withu, parens added for clarity */
		} /* end i */
		e[l] = 0.0;
		e[k] = f;
		q[k] = x;
		goto test_f_splitting;
convergence:
		if (z < 0.0) {
/* q[k] is made non-negative */
			q[k] = - z;
			if (withv) {
				for (j=0;j<n;j++)
					v[j][k] = -v[j][k];
			} /* end withv, parens added for clarity */
		} /* end z */
	} /* end k */
	
	return retval;
}

#define SQR(a) ((a)*(a))
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define FMAX(a,b) ((a) > (b) ? (a) : (b))
#define IMIN(a,b) ((a) > (b) ? (b) : (a))

//Computes (a2 + b2)1/2 without destructive underflow or overflow.
float pythag(float a, float b)
{
	float absa,absb;
	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) 
		return absa*sqrt(1.0+SQR(absb/absa));
	else 
		return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

//Given a matrix a[1..m][1..n], this routine computes its singular value decomposition, A =
//U¡¤W¡¤V T. Thematrix U replaces a on output. The diagonal matrix of singular values W is output
//as a vector w[1..n]. Thematrix V (not the transpose V T ) is output as v[1..n][1..n].
void svd_nr(float a[][4], int m, int n, float w[], float v[][4], float *rv1)
{
	int flag,i,its,j,jj,k,l,nm;
	float anorm,c,f,g,h,s,scale,x,y,z;

	g=scale=anorm=0.0; //Householder reduction to bidiagonal form.
	for (i=1;i<=n;i++) 
	{
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) 
		{
			for(k=i;k<=m;k++) 
				scale += fabs(a[k][i]);
			if (scale) 
			{
				for (k=i;k<=m;k++) 
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=i;k<=m;k++) 
						s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) 
						a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) 
					a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) 
		{
			for (k=l;k<=n;k++) 
				scale += fabs(a[i][k]);
			if (scale) 
			{
				for (k=l;k<=n;k++) 
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for(k=l;k<=n;k++) 
					rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) 
						a[j][k] += s*rv1[k];
				}
				for (k=l;k<=n;k++) 
					a[i][k] *= scale;
			}
		}
		anorm=FMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
	}

	for (i=n;i>=1;i--) 
	{ //Accumulation of right-hand transformations.
		if (i < n) 
		{
			if(g) 
			{
				for (j=l;j<=n;j++) //Double division to avoid possible underflow.
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) 
						v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) 
				v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}

	for (i=IMIN(m,n);i>=1;i--) 
	{ //Accumulation of left-hand transformations.
		l=i+1;
		g=w[i];
		for(j=l;j<=n;j++) 
			a[i][j]=0.0;
		if (g) 
		{
			g=1.0/g;
			for (j=l;j<=n;j++) 
			{
				for (s=0.0,k=l;k<=m;k++) 
					s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) 
					a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) 
				a[j][i] *= g;
		} 
		else 
		{
			for (j=i;j<=m;j++) 
				a[j][i]=0.0;
		}
		++a[i][i];
	}
	for (k=n;k>=1;k--) 
	{ //Diagonalization of the bidiagonal form: Loop over singular values, 
		for (its=1;its<=30;its++) 
		{				//and over allowed iterations.						
			flag=1;
			for(l=k;l>=1;l--) 
			{	//Test for splitting.
				nm=l-1; //Note that rv1[1] is always zero.
				if ((float)(fabs(rv1[l])+anorm) == anorm) 
				{
					flag=0;
					break;
				}
				if ((float)(fabs(w[nm])+anorm) == anorm) 
					break;			
			}
			if (flag) 
			{
				c=0.0; //Cancellation of rv1[l], if l > 1.
				s=1.0;
				for (i=l;i<=k;i++) 
				{
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if ((float)(fabs(f)+anorm) == anorm) break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) 
					{
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) 
			{ //Convergence.
				if (z < 0.0)
				{ //Singular value is made nonnegative.
					w[k] = -z;
					for (j=1;j<=n;j++) 
						v[j][k] = -v[j][k];
				}
				break;
			}

			//if (its == 30) 
			//	printf("no convergence in 30 svdcmp iterations");

			x=w[l]; //Shift from bottom 2-by-2 minor.
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0; //Next QR transformation:
			for (j=l;j<=nm;j++) 
			{
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) 
				{
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z; //Rotation can be arbitrary if z = 0.
				if (z) 
				{
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) 
				{
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}			
			}			
			rv1[l]=0.0;			
			rv1[k]=f;			
			w[k]=x;
		}	
	}
}