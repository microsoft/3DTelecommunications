/******************************************************************************
 * VecOperation.inl
 * Template function implementation
 * Mingsong Dou(doums@cs.unc.edu)
 ******************************************************************************/

template<class T>
void VecOperation<T>::printVec(T *vec, int len, int direct)
{
	if( direct == 0)
	{
		for(int i=0; i<len; i++)
		{
			printf("%f	", vec[i]);
		}
		printf("\n");
	}
	else
	{
		for(int i=0; i<len; i++)
		{
			printf("%f\n", vec[i]);
		}
	}
}



template<class T>
T VecOperation<T>::GetMax(T *vec, int len)
{
	T max_val = vec[0];
	for(int i=1; i<len; i++)
	{
		if(vec[i] > max_val)
			max_val = vec[i];
	}
	return max_val;
}

template<class T>
T VecOperation<T>::GetMax(T *vec, int len, int *idx_max)
{
	T max_val = vec[0];
	*idx_max = 0;
	for(int i=1; i<len; i++)
	{
		if(vec[i] > max_val)
		{
			max_val = vec[i];
			*idx_max = i;
		}
	}
	return max_val;
}

template<class T>
T VecOperation<T>::GetMin(T *vec, int len, int *idx_min)
{
	T min_val = vec[0];
	*idx_min = 0;
	for(int i=1; i<len; i++)
	{
		if(vec[i] < min_val)
		{
			min_val = vec[i];
			*idx_min = i;
		}
	}
	return min_val;
}

template<class T>
T VecOperation<T>::GetMean(T *vec, int len)
{
	T ret = 0;
	for(int i=0; i<len; i++)
	{
		ret += vec[i];
	}

	return ret/len;
}

template<class T>
T VecOperation<T>::GetStd(T *vec, int len, int mod)
{
	T m = GetMean(vec, len);

	return GetStd(vec, m, len, mod);
}

template<class T>
T VecOperation<T>::GetMedian(T *vec, int len)
{
	vector<T> wrapper(vec, vec+len); // copy data from vec

	nth_element(wrapper.begin(), wrapper.begin()+len/2, wrapper.end());

	return wrapper[len/2];
}

//order=0: sort in increasing order, 1: decreasing order
template<class T>
void VecOperation<T>::Sort(T *vec, int len, int order)
{
	vector<T> wrapper(vec, vec+len); // copy data from vec
	sort(wrapper.begin(), wrapper.end());

	if( order == 0)
	{
		for(int i=0; i<len; i++)
			vec[i] = wrapper[i];
	}		
	else
	{
		for(int i=0; i<len; i++)
			vec[i] = wrapper[len-i-1];
	}
}

template<class T>
T VecOperation<T>::GetMoment(T *vec, int len)
{
	T ret=0;
	for(int i=0; i<len; i++)
	{
		ret += vec[i]*vec[i];
	}
	return sqrt(ret/len);
}

template<class T>
T VecOperation<T>::GetStd(T* vec, T mean, int len, int mod)
{
	T ret = 0;
	for(int i=0; i<len; i++)
	{
		ret += (vec[i] - mean)*(vec[i] - mean);
	}
	if( mod == 0 )
		ret = std::sqrt(ret/len);
	else
		ret = std::sqrt(ret/max(len-1, 1));

	return ret;
}

template<class T>
void VecOperation<T>::Normalize(T *vec, int len)
{
	T m = GetMean(vec, len);
	T std = GetStd(vec, m, len, 0);

	Normalize(vec, m, std, len);
}

template<class T>
void VecOperation<T>::Normalize(T *vec, T mean, T std, int len)
{
	for(int i=0; i<len; i++)
	{
		vec[i] = (vec[i]-mean)/(std+1.0E-20);
	}
}

template<class T>
T VecOperation<T>::InnerProd(T *vec1, T *vec2, int len)
{
	T ret = 0;
	for(int i=0; i<len; i++)
	{
		ret += vec1[i]*vec2[i];
	}
	return ret;
}


template<class T>
T VecOperation<T>::NormalizedCorrelation(T *vec1, T *vec2, int len)
{
	Normalize(vec1, len);
	Normalize(vec2, len);

	return InnerProd(vec1, vec2, len)/len;
}

template<class T>
T VecOperation<T>::Correlation(T *vec1, T *vec2, int len)
{
	T mo1 = GetMoment(vec1, len);
	T mo2 = GetMoment(vec2, len);

	T ret = InnerProd(vec1, vec2, len)/len;

	return ret/(mo1*mo2);
}

template<class T>
T VecOperation<T>::CorrelationImagePatch(T *vec1, T *vec2, int len)
{
	T std1 = GetStd(vec1, 125.0, len, 0);
	T std2 = GetStd(vec2, 125.0, len, 0);

	Normalize(vec1, 125.0, std1, len);
	Normalize(vec2, 125.0, std2, len);

	T ret = InnerProd(vec1, vec2, len)/len;

	return ret;
}

template<class T>
T VecOperation<T>::Distance(T const* vec1, T const*vec2, int len)
{
	T ret = 0.0;
	for(int i=0; i<len; i++)
	{
		ret += (vec1[i] - vec2[i])*(vec1[i] - vec2[i]);
	}
	return sqrt(ret);
}

template<class T>
void VecOperation<T>::SaveVectorAscii(const char* filename, T* vec, int len)
{
	ofstream matFile(filename);
	if(matFile.fail())
	{
		cout<<"error write matrix file"<<endl;
		return;
	}

	for(int i=0; i < len; i++)
	{
		matFile.precision(15);
		matFile<<vec[i];
		matFile<<endl;
	}
	matFile.close();
}

template<class T>
inline T VecOperation<T>::DotProd(T const*vec1, T const*vec2, int len)
{
	T ret = 0;
	for(int i=0; i<len; i++)
	{
		ret += vec1[i]*vec2[i];
	}

	return ret;
}

template<class T>
T VecOperation<T>::Norm(T *vec, int len)
{
	return sqrt(DotProd(vec, vec, len));
}

template<class T>
inline void VecOperation<T>::CrossProdVec3(T *a, T *b, T *c)
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
}

template<class T>
void VecOperation<T>::Unitalization(T *vec, int len)
{
	T r = DotProd(vec, vec, len);
	if( r != 0 )
	{	
		r = std::sqrt(r);
		for(int i=0; i<len; i++)
			vec[i] /= r;
	}
}

template<class T>
inline void VecOperation<T>::VecSub(T const*vec1, T const*vec2, T *res, int len)
{
	for(int i=0; i<len; i++)
		res[i] = vec1[i] - vec2[i];
}

template<class T>
inline void VecOperation<T>::VecAdd(T const*vec1, T const*vec2, T *res, int len)
{
	for(int i=0; i<len; i++)
		res[i] = vec1[i] + vec2[i];
}

template<class T>
void VecOperation<T>::VecAddWeighted(T const*vec1, T alpha, T const*vec2, T beta, T gamma, T *res, int len)
{
	for(int i=0; i<len; i++)
		res[i] = vec1[i]*alpha + vec2[i]*beta + gamma;
}

template<class T>
inline void VecOperation<T>::VecCopy(T const*src, T *dst, int len)
{
	memcpy(dst, src, sizeof(T)*len);
}

template<class T>
inline void VecOperation<T>::VecScale(T *vec, T scalar, int len)
{
	for(int i=0; i<len; i++)
		vec[i] *= scalar;
}

template<class T>
T VecOperation<T>::AngleBtwVecs(T const* vec1, T const* vec2, int len)
{
	T *vec1_ = new T[len]; assert(vec1_ != 0 );
	T *vec2_ = new T[len]; assert(vec2_ != 0 );
	VecCopy(vec1, vec1_, len);
	VecCopy(vec2, vec2_, len);
	Unitalization(vec1_, len);
	Unitalization(vec2_, len);

	double a = MAX(-1.0, MIN(1.0, InnerProd(vec1_, vec2_, len)));
	delete [] vec1_;
	delete [] vec2_;

	return acos(a);
}

template<class T>
void VecOperation<T>::VecSetRandom(T *vec, int len)
{
	for(int i=0; i<len; i++)
		vec[i] = RANDOM;
}

template<class T>
vector<int> VecOperation<T>::FindPeaks(T *vec, int len, int max_suppr_radius, double thres)
{
	vector<int> maxima;
	int max_suppr_size = 2*max_suppr_radius+1;

//============= step 0: smooth the vector===========
	T *vec_bak = new T[len];
	VecCopy(vec, vec_bak, len);
	for(int i=0; i<len; i++)
	{
		T sum = 0;
		int count = 0;
		for (int c = -3; c < 3; c++)
		{
			if (i + c < 0 || i + c >= len)
				continue;

			sum += vec_bak[i + c];
			count++;
		}
		vec[i] = sum/count;		
	}
	delete [] vec_bak;

//============= step 1: find the vec_max =============================
	T *vec_max = new T[len];
	T max_val = GetMax(vec, MIN(len, max_suppr_size)); // the max value of first max_suppr_radius points
	for(int i=0; i<=MIN(len-1, max_suppr_radius); i++)
	{
		vec_max[i] = max_val;
	}

	int idx_drop = 0;
	int idx_new = max_suppr_size;
	for(int i=max_suppr_radius+1; i<len-max_suppr_radius; i++)
	{
		if( vec[idx_drop] < max_val)
		{
			if( vec[idx_new] > max_val )
				max_val = vec[idx_new];
		}
		else
		{
			//recompute the max_val
			max_val = GetMax(&vec[idx_drop+1], MIN(max_suppr_size, len - idx_drop - 1));
		}
		vec_max[i] = max_val;
		idx_drop ++;
		idx_new ++;
	}

	int s_idx = MAX(0, len - max_suppr_size);
	max_val = GetMax(&vec[s_idx], len - s_idx);
	for(int i=s_idx; i<len; i++)
	{
		vec_max[i] = max_val;
	}

//================step 2: find the maxima =================
	for(int i=0; i<len; i++)
	{
		if( vec[i] == vec_max[i] &&	vec[i] > thres )
			maxima.push_back(i);
	}
	delete [] vec_max;

	return maxima;
}

//exact suppression
template<class T>
vector<int> VecOperation<T>::FindPeaks2(T *vec, int len, int max_suppr_radius, double thres)
{
	vector<int> maxima;
	int max_suppr_size = 2 * max_suppr_radius + 1;

	//============= step 1: find the vec_max =============================
	T *vec_max = new T[len];
	for (int i = 0; i < len; i++)
	{
		int s_idx = MAX(0, i - max_suppr_radius);
		int e_idx = MIN(len-1, i + max_suppr_radius);
		T max = GetMax(&(vec[s_idx]), e_idx - s_idx + 1);
		vec_max[i] = max;
	}

	//================step 2: find the maxima =================
	for (int i = 0; i<len; i++)
	{
		if (vec[i] == vec_max[i] && vec[i] > thres)
			maxima.push_back(i);
	}
	delete [] vec_max;

	return maxima;
}
