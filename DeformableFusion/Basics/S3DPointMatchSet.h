#ifndef __S3DPOINTMATCHSET_H__
#define __S3DPOINTMATCHSET_H__
#include <vector>
#include "UtilVnlMatrix.h"
#include "CSurface.h"
#include "utility.h"

class S3DPointMatchSet
{
public:
	S3DPointMatchSet(int frmIdx1 = -1, int frmIdx2 = -1)
		: frm_idx1(frmIdx1),
		frm_idx2(frmIdx2)
	{
		;
	}
public:
	int frm_idx1;
	int frm_idx2;
	std::vector<vnl_vector_fixed<double, 3>> points_1;
	std::vector<vnl_vector_fixed<double, 3>> points_2;

	template<class T1, class T2>
	void push_back(vnl_vector_fixed<T1, 3> const&p1, vnl_vector_fixed<T2, 3> const&p2)
	{
		points_1.push_back(vnl_vector_fixed<double, 3>(p1[0], p1[1], p1[2]));
		points_2.push_back(vnl_vector_fixed<double, 3>(p2[0], p2[1], p2[2]));
	}
	int size() const
	{
		return this->points_1.size();
	}
	virtual void clear()
	{
		points_1.clear();
		points_2.clear();
	}

	int filter_outlier();
};
bool save_3D_match_set(S3DPointMatchSet const&match_set_3d,
	const char* filename);
bool load_3D_match_set(S3DPointMatchSet &match_set_3d,
	const char* filename);
bool save_3D_match_set_BIN(S3DPointMatchSet const&match_set_3d,
	const char* filename);
bool load_3D_match_set_BIN(S3DPointMatchSet &match_set_3d,
	const char* filename);

class S3DPointMatchSetIndexed : public S3DPointMatchSet
{
public:
	vector<int> indices_1;
	vector<int> indices_2;

	void clear()
	{
		S3DPointMatchSet::clear();
		indices_1.clear();
		indices_2.clear();
		frm_idx1 = -1;
		frm_idx2 = -1;
	}

	bool fill_in_index_1(CSurface<float> const& surface);
	bool fill_in_index_2(CSurface<float> const& surface);

	bool save_to_txt(char const* filename) const;
	bool load_from_txt(char const* filename);
	//bool save_to_bin(char const* filename) const;
	//bool load_from_bin(char const* filename);
};


#endif