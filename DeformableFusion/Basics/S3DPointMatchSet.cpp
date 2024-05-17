#include "stdafx.h"
#include "S3DPointMatchSet.h"

int S3DPointMatchSet::filter_outlier()
{
	vector< double > disps_vec_all;
	for (int i = 0; i<this->size(); i++)
	{
		double disp = dist_3d(points_1[i], points_2[i]);
		disps_vec_all.push_back(disp);
	}

	int outlier_num = 0;
	int loops = this->size()*0.3;
	for (int k = 0; k<loops; k++)
	{
		//find the pair with the most dispacement
		int max_idx = 0;
		double max_val = disps_vec_all[0];
		for (int c = 1; c<disps_vec_all.size(); c++)
		{
			if (disps_vec_all[c] > max_val)
			{
				max_idx = c;
				max_val = disps_vec_all[c];
			}
		}
		int i = max_idx;
		vnl_vector_fixed<double, 3> &p = this->points_1[i];

		//find its neighboring points
		vector< DoubleIndexPair > dist_vecs;
		dist_vecs.clear();
		for (int j = 0; j<this->points_1.size(); j++)
		{
			if (j == i)
				continue;
			double d = dist_3d(p, this->points_1[j]);
			dist_vecs.push_back(DoubleIndexPair(d, j));
		}
		sort(dist_vecs.begin(), dist_vecs.end(), less_comparator_DoubleIndexPair);

		//find the average displacement of its neighboring points
		int num = MIN(50, dist_vecs.size()*0.3);
		vnl_vector<double> disp_vec(num);
		for (int idx = 0; idx<num; idx++)
		{
			int j = dist_vecs[idx].second;
			double disp = dist_3d(points_1[j], points_2[j]);
			disp_vec[idx] = disp;
		}
		double mean_val = VecOperation<double>::GetMean(disp_vec.data_block(), num);
		double std_val = VecOperation<double>::GetStd(disp_vec.data_block(), mean_val, num);
		double tolerance = MAX(std_val*3.0, 2.0);

		double cur_disp = dist_3d(points_1[i], points_2[i]);
		if (cur_disp > mean_val + tolerance)
		{
			//printf("Warning: delete one matched pair <idx=%d>\n", i);
			points_1.erase(points_1.begin() + i);
			points_2.erase(points_2.begin() + i);
			disps_vec_all.erase(disps_vec_all.begin() + i);
			outlier_num++;
		}
		//delete [] disp_vec;
	}
	printf("Info: %d matched pair(s) are deleted!\n", outlier_num);
	return outlier_num;
}

bool S3DPointMatchSetIndexed::fill_in_index_1(CSurface<float> const& surface)
{
	vector<vnl_vector_fixed<double, 3>> qs;
	closest_point_on_surface(surface, this->points_1, qs, this->indices_1, DistanceMode_Point2Point);
	this->points_1 = qs;
	return true;
}

bool S3DPointMatchSetIndexed::fill_in_index_2(CSurface<float> const& surface)
{
	vector<vnl_vector_fixed<double, 3>> qs;
	closest_point_on_surface(surface, this->points_2, qs, this->indices_2, DistanceMode_Point2Point);
	this->points_2 = qs;
	return true;
}

bool S3DPointMatchSetIndexed::save_to_txt(char const* filename) const
{
	FILE *file = fopen(filename, "w");
	if (file == NULL)
	{
		printf("Error<S3DPointMatchSetIndexed::save_to_txt>: Cannot Open file %s!\n", filename);
		return false;
	}

	fprintf(file, "<%d, %d>: %zd pairs\n", this->frm_idx1, this->frm_idx2, this->points_1.size());

	for (unsigned int i = 0; i<this->size(); i++)
	{
		fprintf(file, "%.15f %.15f %.15f %.15f %.15f %.15f\n", points_1[i][0], points_1[i][1], points_1[i][2],
			points_2[i][0], points_2[i][1], points_2[i][2]);
	}
	fprintf(file, "Indices: <%zd, %zd>\n", this->indices_1.size(), this->indices_2.size());
	for (int i = 0; i < MAX(this->indices_1.size(), this->indices_2.size()); i++)
	{
		if (i < this->indices_1.size())
			fprintf(file, "%09d ", this->indices_1[i]);
		else
			fprintf(file, "          ");

		if (i < this->indices_2.size())
			fprintf(file, "%09d\n", this->indices_2[i]);
		else
			fprintf(file, "\n");
	}

	fclose(file);
	return true;
}

bool S3DPointMatchSetIndexed::load_from_txt(char const* filename)
{
	FILE *file = fopen(filename, "r");
	if (file == NULL)
	{
		LOGGER()->error("S3DPointMatchSetIndexed::load_from_txt", "Cannot Open file %s!", filename);
		return false;
	}

	this->clear();

	int point_num = 0;
	fscanf(file, "<%d, %d>: %d pairs\n", &frm_idx1, &frm_idx2, &point_num);
	this->points_1.resize(point_num);
	this->points_2.resize(point_num);

	for (unsigned int i = 0; i<point_num; i++)
	{
		fscanf(file, "%lf %lf %lf %lf %lf %lf\n", &(points_1[i][0]), &(points_1[i][1]), &(points_1[i][2]),
			&(points_2[i][0]), &(points_2[i][1]), &(points_2[i][2]));
	}

	if (!feof(file))
	{
		int idx1_size = 0;
		int idx2_size = 0;
		fscanf(file, "Indices: <%d, %d>\n", &idx1_size, &idx2_size);
		this->indices_1.resize(idx1_size);
		this->indices_2.resize(idx2_size);
		for (int i = 0; i < MAX(idx1_size, idx2_size); i++)
		{
			if (i < idx1_size)
				fscanf(file, "%d ", &(indices_1[i]));
			else
				fscanf(file, "          ");

			if (i < idx2_size)
				fscanf(file, "%09d\n", &(indices_2[i]));
			else
				fscanf(file, "\n");
		}
	}

	fclose(file);
	return true;
}

bool save_3D_match_set(S3DPointMatchSet const&match_set_3d,
	const char* filename)
{
	FILE *file = fopen(filename, "w");
	if (file == NULL)
	{
		printf("Cannot Open file %s for writing S3DPointMatchSet!\n", filename);
		return false;
	}

	fprintf(file, "<%d, %d>: %zd pairs\n", match_set_3d.frm_idx1, match_set_3d.frm_idx2, match_set_3d.points_1.size());

	for (unsigned int i = 0; i<match_set_3d.size(); i++)
	{
		fprintf(file, "%.15f %.15f %.15f %.15f %.15f %.15f\n", match_set_3d.points_1[i][0], match_set_3d.points_1[i][1], match_set_3d.points_1[i][2],
			match_set_3d.points_2[i][0], match_set_3d.points_2[i][1], match_set_3d.points_2[i][2]);
	}

	fclose(file);
	return true;
}

bool load_3D_match_set(S3DPointMatchSet &match_set_3d,
	const char* filename)
{
	FILE *file = fopen(filename, "r");
	if (file == NULL)
	{
		printf("Cannot Open file %s for reading S3DPointMatchSet file!\n", filename);
		return false;
	}

	//int frm_idx1, frm_idx2;
	int pairs_num = 0;
	fscanf(file, "<%d, %d>: %d pairs\n", &(match_set_3d.frm_idx1), &(match_set_3d.frm_idx2), &pairs_num);
	match_set_3d.points_1.clear();
	match_set_3d.points_2.clear();

	for (unsigned int i = 0; i<pairs_num; i++)
	{
		double x1, y1, z1, x2, y2, z2;
		fscanf(file, "%lf %lf %lf %lf %lf %lf\n", &x1, &y1, &z1, &x2, &y2, &z2);
		match_set_3d.points_1.push_back(vnl_vector_fixed<double, 3>(x1, y1, z1));
		match_set_3d.points_2.push_back(vnl_vector_fixed<double, 3>(x2, y2, z2));
	}

	fclose(file);
	return true;
}

bool save_3D_match_set_BIN(S3DPointMatchSet const& match_set_3d,
	const char* filename)
{
	FILE *file = fopen(filename, "wb");
	if (file == NULL)
	{
		printf("Cannot Open file %s for writing S3DPointMatchSet!\n", filename);
		return false;
	}

	fwrite("#3DMatchSet#", 1, 12, file);
	fwrite(&match_set_3d.frm_idx1, sizeof(int), 1, file);
	fwrite(&match_set_3d.frm_idx2, sizeof(int), 1, file);

	int pairs_num = match_set_3d.size();
	fwrite(&pairs_num, sizeof(int), 1, file);

	for (int i = 0; i<pairs_num; i++)
	{
		fwrite(match_set_3d.points_1[i].data_block(), sizeof(double), 3, file);
		fwrite(match_set_3d.points_2[i].data_block(), sizeof(double), 3, file);
	}

	fclose(file);
	return true;
}


bool load_3D_match_set_BIN(S3DPointMatchSet & match_set_3d,
	const char* filename)
{
	FILE *file = fopen(filename, "rb");
	if (file == NULL)
	{
		printf("Cannot Open file %s for writing S3DPointMatchSet!\n", filename);
		return false;
	}

	char buf[13];
	memset(buf, 0, 13);
	fread(buf, 1, 12, file);
	if (strcmp(buf, "#3DMatchSet#") != 0)
	{
		printf("Error<load_3D_match_set_BIN>: file format doesnot match!\n");
		return false;
	}

	fread(&match_set_3d.frm_idx1, sizeof(int), 1, file);
	fread(&match_set_3d.frm_idx2, sizeof(int), 1, file);

	int pairs_num = match_set_3d.size();
	fread(&pairs_num, sizeof(int), 1, file);

	match_set_3d.points_1.resize(pairs_num);
	match_set_3d.points_2.resize(pairs_num);
	for (int i = 0; i<pairs_num; i++)
	{
		fread(match_set_3d.points_1[i].data_block(), sizeof(double), 3, file);
		fread(match_set_3d.points_2[i].data_block(), sizeof(double), 3, file);
	}

	fclose(file);
	return true;
}