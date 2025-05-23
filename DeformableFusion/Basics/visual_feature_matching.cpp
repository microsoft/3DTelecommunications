// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "visual_feature_matching.h"

// implementation of feature extraction 
// copied form bundler implmentation at <vpgl/algo/vpgl_bundler_tracks_impl.cxx>
SImageVisualFeatureSet_sptr visual_feature_extraction(vil_image_resource_sptr &img, double keypoint_curve_ratio)
{
	vector<bapl_keypoint_sptr> keypoints;

	bapl_keypoint_extractor(img, keypoints, keypoint_curve_ratio, false);

	SImageVisualFeatureSet_sptr feature_set(new SImageVisualFeatureSet);
	//feature_set->source_image.source = img;

	int img_height = img->nj();
	int img_width = img->ni();

	//vnl_matrix<bool> flag_mat(img_height, img_width);
	//flag_mat.fill(false);
	vil_image_view<vxl_byte> img_view = img->get_view();

	for(unsigned int i=0; i<keypoints.size(); i++)
	{
        // Cast the key point to a bapl_lowe_keypoint. Normal keypoints
        // don't have a way of getting the location of the feature, and
        // what comes out of the extractors are all lowe kps anyway.
		bapl_lowe_keypoint_sptr lkp;
		lkp.vertical_cast(keypoints[i]);

		//create out feature
		int row = int(lkp->location_j() + 0.5);
		int col = int(lkp->location_i() + 0.5);

		//throw away keypoints at the image boundary
		if( row <= 10 || row >= img_height - 10 ||
			col <= 10 || col >= img_width - 10 )
			continue;

		//if the pixel is around black edges, throw it away
		bool bThrowPt = false;
		for(int m = row-5; m<=row+5; m++)
		{
			for(int n=col-5; n<=row+5; n++)
			{
				if( img_view(n, m) == 0 )
					bThrowPt = true;
			}
		}
		if(bThrowPt)
			continue;

		////throw away duplicate keypoints
		//if( flag_mat[row][col] )
		//	continue;
		//flag_mat[row][col] = true;

		double scale = lkp->scale();
		double orientation = lkp->orientation();
	
		SImageVisualFeature_sptr f( new SImageVisualFeature(row, col, scale, orientation) );
		f->descriptor = lkp->descriptor().as_vector();
		//f->source_image = feature_set->source_image;
		f->image = feature_set;

		feature_set->features.push_back(f);
	}

	return feature_set;
}

SImageVisualFeatureSet_sptr visual_feature_extraction( const char* filename, 
													   double keypoint_curve_ratio)
{
	vil_image_view<vxl_byte> img_rgb = vil_load(filename);
	vil_image_view<vxl_byte> img_gray;
	vil_convert_planes_to_grey<vxl_byte, vxl_byte>(img_rgb, img_gray);
	vil_save(img_gray, "./gray_img.bmp");
	vil_image_resource_sptr img_grey_src = vil_load_image_resource("./gray_img.bmp");
	return visual_feature_extraction(img_grey_src, keypoint_curve_ratio);
}

SImageVisualFeatureSet_sptr visual_feature_extraction( cv::Mat const& img, 
													   double keypoint_curve_ratio,
													   bool bHistEq)
{
	if( img.empty() )
		return NULL;
	cv::Mat gray_in = cv::Mat(img.rows, img.cols, CV_8UC1);
	cv::cvtColor(img, gray_in, cv::COLOR_RGB2GRAY);
	cv::Mat gray_out= gray_in.clone();
	if( bHistEq)
		cv::equalizeHist(gray_in, gray_out);

	cv::imwrite("./gray_img.bmp", gray_out);
	gray_in.release();
	gray_out.release();
	vil_image_resource_sptr img_grey_src = vil_load_image_resource("./gray_img.bmp");
	return visual_feature_extraction(img_grey_src, keypoint_curve_ratio);
}

SImageVisualFeatureSet_sptr visual_feature_extraction( vil_image_view<vxl_byte> &img, 
													   double keypoint_curve_ratio)
{
	if( img.nplanes() == 3)
	{
		vil_image_view<vxl_byte> img_gray;
		vil_convert_planes_to_grey<vxl_byte, vxl_byte>(img, img_gray);
		vil_save(img_gray, "./gray_img.bmp");
	}
	else if( img.nplanes() == 1)
	{
		vil_save(img, "./gray_img.bmp");
	}

	vil_image_resource_sptr img_grey_src = vil_load_image_resource("./gray_img.bmp");
	return visual_feature_extraction(img_grey_src, keypoint_curve_ratio);
}

void undistort_visual_feature_points( SImageVisualFeatureSet_sptr &feature_set,
									  vnl_matrix_fixed<double, 3, 3> &K,
									  vnl_vector<double> &distort_coef)
{
	for(unsigned int i=0; i<feature_set->features.size(); i++)
	{
		double x = feature_set->features[i]->point.y();
		double y = feature_set->features[i]->point.x();
		double x_out, y_out;
		undistort_a_point(x, y, x_out, y_out, K, distort_coef);

		feature_set->features[i]->point.set(x_out, y_out);
	}
}

void distort_visual_feature_points( SImageVisualFeatureSet_sptr &feature_set,
									vnl_matrix_fixed<double, 3, 3> &K,
									vnl_vector<double> &distort_coef)
{
	for(unsigned int i=0; i<feature_set->features.size(); i++)
	{
		double x = feature_set->features[i]->point.y();
		double y = feature_set->features[i]->point.x();
		double x_out, y_out;
		distort_a_point(x, y, x_out, y_out, K, distort_coef);

		feature_set->features[i]->point.set(x_out, y_out);
	}
}

bool visual_feature_matching_and_refining( SImageVisualFeatureSet_sptr &feature_set_1, 
										   SImageVisualFeatureSet_sptr &feature_set_2,
										   SImageVisualFeatureMatchSet &match_set,
										   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
										   int &num_before_refinement,
										   double min_dist_ratio_init_matching,
										   int min_inliers,
										   double outlier_threshold,
										   double probability_good_f_mat,
										   double max_outlier_frac,
										   bool recompute_after_ransac
										   )
{
	visual_feature_matching(feature_set_1, feature_set_2, match_set, min_dist_ratio_init_matching);
	printf("Initial Matched %d pairs\n", match_set.num_features());

	num_before_refinement = match_set.num_features();
	bool bMatchSucceed = estimate_fundamental_matrix(match_set, fundamental_mat, min_inliers, 
														   outlier_threshold, probability_good_f_mat, 
														   max_outlier_frac, recompute_after_ransac);
	double out_good_match_ratio = match_set.num_features()/double(num_before_refinement);

	if( out_good_match_ratio < 0.25 ||
		!bMatchSucceed )
		return false;
	else
		return true;

}

bool visual_feature_matching_and_refining2( SImageVisualFeatureSet_sptr &feature_set_1, 
										   SImageVisualFeatureSet_sptr &feature_set_2,
										   PairwiseMatchIdxSet &match_set_idx,
										   SImageVisualFeatureMatchSet &match_set_pts,
										   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
										   int &num_before_refinement,
										   double min_dist_ratio_init_matching,
										   int min_inliers,
										   double outlier_threshold,
										   double probability_good_f_mat,
										   double max_outlier_frac,
	 									   bool recompute_after_ransac,
										   int frmIdx_1,
										   int frmIdx_2
										   )
{
	if( visual_feature_matching_and_refining( feature_set_1, feature_set_2, match_set_pts, fundamental_mat,
										     num_before_refinement, 
											 min_dist_ratio_init_matching,
											 min_inliers,
											 outlier_threshold,
											 probability_good_f_mat,
											 max_outlier_frac,
											 recompute_after_ransac) )
	{
		match_set_idx.frm_idx1 = frmIdx_1;
		match_set_idx.frm_idx2 = frmIdx_2;
		match_set_idx.match.clear();
		for(int i=0; i<match_set_pts.num_features(); i++)
		{
			int idx1 = -1;
			feature_set_index(feature_set_1, match_set_pts.matches[i].first->point.y(), match_set_pts.matches[i].first->point.x(), &idx1);
			int idx2 = -1;
			feature_set_index(feature_set_2, match_set_pts.matches[i].second->point.y(), match_set_pts.matches[i].second->point.x(), &idx2);
			match_set_idx.match.push_back(IntPair(idx1, idx2));
		}
		return true;
	}
	return false;
}

bool visual_feature_matching( SImageVisualFeatureSet_sptr &feature_set_1, 
							  SImageVisualFeatureSet_sptr &feature_set_2,
							  SImageVisualFeatureMatchSet &match_set,
							  double min_dist_ratio)
{
	bundler_settings_match_ann setting;
	setting.min_dists_ratio = min_dist_ratio;
	bundler_tracks_impl_match_ann *matchAlgo_ann = new bundler_tracks_impl_match_ann(setting);
	
	bundler_inters_image_pair to_match;
	to_match.f1 = feature_set_1;
	to_match.f2 = feature_set_2;

	match_set.clear();
	(*matchAlgo_ann)(to_match, match_set);

	delete matchAlgo_ann;

	// get the size of the image working on
	int img_height = 480;
	int img_width = 640;
	//if( feature_set_1->source_image.source != NULL )
	//{
	//	img_height = feature_set_1->source_image.source->nj();
	//	img_width = feature_set_1->source_image.source->ni();
	//}

	//remove duplicates
	vnl_matrix< vector< vnl_vector_fixed<int, 2> > > buckets(img_height, img_width);
	vector<bool> flag_delete(match_set.num_features());
	for(int i=0; i<flag_delete.size(); i++)
		flag_delete[i] = false;

	for(int i=0; i<match_set.num_features(); i++)
	{
		int col1 = ROUND(match_set.matches[i].first->point.y());
		int row1 = ROUND(match_set.matches[i].first->point.x());

		int col2 = ROUND(match_set.matches[i].second->point.y());
		int row2 = ROUND(match_set.matches[i].second->point.x());
		
		bool bDelete = false;
		if( buckets[row1][col1].size() != 0)
		{
			for(int j=0; j<buckets[row1][col1].size(); j++)
			{
				vnl_vector_fixed<int, 2> &p = buckets[row1][col1][j];
				if( p[0] == row2 && p[1] == col2)
				{
					flag_delete[i] = true;
					bDelete = true;
					break;
				}
			}
		}
		
		if( !bDelete )
			buckets[row1][col1].push_back(vnl_vector_fixed<int, 2>(row2, col2));
	}

	int deleted_num = 0;
	for(int i=0; i<flag_delete.size(); i++)
	{
		if( flag_delete[i] )
		{
			match_set.remove_feature(i-deleted_num);
			deleted_num++;
		}
	}

	return true;
}

bool visual_feature_matching(SImageVisualFeatureSet_sptr &feature_set_1, 
							 SImageVisualFeatureSet_sptr &feature_set_2,
							 PairwiseMatchIdxSet &match_set_indices,
							 double min_dist_ratio,
							 int frmIdx_1,
							 int frmIdx_2)
{
	SImageVisualFeatureMatchSet match_set;
	if( visual_feature_matching(feature_set_1, feature_set_2, match_set, min_dist_ratio) )
	{
		match_set_indices.frm_idx1 = frmIdx_1;
		match_set_indices.frm_idx2 = frmIdx_2;
		match_set_indices.match.clear();
		for(int i=0; i<match_set.num_features(); i++)
		{
			int idx1 = -1;
			feature_set_index(feature_set_1, match_set.matches[i].first->point.y(), match_set.matches[i].first->point.x(), &idx1);
			int idx2 = -1;
			feature_set_index(feature_set_2, match_set.matches[i].second->point.y(), match_set.matches[i].second->point.x(), &idx2);
			match_set_indices.match.push_back(IntPair(idx1, idx2));
		}
		return true;
	}
	else
		return false;
}

/*----------copy the code from the vpgl bundler implementation ------**/
// This templated class takes in a vector of bools, and a vector of Ts.
// It deleted every element in the Ts vector whose corresponding element
// in the bool vector is true. Checks that the vectors are the same size.
void remove_if_true( vector<bool> &checks,
					 SImageVisualFeatureMatchSet &to_prune)
{
    assert(checks.size() == to_prune.num_features());

    int num_removed = 0;
    for (unsigned int i = 0; i < checks.size(); i++) {
        // If this is an outlier, remove it.
        if (checks[i]) 
		{
            to_prune.remove_feature(i - num_removed);
            num_removed++;
        }
    }
}

// Make sure each feature only appears once in the list. Remove any
// pair that has a duplicate.
void remove_all_duplicates(SImageVisualFeatureMatchSet &matches)
{
    vector<bool> checks;

/*    for (int i = 0; i < matches.num_features(); i++)
    {
        bool to_kill = false;
        for (int j = 0; j < matches.num_features(); j++)
        {
            // If we are not looking at the same feature pair,
            // then say we have (a,b) and (c,d). If a is the same
            // as either c or d, or b is the same as either c or d,
            // then we need to remove i.
            if (i != j && (
                matches.side1[i] == matches.side1[j] ||
                matches.side1[i] == matches.side2[j] ||
                matches.side2[i] == matches.side1[j] ||
                matches.side2[i] == matches.side2[j]))
            {
                to_kill = true;
                break;
            }
        }

        checks.push_back(to_kill);
    }
*/
    remove_if_true(checks, matches);
}

bool estimate_fundamental_matrix( SImageVisualFeatureMatchSet &matches,
								   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
								   int min_inliers,
								   double feature_loc_std,
								   bool recompute_after_ransac)
{
    // First, remove any matches where feature a in image 1 matches to
    // both feature b and c in image 2. In other words, a feature
    // may only appear once in this list.
     remove_all_duplicates(matches);

    if (matches.num_features() < min_inliers) {
        matches.clear();
        return false;
    }

    // Now, we will estimate an F matrix, and use it to get rid of spurious
    // matches. There is a function to do it for us, which rocks.

    // Get the rhs and lhs vectors
    // Create the ransac problem solver
	FMatrixComputeRANSAC ransac(true, feature_loc_std);
	
    // Run the ransac
	vector<vgl_homg_point_2d<double> > rhs, lhs;
    for (int i = 0; i < matches.num_features(); i++) {
		vgl_homg_point_2d<double> p1(matches.matches[i].first->point.y(), matches.matches[i].first->point.x());
		vgl_homg_point_2d<double> p2(matches.matches[i].second->point.y(), matches.matches[i].second->point.x());
        rhs.push_back(p1);
        lhs.push_back(p2);
    }

	FMatrix fm_;
    ransac.compute(rhs, lhs, fm_);

	vector<bool> inlier = ransac.get_inliers();
	int num_inliers = 0;
	for(unsigned int i=0; i<inlier.size(); i++)
	{
		if( inlier[i] )
			num_inliers++;
	}

    // Finally, remove everything if there are fewer than the minimum number
    // of inliers.
    if (num_inliers < min_inliers) 
	{
        matches.clear();
		return false;
    }

	if( recompute_after_ransac )
	{
		vector< vgl_homg_point_2d<double> > pr;
		vector< vgl_homg_point_2d<double> > pl;
		for(int i=0; i<matches.num_features(); i++)
		{
			if( inlier[i] )
			{
				pr.push_back(vgl_homg_point_2d<double>(matches.matches[i].first->point));
				pl.push_back(vgl_homg_point_2d<double>(matches.matches[i].second->point));	
			}
		}

		CFundamentalMatrix fm;
		vpgl_fm_compute_8_point fm_computer(true);
		fm_computer.compute(pr, pl, fm);

		vector<bool> outlier_flag;
		for(int i=0; i<matches.num_features(); i++)
		{
			double d = fm.get_residual(matches.matches[i].first->point, matches.matches[i].second->point);
			if( d < 1.96*1.96*feature_loc_std*feature_loc_std )
				outlier_flag.push_back(false);
			else
				outlier_flag.push_back(true);
		}

		remove_if_true(outlier_flag, matches);

		if (matches.num_features() < min_inliers) 
		{
			matches.clear();
			return false;
		}

		fundamental_mat = fm.get_matrix();
		//fm.printMatrix();
		//printf("inliner: %d\n", matches.num_features());
	}
	else
	{
		vector<bool> outlier_flag;
		for(unsigned int i=0; i<inlier.size(); i++)
			outlier_flag.push_back(!inlier[i]);
		remove_if_true(outlier_flag, matches);
		if (matches.num_features() < min_inliers) 
		{
			matches.clear();
			return false;
		}
		fundamental_mat = fm_.get_matrix();
	}

	return true;
}

bool estimate_fundamental_matrix( SImageVisualFeatureMatchSet &matches,
								   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
								   int min_inliers,
								   double outlier_threshold,
								   double probability_good_f_mat,
								   double max_outlier_frac,
								   bool recompute_after_ransac)
{
    // First, remove any matches where feature a in image 1 matches to
    // both feature b and c in image 2. In other words, a feature
    // may only appear once in this list.
     remove_all_duplicates(matches);

    if (matches.num_features() < min_inliers) {
        matches.clear();
        return false;
    }

    // Now, we will estimate an F matrix, and use it to get rid of spurious
    // matches. There is a function to do it for us, which rocks.

/*    // Get the rhs and lhs vectors
    // Create the ransac problem solver
    fm_compute_ransac ransac;
    ransac.set_desired_prob_good(probability_good_f_mat);
    ransac.set_outlier_threshold(outlier_threshold);
    ransac.set_max_outlier_frac(max_outlier_frac);

    // Run the ransac
    vector<vgl_point_2d<double> > rhs, lhs;
    for (int i = 0; i < matches.num_features(); i++) {
        rhs.push_back(matches.matches[i].first->point);
        lhs.push_back(matches.matches[i].second->point);
    }

	CFundamentalMatrix fm;
    ransac.compute(rhs, lhs, fm);

    //// We'll ignore the fundemental matrix, and just look at the outliers.
    //remove_if_true(ransac.outliers, matches);
	int num_inliers_ransac = 0;
	for(unsigned int i=0; i<ransac.outliers.size(); i++)
	{
		if( !ransac.outliers[i] )
			num_inliers_ransac++;
	}

    // Finally, remove everything if there are fewer than the minimum number
    // of inliers.
    if (num_inliers_ransac < min_inliers) 
	{
        matches.clear();
		return false;
    }

	//fm.printMatrix();
	//printf("inliner: %d\n", num_inliers);

	if( recompute_after_ransac )
	{
		vector< vgl_homg_point_2d<double> > pr;
		vector< vgl_homg_point_2d<double> > pl;
		for(int i=0; i<matches.num_features(); i++)
		{
			if( !ransac.outliers[i] )
			{
				pr.push_back(vgl_homg_point_2d<double>(matches.side1[i]->point));
				pl.push_back(vgl_homg_point_2d<double>(matches.side2[i]->point));	
			}
		}

		vpgl_fm_compute_8_point fm_computer(true);
		fm_computer.compute(pr, pl, fm);

		vector<bool> outlier_flag;
		double residual_total = 0.0;
		int inlier_num = 0;
		for(int i=0; i<matches.num_features(); i++)
		{
			if( ransac.outliers[i] )
			{
				outlier_flag.push_back(true);
				continue;
			}

			double d = fm.get_residual(matches.matches[i].first->point, matches.matches[i].second->point);
			if( d < outlier_threshold )
			{
				outlier_flag.push_back(false);
				inlier_num++;
				residual_total += d;
			}
			else
				outlier_flag.push_back(true);
		}
		residual_total /= inlier_num;
		printf("match<inlier recompute, inlier_ransac, original><%d/%d/%d>, residual=%f\n", inlier_num,  num_inliers_ransac,  matches.num_features(), residual_total);

		remove_if_true(outlier_flag, matches);

		if (matches.num_features() < min_inliers) 
		{
			matches.clear();
			return false;
		}
	}
	else
	{
		remove_if_true(ransac.outliers, matches);
	}


	fundamental_mat.copy_in(fm.get_matrix().data_block());
*/
	return true;
}
/*-----end----------*/

bool extract_transform( vector< vnl_vector_fixed<double, 3> > const &points_1,
						vector< vnl_vector_fixed<double, 3> > const &points_2,
						vnl_matrix_fixed<double, 3, 3> &R,
						vnl_vector_fixed<double, 3> &T)
{
	S3DPointMatchSet match_set_3d;
	match_set_3d.points_1 = points_1;
	match_set_3d.points_2 = points_2;
	return extract_transform(match_set_3d, R, T);
}

bool extract_transform( S3DPointMatchSet const& match_set_3d,
						vnl_matrix_fixed<double, 3, 3> &R,
						vnl_vector_fixed<double, 3> &T)
{
	int n_anchor = match_set_3d.size();
	if( n_anchor < 3)
		return false;
	vnl_matrix<double> points_anchor_1(n_anchor, 3);
	vnl_matrix<double> points_anchor_2(3, n_anchor);
	for(int i=0; i<n_anchor; i++)
	{
		points_anchor_1.set_row(i, match_set_3d.points_1[i]);
		points_anchor_2.set_column(i, match_set_3d.points_2[i]);
	}

	vnl_vector_fixed<double, 3> mean_vec_anchor_1 = get_mean_vector(points_anchor_1, 1);
	vnl_vector_fixed<double, 3> mean_vec_anchor_2 = get_mean_vector(points_anchor_2, 2);
	sub_vec_from_mat(points_anchor_1, mean_vec_anchor_1, 1);
	sub_vec_from_mat(points_anchor_2, mean_vec_anchor_2, 2);

	vnl_matrix_fixed<double, 3, 3> R_ = points_anchor_2 * points_anchor_1;	
	
	// make R unitary
	vnl_svd<double> svd_solver(R_);
	R = svd_solver.U() * svd_solver.V().transpose();
	if( vnl_det<double>(R) < 0 )
	{
		//printf("Warning...\n");
		R = -R;
	}

	T = mean_vec_anchor_2 - R*mean_vec_anchor_1;
	return true;
}

bool extract_transform_ransac( S3DPointMatchSet &match_set_3d,
							   vnl_matrix_fixed<double, 3, 3> &R,
							   vnl_vector_fixed<double, 3> &T)
{
	int sample_num = 50;
	int points_num = match_set_3d.size();
	vnl_matrix<double> X(3, 3);
	vnl_matrix<double> Y(3, 3);		
	vector< S3DPointMatchSet > matches_3d_refined_vec;
	for(int i=0; i<sample_num; i++)
	{
		int idx1 = 0;
		int idx2 = 0;
		int idx3 = 0;
		while( true )
		{
			idx1 = int(RANDOM * points_num) % points_num;
			idx2 = int(RANDOM * points_num) % points_num;
			idx3 = int(RANDOM * points_num) % points_num;
			if( idx1 != idx2 && idx2!= idx3 && idx1 != idx3)
				break;
		}

		X.set_row(0, match_set_3d.points_1[idx1]);
		X.set_row(1, match_set_3d.points_1[idx2]);
		X.set_row(2, match_set_3d.points_1[idx3]);
		Y.set_row(0, match_set_3d.points_2[idx1]);
		Y.set_row(1, match_set_3d.points_2[idx2]);
		Y.set_row(2, match_set_3d.points_2[idx3]);

		calcTransform(X, Y, R, T);

		S3DPointMatchSet match_set_3d_refined;
		compute_inliers(match_set_3d, match_set_3d_refined, R, T, 5.0, false);
		matches_3d_refined_vec.push_back(match_set_3d_refined);
	}

	//find the best match
	int max_idx = -1;
	int max_match_num = 0;
	for(int i=0; i<matches_3d_refined_vec.size(); i++)
	{
		if( max_match_num < matches_3d_refined_vec[i].size() )
		{
			max_idx = i;
			max_match_num = matches_3d_refined_vec[i].size();
		}
	}

	if( max_idx == -1 ||
		max_match_num <= 2)
	{
		return false;
	}

	//recompute the transformation
	S3DPointMatchSet &match_set_3d_refined_max = matches_3d_refined_vec[max_idx];
	return calcTransform(match_set_3d_refined_max, R, T);
}

bool extract_transform(vnl_matrix_fixed<double,3,3> const&F, 
						vnl_matrix_fixed<double,3,3> const&K_left, //intriniscs
						vnl_matrix_fixed<double,3,3> const&K_right, //intriniscs
						vgl_point_2d<double> left_corr, 
						vgl_point_2d<double> right_corr,
						vnl_matrix_fixed<double,3,3> &R, 
						vnl_vector_fixed<double,3> &T,
						double T_mag)
{

/*	double data_block[9] = {2.56107e-006, -2.16656e-005, 0.0105156,
							2.56086e-005, -1.59827e-006, 0.00215137,
							-0.0117936, -0.0047798, 1.65112};
	vnl_matrix_fixed<double, 3, 3> F_(data_block);
*/
	vnl_matrix_fixed<double, 3, 3> E = K_left.transpose() * F * K_right;
	
	//printf("========Essential Matrix====\n");
	//E.print(vcl_cout);

/*	vnl_svd<double> E_svd(E);
	double data_block[9] = {0.0, -1.0, 0.0,
							1.0, 0.0, 0.0,
							0.0, 0.0, 1.0};
	vnl_matrix_fixed<double, 3, 3> S(data_block);
	R = E_svd.U() * S * E_svd.V().inplace_transpose();

	double data_block_2[9] = {0.0, 1.0, 0.0,
							-1.0, 0.0, 0.0,
							0.0, 0.0, 0.0};
	vnl_matrix_fixed<double, 3, 3> S_2(data_block_2);
	vnl_matrix_fixed<double, 3, 3> T_matrix = E_svd.U() * S_2 * E_svd.U().inplace_transpose();

	T = vnl_vector_fixed<double, 3>( T_matrix(2, 1), T_matrix(0, 2), T_matrix(1, 0) );
*/
	CEssentialMatrix EssentialMat(E);

	vgl_point_2d<double> left_point_in_image_plane( (left_corr.x() - K_left[0][2])/K_left[0][0], 
													(left_corr.y() - K_left[1][2])/K_left[1][1] );
	vgl_point_2d<double> right_point_in_image_plane( (right_corr.x() - K_right[0][2])/K_right[0][0], 
													 (right_corr.y() - K_right[1][2])/K_right[1][1] );

	vpgl_perspective_camera<double> p_left;
	bool ret = extract_left_camera( EssentialMat, 
									left_point_in_image_plane, 
									right_point_in_image_plane, 
									p_left, T_mag);
	if( !ret )
		return false;

	R = p_left.get_rotation().as_matrix();
	vgl_vector_3d<double> T_tmp = p_left.get_translation();
	T[0] = T_tmp.x();
	T[1] = T_tmp.y();
	T[2] = T_tmp.z();

	return true;
}

bool triangulation( SImageVisualFeatureMatchSet &match_set,
					 vpgl_proj_camera<double> &cam1,
					 vpgl_proj_camera<double> &cam2,
					 vector< vgl_point_3d<double> > &points_3d)									 
{
	vnl_matrix_fixed<double,4,4> A;
	vnl_matrix_fixed<double,3,4> P1 = cam1.get_matrix();
	vnl_matrix_fixed<double,3,4> P2 = cam2.get_matrix();

	for(int c=0; c<match_set.num_features(); c++)
	{
		vgl_point_2d<double> x1 = match_set.matches[c].first->point;
		vgl_point_2d<double> x2 = match_set.matches[c].first->point;

		/* x1 x P1 * X = 0
		 * x2 x P2 * X = 0
		 */
		for(int i=0; i<4; i++) 
		{
			A[0][i] = -x1.x()*P1[2][i] + P1[0][i];
			A[1][i] =  x1.y()*P1[2][i] - P1[1][i];
			A[2][i] = -x2.x()*P2[2][i] + P2[0][i];
			A[3][i] =  x2.y()*P2[2][i] - P2[1][i];
		}

		vnl_svd<double> svd_solver(A);
		vnl_vector_fixed<double, 4> p = svd_solver.nullvector();
		points_3d.push_back(vgl_point_3d<double>(p[0]/p[3], p[1]/p[3], p[2]/p[3]));

		//vnl_svd<double> svd_solver(A.get_n_columns(0, 3));
		////vnl_vector_fixed<double, 4> p = svd_solver.nullvector();
		////vgl_homg_point_3d<double> hp(p[0],p[1],p[2],p[3]);
		//vnl_vector_fixed<double, 3> p = svd_solver.solve(-A.get_column(3));
		//points_3d.push_back(vgl_point_3d<double>(p[0], p[1], p[2]));
	}

	return true;
}


bool triangulation( SImageVisualFeatureMatchSet &match_set,
					 vnl_matrix_fixed<double, 3, 3> &K,
					 vnl_matrix_fixed<double, 3, 3> &R, 
					 vnl_vector_fixed<double, 3> &T,
					 vector< vgl_point_3d<double> > &points_3d )
{
	vnl_matrix_fixed<double,3,4> P1(0.0);
	for(int i=0; i<3; i++)
		P1[i][i] = 1.0;
	vnl_matrix_fixed<double,3,4> P2(0.0);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			P2[i][j] = R[i][j];
	for(int i=0; i<3; i++)
		P2[i][3] = T[i];

	P1 = K*P1;
	P2 = K*P2;

	vpgl_proj_camera<double> cam1(P1);
	vpgl_proj_camera<double> cam2(P2);

	return triangulation(match_set, cam1, cam2, points_3d);
}

void triangulation( vector< vnl_vector_fixed<double, 2> > const& image_pts,
					vector< vnl_matrix_fixed<double, 3, 4> > const& Ps,
					vnl_vector_fixed<double, 3> &p_3d)
{
	if( image_pts.size() != Ps.size())
	{
		printf("Error<triangulation>: image_pts.size != Ps.size() (%zd vs %zd)!\n", image_pts.size(), Ps.size());
		return;
	}
	int pts_num = image_pts.size();
	vnl_matrix<double> A(pts_num*2, 4);
	for(int k=0; k<pts_num; k++)
	{
		vnl_matrix_fixed<double, 3, 4> const& P = Ps[k];
		vnl_vector_fixed<double, 2> const&x = image_pts[k];
		for(int i=0; i<4; i++)
		{
			A[2*k][i]   = -x[0]*P[2][i] + P[0][i];
			A[2*k+1][i] =  x[1]*P[2][i] - P[1][i];
		}
	}
	vnl_svd<double> svd_solver(A);
	vnl_vector_fixed<double, 4> p = svd_solver.nullvector();
	p_3d[0] = p[0]/p[3];
	p_3d[1] = p[1]/p[3];
	p_3d[2] = p[2]/p[3];
}

void triangulation( vector< vnl_vector_fixed<double, 2> > const& image_pts,
					vector< vpgl_perspective_camera<double>* > const& cams, //camera poses
					vnl_vector_fixed<double, 3> &p_3d)
{
	vector< vnl_matrix_fixed<double, 3, 4> > Ps;
	for(int i=0; i<cams.size(); i++)
		Ps.push_back(cams[i]->get_matrix());

	triangulation(image_pts, Ps, p_3d);
}


/* compute the inliers of the 3D point match set under transformation <R, T>
 */
void compute_inliers( S3DPointMatchSet const&match_3d_in, S3DPointMatchSet &match_3d_out,
					  vnl_matrix_fixed<double, 3, 3> const&R, vnl_vector_fixed<double, 3> const&T, 
					  double thres_dist,
					  bool bDeleteOneVsMany)
{
	match_3d_out.clear();
	int inliers_num = 0;
	vector<double> dist_vec;
	for( int k=0; k<match_3d_in.size(); k++)
	{
		vnl_vector_fixed<double, 3> const&p1 = match_3d_in.points_1[k];
		vnl_vector_fixed<double, 3> const&p2 = match_3d_in.points_2[k];

		vnl_vector_fixed<double, 3> p2_ = R*p1 + T;
		vnl_vector_fixed<double, 3> p_dist = p2 - p2_;
		
		if( p_dist.magnitude() < thres_dist )
		{
			match_3d_out.push_back(p1, p2);
			dist_vec.push_back(p_dist.magnitude());
			inliers_num ++;
		}
	}
	
	//if( match_3d_out.size() == 7)
	//{
	//	save_3D_match_set(match_3d_out, "D:/3DRegistration/result/test_3d_match_set.txt");
	//	int debug = 1;
	//}

	if( match_3d_out.size() <= 1 )
		return;

	if( !bDeleteOneVsMany )
		return;

	vector<int> indices_to_be_deleted;

	//check if there is any one-vs-many case(points_1-->points_2)
	vector< vnl_vector_fixed<double, 3> > &points_1 = match_3d_out.points_1;
	double x_s = points_1[0][0];
	double x_e = points_1[0][0];
	for(int i=1; i<points_1.size(); i++)
	{
		if( points_1[i][0] < x_s )
			x_s = points_1[i][0];
		else if( points_1[i][0] > x_e )
			x_e = points_1[i][0];
	}
	
	//build the hash table
	double dx = 0.5*(x_e-x_s)/points_1.size();
	int cell_num = 2*points_1.size()+1;
	vector< vector<int> > buckets(cell_num);
	int idx = 0;
	for(int i=0; i<points_1.size(); i++)
	{
		if( dx == 0)
			idx = 0;
		else
			idx = floor( (points_1[i][0]-x_s)/dx );
		buckets[idx].push_back(i);
	}
	
	for(int i=0; i<cell_num; i++)
	{
		if( buckets[i].size() <= 1)
			continue;

		//perform clustering on cells with 2 or more points.
		vector< vector<int> > clusters(buckets[i].size()-1);
		vnl_vector<bool> bClustered(buckets[i].size());
		bClustered.fill(false);
		for(int j=0; j<buckets[i].size()-1; j++)
		{
			if( bClustered[j] )
				continue;

			int idx1 = buckets[i][j];
			clusters[j].push_back(idx1);
			for(int k=j+1; k<buckets[i].size(); k++)
			{
				int idx2 = buckets[i][k];
				if( points_1[idx1] == points_1[idx2] )
				{
					clusters[j].push_back(idx2);
					bClustered[k] = true;
				}
			}
		}

		//decide keep which point in a cluster
		for(int j=0; j<clusters.size(); j++)
		{
			if( clusters[j].size() <= 1)
				continue;

			double min_val = thres_dist;
			int min_idx = -1;
			for(int k=0; k<clusters[j].size(); k++)
			{
				int idx = clusters[j][k];
				if( dist_vec[idx] < min_val )
				{
					min_val = dist_vec[idx];
					min_idx = k;
				}
			}

			for(int k=0; k<clusters[j].size(); k++)
			{
				if( k == min_idx )
					continue;

				int idx = clusters[j][k];
				indices_to_be_deleted.push_back(idx);
			}
		}
	}

	//check if there is any one-vs-many case(points_2-->points_1)
	vector< vnl_vector_fixed<double, 3> > &points_2 = match_3d_out.points_2;
	x_s = points_2[0][0];
	x_e = points_2[0][0];
	for(int i=1; i<points_2.size(); i++)
	{
		if( points_2[i][0] < x_s )
			x_s = points_2[i][0];
		else if( points_2[i][0] > x_e )
			x_e = points_2[i][0];
	}

	//build the hash table
	dx = 0.5*(x_e-x_s)/points_2.size();
	cell_num = 2*points_2.size()+1;
	buckets.clear();
	buckets.resize(cell_num);
	for(int i=0; i<points_2.size(); i++)
	{
		if( dx == 0)
			idx = 0;
		else
			idx = floor( (points_2[i][0]-x_s)/dx );
		buckets[idx].push_back(i);
	}
	
	for(int i=0; i<cell_num; i++)
	{
		if( buckets[i].size() <= 1)
			continue;

		//perform clustering on cells with 2 or more points.
		vector< vector<int> > clusters(buckets[i].size()-1);
		vnl_vector<bool> bClustered(buckets[i].size());
		bClustered.fill(false);
		for(int j=0; j<buckets[i].size()-1; j++)
		{
			if( bClustered[j] )
				continue;

			int idx1 = buckets[i][j];
			clusters[j].push_back(idx1);
			for(int k=j+1; k<buckets[i].size(); k++)
			{
				int idx2 = buckets[i][k];
				if( points_2[idx1] == points_2[idx2] )
				{
					clusters[j].push_back(idx2);
					bClustered[k] = true;
				}
			}
		}

		//decide keep which point in a cluster
		for(int j=0; j<clusters.size(); j++)
		{
			if( clusters[j].size() <= 1)
				continue;

			double min_val = thres_dist;
			int min_idx = -1;
			for(int k=0; k<clusters[j].size(); k++)
			{
				int idx = clusters[j][k];
				if( dist_vec[idx] < min_val )
				{
					min_val = dist_vec[idx];
					min_idx = k;
				}
			}

			for(int k=0; k<clusters[j].size(); k++)
			{
				if( k == min_idx )
					continue;

				int idx = clusters[j][k];
				indices_to_be_deleted.push_back(idx);
			}
		}
	}

	S3DPointMatchSet match_3d_out_filtered;
	vnl_vector<bool> bDelelted(match_3d_out.size());
	bDelelted.fill(false);
	for(int i=0; i<indices_to_be_deleted.size(); i++)
	{
		int idx = indices_to_be_deleted[i];
		bDelelted[idx] = true;
	}
	
	for(int i=0; i<bDelelted.size(); i++)
	{
		if( !bDelelted[i] )
			match_3d_out_filtered.push_back(match_3d_out.points_1[i], match_3d_out.points_2[i]);
	}

	match_3d_out = match_3d_out_filtered;
}

/* refine the 3D match set with RANSAC
 */
bool refine_3DPointsMatchSet_RANSAC(S3DPointMatchSet &match_set_in, 
									S3DPointMatchSet &match_set_out,
									vnl_matrix_fixed<double, 3, 3> &R,
									vnl_vector_fixed<double, 3> &T,
									double dist_thres,
									double inlier_percentage,
									double ransac_accracy)
{
	int match_num = match_set_in.size();
	if( match_num < 3 )
		return false;
	int samples_num = log(1-ransac_accracy)/log(1.0-pow(inlier_percentage, 3));
	int samples_num_max = match_num*(match_num-1)*(match_num-2)/6;

	//Run RANSAC
	vector< S3DPointMatchSet > match_set_inliers_vec;
	for(int i=0; i<MIN(samples_num, samples_num_max); i++)
	{
		int idx1, idx2, idx3;
		do{
			idx1 = ROUND(RANDOM * match_num) % match_num;
			idx2 = ROUND(RANDOM * match_num) % match_num;
			idx3 = ROUND(RANDOM * match_num) % match_num;
		}while(idx1==idx2 &&idx2==idx3);

		S3DPointMatchSet match_set_3pairs;
		match_set_3pairs.push_back(match_set_in.points_1[idx1], match_set_in.points_2[idx1]);
		match_set_3pairs.push_back(match_set_in.points_1[idx2], match_set_in.points_2[idx2]);
		match_set_3pairs.push_back(match_set_in.points_1[idx3], match_set_in.points_2[idx3]);

		vnl_matrix_fixed<double, 3, 3> R_t;
		vnl_vector_fixed<double, 3> T_t;
		extract_transform(match_set_3pairs, R_t, T_t);

		S3DPointMatchSet match_set_inliers;
		compute_inliers(match_set_in, match_set_inliers, R_t, T_t, dist_thres);
		match_set_inliers_vec.push_back(match_set_inliers);
	}

	//find the one with most inliers
	int num_inliers_max = 0;
	int idx_max = -1;
	for(int i=0; i<match_set_inliers_vec.size(); i++)
	{
		if( match_set_inliers_vec[i].size() > num_inliers_max )
		{
			num_inliers_max = match_set_inliers_vec[i].size();
			idx_max = i;
		}
	}

	if( num_inliers_max < 3 )
		return false;

	match_set_out = match_set_inliers_vec[idx_max];
	return extract_transform(match_set_out, R, T);
	//compute_inliers(match_set_inliers_vec[idx_max], match_set_out, R, T, dist_thres);
	//return true;
}

void calc_Point3DSet( SImageVisualFeatureSet_sptr fea, 
					   vnl_matrix<double> const&depthMat,
					   vnl_matrix_fixed<double, 3, 3> const&K, 
					   std::vector< vnl_vector_fixed<double, 3> > &points_3d,
					   depth_extract_method method)
{
	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];

	points_3d.clear();
	for(int i=0; i<fea->features.size(); i++)
	{
		double row = fea->features[i]->point.x();
		double col = fea->features[i]->point.y();
		double z = extract_depth_at_visual_feature(depthMat, col, row, method);
		if( z > 0.0)
		{
			double x = (col-cx)*z/fx;
			double y = (row-cy)*z/fy;
			points_3d.push_back( vnl_vector_fixed<double, 3>(x, y, z) );
		}
		else
		{
			points_3d.push_back( vnl_vector_fixed<double, 3>(0.0) );
		}
	}
}

void Point3DSet_to_IndexSet( std::vector< vnl_vector_fixed<double, 3> > const&points_full,
							 std::vector< vnl_vector_fixed<double, 3> > const&points_part,
							 std::vector<int> &pt_indices)
{
	pt_indices.clear();
	for(int i=0; i<points_part.size(); i++)
	{
		int idx = search_val_list(points_full, points_part[i]);
		pt_indices.push_back(idx);
	}
}

void calc_3DPointsMatchSet( SImageVisualFeatureMatchSet const&match_set,
							 vnl_matrix<double> const&depthMap_1,
							 vnl_matrix<double> const&depthMap_2,
							 vnl_matrix_fixed<double, 3, 3> const&K1,
							 vnl_matrix_fixed<double, 3, 3> const&K2,
							 S3DPointMatchSet &match_set_3d,
							 depth_extract_method method)
{
	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	match_set_3d.clear();

	for( int i=0; i<match_set.num_features(); i++)
	{
		double row_1 = match_set.matches[i].first->point.y();
		double col_1 = match_set.matches[i].first->point.x();
		double row_2 = match_set.matches[i].second->point.y();
		double col_2 = match_set.matches[i].second->point.x();
		double z1 = extract_depth_at_visual_feature(depthMap_1, col_1, row_1, method);
		if(z1 <= 0.0)
			continue;
		double z2 = extract_depth_at_visual_feature(depthMap_2, col_2, row_2, method);
		if(z2 <= 0.0)
			continue;

		//if( i== 18)
		//	int debug = 1;

		double x1 = (col_1-cx1)*z1/fx1;
		double y1 = (row_1-cy1)*z1/fy1;
		vnl_vector_fixed<double, 3> point_1(x1, y1, z1);

		double x2 = (col_2-cx2)*z2/fx2;
		double y2 = (row_2-cy2)*z2/fy2;
		vnl_vector_fixed<double, 3> point_2(x2, y2, z2);

		match_set_3d.push_back(point_1, point_2);
	}
}

void calc_3DPointsMatchSet(  vnl_matrix<double> const&matched_corners,
							 vnl_matrix<double> const&depthMap_1,
							 vnl_matrix<double> const&depthMap_2,
							 vnl_matrix_fixed<double, 3, 3> const&K1,
							 vnl_matrix_fixed<double, 3, 3> const&K2,
							 S3DPointMatchSet &match_set_3d,
							 depth_extract_method method)
{
	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	match_set_3d.clear();

	for( int i=0; i<matched_corners.rows(); i++)
	{
		double row_1 = matched_corners[i][1];
		double col_1 = matched_corners[i][0];
		double row_2 = matched_corners[i][3];
		double col_2 = matched_corners[i][2];
		double z1 = extract_depth_at_visual_feature(depthMap_1, col_1, row_1, method);
		if(z1 <= 0.0)
			continue;
		double z2 = extract_depth_at_visual_feature(depthMap_2, col_2, row_2, method);
		if(z2 <= 0.0)
			continue;

		double x1 = (col_1-cx1)*z1/fx1;
		double y1 = (row_1-cy1)*z1/fy1;
		vnl_vector_fixed<double, 3> point_1(x1, y1, z1);

		double x2 = (col_2-cx2)*z2/fx2;
		double y2 = (row_2-cy2)*z2/fy2;
		vnl_vector_fixed<double, 3> point_2(x2, y2, z2);

		match_set_3d.push_back(point_1, point_2);
	}
}

void calc_3DPointsMatchSet( vnl_matrix<double> const&matched_corners,
							vnl_matrix<double> const&depthMap_1,
							vnl_matrix<double> const&depthMap_2,
							vpgl_perspective_camera<double> const&cam1,
							vpgl_perspective_camera<double> const&cam2,
							S3DPointMatchSet &match_set_3d,
							depth_extract_method method)
{
	vnl_matrix_fixed<double, 3, 3> K1 = cam1.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> K2 = cam2.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R1;
	vnl_matrix_fixed<double, 3, 3> R2;
	vnl_vector_fixed<double, 3> T1;
	vnl_vector_fixed<double, 3> T2;
	get_camera_pose(cam1, R1, T1);
	get_camera_pose(cam2, R2, T2);

	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	match_set_3d.clear();

	for( int i=0; i<matched_corners.rows(); i++)
	{
		double row_1 = matched_corners[i][1];
		double col_1 = matched_corners[i][0];
		double row_2 = matched_corners[i][3];
		double col_2 = matched_corners[i][2];
		double z1 = extract_depth_at_visual_feature(depthMap_1, col_1, row_1, method);
		if(z1 <= 0.0)
			continue;
		double z2 = extract_depth_at_visual_feature(depthMap_2, col_2, row_2, method);
		if(z2 <= 0.0)
			continue;

		double x1 = (col_1-cx1)*z1/fx1;
		double y1 = (row_1-cy1)*z1/fy1;
		vnl_vector_fixed<double, 3> point_1(x1, y1, z1);

		double x2 = (col_2-cx2)*z2/fx2;
		double y2 = (row_2-cy2)*z2/fy2;
		vnl_vector_fixed<double, 3> point_2(x2, y2, z2);

		point_1 = R1.transpose() * (point_1-T1);
		point_2 = R2.transpose() * (point_2-T2);

		match_set_3d.push_back(point_1, point_2);
	}
}

void calc_3DPointsMatchSet(vnl_matrix<double> const&matched_corners,
						   cv::Mat const& depthMap_1,
						   cv::Mat const& depthMap_2,
						   GCameraView const&cam1,
						   GCameraView const&cam2,
						   S3DPointMatchSet &match_set_3d,
						   depth_extract_method method,
						   bool bStartOver)
{
	vnl_matrix_fixed<double, 3, 3> R1;
	vnl_matrix_fixed<double, 3, 3> R2;
	vnl_vector_fixed<double, 3> T1;
	vnl_vector_fixed<double, 3> T2;
	get_camera_pose(cam1, R1, T1);
	get_camera_pose(cam2, R2, T2);

	vnl_matrix_fixed<double, 3, 3> K1;
	vnl_matrix_fixed<double, 3, 3> K2;
	get_calibration_matrix(cam1, K1);
	get_calibration_matrix(cam2, K2);
	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	if (bStartOver)
		match_set_3d.clear();

	for (int i = 0; i<matched_corners.rows(); i++)
	{
		double row_1 = matched_corners[i][1];
		double col_1 = matched_corners[i][0];
		double row_2 = matched_corners[i][3];
		double col_2 = matched_corners[i][2];
		double z1 = extract_depth_at_visual_feature(depthMap_1, col_1, row_1, method);
		if (z1 <= 0.0)
			continue;
		double z2 = extract_depth_at_visual_feature(depthMap_2, col_2, row_2, method);
		if (z2 <= 0.0)
			continue;

		double x1 = (col_1 - cx1)*z1 / fx1;
		double y1 = (row_1 - cy1)*z1 / fy1;
		vnl_vector_fixed<double, 3> point_1(x1, y1, z1);

		double x2 = (col_2 - cx2)*z2 / fx2;
		double y2 = (row_2 - cy2)*z2 / fy2;
		vnl_vector_fixed<double, 3> point_2(x2, y2, z2);

		point_1 = R1.transpose() * (point_1 - T1);
		point_2 = R2.transpose() * (point_2 - T2);

		match_set_3d.push_back(point_1, point_2);
	}
}


void PointMatchSet_3DTo2D(  S3DPointMatchSet const&match_set_3d,
						    vnl_matrix_fixed<double, 3, 3> const&K1,
						    vnl_matrix_fixed<double, 3, 3> const&K2,
						    SImageVisualFeatureMatchSet &match_set_2d )
{
	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	match_set_2d.clear();

	for( int i=0; i<match_set_3d.size(); i++)
	{
		double x1 = fx1 * match_set_3d.points_1[i][0]/match_set_3d.points_1[i][2] + cx1;
		double y1 = fy1 * match_set_3d.points_1[i][1]/match_set_3d.points_1[i][2] + cy1;
		double x2 = fx2 * match_set_3d.points_2[i][0]/match_set_3d.points_2[i][2] + cx2;
		double y2 = fx2 * match_set_3d.points_2[i][1]/match_set_3d.points_2[i][2] + cy2;

		SImageVisualFeature_sptr f1( new SImageVisualFeature(ROUND(y1), ROUND(x1), 0.0, 0.0) );
		SImageVisualFeature_sptr f2( new SImageVisualFeature(ROUND(y2), ROUND(x2), 0.0, 0.0) );

		match_set_2d.matches.push_back(make_pair(f1, f2));
	}
}

void PointMatchSet_3DTo2D( S3DPointMatchSet const&match_set_3d,
						   vnl_matrix_fixed<double, 3, 3> const&K1,
						   vnl_matrix_fixed<double, 3, 3> const&K2,
						   vnl_matrix<double> &matched_points )
{
	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	int num_matches = match_set_3d.size();
	matched_points.set_size(num_matches, 4);
	for( int i=0; i<match_set_3d.size(); i++)
	{
		double x1 = fx1 * match_set_3d.points_1[i][0]/match_set_3d.points_1[i][2] + cx1;
		double y1 = fy1 * match_set_3d.points_1[i][1]/match_set_3d.points_1[i][2] + cy1;
		double x2 = fx2 * match_set_3d.points_2[i][0]/match_set_3d.points_2[i][2] + cx2;
		double y2 = fx2 * match_set_3d.points_2[i][1]/match_set_3d.points_2[i][2] + cy2;

		matched_points[i][0] = x1;
		matched_points[i][1] = y1;
		matched_points[i][2] = x2;
		matched_points[i][3] = y2;
	}
}

void PointMatchSet_3DTo2D( S3DPointMatchSet const&match_set_3d,
						   vpgl_perspective_camera<double> const&cam1,
						   vpgl_perspective_camera<double> const&cam2,
						   vnl_matrix<double> &matched_points )
{
	vnl_matrix_fixed<double, 3, 3> K1 = cam1.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> K2 = cam2.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R1;
	vnl_matrix_fixed<double, 3, 3> R2;
	vnl_vector_fixed<double, 3> T1;
	vnl_vector_fixed<double, 3> T2;
	get_camera_pose(cam1, R1, T1);
	get_camera_pose(cam2, R2, T2);

	double fx1 = K1[0][0];
	double cx1 = K1[0][2];
	double fy1 = K1[1][1];
	double cy1 = K1[1][2];

	double fx2 = K2[0][0];
	double cx2 = K2[0][2];
	double fy2 = K2[1][1];
	double cy2 = K2[1][2];

	int num_matches = match_set_3d.size();
	matched_points.set_size(num_matches, 4);
	for( int i=0; i<match_set_3d.size(); i++)
	{
		vnl_vector_fixed<double, 3> point_1 = R1*match_set_3d.points_1[i] + T1;
		vnl_vector_fixed<double, 3> point_2 = R2*match_set_3d.points_2[i] + T2;

		double x1 = fx1 * point_1[0]/point_1[2] + cx1;
		double y1 = fy1 * point_1[1]/point_1[2] + cy1;
		double x2 = fx2 * point_2[0]/point_2[2] + cx2;
		double y2 = fx2 * point_2[1]/point_2[2] + cy2;

		matched_points[i][0] = x1;
		matched_points[i][1] = y1;
		matched_points[i][2] = x2;
		matched_points[i][3] = y2;
	}
}

SImageVisualFeature_sptr feature_set_index( SImageVisualFeatureSet_sptr fea_set, 
											double row, double col, int *idx)
{
	for(int i=0; i<fea_set->features.size(); i++)
	{
		if( abs(fea_set->features[i]->point.x() - col) < 1.5 &&
			abs(fea_set->features[i]->point.y() - row) < 1.5 )
		{
			if( idx != NULL ) 
				*idx = i;
			return fea_set->features[i];
		}
	}

	if( idx != NULL ) *idx = -1;
	return SImageVisualFeature_sptr(NULL);
}

/* test code
 * save the visual features (location and feature vector separately)
 */
bool save_visual_feature_set_ASCII( SImageVisualFeatureSet_sptr feature_set, 
									const char *filename_loc_scale_ori, 
									const char *filename_features)
{
	FILE *file_loc = fopen(filename_loc_scale_ori, "w");
	if( file_loc == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename_loc_scale_ori);
		return false;
	}

	FILE *file_fea = fopen(filename_features, "w");
	if( file_fea == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename_features);
		return false;
	}

	vector<SImageVisualFeature_sptr> features = feature_set->features;

	for(unsigned int i=0; i<features.size(); i++)
	{
		SImageVisualFeature* f = (SImageVisualFeature*) features[i].ptr();
		fprintf(file_loc, "%f, %f %f %f\n", f->x(),f->y(), f->scale, f->orientation);
		for(unsigned int j=0; j<features[i]->descriptor.size(); j++)
		{
			fprintf(file_fea, "%f ", features[i]->descriptor[j]);
		}
		fprintf(file_fea, "\n");
	}
	
	fclose(file_fea);
	fclose(file_loc);

	return true;
}

bool save_visual_feature_set_ASCII( SImageVisualFeatureSet_sptr feature_set, 
									const char *filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if(!fp)
	{
		printf("Cannot Open file %s for writing visual features!\n", filename);
		return false;
	}

	vector<SImageVisualFeature_sptr> features = feature_set->features;

	fprintf(fp, "SIFT Image Feature Data(#version 1.0)\n");
	fprintf(fp, "Number of features %zd\n", features.size());
	for(int i=0; i<features.size(); i++)
	{
		SImageVisualFeature* f = (SImageVisualFeature*) features[i].ptr();
		fprintf(fp, "Feature %04d: loc<%f %f>, scale<%f>, ori<%f>\n", i, f->x(),f->y(), 
						f->scale, f->orientation);
		fprintf(fp, "\tDiscrpt<%zd>:", features[i]->descriptor.size());
		for(int j=0; j<features[i]->descriptor.size(); j++)
			fprintf(fp, " %f", features[i]->descriptor[j]);

		fprintf(fp, "\n");
	}

	fclose(fp);

	return true;
}

SImageVisualFeatureSet_sptr load_visual_feature_set_ASCII(const char *filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "r");
	if(!fp)
	{
		printf("Cannot Open file %s for reading visual features!\n", filename);
		return NULL;
	}

	double version = 0;
	fscanf(fp, "SIFT Image Feature Data(#version %lf)\n", &version);

	int fea_num = 0;
	fscanf(fp, "Number of features %d\n", &fea_num);

	SImageVisualFeatureSet_sptr feature_set(new SImageVisualFeatureSet);

	for(int i=0; i<fea_num; i++)
	{
		double row = 0;
		double col = 0;
		double scale = 0;
		double ori = 0;
		int idx;
		fscanf(fp, "Feature %04d: loc<%lf %lf>, scale<%lf>, ori<%lf>\n", &idx, &col, &row, &scale, &ori);
		SImageVisualFeature_sptr f(new SImageVisualFeature(ROUND(row), ROUND(col), scale, ori));

		int descriptor_dim = 0;
		fscanf(fp, "\tDiscrpt<%d>:", &descriptor_dim);
		
		f->descriptor.set_size(descriptor_dim);
		for(int j=0; j<descriptor_dim; j++)
			fscanf(fp, " %lf", &(f->descriptor[j]));

		fscanf(fp, "\n");

		feature_set->features.push_back(f);
	}

	fclose(fp);

	return feature_set;
}

bool save_visual_feature_set_BIN( SImageVisualFeatureSet_sptr fea, 
									const char *filename)
{
	FILE *file = fopen(filename, "wb");
	if( file == NULL )
	{
		printf("Error<save_visual_feature_set_BIN>: Cannot Open file <%s> for written!\n", filename);
		return false;
	}

	int num_feas = fea->features.size();
	fwrite(&num_feas, sizeof(int), 1, file);
	for(int j=0; j<num_feas; j++)
	{
		SImageVisualFeature *f = (SImageVisualFeature*) fea->features[j].as_pointer();
		fwrite(&(f->orientation), sizeof(double), 1, file);
		fwrite(&(f->point), sizeof(vgl_point_2d<double>), 1, file);
		fwrite(&(f->scale), sizeof(double), 1, file);
		int num_desc = f->descriptor.size();
		fwrite(&num_desc, sizeof(int), 1, file);
		fwrite(f->descriptor.data_block(), sizeof(double), num_desc, file);		
	}

	fclose(file);
	return true;
}

SImageVisualFeatureSet_sptr load_visual_feature_set_BIN(const char *filename)
{
	FILE *file = fopen(filename, "rb");
	if( file == NULL )
	{
		printf("Error<load_visual_feature_set_BIN>: Cannot Open file <%s> for reading!\n", filename);
		return SImageVisualFeatureSet_sptr(NULL);
	}

	SImageVisualFeatureSet_sptr fea = SImageVisualFeatureSet_sptr(new SImageVisualFeatureSet());
	int num_feas = 0;
	fread(&num_feas, sizeof(int), 1, file);
	fea->features.resize(num_feas);
	for(int j=0; j<num_feas; j++)
	{
		SImageVisualFeature *f = new SImageVisualFeature();
		fread(&(f->orientation), sizeof(double), 1, file);
		fread(&(f->point), sizeof(vgl_point_2d<double>), 1, file);
		fread(&(f->scale), sizeof(double), 1, file);
		int num_desc = 0;
		fread(&num_desc, sizeof(int), 1, file);
		f->descriptor.set_size(num_desc);
		fread(f->descriptor.data_block(), sizeof(double), num_desc, file);		
		fea->features[j] = SImageVisualFeature_sptr(f);
	}

	fclose(file);
	return fea;
}

SImageVisualFeatureSet_sptr load_visual_feature_set(const char *filename)
{
	int len = strlen(filename);
	if( len > 3 )
	{
		if(strcmp("bin", &filename[len-3])==0)
		{
			return load_visual_feature_set_BIN(filename);
		}
		else if (strcmp("txt", &filename[len-3])==0)
		{
			return load_visual_feature_set_ASCII(filename);
		}
	}

	printf("Error<load_visual_feature_set>: file name <%s> is illegal!\n", filename);
	return SImageVisualFeatureSet_sptr(NULL);
}



/* test code
 * save match set (corresponding location pairs)
 */

bool save_visual_feature_match_set_ASCII( SImageVisualFeatureMatchSet &match_set, 
										  const char *filename)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename);
		return false;
	}

	//assert(match_set.side1.size() == match_set.side2.size());

	for(unsigned int i=0; i<match_set.matches.size(); i++)
	{
		fprintf(file, "%f %f %f %f\n", match_set.matches[i].first->point.x(), match_set.matches[i].first->point.y(),
			match_set.matches[i].second->point.x(), match_set.matches[i].second->point.y());
	}

	fclose(file);
	return true;
}

bool draw_visual_feature_match_set( SImageVisualFeatureMatchSet &match_set,
									cv::Mat const& img1,
									cv::Mat const& img2,
									const char* filename_out )
{
	if( img1.empty()|| img2.empty())
		return false;
	if( img1.rows != img2.rows ||
		img1.cols != img2.cols ||
		img1.channels() != 3 ||
		img2.channels() != 3)
		return false;
	int h = img1.rows;
	int w = img1.cols;
	cv::Mat img_out = cv::Mat(h, w * 2, CV_8UC3);

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			img_out.at<cv::Vec3b>(i, j)[0] = img1.at<cv::Vec3b>(i, j)[0];
			img_out.at<cv::Vec3b>(i, j)[1] = img1.at<cv::Vec3b>(i, j)[1];
			img_out.at<cv::Vec3b>(i, j)[2] = img1.at<cv::Vec3b>(i, j)[2];
		}
	}
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			img_out.at<cv::Vec3b>(i, w+j)[0] = img2.at<cv::Vec3b>(i, j)[0];
			img_out.at<cv::Vec3b>(i, w+j)[1] = img2.at<cv::Vec3b>(i, j)[1];
			img_out.at<cv::Vec3b>(i, w+j)[2] = img2.at<cv::Vec3b>(i, j)[2];
		}
	}

	cv::Point p1, p2;
	for(int i=0; i<match_set.num_features(); i++)
	{
		int x1 = match_set.matches[i].first->point.x();
		int y1 = match_set.matches[i].first->point.y();
		p1.x = MAX(x1-2, 0);
		p1.y = MAX(y1-2, 0);
		p2.x = x1+2;
		p2.y = MIN(y1+2, h-1);
		cv::rectangle(img_out, p1, p2, cv::Scalar(0, 255, 0, 0));

		int x2 = match_set.matches[i].second->point.x()+w;
		int y2 = match_set.matches[i].second->point.y();
		p1.x = MAX(x2-2, 0);
		p1.y = MAX(y2-2, 0);
		p2.x = MIN(x2+2, 2*w-1);
		p2.y = MIN(y2+2, h-1);
		cv::rectangle(img_out, p1, p2, cv::Scalar(0, 255, 0, 0));

		p1.x = x1; p1.y = y1;
		p2.x = x2; p2.y = y2;
		cv::line(img_out, p1, p2, cv::Scalar(255, 0, 0, 0), 1);
	}

	cv::imwrite(filename_out, img_out);
	img_out.release();
	return true;
}

bool draw_visual_features_on_image( SImageVisualFeatureSet_sptr feas, 
									cv::Mat* img )
{
	if( img == NULL ||
		img->channels() != 3 )
		return false;
	int h = img->rows;
	int w = img->cols;
	
	cv::Point p1, p2;
	for(int i=0; i<feas->features.size(); i++)
	{
		int x = feas->features[i]->point.x();
		int y = feas->features[i]->point.y();
		p1.x = MAX(x-2, 0);
		p1.y = MAX(y-2, 0);
		p2.x = x+2;
		p2.y = MIN(y+2, h-1);
		cv::rectangle(*img, p1, p2, cv::Scalar(0, 255, 0, 0));
	}

	return true;
}

bool load_visual_feature_match_set_ASCII(SImageVisualFeatureMatchSet &match_set, 
										 const char *filename)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading!\n", filename);
		return false;
	}

	while( 1 )
	{
		float x1, y1, x2, y2;
		int ret = fscanf(file, "%f %f %f %f\n", &x1, &y1, &x2, &y2);
		if( ret != 4 ) //if(ret == EOF)
			break;

		SImageVisualFeature_sptr f1(new SImageVisualFeature(y1, x1));
		SImageVisualFeature_sptr f2(new SImageVisualFeature(y2, x2));
		match_set.add_match(f1, f2);
	}

	fclose(file);
	return true;
}

/* test code 
 * save 3d point clouds
 */
bool save_3D_points(const char* filename, vector< vgl_point_3d<double> > &points_3d)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename);
		return false;
	}

	fprintf(file, "#3d points file\n");
	fprintf(file, "Point num = %zd\n", points_3d.size());
	for(unsigned int i=0; i<points_3d.size(); i++)
	{
		fprintf(file, "%f %f %f\n", points_3d[i].x(), points_3d[i].y(), points_3d[i].z());
	}
	fclose(file);
	return true;
}

bool save_3D_points(const char* filename, vector< vnl_vector_fixed<double, 3> > &points_3d)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename);
		return false;
	}

	fprintf(file, "#3d points file\n");
	fprintf(file, "Point num = %zd\n", points_3d.size());
	for(unsigned int i=0; i<points_3d.size(); i++)
	{
		fprintf(file, "%f %f %f\n", points_3d[i][0], points_3d[i][1], points_3d[i][2]);
	}
	fclose(file);
	return true;
}

bool load_3D_points(const char* filename, vector< vgl_point_3d<double> > &points_3d)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading 3d points!\n", filename);
		return false;
	}

	fscanf(file, "#3d points file\n");
	int points_num = 0;
	fscanf(file, "Point num = %d\n", &points_num);
	
	points_3d.clear();
	for(unsigned int i=0; i<points_num; i++)
	{
		double x, y, z;
		fscanf(file, "%lf %lf %lf\n", &x, &y, &z);
		points_3d.push_back( vgl_point_3d<double>(x, y, z) );
	}
	fclose(file);
	return true;
}

bool load_3D_points(const char* filename, vector< vnl_vector_fixed<double, 3> > &points_3d)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading 3d points!\n", filename);
		return false;
	}

	fscanf(file, "#3d points file\n");
	int points_num = 0;
	fscanf(file, "Point num = %d\n", &points_num);
	
	points_3d.clear();
	for(unsigned int i=0; i<points_num; i++)
	{
		double x, y, z;
		fscanf(file, "%lf %lf %lf\n", &x, &y, &z);
		points_3d.push_back( vnl_vector_fixed<double, 3>(x, y, z) );
	}
	fclose(file);
	return true;
}


/* test code 
 * save the reconstructure result to a bundle file
 */
bool saveAsBundleFileOnePair( const char* filename,
							  SImageVisualFeatureMatchSet &match_set,
							  vpgl_perspective_camera<double> &cam1,
							  vpgl_perspective_camera<double> &cam2,
							  vector< vgl_point_3d<double> > &points_3d )
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for written!\n", filename);
		return false;
	}
	fprintf(file, "# Bundle file v1.0, Reconstruction Result of *ONE* Image Pair\n");

	//save the number of cameras and points
	fprintf(file, "2 %zd\n", points_3d.size());

	//save camera parameters
	vnl_matrix_fixed<double, 3, 3> K1 = cam1.get_calibration().get_matrix();
	fprintf(file, "%15.15f 0.0 0.0\n", (K1[0][0] + K1[1][1])/2.0);
	vnl_matrix_fixed<double, 3, 3> R1 = cam1.get_rotation().as_matrix();
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			fprintf(file, "%15.15f ", R1[i][j]);
		}
		fprintf(file, "\n");
	}

	vgl_vector_3d<double> T1 = cam1.get_translation();
	fprintf(file, "%15.15f %15.15f %15.15f\n", T1.x(), T1.y(), T1.z());

	vnl_matrix_fixed<double, 3, 3> K2 = cam2.get_calibration().get_matrix();
	fprintf(file, "%15.15f 0.0 0.0\n", (K2[0][0] + K2[1][1])/2.0);
	vnl_matrix_fixed<double, 3, 3> R2 = cam2.get_rotation().as_matrix();
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			fprintf(file, "%15.15f ", R2[i][j]);
		}
		fprintf(file, "\n");
	}

	vgl_vector_3d<double> T2 = cam2.get_translation();
	fprintf(file, "%15.15f %15.15f %15.15f\n", T2.x(), T2.y(), T2.z());

	//save 3d point data
	//assert(points_3d.size() == match_set.num_features());

	//vil_image_view<vxl_byte> img1 = match_set.image1.source->get_view();
	//vil_image_view<vxl_byte> img2 = match_set.image2.source->get_view();
	for(unsigned int i=0; i<points_3d.size(); i++)
	{
		//3D point position
		fprintf(file, "%15.15f %15.15f %15.15f\n", points_3d[i].x(), points_3d[i].y(), points_3d[i].z());

		vgl_point_2d<double> loc1 = match_set.matches[i].first->point;
		vgl_point_2d<double> loc2 = match_set.matches[i].second->point;
		
		//vil_rgb<vxl_byte> color1 = img1(i, j, 0);

		//point color
		fprintf(file, "%d %d %d\n", 125, 125, 125);

		//track
		fprintf(file, "2 0 0 %f %f 1 0 %f %f\n", loc1.x(), loc1.y(), loc2.x(), loc2.y());
	}

	fclose(file);

	return true;
}

