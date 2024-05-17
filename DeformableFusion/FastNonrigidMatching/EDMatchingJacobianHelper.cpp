#include "stdafx.h"
#include "EDMatchingJacobianHelper.h"
#include "VoxelMatrix.h"
#include "normal_estimation.h"
#include "DeformGraph.h"

#define VOL_FUSION_WEIGHT_MAX 10.0

using namespace NonrigidMatching;

bool EDMatchingJacobianHelper::
volumetric_fusion(DDF &ddf, NonrigidMatching::DeformGraph const&graph,
vector<cv::Mat> depthMats, vector<GCameraView*> cams_d,
vector<cv::Mat> imgs, vector<GCameraView*> cams_clr)
{
	int views_num = depthMats.size();
	//estimate the normal
	vector<CDepthMap> depth_maps(views_num);
	for (int i = 0; i < views_num; i++)
	{
		vnl_matrix<double> depthMat_vnl;
		cvmat_to_vnlmatrix(depthMats[i], depthMat_vnl);
		vpgl_perspective_camera<double> cam_d;
		GCameraView_to_vpgl_camera_view(cams_d[i], cam_d);
	}

	vector<DeformGraphNode> const&ed_nodes = graph.nodes;
	double nodes_min_dist = graph.nodes_dist;

	vector < vnl_matrix_fixed<double, 3, 3>> Rds(cams_d.size());
	vector< vnl_vector_fixed<double, 3>> Tds(cams_d.size());
	vector < vnl_matrix_fixed<double, 3, 3>> Ks(cams_d.size());
	for (int i = 0; i < cams_d.size(); i++)
	{
		get_camera_pose(*(cams_d[i]), Rds[i], Tds[i]);
		get_calibration_matrix(*(cams_d[i]), Ks[i]);
	}

	vector < vnl_matrix_fixed<double, 3, 3>> Rcs(cams_clr.size());
	vector< vnl_vector_fixed<double, 3>> Tcs(cams_clr.size());
	vector < vnl_matrix_fixed<double, 3, 3>> Kcs(cams_clr.size());
	for (int i = 0; i < cams_d.size(); i++)
	{
		get_camera_pose(*(cams_clr[i]), Rcs[i], Tcs[i]);
		get_calibration_matrix(*(cams_clr[i]), Kcs[i]);
	}

	vector<CDepthMap> depthMaps(cams_d.size());
	for (int i = 0; i < cams_d.size(); i++)
	{
		LOGGER()->debug("normal: %d/%zd\r", i, cams_d.size());
		vnl_matrix<double> depthMat_vnl;
		cvmat_to_vnlmatrix(depthMats[i], depthMat_vnl);
		local_plane_estimation_for_depthMap2(depthMat_vnl, Ks[i], depthMaps[i], 3, 3.0, 1, 7, false);
	}

	//find the voxels that neear current zero crossings
	int r = ceil(1.2*ddf.mu/ddf.xres);
	VoxelMatrix<char> voxels_todo(ddf.nx, ddf.ny, ddf.nz, 0, 0);
	voxels_todo.fill(0);
	__tic__();
	for (int k = 1; k < ddf.nz-1; k++)
	{
		LOGGER()->debug("k: %03d/%03d\r", k, ddf.nz);
#pragma omp parallel for schedule(dynamic)
		for (int j = 1; j < ddf.ny-1; j++)
		{
			for (int i = 1; i < ddf.nx-1; i++)
			{
				float val = ddf.func_val(i, j, k);
				if (val == SDF_NULL_VALUE)
					continue;

				if (ddf.func_val(i - 1, j, k)*val <= 0.0 ||
					ddf.func_val(i + 1, j, k)*val <= 0.0 ||
					ddf.func_val(i, j - 1, k)*val <= 0.0 ||
					ddf.func_val(i, j + 1, k)*val <= 0.0 ||
					ddf.func_val(i, j, k - 1)*val <= 0.0 ||
					ddf.func_val(i, j, k + 1)*val <= 0.0
					)
				{
					for (int kk = k - r; kk <= k + r; kk++)
					for (int jj = j - r; jj <= j + r; jj++)
					for (int ii = i - r; ii <= i + r; ii++)
					{
						if ((kk - k)*(kk - k) + (jj - j)*(jj - j) + (ii - i)*(ii - i) > r*r ||
							kk < 0 || kk >= ddf.nz ||
							jj < 0 || jj >= ddf.ny ||
							ii < 0 || ii >= ddf.nx)
							continue;
						voxels_todo(ii, jj, kk) = 1;
					}
				}
			}
		}
	}
	LOGGER()->debug("Time = %f", __toc__());


	__tic__();
	int pt_count = -1;
	double mu_cur = ddf.mu;
	for (int k = 0; k < ddf.nz; k++)
	{
		LOGGER()->debug("k: %03d/%03d\r", k, ddf.nz);
		double z = k*ddf.zres + ddf.zoffset;
#pragma omp parallel for schedule(dynamic)
		for (int j = 0; j < ddf.ny; j++)
		{
			double y = j*ddf.yres + ddf.yoffset;	

			for (int i = 0; i < ddf.nx; i++)
			{
				double x = i*ddf.xres + ddf.xoffset;
				vnl_vector_fixed<double, 3> v(x, y, z);

				if (!voxels_todo(i, j, k))
					continue;

				//find 4 nearest neighboring nodes
				double dists[4];
				int nd_idxs[4];
				for (int c = 0; c < 4; c++)
				{
					dists[c] = dist_3d(v, ed_nodes[c].g);
					nd_idxs[c] = c;
				}
				for (int m = 4; m < ed_nodes.size(); m++)
				{
					double dist_new = dist_3d(v, ed_nodes[m].g);
					int nd_idx_new = m;

					for (int c = 0; c < 4; c++)
					{
						if (dist_new < dists[c])
						{
							std::swap(dist_new, dists[c]);
							std::swap(nd_idx_new, nd_idxs[c]);
						}
					}
				}
				double ws[4];
				double w_sum = 0.0;
				for (int c = 0; c < 4; c++)
				{
					ws[c] = exp(-(dists[c] * dists[c]) / (2 * nodes_min_dist*nodes_min_dist));
					w_sum += ws[c];
				}

				for (int c = 0; c < 4; c++)
					ws[c] /= w_sum;

				NeighborGraphNodesOfPoint ngn;
				ngn.neighborIndices = vector<int>(nd_idxs, nd_idxs + 4);
				ngn.weights = vector<double>(ws, ws + 4);

				//deform the voxel nonrigidly
				vnl_vector_fixed<double, 3> vt;
				transform_one_point_with_DeformGraph(v, vt, graph, ngn, true);

				vnl_vector_fixed<double, 3> n(0.0);
				bool bSDFNormal = ddf.func_val.mid_der_at(i, j, k, n.data_block());
				n *= -1; //make it pointing inward
				n.normalize();
				if (!bSDFNormal)
					n.fill(0.0);


				vnl_vector_fixed<double, 3> nt(0.0);
				transform_one_normal_with_DeformGraph(n, nt, graph, ngn);
				
				if (isnan(nt[0]))
					LOGGER()->debug("nan!\n");
		

				vector<double> weights(cams_d.size());
				vector<double> sdf_vals(cams_d.size());
				for (int c = 0; c < cams_d.size(); c++)
				{
					weights[c] = 0.0;
					sdf_vals[c] = 0.0;

					double const& fx = Ks[c][0][0];
					double const& fy = Ks[c][1][1];
					double const& cx = Ks[c][0][2];
					double const& cy = Ks[c][1][2];

					vnl_vector_fixed<double, 3> nt_cam = Rds[c] * nt;

					bool bColorDepthAligned = false;
					if (cams_d[c] == cams_clr[c])
						bColorDepthAligned = true;

					cv::Mat img = imgs.size() > c ? imgs[c] : cv::Mat();
					cv::Mat depthMat = depthMats[c];

					vnl_vector_fixed<double, 3> X_cam = Rds[c] * vt + Tds[c];
					int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
					int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

					//if the point is behind the camera, then skip it
					if (X_cam[2] <= 0)
						continue;

					// if the point cannot be observed at the current camera pose
					if (u < 0 || u >= depthMat.cols ||
						v < 0 || v >= depthMat.rows)
						continue;

					//TODO: mask from multiview
					double ds = depthMat.at<double>(v, u);;
					if (ds <= 0)
					{
						//if no depth measurement of the surface at this point
						sdf_vals[c] = 0.0;// empty space: cannot use the depth, segmentation maybe? 
						weights[c] = 0.0;
					}
					else
					{
						double sdf_val = ds - X_cam[2];

						//if the point is far away from the observed surface 
						if (sdf_val <= -mu_cur)
						{
							sdf_vals[c] = 0.0;
							weights[c] = 0.0;
						}
						else if (sdf_val >= mu_cur)
						{
							//sdf_vals[c] = 1.0;
							sdf_vals[c] = 0.0;
							weights[c] = 0.0;// 1.0;
						}
						else
						{
							sdf_vals[c] = sdf_val / mu_cur;//*this->mu;
							weights[c] = 1.0;

							//modulate weight with depth normal, and normal vector at voxel
							DepthElement &ele = depthMaps[c][v][u];
							if (ele.bNormal)
							{
								vnl_vector_fixed<double, 3> ray = ele.P_cam;
								ray.normalize();
								double a = MAX(0.0, dot_product(ray, ele.normal));
								weights[c] *= (a*a);

								if (bSDFNormal)
								{
									//double b = dot_product(nt_cam, ele.normal); //acos(0.2) = 78.5 degree
									//if (b <= -0.3)	weights[c] = 0.0;
									double b = MAX(0.0, dot_product(nt_cam, ele.normal));
									weights[c] *= b*b;
								}								
							}
							else
							{
								weights[c] = 0.0;
							}
						}
					}
				}//end of for-cam

				double val_cur = 0.0;
				double weight_cur = 0.0;
				bool bEmptySpaceObserved = false;
				for (int c = 0; c < sdf_vals.size(); c++)
				{
					if (sdf_vals[c] >= 1.0)
					{
						bEmptySpaceObserved = true;
						continue;
					}
					val_cur += (sdf_vals[c] * weights[c]);
					weight_cur += weights[c];
				}

				if (weight_cur > 0.0)
				{
					val_cur /= weight_cur;
				}
				else if (bEmptySpaceObserved)
				{
					//empty space
					val_cur = 1.0;
					weight_cur = 0.5;
				}

				// if the function value at current point is NULL
				double tmp = ddf.func_val(i, j, k);
				if (ddf.func_val(i, j, k) == SDF_NULL_VALUE)
				{
					if (weight_cur > 0.0)
					{
						ddf.func_val(i, j, k) = val_cur;
						ddf.weights(i, j, k) = weight_cur;
					}
				}
				else
				{
					ddf.func_val(i, j, k) = (ddf.func_val(i, j, k)*ddf.weights(i, j, k) + val_cur*weight_cur) / (weight_cur + ddf.weights(i, j, k));
					ddf.weights(i, j, k) = MIN(ddf.weights(i, j, k) + weight_cur, VOL_FUSION_WEIGHT_MAX);
				}

			}
		}
	}
	LOGGER()->debug("Time = %f", __toc__());

	return true;
}

bool EDMatchingJacobianHelper::
allocate_vertices_and_graph_edges(  NonrigidMatching::DeformGraph const& graph,
									NonrigidMatching::NeighborGraphNodesOfPointList const& ngns,
									vnl_matrix<HessianBlockRelatedData*> &hessian_related_data
									)
{
	int nds_num = graph.nodes.size();
	hessian_related_data.set_size(nds_num, nds_num);
	hessian_related_data.fill(NULL);

	for (int vtIdx = 0; vtIdx < ngns.size(); vtIdx++)
	{
		vector<int> const& nd_indices_nn = ngns[vtIdx].neighborIndices;
		vector<double> const& nd_weights_nn = ngns[vtIdx].weights;
		for (int i = 0; i < nd_indices_nn.size(); i++)
		{
			short ndIdx_i = nd_indices_nn[i];
			float weight_i = nd_weights_nn[i];
			for (int j = i; j < nd_indices_nn.size(); j++)
			{
				short ndIdx_j = nd_indices_nn[j];
				float weight_j = nd_weights_nn[j];

				int ndIdx_1 = ndIdx_i;
				int ndIdx_2 = ndIdx_j;
				float weight_1 = weight_i;
				float weight_2 = weight_j;
				if (ndIdx_1 < ndIdx_2)
				{
					ndIdx_1 = ndIdx_j;
					ndIdx_2 = ndIdx_i;
					weight_1 = weight_j;
					weight_2 = weight_i;
				}

				if (hessian_related_data[ndIdx_1][ndIdx_2] == NULL)
				{
					hessian_related_data[ndIdx_1][ndIdx_2] = new HessianBlockRelatedData();
					hessian_related_data[ndIdx_1][ndIdx_2]->vt_indices = new vector<int>();
					hessian_related_data[ndIdx_1][ndIdx_2]->weights = new vector<pair<float, float>>();
					hessian_related_data[ndIdx_1][ndIdx_2]->bGraphEdge = false;
				}

				hessian_related_data[ndIdx_1][ndIdx_2]->vt_indices->push_back(vtIdx);
				hessian_related_data[ndIdx_1][ndIdx_2]->weights->push_back(make_pair(weight_1, weight_2));
			}
		}
	}

	for (int ndIdx = 0; ndIdx < graph.nodes.size(); ndIdx++)
	{
		vector<int> const& neighbors = graph.nodes[ndIdx].neighborIndices;
		for (int k = 0; k < neighbors.size(); k++)
		{
			int ndIdx_n = neighbors[k];
			int ndIdx_1 = MAX(ndIdx, ndIdx_n);
			int ndIdx_2 = MIN(ndIdx, ndIdx_n);

			if (hessian_related_data[ndIdx_1][ndIdx_2] == NULL)
				hessian_related_data[ndIdx_1][ndIdx_2] = new HessianBlockRelatedData();
			hessian_related_data[ndIdx_1][ndIdx_2]->bGraphEdge = true;
		}
	}

	return true;
}

bool EDMatchingJacobianHelper::
init_hessian_blocks(vnl_matrix<HessianBlockRelatedData*> &hessian_related_data,
					BlockedHessianMatrix &hessian,
					vnl_vector<float> & b,
					int* &vt_indices,
					WeightsPair *&vt_node_weights,
					int& vt_indices_count)
{
	const int vt_num_thres_per_block = 300;

	int blk_num = 0;
	int vti_num = 0;
	for (int i = 0; i < hessian_related_data.rows(); i++)
	{
		for (int j = 0; j <= i; j++)
		{
			HessianBlockRelatedData* h_info = hessian_related_data[i][j];
			//delete blocks with few vertices associated with it
			if (hessian_related_data[i][j] &&
				hessian_related_data[i][j]->vt_indices &&
				hessian_related_data[i][j]->vt_indices->size() <= vt_num_thres_per_block &&
				i != j)
			{
				delete hessian_related_data[i][j]->vt_indices;
				delete hessian_related_data[i][j]->weights;
				hessian_related_data[i][j]->vt_indices = NULL;
				hessian_related_data[i][j]->weights = NULL;

				if (!hessian_related_data[i][j]->bGraphEdge)
				{
					delete hessian_related_data[i][j];
					hessian_related_data[i][j] = NULL;
				}
			}

			//diagonal block is always on
			if (hessian_related_data[i][j] ||
				i == j)
				blk_num++;

			if (hessian_related_data[i][j] &&
				hessian_related_data[i][j]->vt_indices)
				vti_num += hessian_related_data[i][j]->vt_indices->size();
		}
	}

	hessian.blk_num = blk_num;
	hessian.blocks = new HessianBlock[blk_num];
	hessian.allocate_space_equal_block_size(12);

	hessian.para_blk_num = hessian_related_data.rows();
	vt_indices = new int[vti_num];
	vt_node_weights = new WeightsPair[vti_num];
	vt_indices_count = vti_num;

	b.set_size(hessian_related_data.rows() * 12);
	b.fill(0.0);

	//first allocate space for diagnal blocks
	int vti_count = 0;
	int blk_count = 0;
	HessianBlock *hessian_blocks = hessian.blocks;
	for (int i = 0; i < hessian_related_data.rows(); i++)
	{
		hessian_blocks[blk_count].row = i;
		hessian_blocks[blk_count].col = i;
		hessian_blocks[blk_count].bGraphEdge = false;

		hessian_blocks[blk_count].vtii_first = vti_count;
		hessian_blocks[blk_count].vts_num = 0;

		if (hessian_related_data[i][i] &&
			hessian_related_data[i][i]->vt_indices)
		{
			vector<int> const& vt_indices_cur = *(hessian_related_data[i][i]->vt_indices);
			std::copy(vt_indices_cur.begin(), vt_indices_cur.end(), vt_indices + vti_count);
			hessian_blocks[blk_count].vts_num = vt_indices_cur.size();

			vector<pair<float, float>> const& weights_cur = *(hessian_related_data[i][i]->weights);
			for (int k = 0; k < weights_cur.size(); k++)
			{
				vt_node_weights[vti_count + k].w1 = weights_cur[k].first;
				vt_node_weights[vti_count + k].w2 = weights_cur[k].second;
			}

			vti_count += vt_indices_cur.size();
		}

		blk_count++;
	}

	//off-diagonal data
	for (int i = 0; i < hessian_related_data.rows(); i++)
	{
		for (int j = 0; j < i; j++)
		{
			if (hessian_related_data[i][j] == NULL)
				continue;

			hessian_blocks[blk_count].row = i;
			hessian_blocks[blk_count].col = j;
			hessian_blocks[blk_count].bGraphEdge = hessian_related_data[i][j]->bGraphEdge;

			hessian_blocks[blk_count].vtii_first = vti_count;
			hessian_blocks[blk_count].vts_num = 0;

			if (hessian_related_data[i][j] &&
				hessian_related_data[i][j]->vt_indices)
			{
				vector<int> const& vt_indices_cur = *(hessian_related_data[i][j]->vt_indices);
				std::copy(vt_indices_cur.begin(), vt_indices_cur.end(), vt_indices + vti_count);
				hessian_blocks[blk_count].vts_num = vt_indices_cur.size();

				vector<pair<float, float>> const& weights_cur = *(hessian_related_data[i][j]->weights);
				for (int k = 0; k < weights_cur.size(); k++)
				{
					vt_node_weights[vti_count + k].w1 = weights_cur[k].first;
					vt_node_weights[vti_count + k].w2 = weights_cur[k].second;
				}

				vti_count += vt_indices_cur.size();
			}

			blk_count++;
		}
	}
	assert(vti_count = vti_num && blk_count == blk_num);
	return true;
}

double EDMatchingJacobianHelper::
evaluate_data_term( CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K,
					float w_data)
{
	const float mu = 1.0;
	float fx = K[0][0];
	float fy = K[1][1];
	float cx = K[0][2];
	float cy = K[1][2];
	int depth_width = depthMat_trg.cols;
	int depth_height = depthMat_trg.rows;
	float w_data_sqrt = std::sqrt(w_data);

	vector<float> costs(surface_t.vtNum, 0.0);

#pragma omp parallel for
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		vnl_vector_fixed<float, 3> v_t(surface_t.vt_data_block(vtIdx));
		vnl_vector_fixed<float, 3> n_t(surface_t.vt_normal(vtIdx));

		//if the point is behind the camera, then skip it
		if (v_t[2] <= 0.0)
			continue;

		int u = ROUND(fx*v_t[0] / v_t[2] + cx);
		int v = ROUND(fy*v_t[1] / v_t[2] + cy);

		// if the point cannot be observed at the current camera pose
		if (u < 0 || u >= depth_width ||
			v < 0 || v >= depth_height)
			continue;

		float d_s = depthMat_trg.at<double>(v, u);;
		if (d_s <= 0)
			continue;

		vnl_vector_fixed<float, 3> p;
		p[0] = (u - cx)*d_s / fx;
		p[1] = (v - cy)*d_s / fy;
		p[2] = d_s;

		//cost
		float f = MIN(mu, MAX(-mu, dot_product(n_t, p - v_t)));

		f *= w_data_sqrt;
		costs[vtIdx] = f*f;
	}

	double cost = 0.0;
	for (int i = 0; i < costs.size(); i++)
		cost += costs[i];

	return cost;
}


double EDMatchingJacobianHelper::
evaluate_data_term( CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,					
					vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
					float w_data, float mu)
{
	int depth_width = depthMats_trg[0].cols;
	int depth_height = depthMats_trg[0].rows;
	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> R = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> T = graph.global_rigid.t;

	vector<vnl_matrix_fixed<double, 3, 3>> Rs(cams.size());
	vector<vnl_vector_fixed<double, 3>> Ts(cams.size());
	vector<vnl_matrix_fixed<double, 3, 3>> Ks(cams.size());
	for (int i = 0; i < cams.size(); i++)
	{
		get_camera_pose(*(cams[i]), Rs[i], Ts[i]);
		get_calibration_matrix(*(cams[i]), Ks[i]);
	}

	vector<float> costs(surface_t.vtNum, 0.0);

#pragma omp parallel for
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		float const*v_t_ = surface_t.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
		float const*n_t_ = surface_t.vt_normal(vtIdx);
		vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);

		vnl_vector_fixed<double, 3> p_avg(0.0);
		int count = 0;
		for (int vId = 0; vId < cams.size(); vId++)
		{
			vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

			//if the point is behind the camera, then skip it
			if (X_cam[2] <= 0)
				continue;

			float const&fx = Ks[vId][0][0];
			float const&fy = Ks[vId][1][1];
			float const&cx = Ks[vId][0][2];
			float const&cy = Ks[vId][1][2];

			int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
			int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

			// if the point cannot be observed at the current camera pose
			if (u < 0 || u >= depth_width ||
				v < 0 || v >= depth_height)
				continue;

			float d_s = depthMats_trg[vId].at<double>(v, u);
			if (d_s <= 0)
				continue;

			vnl_vector_fixed<double, 3> p;
			p[0] = (u - cx)*d_s / fx;
			p[1] = (v - cy)*d_s / fy;
			p[2] = d_s;

			p = Rs[vId].transpose()*(p - Ts[vId]);
			double f = dot_product(n_t, p - v_t);
			if (std::abs(f) < mu)
			{
				p_avg += p;
				count++;
			}
		}
		
		if (count > 0)
		{
			p_avg /= count;

			//cost
			float f = dot_product(n_t, p_avg - v_t);

			f *= w_data_sqrt;
			costs[vtIdx] = f*f;
		}
	}

	double cost = 0.0;
	for (int i = 0; i < costs.size(); i++)
		cost += costs[i];

	return cost;
}

double EDMatchingJacobianHelper::
evaluate_data_term2(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					vector<int2> & vts_mask,
					vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
					float w_data, float mu, int frmIdx, int iter)
{
	int depth_width = depthMats_trg[0].cols;
	int depth_height = depthMats_trg[0].rows;
	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> R = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> T = graph.global_rigid.t;

	vector<vnl_matrix_fixed<double, 3, 3>> Rs(cams.size());
	vector<vnl_vector_fixed<double, 3>> Ts(cams.size());
	vector<vnl_matrix_fixed<double, 3, 3>> Ks(cams.size());
	for (int i = 0; i < cams.size(); i++)
	{
		get_camera_pose(*(cams[i]), Rs[i], Ts[i]);
		get_calibration_matrix(*(cams[i]), Ks[i]);
	}

	vector<float> costs(surface_t.vtNum, 0.0);
	vector<int> counts(surface_t.vtNum, 0);
	vector<bool> bHasCorrs(surface_t.vtNum, false);

#pragma omp parallel for
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		float const*v_t_ = surface_t.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
		float const*n_t_ = surface_t.vt_normal(vtIdx);
		vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);		

		for (int vId = 0; vId < cams.size(); vId++)
		{
			vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

			//if the point is behind the camera, then skip it
			if (X_cam[2] <= 0)
				continue;

			float const&fx = Ks[vId][0][0];
			float const&fy = Ks[vId][1][1];
			float const&cx = Ks[vId][0][2];
			float const&cy = Ks[vId][1][2];

			int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
			int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

			// if the point cannot be observed at the current camera pose
			if (u < 0 || u >= depth_width ||
				v < 0 || v >= depth_height)
				continue;

			float d_s = depthMats_trg[vId].at<double>(v, u);
			if (d_s <= 0)
				continue;

			vnl_vector_fixed<double, 3> p;
			p[0] = (u - cx)*d_s / fx;
			p[1] = (v - cy)*d_s / fy;
			p[2] = d_s;
			p = Rs[vId].transpose()*(p - Ts[vId]);

			//cost
			if (dist_3d(p, v_t) > mu)
				continue;

			float f = dot_product(n_t, p - v_t);
			f *= w_data_sqrt;
			costs[vtIdx] += f*f;
			counts[vtIdx]++;

			bHasCorrs[vtIdx] = true;
		}
	}

	double cost = 0.0;
	int count = 0;
	for (int i = 0; i < costs.size(); i++)
	{
		cost += costs[i];
		count += counts[i];
	}

	return cost/count;
}

double EDMatchingJacobianHelper::
evaluate_data_term3(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
					float w_data, float mu)
{
	double cost_sum = 0;
	for (int i = 0; i < cams.size(); i++)
	{
		vector<cv::Mat> depthMats_one;
		depthMats_one.push_back(depthMats_trg[i]);
		vector<GCameraView*> cams_one;
		cams_one.push_back(cams[i]);

		cost_sum += evaluate_data_term(surface, graph, surface_t, depthMats_one, cams_one, w_data, mu);
	}

	return cost_sum;
}

bool EDMatchingJacobianHelper::
fill_jacobian_data(vector<Triplet> &jac, CSurface<float> const& surface,
					NonrigidMatching::DeformGraph const& graph,
					NonrigidMatching::NeighborGraphNodesOfPointList const& ngns,
					CSurface<float> const& surface_t,
					cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K,
					float w_data)
{
	const float mu = 1.0;
	float fx = K[0][0];
	float fy = K[1][1];
	float cx = K[0][2];
	float cy = K[1][2];
	int depth_width = depthMat_trg.cols;
	int depth_height = depthMat_trg.rows;

	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> Rd = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> Td = graph.global_rigid.t;
	vnl_matrix_fixed<float, 3, 3> R;
	R[0][0] = Rd[0][0]; R[0][1] = Rd[0][1]; R[0][2] = Rd[0][2];
	R[1][0] = Rd[1][0]; R[1][1] = Rd[1][1]; R[1][2] = Rd[1][2];
	R[2][0] = Rd[2][0]; R[2][1] = Rd[2][1]; R[2][2] = Rd[2][2];
	vnl_vector_fixed<float, 3> T(Td[0], Td[1], Td[2]);

	int count = 0;
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		vnl_vector_fixed<float, 3> v_t(surface_t.vt_data_block(vtIdx));
		vnl_vector_fixed<float, 3> n_t(surface_t.vt_normal(vtIdx));
		vnl_vector_fixed<float, 3> vt(surface.vt_data_block(vtIdx));

		//if the point is behind the camera, then skip it
		if (v_t[2] <= 0)
			continue;

		int u = ROUND(fx*v_t[0] / v_t[2] + cx);
		int v = ROUND(fy*v_t[1] / v_t[2] + cy);

		// if the point cannot be observed at the current camera pose
		if (u < 0 || u >= depth_width ||
			v < 0 || v >= depth_height)
			continue;

		float d_s = depthMat_trg.at<double>(v, u);;
		if (d_s <= 0)
			continue;

		vnl_vector_fixed<float, 3> p;
		p[0] = (u - cx)*d_s / fx;
		p[1] = (v - cy)*d_s / fy;
		p[2] = d_s;

		//cost
		float f = dot_product(n_t, p - v_t);
		if (abs(f) > mu)
			continue;

		f *= w_data_sqrt;

		vnl_vector_fixed<float, 3> df_dvtn = -n_t*R;

		NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];

		for (int i = 0; i < ngn.neighborIndices.size(); i++)
		{
			int ndIdx_i = ngn.neighborIndices[i];
			DeformGraphNode const&nd = graph.nodes[ndIdx_i];
			vnl_vector_fixed<double, 3> const& gi = nd.g;
			double w_i = ngn.weights[i] * w_data_sqrt;
			//jacobian
			vnl_vector_fixed<float, 12> df_dpi;
			df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
			df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
			df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
			df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
			df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
			df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
			df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
			df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
			df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
			df_dpi[9] = df_dvtn[0] * w_i;
			df_dpi[10] = df_dvtn[1] * w_i;
			df_dpi[11] = df_dvtn[2] * w_i;

			for (int k = 0; k < 12; k++)
				jac.push_back(Triplet(count, ndIdx_i * 12 + k, df_dpi[k]));
		}
		count++;
	}

	return true;
}

bool EDMatchingJacobianHelper::
fill_hessians_data( BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
					int const* vt_indices, WeightsPair const* vt_node_weights,
					CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K, float w_data)
{
	const float mu = 1.0;
	float fx = K[0][0];
	float fy = K[1][1];
	float cx = K[0][2];
	float cy = K[1][2];
	int depth_width = depthMat_trg.cols;
	int depth_height = depthMat_trg.rows;

	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> Rd = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> Td = graph.global_rigid.t;
	vnl_matrix_fixed<float, 3, 3> R;
	R[0][0] = Rd[0][0]; R[0][1] = Rd[0][1]; R[0][2] = Rd[0][2];
	R[1][0] = Rd[1][0]; R[1][1] = Rd[1][1]; R[1][2] = Rd[1][2];
	R[2][0] = Rd[2][0]; R[2][1] = Rd[2][1]; R[2][2] = Rd[2][2];
	vnl_vector_fixed<float, 3> T(Td[0], Td[1], Td[2]);

	int blk_num = hessian.blk_num;
	int para_blk_num = hessian.para_blk_num;
	HessianBlock *hessian_blocks = hessian.blocks;

	assert(para_blk_num * 12 == b.size());

	vnl_vector<float> costs_per_block;
	costs_per_block.set_size(para_blk_num);
	costs_per_block.fill(0.0);

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < para_blk_num; i++)
	{
		//parameter block index
		int piIdx = hessian_blocks[i].row;
		int pjIdx = hessian_blocks[i].col;
		assert(piIdx == pjIdx);
		float *h_val = &(hessian_blocks[i].vals[0]);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		float cost_cur = 0.0;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			vnl_vector_fixed<float, 3> v_t(surface_t.vt_data_block(vtIdx));
			vnl_vector_fixed<float, 3> n_t(surface_t.vt_normal(vtIdx));
			vnl_vector_fixed<float, 3> vt(surface.vt_data_block(vtIdx));

			//if the point is behind the camera, then skip it
			if (v_t[2] <= 0)
				continue;

			int u = ROUND(fx*v_t[0] / v_t[2] + cx);
			int v = ROUND(fy*v_t[1] / v_t[2] + cy);

			// if the point cannot be observed at the current camera pose
			if (u < 0 || u >= depth_width ||
				v < 0 || v >= depth_height)
				continue;

			float d_s = depthMat_trg.at<double>(v, u);;
			if (d_s <= 0)
				continue;

			vnl_vector_fixed<float, 3> p;
			p[0] = (u - cx)*d_s / fx;
			p[1] = (v - cy)*d_s / fy;
			p[2] = d_s;

			//cost
			float f = dot_product(n_t, p - v_t);
			if (abs(f) > mu)
				continue;

			f *= w_data_sqrt;
			cost_cur += f*f;

			double w_i = vt_node_weights[k].w1 * w_data_sqrt;
			vnl_vector_fixed<float, 3> df_dvtn = -n_t*R;

			//jacobian
			vnl_vector_fixed<float, 12> df_dpi;
			df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
			df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
			df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
			df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
			df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
			df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
			df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
			df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
			df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
			df_dpi[9] = df_dvtn[0] * w_i;
			df_dpi[10] = df_dvtn[1] * w_i;
			df_dpi[11] = df_dvtn[2] * w_i;

			//gradient
			for (int m = 0; m < 12; m++)
				b[i * 12 + m] += f*df_dpi[m];

			//hessian
			for (int ii = 0; ii < 12; ii++)
			for (int jj = 0; jj < 12; jj++)
				h_val[ii * 12 + jj] += df_dpi[ii] * df_dpi[jj];
		}

		costs_per_block[i] = cost_cur;
	}

	if (cost != NULL)
	{
		(*cost) += costs_per_block.sum() / 4;
	}

	//=====dealing with cross-talks between different parameter blocks
#pragma omp parallel for schedule(dynamic)
	for (int i = para_blk_num; i < blk_num; i++)
	{
		//parameter block index
		HessianBlock &blk = hessian_blocks[i];
		int piIdx = blk.row;
		int pjIdx = blk.col;
		assert(piIdx > pjIdx);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<double, 3> const& gjd = graph.nodes[pjIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);
		vnl_vector_fixed<float, 3> gj(gjd[0], gjd[1], gjd[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			vnl_vector_fixed<float, 3> v_t(surface_t.vt_data_block(vtIdx));
			vnl_vector_fixed<float, 3> n_t(surface_t.vt_normal(vtIdx));
			vnl_vector_fixed<float, 3> vt(surface.vt_data_block(vtIdx));

			//if the point is behind the camera, then skip it
			if (v_t[2] <= 0)
				continue;

			int u = ROUND(fx*v_t[0] / v_t[2] + cx);
			int v = ROUND(fy*v_t[1] / v_t[2] + cy);

			// if the point cannot be observed at the current camera pose
			if (u < 0 || u >= depth_width ||
				v < 0 || v >= depth_height)
				continue;

			float d_s = depthMat_trg.at<double>(v, u);;
			if (d_s <= 0.0)
				continue;

			vnl_vector_fixed<float, 3> p;
			p[0] = (u - cx)*d_s / fx;
			p[1] = (v - cy)*d_s / fy;
			p[2] = d_s;

			//cost
			float f = dot_product(n_t, p - v_t);
			if (abs(f) > mu)
				continue;
			f *= w_data_sqrt;

			double w_i = vt_node_weights[k].w1 * w_data_sqrt;
			double w_j = vt_node_weights[k].w2 * w_data_sqrt;
			vnl_vector_fixed<float, 3> df_dvtn = -n_t*R;

			//jocobian
			vnl_vector_fixed<float, 12> df_dpi;
			df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
			df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
			df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
			df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
			df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
			df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
			df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
			df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
			df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
			df_dpi[9] = df_dvtn[0] * w_i;
			df_dpi[10] = df_dvtn[1] * w_i;
			df_dpi[11] = df_dvtn[2] * w_i;

			//jocobian
			vnl_vector_fixed<float, 12> df_dpj;
			df_dpj[0] = df_dvtn[0] * (vt[0] - gj[0])*w_j;
			df_dpj[1] = df_dvtn[0] * (vt[1] - gj[1])*w_j;
			df_dpj[2] = df_dvtn[0] * (vt[2] - gj[2])*w_j;
			df_dpj[3] = df_dvtn[1] * (vt[0] - gj[0])*w_j;
			df_dpj[4] = df_dvtn[1] * (vt[1] - gj[1])*w_j;
			df_dpj[5] = df_dvtn[1] * (vt[2] - gj[2])*w_j;
			df_dpj[6] = df_dvtn[2] * (vt[0] - gj[0])*w_j;
			df_dpj[7] = df_dvtn[2] * (vt[1] - gj[1])*w_j;
			df_dpj[8] = df_dvtn[2] * (vt[2] - gj[2])*w_j;
			df_dpj[9] = df_dvtn[0] * w_j;
			df_dpj[10] = df_dvtn[1] * w_j;
			df_dpj[11] = df_dvtn[2] * w_j;

			//hessian
			for (int ii = 0; ii < 12; ii++)
			for (int jj = 0; jj < 12; jj++)
				blk.at(ii, jj) += df_dpi[ii] * df_dpj[jj];
		}
	}

	return true;
}

bool EDMatchingJacobianHelper::
fill_hessians_data( BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
					int const* vt_indices, WeightsPair const* vt_node_weights,
					CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams, float w_data,
					float mu)
{
	int depth_width = depthMats_trg[0].cols;
	int depth_height = depthMats_trg[0].rows;

	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> R = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> T = graph.global_rigid.t;

	vector<vnl_matrix_fixed<double, 3, 3>> Rs(cams.size());
	vector<vnl_vector_fixed<double, 3>> Ts(cams.size());
	vector<vnl_matrix_fixed<double, 3, 3>> Ks(cams.size());
	for (int i = 0; i < cams.size(); i++)
	{
		get_camera_pose(*(cams[i]), Rs[i], Ts[i]);
		get_calibration_matrix(*(cams[i]), Ks[i]);
	}

	int blk_num = hessian.blk_num;
	int para_blk_num = hessian.para_blk_num;
	HessianBlock *hessian_blocks = hessian.blocks;

	assert(para_blk_num * 12 == b.size());

	vnl_vector<float> costs_per_block;
	costs_per_block.set_size(para_blk_num);
	costs_per_block.fill(0.0);

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < para_blk_num; i++)
	{
		//parameter block index
		int piIdx = hessian_blocks[i].row;
		int pjIdx = hessian_blocks[i].col;
		assert(piIdx == pjIdx);
		float *h_val = &(hessian_blocks[i].vals[0]);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		float cost_cur = 0.0;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			float const*v_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
			float const*n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);
			float const*vt_ = surface.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

			vnl_vector_fixed<double, 3> p_avg(0.0);
			int count = 0;
			for (int vId = 0; vId < cams.size(); vId++)
			{
				vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];
				
				//if the point is behind the camera, then skip it
				if (X_cam[2] <= 0)
					continue;

				float const&fx = Ks[vId][0][0];
				float const&fy = Ks[vId][1][1];
				float const&cx = Ks[vId][0][2];
				float const&cy = Ks[vId][1][2];

				int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u < 0 || u >= depth_width ||
					v < 0 || v >= depth_height)
					continue;

				float d_s = depthMats_trg[vId].at<double>(v, u);
				if (d_s <= 0)
					continue;

				vnl_vector_fixed<double, 3> p;
				p[0] = (u - cx)*d_s / fx;
				p[1] = (v - cy)*d_s / fy;
				p[2] = d_s;

				p = Rs[vId].transpose()*(p - Ts[vId]);
				double f = dot_product(n_t, p - v_t);
				if (std::abs(f) < mu)
				{
					p_avg += p;
					count++;
				}
			}

			if (count > 0)
			{
				p_avg /= count;

				//cost
				float f = dot_product(n_t, p_avg - v_t);
				if (abs(f) > mu)
					continue;

				f *= w_data_sqrt;
				cost_cur += f*f;

				double w_i = vt_node_weights[k].w1 * w_data_sqrt;
				vnl_vector_fixed<double, 3> df_dvtn = -n_t*R;

				//jacobian
				vnl_vector_fixed<float, 12> df_dpi;
				df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
				df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
				df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
				df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
				df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
				df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
				df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
				df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
				df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;

				//gradient
				for (int m = 0; m < 12; m++)
					b[i * 12 + m] += f*df_dpi[m];

				//hessian
				for (int ii = 0; ii < 12; ii++)
				for (int jj = 0; jj < 12; jj++)
					h_val[ii * 12 + jj] += df_dpi[ii] * df_dpi[jj];
			}
		}

		costs_per_block[i] = cost_cur;
	}

	if (cost != NULL)
	{
		(*cost) += costs_per_block.sum() / 4;
	}

	//=====dealing with cross-talks between different parameter blocks
#pragma omp parallel for schedule(dynamic)
	for (int i = para_blk_num; i < blk_num; i++)
	{
		//parameter block index
		HessianBlock &blk = hessian_blocks[i];
		int piIdx = blk.row;
		int pjIdx = blk.col;

		assert(piIdx > pjIdx);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<double, 3> const& gjd = graph.nodes[pjIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);
		vnl_vector_fixed<float, 3> gj(gjd[0], gjd[1], gjd[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			float const*v_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
			float const*n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);
			float const*vt_ = surface.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

			vnl_vector_fixed<double, 3> p_avg(0.0);
			int count = 0;
			for (int vId = 0; vId < cams.size(); vId++)
			{
				vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

				//if the point is behind the camera, then skip it
				if (X_cam[2] <= 0)
					continue;

				float const&fx = Ks[vId][0][0];
				float const&fy = Ks[vId][1][1];
				float const&cx = Ks[vId][0][2];
				float const&cy = Ks[vId][1][2];

				int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u < 0 || u >= depth_width ||
					v < 0 || v >= depth_height)
					continue;

				float d_s = depthMats_trg[vId].at<double>(v, u);
				if (d_s <= 0)
					continue;

				vnl_vector_fixed<double, 3> p;
				p[0] = (u - cx)*d_s / fx;
				p[1] = (v - cy)*d_s / fy;
				p[2] = d_s;

				p = Rs[vId].transpose()*(p - Ts[vId]);
				double f = dot_product(n_t, p - v_t);
				if (std::abs(f) < mu)
				{
					count++;
					p_avg += p;
				}
			}

			if (count > 0)
			{
				p_avg /= count;

				//cost
				float f = dot_product(n_t, p_avg - v_t);
				if (abs(f) > mu)
					continue;
				f *= w_data_sqrt;

				double w_i = vt_node_weights[k].w1 * w_data_sqrt;
				double w_j = vt_node_weights[k].w2 * w_data_sqrt;
				vnl_vector_fixed<double, 3> df_dvtn = -n_t*R;

				//jocobian
				vnl_vector_fixed<float, 12> df_dpi;
				df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
				df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
				df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
				df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
				df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
				df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
				df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
				df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
				df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;

				//jocobian
				vnl_vector_fixed<float, 12> df_dpj;
				df_dpj[0] = df_dvtn[0] * (vt[0] - gj[0])*w_j;
				df_dpj[1] = df_dvtn[0] * (vt[1] - gj[1])*w_j;
				df_dpj[2] = df_dvtn[0] * (vt[2] - gj[2])*w_j;
				df_dpj[3] = df_dvtn[1] * (vt[0] - gj[0])*w_j;
				df_dpj[4] = df_dvtn[1] * (vt[1] - gj[1])*w_j;
				df_dpj[5] = df_dvtn[1] * (vt[2] - gj[2])*w_j;
				df_dpj[6] = df_dvtn[2] * (vt[0] - gj[0])*w_j;
				df_dpj[7] = df_dvtn[2] * (vt[1] - gj[1])*w_j;
				df_dpj[8] = df_dvtn[2] * (vt[2] - gj[2])*w_j;
				df_dpj[9] = df_dvtn[0] * w_j;
				df_dpj[10] = df_dvtn[1] * w_j;
				df_dpj[11] = df_dvtn[2] * w_j;

				//hessian
				for (int ii = 0; ii < 12; ii++)
				for (int jj = 0; jj < 12; jj++)
					blk.at(ii, jj) += df_dpi[ii] * df_dpj[jj];
			}
		}
	}

	return true;
}

bool EDMatchingJacobianHelper::
fill_hessians_data2(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
				vector<int2> &vts_mask,
				int const* vt_indices, WeightsPair const* vt_node_weights,
				CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
				CSurface<float> const& surface_t,
				vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams, 
				float w_data, float mu,
				int frmIdx, int iter)
{
	int depth_width = depthMats_trg[0].cols;
	int depth_height = depthMats_trg[0].rows;

	float w_data_sqrt = std::sqrt(w_data);

	vnl_matrix_fixed<double, 3, 3> R = graph.global_rigid.rotation();
	vnl_vector_fixed<double, 3> T = graph.global_rigid.t;

	vector<vnl_matrix_fixed<double, 3, 3>> Rs(cams.size());
	vector<vnl_vector_fixed<double, 3>> Ts(cams.size());
	vector<vnl_matrix_fixed<double, 3, 3>> Ks(cams.size());
	for (int i = 0; i < cams.size(); i++)
	{
		get_camera_pose(*(cams[i]), Rs[i], Ts[i]);
		get_calibration_matrix(*(cams[i]), Ks[i]);
	}

	vts_mask.clear();
	vts_mask.resize(surface.vtNum);
#pragma omp parallel for
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		float const*v_t_ = surface_t.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
		float const*n_t_ = surface_t.vt_normal(vtIdx);
		vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);
		float const*vt_ = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

		int2 vt_mask;
		vt_mask.x = 0;
		vt_mask.y = 0;
		for (int vId = 0; vId < cams.size(); vId++)
		{
			vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

			//if the point is behind the camera, then skip it
			if (X_cam[2] <= 0)
				continue;

			float const&fx = Ks[vId][0][0];
			float const&fy = Ks[vId][1][1];
			float const&cx = Ks[vId][0][2];
			float const&cy = Ks[vId][1][2];

			int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
			int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

			// if the point cannot be observed at the current camera pose
			if (u < 0 || u >= depth_width ||
				v < 0 || v >= depth_height)
				continue;

			float d_s = depthMats_trg[vId].at<double>(v, u);
			if (d_s <= 0)
				continue;

			vnl_vector_fixed<double, 3> p;
			p[0] = (u - cx)*d_s / fx;
			p[1] = (v - cy)*d_s / fy;
			p[2] = d_s;

			p = Rs[vId].transpose()*(p - Ts[vId]);

			if (dist_3d(p, v_t) < mu)
			{
				vt_mask.x++;
				vt_mask.y |= (1 << vId);
			}
		}
		vts_mask[vtIdx] = vt_mask;
	}

	int count = 0;
	for (int i = 0; i < vts_mask.size(); i++)
		count += vts_mask[i].x;

	int blk_num = hessian.blk_num;
	int para_blk_num = hessian.para_blk_num;
	HessianBlock *hessian_blocks = hessian.blocks;

	assert(para_blk_num * 12 == b.size());

	vnl_vector<float> costs_per_block;
	costs_per_block.set_size(para_blk_num);
	costs_per_block.fill(0.0);

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < para_blk_num; i++)
	{
		//parameter block index
		int piIdx = hessian_blocks[i].row;
		int pjIdx = hessian_blocks[i].col;
		assert(piIdx == pjIdx);
		float *h_val = &(hessian_blocks[i].vals[0]);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		float cost_cur = 0.0;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			float const*v_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
			float const*n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);
			float const*vt_ = surface.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);			

			int2 vt_mask = vts_mask[vtIdx];
			if (vt_mask.x == 0)
				continue;

			for (int vId = 0; vId < cams.size(); vId++)
			{
				vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

				//if the point is behind the camera, then skip it
				if (X_cam[2] <= 0)
					continue;

				float const&fx = Ks[vId][0][0];
				float const&fy = Ks[vId][1][1];
				float const&cx = Ks[vId][0][2];
				float const&cy = Ks[vId][1][2];

				int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u < 0 || u >= depth_width ||
					v < 0 || v >= depth_height)
					continue;

				float d_s = depthMats_trg[vId].at<double>(v, u);
				if (d_s <= 0)
					continue;

				vnl_vector_fixed<double, 3> p;
				p[0] = (u - cx)*d_s / fx;
				p[1] = (v - cy)*d_s / fy;
				p[2] = d_s;
				p = Rs[vId].transpose()*(p - Ts[vId]);

				//cost
				if (dist_3d(p, v_t) > mu)
					continue;

				float f = dot_product(n_t, p - v_t);
				f *= w_data_sqrt*sqrt(1.0/count);// *sqrt(1.0 / vt_mask.x);
				cost_cur += f*f;

				double w_i = vt_node_weights[k].w1 * w_data_sqrt * sqrt(1.0 / count);// *sqrt(1.0 / vt_mask.x);
				vnl_vector_fixed<double, 3> df_dvtn = -n_t*R;

				//jacobian
				vnl_vector_fixed<float, 12> df_dpi;
				df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
				df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
				df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
				df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
				df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
				df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
				df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
				df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
				df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;

				//gradient
				for (int m = 0; m < 12; m++)
					b[i * 12 + m] += f*df_dpi[m];

				//hessian
				for (int ii = 0; ii < 12; ii++)
				for (int jj = 0; jj < 12; jj++)
					h_val[ii * 12 + jj] += df_dpi[ii] * df_dpi[jj];
			}
		}

		costs_per_block[i] = cost_cur;
	}

	if (cost != NULL)
	{
		(*cost) += costs_per_block.sum() / 4;
	}

	//=====dealing with cross-talks between different parameter blocks
#pragma omp parallel for schedule(dynamic)
	for (int i = para_blk_num; i < blk_num; i++)
	{
		//parameter block index
		HessianBlock &blk = hessian_blocks[i];
		int piIdx = blk.row;
		int pjIdx = blk.col;
		//float *h_val = &(hessian_blocks[i].vals[0]);
		assert(piIdx > pjIdx);

		vnl_vector_fixed<double, 3> const& gid = graph.nodes[piIdx].g;
		vnl_vector_fixed<double, 3> const& gjd = graph.nodes[pjIdx].g;
		vnl_vector_fixed<float, 3> gi(gid[0], gid[1], gid[2]);
		vnl_vector_fixed<float, 3> gj(gjd[0], gjd[1], gjd[2]);

		//data term
		int vts_num = hessian_blocks[i].vts_num;
		int vtii_first = hessian_blocks[i].vtii_first;
		for (int k = vtii_first; k < vtii_first + vts_num; k++)
		{
			int vtIdx = vt_indices[k];

			float const*v_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> v_t(v_t_[0], v_t_[1], v_t_[2]);
			float const*n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);
			float const*vt_ = surface.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

			int2 vt_mask = vts_mask[vtIdx];
			if (vt_mask.x == 0)
				continue;

			for (int vId = 0; vId < cams.size(); vId++)
			{
				vnl_vector_fixed<double, 3> X_cam = Rs[vId] * v_t + Ts[vId];

				//if the point is behind the camera, then skip it
				if (X_cam[2] <= 0)
					continue;

				float const&fx = Ks[vId][0][0];
				float const&fy = Ks[vId][1][1];
				float const&cx = Ks[vId][0][2];
				float const&cy = Ks[vId][1][2];

				int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u < 0 || u >= depth_width ||
					v < 0 || v >= depth_height)
					continue;

				float d_s = depthMats_trg[vId].at<double>(v, u);
				if (d_s <= 0.0)
					continue;

				vnl_vector_fixed<double, 3> p;
				p[0] = (u - cx)*d_s / fx;
				p[1] = (v - cy)*d_s / fy;
				p[2] = d_s;
				p = Rs[vId].transpose()*(p - Ts[vId]);

				//cost
				if (dist_3d(p, v_t) > mu)
					continue;

				float f = dot_product(n_t, p - v_t);
				f *= w_data_sqrt;// *sqrt(1.0 / count);

				double w_i = vt_node_weights[k].w1 * w_data_sqrt * sqrt(1.0/count);// *sqrt(1.0 / vt_mask.x);
				double w_j = vt_node_weights[k].w2 * w_data_sqrt * sqrt(1.0/count);// *sqrt(1.0 / vt_mask.x);
				vnl_vector_fixed<double, 3> df_dvtn = -n_t*R;

				//jocobian
				vnl_vector_fixed<float, 12> df_dpi;
				df_dpi[0] = df_dvtn[0] * (vt[0] - gi[0])*w_i;
				df_dpi[1] = df_dvtn[0] * (vt[1] - gi[1])*w_i;
				df_dpi[2] = df_dvtn[0] * (vt[2] - gi[2])*w_i;
				df_dpi[3] = df_dvtn[1] * (vt[0] - gi[0])*w_i;
				df_dpi[4] = df_dvtn[1] * (vt[1] - gi[1])*w_i;
				df_dpi[5] = df_dvtn[1] * (vt[2] - gi[2])*w_i;
				df_dpi[6] = df_dvtn[2] * (vt[0] - gi[0])*w_i;
				df_dpi[7] = df_dvtn[2] * (vt[1] - gi[1])*w_i;
				df_dpi[8] = df_dvtn[2] * (vt[2] - gi[2])*w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;

				//jocobian
				vnl_vector_fixed<float, 12> df_dpj;
				df_dpj[0] = df_dvtn[0] * (vt[0] - gj[0])*w_j;
				df_dpj[1] = df_dvtn[0] * (vt[1] - gj[1])*w_j;
				df_dpj[2] = df_dvtn[0] * (vt[2] - gj[2])*w_j;
				df_dpj[3] = df_dvtn[1] * (vt[0] - gj[0])*w_j;
				df_dpj[4] = df_dvtn[1] * (vt[1] - gj[1])*w_j;
				df_dpj[5] = df_dvtn[1] * (vt[2] - gj[2])*w_j;
				df_dpj[6] = df_dvtn[2] * (vt[0] - gj[0])*w_j;
				df_dpj[7] = df_dvtn[2] * (vt[1] - gj[1])*w_j;
				df_dpj[8] = df_dvtn[2] * (vt[2] - gj[2])*w_j;
				df_dpj[9] = df_dvtn[0] * w_j;
				df_dpj[10] = df_dvtn[1] * w_j;
				df_dpj[11] = df_dvtn[2] * w_j;

				//hessian
				for (int ii = 0; ii < 12; ii++)
				for (int jj = 0; jj < 12; jj++)
					blk.at(ii, jj) += df_dpi[ii] * df_dpj[jj];
			}
		}
	}

	return true;
}

bool EDMatchingJacobianHelper::
fill_hessians_data3(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
					int const* vt_indices, WeightsPair const* vt_node_weights,
					CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
					CSurface<float> const& surface_t,
					vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams, float w_data,
					float mu)
{
	for (int i = 0; i < cams.size(); i++)
	{
		vector<cv::Mat> depthMats_one;
		depthMats_one.push_back(depthMats_trg[i]);
		vector<GCameraView*> cams_one;
		cams_one.push_back(cams[i]);

		double cost_one;
		fill_hessians_data(hessian, b, &cost_one, vt_indices, vt_node_weights, surface, graph, surface_t, 
			depthMats_one, cams_one, w_data, mu);
		*cost += cost_one;
	}
	return true;
}


double EDMatchingJacobianHelper::evaluate_rot_term(NonrigidMatching::DeformGraph const& graph, float w_rot)
{
	float w_rot_sqrt = std::sqrt(w_rot);

	double cost = 0.0;
	for (int ndIdx = 0; ndIdx < graph.nodes.size(); ndIdx++)
	{
		vnl_matrix_fixed<double, 3, 3> const& A = graph.nodes[ndIdx].A;

		for (int k = 0; k < 3; k++)
		{
			vnl_vector_fixed<double, 3> r = A.get_row(k);
			float f = (dot_product(r, r) - 1.0)*w_rot_sqrt;
			cost += f*f;
		}

		for (int i = 0; i < 3; i++)
		{
			vnl_vector_fixed<double, 3> r_i = A.get_row(i);
			for (int j = i + 1; j < 3; j++)
			{
				vnl_vector_fixed<double, 3> r_j = A.get_row(j);

				//cost
				float f = dot_product(r_i, r_j) * w_rot_sqrt;

				cost += f*f;
			}
		}

		//det(A)
		double w_detA = w_rot_sqrt * 4;
		float f = (vnl_det(A) - 1.0) * w_detA;
		cost += f*f;
	}

	return cost;
}

bool EDMatchingJacobianHelper::
fill_hessians_rot(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost, NonrigidMatching::DeformGraph const& graph, float w_rot)
{
	int para_blk_num = hessian.para_blk_num;
	float w_rot_sqrt = std::sqrt(w_rot);

	vnl_vector<double> costs_per_block;
	costs_per_block.set_size(para_blk_num);

#pragma omp parallel for
	for (int blkIdx = 0; blkIdx < para_blk_num; blkIdx++)
	{
		HessianBlock *h_blk = &(hessian.blocks[blkIdx]);
		assert(h_blk->row == h_blk->col);
		int ndIdx = h_blk->row;

		vnl_matrix_fixed<double, 3, 3> const& A = graph.nodes[ndIdx].A;

		double cost_cur = 0.0;

		// ||r||^2-1
		for (int k = 0; k < 3; k++)
		{
			vnl_vector_fixed<double, 3> r = A.get_row(k);

			//cost
			float f = (dot_product(r, r) - 1.0)*w_rot_sqrt;

			cost_cur += f*f;

			//jacobian
			vnl_vector_fixed<double, 3> df_dr;
			df_dr[0] = 2.0*r[0] * w_rot_sqrt;
			df_dr[1] = 2.0*r[1] * w_rot_sqrt;
			df_dr[2] = 2.0*r[2] * w_rot_sqrt;

			//gradient
			for (int m = 0; m < 3; m++)
				b[blkIdx * 12 + 3 * k + m] += df_dr[m] * f;

			//hessian
			for (int m = 0; m < 3; m++)
			for (int n = 0; n <= m; n++)
				h_blk->at(3 * k + m, 3 * k + n) += df_dr[m] * df_dr[n];
		}

		// r_i^T r_j 
		for (int i = 0; i < 3; i++)
		{
			vnl_vector_fixed<double, 3> r_i = A.get_row(i);
			for (int j = i + 1; j < 3; j++)
			{
				vnl_vector_fixed<double, 3> r_j = A.get_row(j);

				//cost
				float f = dot_product(r_i, r_j) * w_rot_sqrt;

				cost_cur += f*f;

				//jacobian
				vnl_vector_fixed<double, 3> df_dr_i = r_j * w_rot_sqrt;
				vnl_vector_fixed<double, 3> df_dr_j = r_i * w_rot_sqrt;

				//gradient
				for (int m = 0; m < 3; m++)
				{
					b[blkIdx * 12 + 3 * i + m] += df_dr_i[m] * f;
					b[blkIdx * 12 + 3 * j + m] += df_dr_j[m] * f;
				}

				//hessian
				for (int m = 0; m < 3; m++)
				for (int n = 0; n <= m; n++)
					h_blk->at(3 * i + m, 3 * i + n) += df_dr_i[m] * df_dr_i[n];

				for (int m = 0; m < 3; m++)
				for (int n = 0; n <= m; n++)
					h_blk->at(3 * j + m, 3 * j + n) += df_dr_j[m] * df_dr_j[n];

				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					h_blk->at(3 * j + m, 3 * i + n) += df_dr_j[m] * df_dr_i[n];
			}
		}

		//det(A)
		double w_detA = w_rot_sqrt * 4;
		float f = (vnl_det(A) - 1.0) * w_detA;

		cost_cur += f*f;

		//jacobian
		vnl_vector_fixed<double, 9> df_dA;
		df_dA[0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * w_detA;
		df_dA[1] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * w_detA;
		df_dA[2] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * w_detA;
		df_dA[3] = (A[2][1] * A[0][2] - A[0][1] * A[2][2]) * w_detA;
		df_dA[4] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * w_detA;
		df_dA[5] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * w_detA;
		df_dA[6] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * w_detA;
		df_dA[7] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * w_detA;
		df_dA[8] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * w_detA;

		//gradient
		for (int m = 0; m < 9; m++)
			b[blkIdx * 12 + m] += df_dA[m] * f;

		//hessian
		for (int m = 0; m < 9; m++)
		for (int n = 0; n <= m; n++)
			h_blk->at(m, n) += df_dA[m] * df_dA[n];

		costs_per_block[blkIdx] = cost_cur;
	}

	if (cost != NULL)
		*cost += costs_per_block.sum();

	return true;
}


double EDMatchingJacobianHelper::evaluate_reg_term(NonrigidMatching::DeformGraph const& graph,
	vnl_matrix<HessianBlockRelatedData*> const& hessian_related_data, float w_reg)
{
	float w_reg_sqrt = std::sqrt(w_reg);

	double cost = 0.0;
	for (int ndIdx_i = 0; ndIdx_i < graph.nodes.size(); ndIdx_i++)
	{
		for (int ndIdx_j = 0; ndIdx_j < graph.nodes.size(); ndIdx_j++)
		{
			if (hessian_related_data[ndIdx_i][ndIdx_j] == NULL ||
				!hessian_related_data[ndIdx_i][ndIdx_j]->bGraphEdge)
				continue;

			vnl_vector_fixed<double, 3> const& gi = graph.nodes[ndIdx_i].g;
			vnl_vector_fixed<double, 3> const& gj = graph.nodes[ndIdx_j].g;
			vnl_matrix_fixed<double, 3, 3> const& Ai = graph.nodes[ndIdx_i].A;
			vnl_matrix_fixed<double, 3, 3> const& Aj = graph.nodes[ndIdx_j].A;
			vnl_vector_fixed<double, 3> const& ti = graph.nodes[ndIdx_i].t;
			vnl_vector_fixed<double, 3> const& tj = graph.nodes[ndIdx_j].t;

			for (int k = 0; k < 3; k++)
			{
				vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
				vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

				//cost 
				vnl_vector_fixed<double, 3> tmp = gj - gi;
				double f = dot_product(r_i, tmp) + gi[k] + ti[k] - gj[k] - tj[k];
				f *= w_reg_sqrt;

				cost += f*f;
			}

			for (int k = 0; k < 3; k++)
			{
				vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
				vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

				//cost 
				vnl_vector_fixed<double, 3> tmp = gi - gj;
				double f = dot_product(r_j, tmp) + gj[k] + tj[k] - gi[k] - ti[k];
				f *= w_reg_sqrt;

				cost += f*f;
			}

		}
	}

	return cost;

}

double EDMatchingJacobianHelper::evaluate_reg_term(NonrigidMatching::DeformGraph const& graph,
	vnl_matrix<bool> const& reg_mat, float w_reg)
{
	float w_reg_sqrt = std::sqrt(w_reg);

	double cost = 0.0;
	for (int ndIdx_i = 0; ndIdx_i < graph.nodes.size(); ndIdx_i++)
	{
		for (int ndIdx_j = 0; ndIdx_j < graph.nodes.size(); ndIdx_j++)
		{
			if (!reg_mat[ndIdx_i][ndIdx_j])
				continue;

			vnl_vector_fixed<double, 3> const& gi = graph.nodes[ndIdx_i].g;
			vnl_vector_fixed<double, 3> const& gj = graph.nodes[ndIdx_j].g;
			vnl_matrix_fixed<double, 3, 3> const& Ai = graph.nodes[ndIdx_i].A;
			vnl_matrix_fixed<double, 3, 3> const& Aj = graph.nodes[ndIdx_j].A;
			vnl_vector_fixed<double, 3> const& ti = graph.nodes[ndIdx_i].t;
			vnl_vector_fixed<double, 3> const& tj = graph.nodes[ndIdx_j].t;

			for (int k = 0; k < 3; k++)
			{
				vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
				vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

				//cost 
				vnl_vector_fixed<double, 3> tmp = gj - gi;
				double f = dot_product(r_i, tmp) + gi[k] + ti[k] - gj[k] - tj[k];
				f *= w_reg_sqrt;

				cost += f*f;
			}

			for (int k = 0; k < 3; k++)
			{
				vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
				vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

				//cost 
				vnl_vector_fixed<double, 3> tmp = gi - gj;
				double f = dot_product(r_j, tmp) + gj[k] + tj[k] - gi[k] - ti[k];
				f *= w_reg_sqrt;

				cost += f*f;
			}
		}
	}

	return cost;

}

bool EDMatchingJacobianHelper::
fill_hessians_reg(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost, NonrigidMatching::DeformGraph const& graph, float w_reg)
{
	int blk_num = hessian.blk_num;
	int para_blk_num = hessian.para_blk_num;
	HessianBlock* hessian_blocks = hessian.blocks;

	float w_reg_sqrt = std::sqrt(w_reg);

	for (int i = para_blk_num; i < blk_num; i++)
	{
		int piIdx = hessian_blocks[i].row;
		int pjIdx = hessian_blocks[i].col;

		if (!hessian_blocks[i].bGraphEdge)
			continue;

		vnl_vector_fixed<double, 3> const& gi = graph.nodes[piIdx].g;
		vnl_vector_fixed<double, 3> const& gj = graph.nodes[pjIdx].g;
		vnl_matrix_fixed<double, 3, 3> const& Ai = graph.nodes[piIdx].A;
		vnl_matrix_fixed<double, 3, 3> const& Aj = graph.nodes[pjIdx].A;
		vnl_vector_fixed<double, 3> const& ti = graph.nodes[piIdx].t;
		vnl_vector_fixed<double, 3> const& tj = graph.nodes[pjIdx].t;

		for (int k = 0; k < 3; k++)
		{
			vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
			vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

			//cost 
			vnl_vector_fixed<double, 3> tmp = gj - gi;
			double f = dot_product(r_i, tmp) + gi[k] + ti[k] - gj[k] - tj[k];
			f *= w_reg_sqrt;

			*cost += f*f;

			//jacobian
			vnl_vector_fixed<float, 5> jac;
			jac[0] = tmp[0] * w_reg_sqrt; //df_dri
			jac[1] = tmp[1] * w_reg_sqrt;
			jac[2] = tmp[2] * w_reg_sqrt;
			jac[3] = w_reg_sqrt; //df_dt_i
			jac[4] = -w_reg_sqrt; //df_dt_j

			//gradient
			for (int m = 0; m < 3; m++)
				b[piIdx * 12 + 3 * k + m] += jac[m] * f;
			b[piIdx * 12 + 9 + k] += jac[3] * f;
			b[pjIdx * 12 + 9 + k] += jac[4] * f;


			//Hessian
			//hessian block pi
			for (int m = 0; m < 3; m++)
			for (int n = 0; n <= m; n++)
				hessian_blocks[piIdx].at(3 * k + m, 3 * k + n) += jac[m] * jac[n];
			for (int n = 0; n < 3; n++)
				hessian_blocks[piIdx].at(9 + k, 3 * k + n) += jac[3] * jac[n];

			hessian_blocks[piIdx].at(9 + k, 9 + k) += jac[3] * jac[3];

			//hessian block pj
			hessian_blocks[pjIdx].at(9 + k, 9 + k) += jac[4] * jac[4];

			//hessian block (pi, pj)
			for (int n = 0; n < 3; n++)
				hessian_blocks[i].at(3 * k + n, 9 + k) += jac[n] * jac[4];
			hessian_blocks[i].at(9 + k, 9 + k) += jac[4] * jac[3];
		}

		for (int k = 0; k < 3; k++)
		{
			vnl_vector_fixed<double, 3> r_i = Ai.get_row(k);
			vnl_vector_fixed<double, 3> r_j = Aj.get_row(k);

			//cost 
			vnl_vector_fixed<double, 3> tmp = gi - gj;
			double f = dot_product(r_j, tmp) + gj[k] + tj[k] - gi[k] - ti[k];
			f *= w_reg_sqrt;

			*cost += f*f;

			//jacobian
			vnl_vector_fixed<float, 5> jac;
			jac[0] = tmp[0] * w_reg_sqrt; //df_drj
			jac[1] = tmp[1] * w_reg_sqrt;
			jac[2] = tmp[2] * w_reg_sqrt;
			jac[3] = w_reg_sqrt; //df_dt_j
			jac[4] = -w_reg_sqrt; //df_dt_i

			//gradient
			for (int m = 0; m < 3; m++)
				b[pjIdx * 12 + 3 * k + m] += jac[m] * f;
			b[pjIdx * 12 + 9 + k] += jac[3] * f;
			b[piIdx * 12 + 9 + k] += jac[4] * f;

			//Hessian
			//hessian block pj
			for (int m = 0; m < 3; m++)
			for (int n = 0; n <= m; n++)
				hessian_blocks[pjIdx].at(3 * k + m, 3 * k + n) += jac[m] * jac[n];
			for (int n = 0; n < 3; n++)
				hessian_blocks[pjIdx].at(9 + k, 3 * k + n) += jac[3] * jac[n];
			hessian_blocks[pjIdx].at(9 + k, 9 + k) += jac[3] * jac[3];

			//hessian block pi
			hessian_blocks[piIdx].at(9 + k, 9 + k) += jac[4] * jac[4];

			//hessian block (pi, pj)
			for (int n = 0; n < 3; n++)
				hessian_blocks[i].at(9 + k, 3 * k + n) += jac[4] * jac[n];
			hessian_blocks[i].at(9 + k, 9 + k) += jac[4] * jac[3];
		}
	}

	return true;
}


bool EDMatchingJacobianHelper::save_triplets_matlab(vector<Triplet> const&triplets, char const* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if (!fp)
	{
		LOGGER()->error("EDMatchingJacobianHelper::save_triplets", "Cannot open the file <%s>.", file_name);
		return false;
	}

	for (int i = 0; i < triplets.size(); i++)
		fprintf(fp, "%d  %d  %f\n", triplets[i].row + 1, triplets[i].col + 1, triplets[i].val);

	fclose(fp);
	return true;
}
