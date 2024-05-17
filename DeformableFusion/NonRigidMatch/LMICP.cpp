#include "LMICP.h"
#include "DDF.h"
#include <omp.h>
using namespace NonrigidMatching;

void LMICP::operator() ( CSurface<float> const& surface_ref, 
						 vector<cv::Mat> depthMats_t, //without len distortion
						 vector<cv::Mat> imgs_t, //without lens distortion, 
						 vector<GCameraView*> cams_t,
						 vector<GCameraView*> cams_clr_t,
						 vnl_matrix_fixed<double, 3, 3> &R, 
						 vnl_vector_fixed<double, 3> &T,
						 int frmIdx_t,
						 int level,
						 double res_ddf,
						 bool bUseRoughDDFForMatching)
{
	if( !surface_ref.haveNormalInfo() )
	{
		printf("Error<LMICP>: the surface normal does not exist!(the vertices should be in couter-clockwise order)\n");
		return;
	}

	//extract bounding box
	BoundingBox3D bbox1 = extract_bounding_box(depthMats_t, cams_t);
	CSurface<float> surface_ref_t = surface_ref;
	transform_Surface_with_RigidModel(surface_ref_t, R, T);
	BoundingBox3D bbox2 = surface_ref_t.get_bbox();
	BoundingBox3D bbox = bbox_min(bbox1, bbox2);
	bbox.extend(5.0);

	DDF ddf_rough;
	if (bUseRoughDDFForMatching)
	{
		ddf_rough.Init(bbox, res_ddf * 3, res_ddf * 3, res_ddf * 3, 8.0, false, false, imgs_t.size() != 0);
		for (int i = 0; i < depthMats_t.size(); i++)
			ddf_rough.add_a_frame(depthMats_t[i], cams_t[i], imgs_t.size() <= i ? cv::Mat() : imgs_t[i], cams_clr_t[i]);
	}

	//load ddf if I can, otherwise compute DDF
	DDF ddf;
	if( this->ddf_dir != NULL )
	{
		char name[500];
		sprintf(name, "%s/lmicp_ddf_%04d.z", this->ddf_dir, frmIdx_t);
		if( !ddf.load_data_from_file_zip(name) )
		{
			//compute ddf
			ddf.Init(bbox, res_ddf, res_ddf, res_ddf, 2.0, false, false, imgs_t.size()==0?false:true);
			for(int i=0; i<depthMats_t.size(); i++)
				ddf.add_a_frame(depthMats_t[i], cams_t[i], imgs_t.size() <= i ? cv::Mat() : imgs_t[i], cams_clr_t[i]);
			ddf.save_data_to_file_zip(name);
		}
	}
	else
	{
		//compute ddf
		ddf.Init(bbox, res_ddf, res_ddf, res_ddf, 2.0, false, false, imgs_t.size() == 0 ? false : true);
		for (int i = 0; i<depthMats_t.size(); i++)
			ddf.add_a_frame(depthMats_t[i], cams_t[i], imgs_t.size() <= i ? cv::Mat() : imgs_t[i], cams_clr_t[i]);
	}

	if (this->ddf_dir != NULL)
	{
		CSurface<float> surface_t;
		ddf.generate_surface(surface_t, 0, true);

		char name[500];
		sprintf(name, "%s/lmicp_surface_%04d.bin", this->ddf_dir, frmIdx_t);
		surface_t.writeToFileBIN(name);
	}

	vnl_vector_fixed<double, 3> rod;
	matrix_to_rodrigues(R, rod);
	RigidTransformModel rigid_transf;
	rigid_transf.rod = rod;
	rigid_transf.t = T;

	int kernal_radius = 2;
	vector< cv::Mat > imgs_f;
	vector< cv::Mat > mats_Rx;
	vector< cv::Mat > mats_Ry;
	vector< cv::Mat > mats_Gx;
	vector< cv::Mat > mats_Gy;
	vector< cv::Mat > mats_Bx;
	vector< cv::Mat > mats_By;
	calc_imgs_gradient( imgs_t, imgs_f, mats_Rx, mats_Ry, mats_Gx, mats_Gy, mats_Bx, mats_By, kernal_radius);

#ifdef USE_CERES_SOLVER
	DeformGraphOptMultiDataDensePtsAndClrStatic::iterIdx = 0;
	DeformGraphOptMultiDataDensePtsAndClrStatic::frmIdx = frmIdx_t;

	DeformGraphOptMultiDataDensePtsAndClr data;
	data.cams = cams_clr_t;
	data.imgs = imgs_f;
	data.mats_Rx = mats_Rx;
	data.mats_Ry = mats_Ry;
	data.mats_Gx = mats_Gx;
	data.mats_Gy = mats_Gy;
	data.mats_Bx = mats_Bx;
	data.mats_By = mats_By;
	data.sdf_target = bUseRoughDDFForMatching? &ddf_rough : &ddf;
	data.rigid_surfaces.push_back((CSurface<float>*)&surface_ref);
	data.rigid_transfs.push_back(&rigid_transf);
	data.w_sdf = 1.0;
	data.w_color = 0.0;

	data.itmax = 10;
	run_opt_ceres(data, 6);
	if( level <= 2 )
	{
		data.itmax = 5;
		data.sdf_target = &ddf;
		run_opt_ceres(data, 2);
	}
	if( level <= 1)
	{
		data.itmax = 10;
		data.sdf_target = &ddf;
		run_opt_ceres(data, 1);
	}

#endif


	for(int i=0; i<imgs_f.size(); i++)
	{
		imgs_f[i].release();
		mats_Rx[i].release();
		mats_Gx[i].release();
		mats_Bx[i].release();
		mats_Ry[i].release();
		mats_Gy[i].release();
		mats_By[i].release();
	}

	rodrigues_to_matrix(rigid_transf.rod, R);
	T = rigid_transf.t;
}

void LMICP::operator() ( CSurface<float> const& surface_ref, 
						 vector<cv::Mat> imgs_t, //without lens distortion, 
						 vector<GCameraView*> cams_t,
						 vnl_matrix_fixed<double, 3, 3> &R, 
						 vnl_vector_fixed<double, 3> &T,
						 int frmIdx_t)
{
	vnl_vector_fixed<double, 3> rod;
	matrix_to_rodrigues(R, rod);
	RigidTransformModel rigid_transf;
	rigid_transf.rod = rod;
	rigid_transf.t = T;

	int kernal_radius = 2;
	vector< cv::Mat > imgs_f;
	vector< cv::Mat > mats_Rx;
	vector< cv::Mat > mats_Ry;
	vector< cv::Mat > mats_Gx;
	vector< cv::Mat > mats_Gy;
	vector< cv::Mat > mats_Bx;
	vector< cv::Mat > mats_By;
	calc_imgs_gradient( imgs_t, imgs_f, mats_Rx, mats_Ry, mats_Gx, mats_Gy, mats_Bx, mats_By, kernal_radius);

#ifdef USE_CERES_SOLVER
	DeformGraphOptMultiDataDensePtsAndClrStatic::iterIdx = 0;
	DeformGraphOptMultiDataDensePtsAndClrStatic::frmIdx = frmIdx_t;

	DeformGraphOptMultiDataDensePtsAndClr data;
	data.cams = cams_t;
	data.imgs = imgs_f;
	data.mats_Rx = mats_Rx;
	data.mats_Ry = mats_Ry;
	data.mats_Gx = mats_Gx;
	data.mats_Gy = mats_Gy;
	data.mats_Bx = mats_Bx;
	data.mats_By = mats_By;
	data.sdf_target = NULL;
	data.rigid_surfaces.push_back((CSurface<float>*)&surface_ref);
	data.rigid_transfs.push_back(&rigid_transf);
	data.w_sdf = 0.0;
	data.w_color = 1.0;

	data.itmax = 30;
	run_opt_ceres(data, 6);
	data.itmax = 15;
	run_opt_ceres(data, 3);
	data.itmax = 10;
	run_opt_ceres(data, 1);

#endif

	for(int i=0; i<imgs_f.size(); i++)
	{
		imgs_f[i].release();
		mats_Rx[i].release();
		mats_Gx[i].release();
		mats_Bx[i].release();
		mats_Ry[i].release();
		mats_Gy[i].release();
		mats_By[i].release();
	}

	rodrigues_to_matrix(rigid_transf.rod, R);
	T = rigid_transf.t;
}

#ifdef USE_CERES_SOLVER
void LMICP::run_opt_ceres(DeformGraphOptMultiDataDensePtsAndClr &data, int step)
{
	__tic__();
	DeformGraphOptMultiDataPlusStatic::rigid_transfs = data.rigid_transfs;
	DeformGraphOptMultiDataPlusStatic::rigid_surfaces = data.rigid_surfaces;
	DeformGraphOptMultiDataStatic::graphs.clear();
	DeformGraphOptMultiDataStatic::matched_points_3d.clear();
	DeformGraphOptMultiDataStatic::ngns_key_points.clear();
	DeformGraphOptMultiDataPlusStatic::surfaces.clear();
	DeformGraphOptMultiDataPlusStatic::ngns_dense.clear();
	DeformGraphOptMultiDataDensePtsAndClrStatic::sdf_target = data.sdf_target;
	DeformGraphOptMultiDataDensePtsAndClrStatic::cams = data.cams;
	DeformGraphOptMultiDataDensePtsAndClrStatic::imgs = data.imgs;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Rx = data.mats_Rx;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Gx = data.mats_Gx;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Bx = data.mats_Bx;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Ry = data.mats_Ry;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Gy = data.mats_Gy;
	DeformGraphOptMultiDataDensePtsAndClrStatic::mats_By = data.mats_By;
	extract_prj_mats(data.cams, DeformGraphOptMultiDataDensePtsAndClrStatic::cam_prj_mats);

	vector<double*> sdf_vals_last_retrieved_rigid_surfs;
	vnl_vector<double> sdf_vals_last_retrieved(data.rigid_surfaces[0]->vtNum);
	sdf_vals_last_retrieved.fill(1.0);
	sdf_vals_last_retrieved_rigid_surfs.push_back(sdf_vals_last_retrieved.data_block());
	DeformGraphOptMultiDataDensePtsAndClrStatic::sdf_vals_last_retrieved_rigid_surfs = sdf_vals_last_retrieved_rigid_surfs;

	ceres::Problem problem;

	RigidTransformModel *rigid_transf = data.rigid_transfs[0];
	CSurface<float> *surface_ref = data.rigid_surfaces[0];

	//add dense points constraints
	if( data.w_sdf > 0 )
	{
		vector<double*> paras;
		paras.push_back(rigid_transf->rod.data_block());
		paras.push_back(rigid_transf->t.data_block());
		int count = 0;
		for(int vtIdx=0; vtIdx<surface_ref->vtNum; vtIdx+=step)
		{
			LossFunction *loss_fun_sdf = new ScaledLoss( new TrivialLoss(), data.w_sdf, TAKE_OWNERSHIP );
			problem.AddResidualBlock(new RigidDensePtsTerm::F(0, vtIdx), loss_fun_sdf, paras);
			count++;
		}

		std::printf("DnsPts<%d>...", count);
	}

	//add color constraints
	if( data.w_color > 0 && data.imgs.size() > 0 )
	{
		LossFunction *loss_fun_color = new ScaledLoss( new CauchyLoss(50.0/255.0), data.w_color, TAKE_OWNERSHIP );
		
		vector< vector<int> > visible_cams;
		calc_camera_visibility( *surface_ref, data.cams, visible_cams);

		vector<double*> paras;
		paras.push_back(rigid_transf->rod.data_block());
		paras.push_back(rigid_transf->t.data_block());

		int count = 0;
		for(int vtIdx=0; vtIdx<surface_ref->vtNum; vtIdx+=step)
		{
			//if no color attached at a vertex, skip
			float const* clr = surface_ref->vt_color(vtIdx);
			if( clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0 )
				continue;

			vector<int> const&visible_cams_vt = visible_cams[vtIdx];
			for(int j=0; j<visible_cams_vt.size(); j++)
			{
				int camIdx = visible_cams_vt[j];
				problem.AddResidualBlock( new RigidDenseColorTerm::F(0, vtIdx, camIdx), loss_fun_color, paras);
				count++;
			}
		}

		std::printf("DnsClr<%d>...", count);
	}

	std::printf("End!\n");

	Solver::Options options;
	options.max_num_iterations = data.itmax;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = omp_get_num_procs();
	options.num_linear_solver_threads = omp_get_num_procs();
	options.use_nonmonotonic_steps = false;
	options.update_state_every_iteration = false;

	IterCallBackTransAndPrjSurface my_call_back;
	ceres::IterationSummary iter_summary;
	my_call_back(iter_summary);
	options.callbacks.push_back(&my_call_back);

	Solver::Summary summary;
	int graph_nodes_num = 0;
	int matches_key_points_num = 0;
	int dense_pts_num = 0;
	Solve(options, &problem, &summary);
	std::cout<<summary.BriefReport()<<endl;

	std::printf("Ceres LMICP time: %f\n", __toc__());

	//clear interal memory of DeformGraphOptDataDensePtsAndClrStatic
	for(int i=0; i<DeformGraphOptMultiDataDensePtsAndClrStatic::depthMats_prj.size(); i++)
	{
		if( !DeformGraphOptMultiDataDensePtsAndClrStatic::depthMats_prj[i].empty() )
			DeformGraphOptMultiDataDensePtsAndClrStatic::depthMats_prj[i].release();
	}
	DeformGraphOptMultiDataDensePtsAndClrStatic::depthMats_prj.clear();
	for(int i=0; i<DeformGraphOptMultiDataDensePtsAndClrStatic::imgs_proj.size(); i++)
	{
		if( !DeformGraphOptMultiDataDensePtsAndClrStatic::imgs_proj[i].empty())
			DeformGraphOptMultiDataDensePtsAndClrStatic::imgs_proj[i].release();
	}
	DeformGraphOptMultiDataDensePtsAndClrStatic::imgs_proj.clear();
}
#endif

void LMICP::operator() ( CSurface<float> const& surface_ref,
						 cv::Mat const& depthMat_t,
						 cv::Mat const& img_t,
						 GCameraView const* cam_t,
						 GCameraView const* cam_clr_t,
						 vnl_matrix_fixed<double, 3, 3> &R, 
						 vnl_vector_fixed<double, 3> &T,
						 int frmIdx_t)
{
	vector< cv::Mat > depthMats_t;
	depthMats_t.push_back((cv::Mat)depthMat_t);
	vector< cv::Mat > imgs_t;
	imgs_t.push_back((cv::Mat)img_t);
	vector< GCameraView* > cams_t;
	cams_t.push_back((GCameraView*)cam_t);
	vector< GCameraView* > cams_clr_t;
	cams_clr_t.push_back((GCameraView*)cam_clr_t);

	(*this)(surface_ref, depthMats_t, imgs_t, cams_t, cams_clr_t, R, T, frmIdx_t);
}

//<R, T>--in and out: will transform points in reference camera space to those in the template camera space.
void LMICP::operator() ( cv::Mat const& depthMat_ref, //reference 
						 cv::Mat const& img_ref,
						 cv::Mat const& depthMat_t, //template
						 cv::Mat const& img_t,
						 vnl_matrix_fixed<double, 3, 3> const& intrinsics_ref,
						 vnl_matrix_fixed<double, 3, 3> const& intrinsics_t,
						 vnl_matrix_fixed<double, 3, 3> &R, 
						 vnl_vector_fixed<double, 3> &T,
						 int frmIdx_t)
{
	GCameraView cam_ref(depthMat_ref.rows, depthMat_ref.cols);
	cam_ref.SetIntrinsics(intrinsics_ref.data_block(), NULL );
	CSurface<float> *surface_ref = DepthMap2Surface<float>(&cam_ref, depthMat_ref, img_ref, 5.0, 0, 0, 1.0, 0, true);

	GCameraView cam_t(depthMat_t.rows, depthMat_t.cols);
	cam_t.SetIntrinsics(intrinsics_t.data_block(), NULL );

	(*this)(*surface_ref, depthMat_t, img_t, &cam_t, &cam_t, R, T, frmIdx_t);

	delete surface_ref;
}
