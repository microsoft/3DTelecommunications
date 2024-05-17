#ifndef __RIGIDTRANSFORMMODEL_H__
#define __RIGIDTRANSFORMMODEL_H__
#include "UtilVnlMatrix.h"
#include "basic_geometry.h"

struct RigidTransformModel
{
public:
	RigidTransformModel() : rod(0.0, 0.0, 0.0), t(0.0, 0.0, 0.0), bRotMatComputed(false) {};
	RigidTransformModel( vnl_matrix_fixed<double, 3, 3> const&R, vnl_vector_fixed<double, 3> const& T)
		: bRotMatComputed(false)
	{
		matrix_to_rodrigues(R, this->rod);
		this->t = T;
	}
	RigidTransformModel( cv::Mat const* R, cv::Mat const* T)
		: bRotMatComputed(false)
	{
		assert( R!=NULL && T!=NULL);
		vnl_matrix_fixed<double, 3, 3> R_(R->ptr<double>());
		matrix_to_rodrigues(R_, this->rod);
		this->t.set(T->ptr<double>());
	}
	RigidTransformModel( vpgl_perspective_camera<double> const& cam)
		: bRotMatComputed(false)
	{
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;
		get_camera_pose(cam, R, T);
		matrix_to_rodrigues(R, this->rod);
		this->t = T;
	}
	RigidTransformModel( GCameraView const* cam)
		: bRotMatComputed(false)
	{
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;
		get_camera_pose(*cam, R, T);
		matrix_to_rodrigues(R, this->rod);
		this->t = T;
	}

public:
	vnl_matrix_fixed<double, 3, 3> rotation() const
	{
		vnl_matrix_fixed<double, 3, 3> R;
		rodrigues_to_matrix(rod, R);		
		return R;
	}

public:
	vnl_vector_fixed<double, 3> rod;
	vnl_vector_fixed<double, 3> t;

private:
	bool bRotMatComputed;
	vnl_matrix_fixed<double, 3, 3> rotMat;

public:
	void save_to_txt(const char *filename) const
	{
		vector< vnl_vector_fixed<double, 3> > vecs;
		vecs.push_back(rod);
		vecs.push_back(t);
		saveVnlVectorSetASCAII(filename, vecs);
	}

	bool save_to_bin(const char *filename) const
	{
		FILE *file = NULL;
		if( (fopen_s(&file, filename, "wb")) != 0 )
		{
			printf("Error<RigidTransformModel::save_to_bin> when open File <%s>\n", filename);
			return false;
		}
		fwrite(rod.data_block(), sizeof(double), 3, file);
		fwrite(t.data_block(), sizeof(double), 3, file);
		fclose(file);
		return true;
	}

	bool load_from_txt( const char *filename)
	{
		std::vector< vnl_vector_fixed<double, 3> > vecs;
		if( !loadVnlVectorSetASCAII(filename, vecs) )
			return false;

		this->rod = vecs[0];
		this->t = vecs[1];
		return true;
	}

	bool load_from_bin(const char *filename)
	{
		FILE *file = NULL;
		if( (fopen_s(&file, filename, "rb")) != 0 )
		{
			printf("Error<RigidTransformModel::load_from_bin> when open File <%s>\n", filename);
			return false;
		}
		fread(rod.data_block(), sizeof(double), 3, file);
		fread(t.data_block(), sizeof(double), 3, file);
		fclose(file);
		return true;
	}
};

inline void releaseRigidTransformModels( std::vector< RigidTransformModel*> rigid_transfs )
{
	for(int i=0; i<rigid_transfs.size(); i++)
		if(rigid_transfs[i] != NULL )
			delete rigid_transfs[i];
	rigid_transfs.clear();
}


#endif