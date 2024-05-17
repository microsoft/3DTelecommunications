#ifndef __GEOMETRY_TYPES_CUDA_H__
#define __GEOMETRY_TYPES_CUDA_H__
#include "cuda_matrix_fixed.cuh"
#include "cuda_vector_fixed.cuh"
#include <stdio.h>
#include <assert.h>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "..\Common\cuda\CudaHelpers.h"

#define SDF_NULL_VALUE -2.0f

#define ED_HIER_LEVEL_NUM_MAX 5
#define NEIGHBOR_EDNODE_NUM 4 //#neighboring ed nodes for each vertex
#define JTJ_BLKS_NUM_INDUCED_PER_VERTEX (NEIGHBOR_EDNODE_NUM+(NEIGHBOR_EDNODE_NUM)*(NEIGHBOR_EDNODE_NUM-1)/2)
#define EDNODE_NN 8 //#neighbors of ed nodes: asymmetric
#define EDNODE_NN_MAX 100 //#neighbors of ed nodes :after symemetric processing
#define ED_NODES_NUM_MAX 4000

#define ED_CUBE_DIM_MAX 128
#define ED_CUBE_NUM_MAX (ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX)

struct RigidTransformCuda
{
public:
	__host__ __device__ RigidTransformCuda() {}
	__host__ __device__ ~RigidTransformCuda() {}

	cuda_matrix_fixed<float, 3, 3> R;
	cuda_vector_fixed<float, 3> T;
};

struct DeformGraphNodeCoreCuda
{
	cuda_matrix_fixed<float, 3, 3> A;
	cuda_vector_fixed<float, 3> t;
	cuda_vector_fixed<float, 3> g;
};

struct DeformGraphNodeCoreCudaL2 : public DeformGraphNodeCoreCuda
{
	cuda_matrix_fixed<float, 3, 3> A_inv_t; //A^(-T)
	cuda_vector_fixed<float, 3> n; //average normal direction 
};

struct DeformGraphNodeCuda : public DeformGraphNodeCoreCudaL2
{
	short neighbors[EDNODE_NN_MAX];
};


struct EDNodesParasGPU
{
	DeformGraphNodeCuda* dev_ed_nodes;
	cuda::gpu_size_data ed_nodes_num_gpu;
	cudaArray* cu_3dArr_ndIds;
	int3* dev_ed_cubes_dim;
	float3* dev_ed_cubes_offset;
	float ed_cubes_res;
	RigidTransformCuda* dev_rigid_transf;
};

inline void allocate_EDNodesParasGPU(EDNodesParasGPU &ed_nodes_paras)
{
	checkCudaErrors(cudaMalloc(&(ed_nodes_paras.dev_ed_nodes), sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX));
	ed_nodes_paras.ed_nodes_num_gpu.max_size = ED_NODES_NUM_MAX;
	checkCudaErrors(cudaMalloc(&(ed_nodes_paras.ed_nodes_num_gpu.dev_ptr), sizeof(int)));
	checkCudaErrors(cudaMalloc(&(ed_nodes_paras.dev_ed_cubes_dim), sizeof(int3)));
	checkCudaErrors(cudaMalloc(&(ed_nodes_paras.dev_ed_cubes_offset), sizeof(float3)));
	checkCudaErrors(cudaMalloc(&(ed_nodes_paras.dev_rigid_transf), sizeof(RigidTransformCuda)));
	
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
	checkCudaErrors(cudaMalloc3DArray(&(ed_nodes_paras.cu_3dArr_ndIds), &channelDesc, 
		make_cudaExtent(ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX), 
		cudaArraySurfaceLoadStore));
}

inline int min_buf_size_for_EDNodesParasGPU()
{
	return sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX +
		ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*sizeof(short)+
		sizeof(int3)+sizeof(float3)+sizeof(RigidTransformCuda)+sizeof(int);
}

inline void pack_EDNodesParasGPU_to_dev_buf(EDNodesParasGPU ed_paras, char* dev_buf, int buf_size, cudaStream_t stream=NULL)
{
	assert(buf_size >= min_buf_size_for_EDNodesParasGPU());

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcArray = ed_paras.cu_3dArr_ndIds;
	paras.dstPtr = make_cudaPitchedPtr(dev_buf, ED_CUBE_DIM_MAX*sizeof(short), ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX);
	paras.extent = make_cudaExtent(ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX);
	paras.kind = cudaMemcpyDeviceToDevice;
	checkCudaErrors(cudaMemcpy3DAsync(&paras, stream));

	int ed_nodes_offset = ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*sizeof(short);
	checkCudaErrors(cudaMemcpyAsync(dev_buf + ed_nodes_offset, ed_paras.dev_ed_nodes, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX, cudaMemcpyDeviceToDevice, stream));

	int rigid_offset = ed_nodes_offset + sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX;
	checkCudaErrors(cudaMemcpyAsync(dev_buf + rigid_offset, ed_paras.dev_rigid_transf, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice, stream));

	int edCubesDim_offset = rigid_offset + sizeof(RigidTransformCuda);
	checkCudaErrors(cudaMemcpyAsync(dev_buf + edCubesDim_offset, ed_paras.dev_ed_cubes_dim, sizeof(int3), cudaMemcpyDeviceToDevice, stream));

	int edCubesOffset_offset = edCubesDim_offset = sizeof(int3);
	checkCudaErrors(cudaMemcpyAsync(dev_buf + edCubesOffset_offset, ed_paras.dev_ed_cubes_offset, sizeof(float3), cudaMemcpyDeviceToDevice, stream));

	int edNodesNum_offset = edCubesOffset_offset + sizeof(float3);
	checkCudaErrors(cudaMemcpyAsync(dev_buf + edNodesNum_offset, ed_paras.ed_nodes_num_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice, stream));
}

inline void unpack_EDNodesParasGPU_from_dev_buf(char const* dev_buf, EDNodesParasGPU ed_paras, cudaStream_t stream=NULL)
{
	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcPtr = make_cudaPitchedPtr((void*)dev_buf, ED_CUBE_DIM_MAX*sizeof(short), ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX);
	paras.dstArray = ed_paras.cu_3dArr_ndIds;
	paras.extent = make_cudaExtent(ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX, ED_CUBE_DIM_MAX);
	paras.kind = cudaMemcpyDeviceToDevice;
	checkCudaErrors(cudaMemcpy3DAsync(&paras, stream));

	int ed_nodes_offset = ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*ED_CUBE_DIM_MAX*sizeof(short);
	checkCudaErrors(cudaMemcpyAsync(ed_paras.dev_ed_nodes, dev_buf + ed_nodes_offset, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX, cudaMemcpyDeviceToDevice, stream));

	int rigid_offset = ed_nodes_offset + sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX;
	checkCudaErrors(cudaMemcpyAsync(ed_paras.dev_rigid_transf, dev_buf + rigid_offset, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice, stream));

	int edCubesDim_offset = rigid_offset + sizeof(RigidTransformCuda);
	checkCudaErrors(cudaMemcpyAsync(ed_paras.dev_ed_cubes_dim, dev_buf + edCubesDim_offset, sizeof(int3), cudaMemcpyDeviceToDevice, stream));

	int edCubesOffset_offset = edCubesDim_offset = sizeof(int3);
	checkCudaErrors(cudaMemcpyAsync(ed_paras.dev_ed_cubes_offset, dev_buf + edCubesOffset_offset, sizeof(float3), cudaMemcpyDeviceToDevice, stream));

	int edNodesNum_offset = edCubesOffset_offset + sizeof(float3);
	checkCudaErrors(cudaMemcpyAsync(ed_paras.ed_nodes_num_gpu.dev_ptr, dev_buf + edNodesNum_offset, sizeof(int), cudaMemcpyDeviceToDevice, stream));
}


struct BoundingBox3DCuda
{
public:
__host__ __device__	BoundingBox3DCuda(float x_s_ = 0.0, float x_e_ = 0.0, float y_s_ = 0.0, float y_e_ = 0.0, float z_s_ = 0.0, float z_e_ = 0.0)
		:x_s(x_s_),
		x_e(x_e_),
		y_s(y_s_),
		y_e(y_e_),
		z_s(z_s_),
		z_e(z_e_) 
{}
public:
	float x_s;
	float x_e;
	float y_s;
	float y_e;
	float z_s;
	float z_e;
};

struct OccupcyCube
{
	int offset; //-1 initially. offset in cubes count, need to multiply by cube_size_in_voxel to get data offset
};

class VolumeTwoLevelHierachy
{
public:
	VolumeTwoLevelHierachy()
		: vxl_res(0.0), 
		cube_res(0.0), cube_size_in_voxel(0.0),
		cubes(NULL), cubes_occpied_capacity(0), cubes_occpied_count(0), data(NULL)
	{
		cubes_offset.x = 0.0f; cubes_offset.y = 0.0f; cubes_offset.z = 0.0f;
		cubes_num.x = 0; cubes_num.y = 0; cubes_num.z = 0;
	}

public:
	float mu;
	float vxl_res; //size of voxel
	float cube_res;
	int cube_size_in_voxel; //size of the cube. power of 2 ideally
	int vxls_per_cube;
	OccupcyCube* cubes;

	int cubes_occpied_capacity;
	int* buf_occupied_cube_ids; //list of occupied cube ids

	float* data;
	float* weights;
	uchar4* colors;

	//the following three property might be on gpu: cpu value might be invalid
	float3 cubes_offset;
	int3 cubes_num;
	int cubes_occpied_count;

	//gpu memory
	float3 *ptr_cubes_offset;
	int3 *ptr_cubes_num;
	cuda::gpu_size_data gpu_cubes_occpied_count;

public:
	bool save_to_txt(char const* filename)
	{
		FILE *file = NULL;
		if ((fopen_s(&file, filename, "w")) != 0)
		{
			printf("Error<VolumeTwoLevelHierachy::save_to_txt>: cannot open file <%s>\n", filename);
			return false;
		}

		fprintf(file, "Two Level Hirarchy Volumetric file.\n");
		fprintf(file, "Volume offset: <%f, %f, %f>\n", cubes_offset.x, cubes_offset.y, cubes_offset.z);
		fprintf(file, "coarse cube num: <%d, %d, %d>\n", cubes_num.x, cubes_num.y, cubes_num.z);
		fprintf(file, "coarse cube res: %f\n", cube_res);
		fprintf(file, "coarse cube size in voxel: %d\n", cube_size_in_voxel);
		fprintf(file, "vxl num per cube: %d\n", vxls_per_cube);
		fprintf(file, "voxel occupancy: <%d/%d>\n", cubes_occpied_count, cubes_occpied_capacity);
		fprintf(file, "voxel res: %f\n", vxl_res);
		fprintf(file, "mu: %f\n", mu);

		fprintf(file, "\ncubes data offset:\n");
		for (int i = 0; i < cubes_num.x*cubes_num.y*cubes_num.z; i++)
			fprintf(file, "%d ", cubes[i].offset);
		fprintf(file, "\n");

		fprintf(file, "\noccupied cubes ID:\n");
		for (int i = 0; i < cubes_occpied_count; i++)
			fprintf(file, "%d ", buf_occupied_cube_ids[i]);
		fprintf(file, "\n");
		
		int vxl_cnt_per_cube = cube_size_in_voxel*cube_size_in_voxel*cube_size_in_voxel;
		if (data != NULL && weights != NULL)
		{
			fprintf(file, "\nvoxel data:\n");
			for (int i = 0; i < cubes_occpied_count; i++)
			{
				fprintf(file, "occupied cube %d:\n", i);
				fprintf(file, "data:\n");
				for (int j = 0; j < vxl_cnt_per_cube; j++)
				{
					fprintf(file, "%f ", data[i*vxl_cnt_per_cube + j]);
				}
				fprintf(file, "weights:\n");
				for (int j = 0; j < vxl_cnt_per_cube; j++)
				{
					fprintf(file, "%f ", weights[i*vxl_cnt_per_cube + j]);
				}
				fprintf(file, "\n");
			}
		}

		fclose(file);
		return true;
	}

	bool load_from_txt(char const* filename)
	{
		FILE *file = NULL;
		if ((fopen_s(&file, filename, "r")) != 0)
		{
			printf("Error<VolumeTwoLevelHierachy::load_from_txt>: cannot open file <%s> for loading\n", filename);
			return false;
		}

		fscanf(file, "Two Level Hirarchy Volumetric file.\n");
		fscanf(file, "Volume offset: <%f, %f, %f>\n", &cubes_offset.x, &cubes_offset.y, &cubes_offset.z);
		fscanf(file, "coarse cube num: <%d, %d, %d>\n", &cubes_num.x, &cubes_num.y, &cubes_num.z);
		fscanf(file, "coarse cube res: %f\n", &cube_res);
		fscanf(file, "coarse cube size in voxel: %d\n", &cube_size_in_voxel);
		fscanf(file, "vxl num per cube: %d\n", &vxls_per_cube);
		fscanf(file, "voxel occupancy: <%d/%d>\n", &cubes_occpied_count, &cubes_occpied_capacity);
		fscanf(file, "voxel res: %f\n", &vxl_res);
		fscanf(file, "mu: %f\n", &mu);

		int cubes_count = cubes_num.x*cubes_num.y*cubes_num.z;

		this->cubes = new OccupcyCube[cubes_count];
		fscanf(file, "\ncubes data offset:\n");
		for (int i = 0; i < cubes_count; i++)
			fscanf(file, "%d ", &(cubes[i].offset));
		fscanf(file, "\n");

		this->buf_occupied_cube_ids = new int[cubes_occpied_count];
		fscanf(file, "\noccupied cubes ID:\n");
		for (int i = 0; i < cubes_occpied_count; i++)
			fscanf(file, "%d ", &(buf_occupied_cube_ids[i]));
		fscanf(file, "\n");

		data = NULL;
		weights = NULL;
		colors = NULL;
		fscanf(file, "\nvoxel data:\n");
		if (!feof(file))
		{
			data = new float[vxls_per_cube*cubes_occpied_count];
			weights = new float[vxls_per_cube*cubes_occpied_count];
			for (int i = 0; i < cubes_occpied_count; i++)
			{
				int id;
				fscanf(file, "occupied cube %d:\n", &id);
				assert(id == i);

				fscanf(file, "data:\n");
				for (int j = 0; j < vxls_per_cube; j++)
					fscanf(file, "%f ", &(data[i*vxls_per_cube + j]));

				fscanf(file, "weights:\n");
				for (int j = 0; j < vxls_per_cube; j++)
					fscanf(file, "%f ", &(weights[i*vxls_per_cube + j]));
				fscanf(file, "\n");
			}
		}
		
		fclose(file);
		return true;
	}
};


struct VolumeDataGPU
{
public:
	VolumeDataGPU(VolumeTwoLevelHierachy &volume)
	{
		this->data = volume.data;
		this->weights = volume.weights;
		this->colors = volume.colors;
		this->cubes = volume.cubes;
		this->occupied_cube_ids = volume.buf_occupied_cube_ids;
		this->count_occu_cubes = volume.gpu_cubes_occpied_count.dev_ptr;
		this->cubes_num = volume.ptr_cubes_num;
		this->cubes_offset = volume.ptr_cubes_offset;
	}

	float* __restrict__ data;
	float* __restrict__ weights;
	uchar4* __restrict__ colors;
	OccupcyCube* __restrict__ cubes;
	int* __restrict__ occupied_cube_ids;
	int* __restrict__ count_occu_cubes;
	float3* __restrict__ cubes_offset;
	int3* __restrict__ cubes_num;
};

struct VolumeConstDataGPU
{
public:
	VolumeConstDataGPU(VolumeTwoLevelHierachy &volume)
	{
		this->data = volume.data;
		this->weights = volume.weights;
		this->colors = volume.colors;
		this->cubes = volume.cubes;
		this->occupied_cube_ids = volume.buf_occupied_cube_ids;
		this->count_occu_cubes = volume.gpu_cubes_occpied_count.dev_ptr;
		this->cubes_num = volume.ptr_cubes_num;
		this->cubes_offset = volume.ptr_cubes_offset;
	}

	float const* __restrict__ data;
	float const* __restrict__ weights;
	uchar4 const* __restrict__ colors;
	OccupcyCube const* __restrict__ cubes; //offset for each cube
	int const* __restrict__ occupied_cube_ids; //list of cubes that are occupied
	int* __restrict__ count_occu_cubes;
	float3* __restrict__ cubes_offset;
	int3* __restrict__ cubes_num; //dimension of first-level cubes
};


struct CameraViewCuda
{
public:
	__host__ __device__ CameraViewCuda() {}
	__host__ __device__ ~CameraViewCuda() {}

	cuda_matrix_fixed<float, 3, 3> K;
	RigidTransformCuda cam_pose;
};

#endif