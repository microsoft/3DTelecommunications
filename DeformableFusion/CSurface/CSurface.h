//===============================================
//			CSurface.h
//			Mingsong Dou (doums@cs.unc.edu)
//			August, 2010
//===============================================
#ifndef __CSURFACE_H__
#define __CSURFACE_H__

//#define USING_GLMOBJ

#ifdef USING_GLMOBJ
#include "glm.h"
#endif
#include <vector>
#include "CIsoSurface.h"
#include "BoundingBox3D.h"

template<class T>
class CSurface
{
public:
	CSurface()
		: vtDim(0),
		  vtNum(0), 
		  vtData(0),
		  triNum(0),
		  triangles(0)
	{
	}

	CSurface(CSurface<T> const&other);

	~CSurface()
	{
		if(vtData != 0)
			delete [] vtData;
		if(triangles != 0)
			delete [] triangles;
		for(int i=0; i<colors_of_view.size(); i++)
		{
			if( colors_of_view[i] != NULL )
			{
				delete [] colors_of_view[i];
				colors_of_view[i] = NULL;
			}
		}
	}
public:
	CSurface<T>& operator=( CSurface<T> const& rhs );

	template<class T2>
	void copy_from_another(CSurface<T2> const& other);

public:
	int vtDim; // the dimensions of vector to represent a vertex;
	int vtNum;
	T* vtData; //vertices, normals, color; data pattern: vertices, normals and colors are interleaved (vertex_x, vertex_y, vertex_z, normal_x, normal_y, normal_z, r, g, b,...)
	int triNum;
	int* triangles; //contains indices to vertices; count from 0;

	bool color; //indicate whether color info is included
	bool normal; //indicate whether normal info is included

public:
	std::vector<float*> colors_of_view; // the colors from different views/cameras
								   // each view must give a color information for each vertex
								   // could be empty

public:
	void extract_boundary();
public:
	//note boundary vertices do NOT include isolated vertices
	std::vector<bool> bBoundary;
	std::vector<int> boundary_vts;
	std::vector< std::pair<int, int> > boundary_edges;

public:
	void build_edges();
	int edge_global_index(int vtIdx_1, int vtIdx_2) const
	{
		if (vtIdx_1 == vtIdx_2)
			return -1;
		std::pair<int, int> vt_pair = (vtIdx_1 < vtIdx_2) ? make_pair(vtIdx_1, vtIdx_2) : make_pair(vtIdx_2, vtIdx_1);
		std::map<std::pair<int, int>, int>::const_iterator iter = edges_map.find(vt_pair);
		if (iter != edges_map.end())
			return iter->second;
		else
			return -1;
	}
public:
	std::vector< std::pair<int, int> > edges;
	std::map<std::pair<int, int>, int> edges_map;

public:
	// if there is no normals in the structure, genereate the normal of one vertex 
	// based on the normals of facades which contains the vertex.
	// looking from the normal direction, the triangle vertices is in couter clockwise order
	void generateNormals();
	void smoothNormals();

	void resize(int vtNum, bool bUseColor, bool bUseNormal);
	void expand_data(bool bExpandColor, bool bExpandNormal);

	//update normals: if normal vector does not exist, return false;
	// looking from the normal direction, the triangle vertices is in couter clockwise order
	bool updateNormals();

	double* get_center() const; 
	BoundingBox3D get_bbox() const;
	double* get_avg_normal();
	void centralize();
	void reverse_normal()
	{
		if( !this->haveNormalInfo() )
			return;
		for(int i=0; i<this->vtNum; i++)
		{
			T* n = this->vt_normal(i);
			n[0] *= -1.0;
			n[1] *= -1.0;
			n[2] *= -1.0;
		}
	}

	bool copy_texture_from_other( CSurface<T> const& other);

	bool CSurface<T>::create_and_init_colors_of_view(int viewNum, bool bInitAsVertexColor=true);

	void reset_clr()
	{
		if( this->haveColorInfo() )
		{
			for(int i=0; i<this->vtNum; i++)
			{
				T *clr = this->vt_color(i);
				memset(clr, 0, sizeof(T)*3);
			}
		}
	}

	void color_surface_with_normal_map()
	{
		if (!this->haveNormalInfo())
			return;
		this->expand_data(true, false);

		for (int vtIdx = 0; vtIdx < vtNum; vtIdx++)
		{
			T* n = vt_normal(vtIdx);
			T* clr = vt_color(vtIdx);
			clr[0] = (-n[0] + 1.0f) / 2.0;
			clr[1] = (n[1] + 1.0f) / 2.0;
			clr[2] = (n[2] + 1.0f) / 2.0;
		}
	}

	void hide_untextured_vertex()
	{
		if( !this->haveColorInfo() )
			return;
		
		vector<bool> bFlags(this->vtNum);
		for(int i=0; i<this->vtNum; i++)
		{
			T* clr = this->vt_color(i);
			if( clr[0]==0 && clr[1]==0 && clr[2]==0)
				bFlags[i] = true;
			else
				bFlags[i] = false;
		}

		for(int i=0; i<this->triNum; i++)
		{
			int& vtIdx1 = this->triangles[3*i];
			int& vtIdx2 = this->triangles[3*i+1];
			int& vtIdx3 = this->triangles[3*i+2];

			if( !bFlags[vtIdx1] && !bFlags[vtIdx2] && !bFlags[vtIdx3] )
				continue;

			if( bFlags[vtIdx1] && bFlags[vtIdx2] && bFlags[vtIdx3] )
			{
				vtIdx1 = 0;
				vtIdx2 = 0;
				vtIdx3 = 0;
			}

		}

	}
	void scale(double s)
	{
		for (int vtIdx = 0; vtIdx < this->vtNum; vtIdx++)
		{
			T *vt = this->vt_data_block(vtIdx);
			vt[0] *= s;
			vt[1] *= s;
			vt[2] *= s;
		}
	}
	void shrink(int num);

	// for the following vertices extraction or deletion operations, colors_of_view is not handled at all.
	// delete vertex: if a vertex is in the list, the vertex connected with it will
	// also be deleted if it is not connected with others
	void delete_vertices( std::vector<int> const& vt_indices);
	void delete_duplicate_vertices();
	CSurface<T>* extract_surface( std::vector<int> const& vt_indices) const;
	void delete_vertices_inside_box( BoundingBox3D bbox );
	void delete_vertices_outside_box( BoundingBox3D bbox );
	void delete_vertices_inside_box( BoundingBox3D bbox, std::vector<int> &vertices_to_delete );
	void delete_vertices_outside_box( BoundingBox3D bbox, std::vector<int> &vertices_to_delete );

	// delete the vertices that are not used in triangles, the order of vertices is preserved
	std::vector<bool> bExtra;//indicate whether a vertex is indexed by triangles (TRUE: NOT indexed)
	void build_bExtra();
	void delete_extra_vertice(std::vector<int> &LUT_new2Old = std::vector<int>());

	bool haveColorInfo() const {return color;}
	bool haveNormalInfo() const {return normal;}

	// read in file based on its extension(.bin or .txt)
	bool readFromFile(const char* filename);

	int writeToFileASCII(const char *filename) const;
	int readFromFileASCII(const char *filename);

	bool writeToFilePLY(const char* filename, char* comments=NULL);
	bool readFromFilePLY(const char* filename);

	int writeToFileBIN(const char *filename) const;
	int readFromFileBIN(const char *filename);

	// I/O for ".mesh"
	bool readFromMesh(const char* filename);

public:
	T* vt_data_block(int vtIdx)
	{
		return &(vtData[vtIdx*vtDim]);
	}

	T const* vt_data_block(int vtIdx) const
	{
		return &(vtData[vtIdx*vtDim]);
	}

	T* vt_normal(int vtIdx)
	{
		return &(vtData[vtIdx*vtDim+3]);
	}

	T const* vt_normal(int vtIdx) const
	{
		return &(vtData[vtIdx*vtDim+3]);
	}

	T* vt_color(int vtIdx)
	{
		return &(vtData[(vtIdx+1)*vtDim-3]);
	}

	T const* vt_color(int vtIdx) const
	{
		return &(vtData[(vtIdx+1)*vtDim-3]);
	}


#ifdef USING_GLMOBJ
	void writeTOFileOBJ(const char* filename);
	
	//Read polygon mesh from Wavefront OBJ format
	//not all the data is read (sometimes only vertex data is loaded, sometimes both vertex and normal are loaded!)
	int readFromObj(const char *filename);
	int readFromObj(GLMmodel *obj_model);

	//color info is abanded
	GLMmodel* convertToObj();
#endif

	int readFromIsoSurface(CIsoSurface<T> *surface_model);

	void freeMemory()
	{
		if(vtData != 0)
		{
			delete [] vtData;
			vtData = NULL;
		}
		if(triangles != 0)
		{
			delete [] triangles;
			triangles = NULL;
		}

		for(int i=0; i<colors_of_view.size(); i++)
		{
			if( colors_of_view[i] != NULL )
			{
				delete [] colors_of_view[i];
				colors_of_view[i] = NULL;
			}
		}
		this->colors_of_view.clear();
		this->color = 0;
		this->normal = 0;
	}
};
template <class T>
CSurface<T>* cloneCSurface(CSurface<T> const* surf);
template <class T>
void releaseSurfaces(std::vector<CSurface<T>*> &surfaces)
{
	for(int i=0; i<surfaces.size(); i++)
	{
		if( surfaces[i] != NULL )
			delete surfaces[i];
	}

	surfaces.clear();
}

typedef CSurface<float> CSurface_F;
typedef CSurface<double> CSurface_D;

#include "CSurface.hpp"
#endif