#include "CSurface.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "VecOperation.h"
using namespace std;

CSurface::CSurface(const CSurface &other)
{
	//printf("Error: Copy function to be implemented!\n");
	this->vtDim = other.vtDim;
	this->vtNum = other.vtNum;
	this->triNum = other.triNum;
	this->color = other.color;
	this->normal = other.normal;

	this->vtData = new float[this->vtNum*this->vtDim];
	memcpy(this->vtData, other.vtData, this->vtNum*this->vtDim*sizeof(float));
	this->triangles = new int[triNum*3];
	memcpy(this->triangles, other.triangles, triNum*3*sizeof(int));
}

CSurface& 
CSurface::operator=( CSurface const& rhs )
{
	this->vtDim = rhs.vtDim;
	this->vtNum = rhs.vtNum;
	this->triNum = rhs.triNum;
	this->color = rhs.color;
	this->normal = rhs.normal;

	if( this->vtData != NULL )
		delete [] this->vtData;
	this->vtData = new float[this->vtNum*this->vtDim];
	memcpy(this->vtData, rhs.vtData, this->vtNum*this->vtDim*sizeof(float));

	if(this->triangles != NULL )
		delete [] this->triangles;
	this->triangles = new int[triNum*3];
	memcpy(this->triangles, rhs.triangles, triNum*3*sizeof(int));

	return *this;
}

bool CSurface::readFromFile(const char* filename)
{
	int len = strlen(filename);
	if( len > 3 )
	{
		if( strcmp("bin", &filename[len-3])==0)
		{
			if( this->readFromFileBIN(filename) < 0)
			{
				printf("Error: file name <%s> is illegal!\n", filename);
				return false;
			}
		}
		else
		{
			if( this->readFromFileASCII(filename) < 0)
			{
						
				printf("Error: file name <%s> is illegal!\n", filename);
				return false;
			}
		}
		return true;
	}
	else
	{
		printf("Error: file name <%s> is illegal!\n", filename);
		return false;
	}	
}

int CSurface::writeToFileASCII(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "w")) != 0 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	fprintf(file, "%d\n", this->vtDim);
	fprintf(file, "%d\n", int(this->normal));
	fprintf(file, "%d\n\n", int(this->color));

	fprintf(file, "%d\n", this->vtNum);
	for( int i=0; i<this->vtNum; i++)
	{
		for(int j=0; j<this->vtDim; j++)
			fprintf(file, "%f ", vtData[vtDim*i+j]);
		fprintf(file, "\n");
	}

	fprintf(file, "%d\n", this->triNum);
	for( int i=0; i<this->triNum; i++)
	{
		fprintf(file, "%d %d %d\n", triangles[3*i], triangles[3*i+1], triangles[3*i+2]);
	}

	fclose(file);
	return 1;
}

bool CSurface::writeToFilePLY(const char* filename, char* comments)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "w")) != 0 )
	{
		printf("Error when Writing File %s\n", filename);
		return false;
	}

	fprintf(file, "ply\n");
	fprintf(file, "format ascii 1.0\n");
	if( comments != NULL )
		fprintf(file, "comment %s\n", comments);
	
	fprintf(file, "element vertex %d\n", this->vtNum);
	fprintf(file, "property float x\n");
	fprintf(file, "property float y\n");
	fprintf(file, "property float z\n");
	if( this->haveNormalInfo() )
	{
		fprintf(file, "property float nx\n");
		fprintf(file, "property float ny\n");
		fprintf(file, "property float nz\n");
	}
	if( this->haveColorInfo() )
	{
		fprintf(file, "property uchar red\n");
		fprintf(file, "property uchar green\n");
		fprintf(file, "property uchar blue\n");
	}

	fprintf(file, "element face %d\n", this->triNum);
	fprintf(file, "property list uchar int vertex_indices\n");
	fprintf(file, "end_header\n");

	for(int i=0; i<this->vtNum; i++)
	{
		for(int j=0; j<this->vtDim-3; j++)
		{
			if( _isnan(this->vtData[i*vtDim+j]) )
				fprintf(file, "0 ");
			else
				fprintf(file, "%f ", this->vtData[i*vtDim+j]);
		}
		fprintf(file, "%d %d %d ", int(this->vtData[i*vtDim+vtDim-3]*255), int(this->vtData[i*vtDim+vtDim-2]*255),
								   int(this->vtData[i*vtDim+vtDim-1]*255));
		fprintf(file, "\n");
	}
	for(int i=0; i<this->triNum; i++)
	{
		fprintf(file, "3 %d %d %d\n", this->triangles[3*i], this->triangles[3*i+1], this->triangles[3*i+2]);
	}
	fclose(file);
	return true;
}

int CSurface::readFromFileASCII(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "r")) != 0 )
	{
		printf("Error when Reading File %s\n", filename);
		return -1;
	}

	int isColor;
	int isNormal;

	fscanf(file, "%d\n", &vtDim);
	fscanf(file, "%d\n", &isNormal);
	fscanf(file, "%d\n", &isColor);

	if(isColor > 0)
		this->color = true;
	else
		this->color = false;
	if(isNormal > 0)
		this->normal = true;
	else
		this->normal = false;

	fscanf(file, "%d\n", &vtNum);
	if( this->vtData != NULL ) 
		delete [] vtData;
	vtData = new float[vtNum*vtDim];

	for( int i=0; i<this->vtNum; i++)
	{
		for(int j=0; j<vtDim; j++)
			fscanf(file, "%f", &vtData[vtDim*i+j]);
	}
	
	fscanf(file, "%d\n", &triNum);
	if( this->triangles != NULL) 
		delete [] triangles;
	triangles = new int[triNum*3];

	for( int i=0; i<this->triNum; i++)
	{
		fscanf(file, "%d %d %d\n", &triangles[3*i], &triangles[3*i+1], &triangles[3*i+2]);
	}

	fclose(file);
	return 1;
}

/*
int CSurface::writeToFileBIN(char *filename)
{
	ofstream file(filename, ofstream::binary);
	file.write((char*)(&vtnmNum), sizeof(int));
	file.write((char*)vtnmData, sizeof(float)*vtnmNum*6);
	file.write((char*)(&triNum), sizeof(int));
	file.write((char*)triangles, sizeof(int)*triNum*3);
	file.close();
	return 1;
}*/


int CSurface::writeToFileBIN(const char *filename)
{
	int numwritten;
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing Surface File\n");
		return -1;
	}

	numwritten = fwrite(&vtDim, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing Surface File\n");
		return -1;
	}

	int isNormal = int(normal);
	numwritten = fwrite(&isNormal, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing Surface File\n");
		return -1;
	}

	int isColor = int(color);
	numwritten = fwrite(&isColor, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing Surface File\n");
		return -1;
	}

	numwritten = fwrite(&vtNum, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing Surface File\n");
		return -1;
	}
	
	numwritten = fwrite(vtData, sizeof(float), vtNum*vtDim, file);
	if( numwritten != vtNum*vtDim )
	{
		printf("Error when writing vertex data\n");
		return -1;
	}

	numwritten = fwrite(&triNum, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	numwritten = fwrite(triangles, sizeof(int), triNum*3, file);
	if( numwritten != triNum*3 )
	{
		printf("Error when Writing triangles\n");
		return -1;
	}

	fclose(file);
	printf("%d vertices and %d triangles have been written into file\n", this->vtNum, this->triNum);
	return 1;
}

/*
int CSurface::readFromFileBIN(char *filename)
{
	ifstream file(filename, ifstream::binary);
	
	file.read((char*)(&vtnmNum), sizeof(int));
	
	if( this->vtnmData != NULL ) delete [] vtnmData;
	vtnmData = new float[vtnmNum*6];
	file.read((char*)vtnmData, sizeof(float)*vtnmNum*6);

	file.read((char*)(&triNum), sizeof(int));
	if( this->triangles != NULL) delete [] triangles;
	triangles = new int[triNum*3];
	file.read((char*)triangles, sizeof(int)*triNum*3);

	file.close();	
	return 1;
}*/


int CSurface::readFromFileBIN(const char *filename)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File1\n");
		return -1;
	}

	numread = fread(&vtDim, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File2\n");
		return -1;
	}

	int isNormal;
	numread = fread(&isNormal, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File3\n");
		return -1;
	}
	if(isNormal > 0)
		normal = true;
	else
		normal = false;

	int isColor;
	numread = fread(&isColor, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File4\n");
		return -1;
	}
	if(isColor>0)
		color = true;
	else
		color = false;


	numread = fread(&vtNum, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File5\n");
		return -1;
	}
	
	if( this->vtData != NULL ) delete [] vtData;
	vtData = new float[vtNum*vtDim];
	numread = fread(vtData, sizeof(float), vtNum*vtDim, file);
	if( numread != vtNum*vtDim )
	{
		if(feof(file)!=0)
			printf("End of File\n");
		else if(ferror(file) != 0)
			printf("Error\n");
		else
			printf("Error when Reading vertex data\n");
		return -1;
	}
	printf("%d vertices are read\n", vtNum);

	numread = fread(&triNum, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		return -1;
	}

	if( this->triangles != NULL) delete [] triangles;
	triangles = new int[triNum*3];
	numread = fread(triangles, sizeof(int), triNum*3, file);
	if( numread != triNum*3 )
	{
		printf("Error when Reading triangles\n");
		return -1;
	}
	printf("%d triangles are read\n", triNum);

	fclose(file);
	return 1;
}

void CSurface::generateNormals()
{
	if(haveNormalInfo())
	{
		printf("Error: normals alread exist!\n");
		return;
	}

	if( this->triangles == 0 || this->vtNum == 0)
	{
		printf("Error: No triangle data or no vertex data!\n");
		return;
	}

	float *normalData = new float[this->vtNum*3];
	int *adjacentFacetNum = new int[this->vtNum];
	memset(adjacentFacetNum, 0, sizeof(int)*vtNum);

	//update the normal of each vertex
	for(int i=0; i<this->triNum; i++)
	{
		int idx1 = this->triangles[i*3];
		int idx2 = this->triangles[i*3+1];
		int idx3 = this->triangles[i*3+2];

		float* p1 = &vtData[idx1*vtDim];
		float* p2 = &vtData[idx2*vtDim];
		float* p3 = &vtData[idx3*vtDim];
		
		float e1[3];
		float e2[3];
		float normal[3];

		VecOperation<float>::VecSub(p1, p2, e1, 3);
		VecOperation<float>::VecSub(p1, p3, e2, 3);
		VecOperation<float>::CrossProdVec3(e1, e2, normal);
		VecOperation<float>::Unitalization(normal, 3);

		if( adjacentFacetNum[idx1] > 0 )
		{
			float *prev_normal = &normalData[idx1*3];
			if(VecOperation<float>::DotProd(normal, prev_normal, 3) > 0)
				VecOperation<float>::VecAdd(normal, prev_normal, prev_normal, 3);
			else
				VecOperation<float>::VecSub(prev_normal, normal, prev_normal, 3 );
			adjacentFacetNum[idx1] ++;
		}
		else
		{
			float *prev_normal = &normalData[idx1*3];
			VecOperation<float>::VecCopy(normal, prev_normal, 3);
			adjacentFacetNum[idx1] = 1;
		}
		
		if( adjacentFacetNum[idx2] > 0 )
		{
			float *prev_normal = &normalData[idx2*3];
			if(VecOperation<float>::DotProd(normal, prev_normal, 3) > 0)
				VecOperation<float>::VecAdd(normal, prev_normal, prev_normal, 3);
			else
				VecOperation<float>::VecSub(prev_normal, normal, prev_normal, 3);
			adjacentFacetNum[idx2] ++;
		}
		else
		{
			float *prev_normal = &normalData[idx2*3];
			VecOperation<float>::VecCopy(normal, prev_normal, 3);
			adjacentFacetNum[idx2] = 1;
		}

		if( adjacentFacetNum[idx3] > 0 )
		{
			float *prev_normal = &normalData[idx3*3];
			if(VecOperation<float>::DotProd(normal, prev_normal, 3) > 0)
				VecOperation<float>::VecAdd(normal, prev_normal, prev_normal, 3);
			else
				VecOperation<float>::VecSub(prev_normal, normal, prev_normal, 3 );
			adjacentFacetNum[idx3] ++;
		}
		else
		{
			float *prev_normal = &normalData[idx3*3];
			VecOperation<float>::VecCopy(normal, prev_normal, 3);
			adjacentFacetNum[idx3] = 1;
		}
	}

	//unitilize the normal vector
	for(int i=0; i<this->vtNum; i++)
	{
		float* cur_normal = &normalData[i*3];
		VecOperation<float>::Unitalization(cur_normal, 3);
	}

	//combine vertex data with normal data
	int new_vtDim = 3 + this->vtDim;
	float *new_vtData = new float[new_vtDim*vtNum];
	for(int i=0; i<vtNum; i++)
	{
		new_vtData[i*new_vtDim] = this->vtData[i*vtDim];
		new_vtData[i*new_vtDim+1] = this->vtData[i*vtDim+1];
		new_vtData[i*new_vtDim+2] = this->vtData[i*vtDim+2];
		new_vtData[i*new_vtDim+3] = normalData[3*i];
		new_vtData[i*new_vtDim+4] = normalData[3*i+1];
		new_vtData[i*new_vtDim+5] = normalData[3*i+2];
	}

	if(this->haveColorInfo())
	{
		//copy color info
		for(int i=0; i<this->vtNum; i++)
		{
			new_vtData[i*new_vtDim+6] = this->vtData[i*vtDim+3];
			new_vtData[i*new_vtDim+7] = this->vtData[i*vtDim+4];
			new_vtData[i*new_vtDim+8] = this->vtData[i*vtDim+5];
		}
	}

	delete [] this->vtData;
	this->vtData = new_vtData;
	this->vtDim = new_vtDim;
	this->normal = true;

	delete [] normalData;
	delete [] adjacentFacetNum;
}

void CSurface::centralize()
{
	double x_cen = 0.0;
	double y_cen = 0.0;
	double z_cen = 0.0;
	for(int i=0; i<this->vtNum; i++)
	{
		x_cen += this->vtData[i*this->vtDim];
		y_cen += this->vtData[i*this->vtDim+1];
		z_cen += this->vtData[i*this->vtDim+2];
	}
	x_cen /= this->vtNum;
	y_cen /= this->vtNum;
	z_cen /= this->vtNum;

	for(int i=0; i<this->vtNum; i++)
	{
		this->vtData[i*this->vtDim  ] -= x_cen;
		this->vtData[i*this->vtDim+1] -= y_cen;
		this->vtData[i*this->vtDim+2] -= z_cen;
	}
}

#ifdef USING_GLMOBJ
int CSurface::readFromObj(const char *filename)
{
	GLMmodel *obj_model = glmReadOBJ(filename);
	if(obj_model == NULL)
		return -1;
	
	int ret = readFromObj(obj_model);
	glmDelete(obj_model);
	return ret;
}

int CSurface::readFromObj(GLMmodel *obj_model)
{
	if( obj_model->numvertices != obj_model->numnormals )
	{
		printf("Warning: The vertex num does not equal to normal num! Only the vertex is read!\n");
		this->freeMemory();
		
		this->vtDim = 3;
		this->color = false;
		this->normal = false;
		this->vtNum = obj_model->numvertices;
		this->vtData = new float[vtNum*3];
		for( int i=0; i<vtNum; i++ )
		{
			/* fetch from index 1 */
			vtData[3*i] = obj_model->vertices[3*(i+1)];
			vtData[3*i+1] = obj_model->vertices[3*(i+1)+1];
			vtData[3*i+2] = obj_model->vertices[3*(i+1)+2];
		}

		this->triNum = obj_model->numtriangles;
		this->triangles = new int[triNum*3];
		for( int i=0; i<triNum; i++)
		{
			//note that OBJ file counts from 1, instead of 0
			this->triangles[3*i] = obj_model->triangles[i].vindices[0] - 1;
			this->triangles[3*i+1] = obj_model->triangles[i].vindices[1] - 1;
			this->triangles[3*i+2] = obj_model->triangles[i].vindices[2] - 1;
		}

		return 2;
	}

	this->freeMemory();
	//============ the following code is based on the assumption that ============
	//============ the i-th vertex corresponds to i-th normal	=================
	this->vtDim = 6;
	this->color = false;
	this->normal = true;
	this->vtNum = obj_model->numvertices;
	this->vtData = new float[vtNum*6];
	for( int i=0; i<vtNum; i++ )
	{
		/* fetch from index 1 */
		vtData[6*i] = obj_model->vertices[3*(i+1)];
		vtData[6*i+1] = obj_model->vertices[3*(i+1)+1];
		vtData[6*i+2] = obj_model->vertices[3*(i+1)+2];

		vtData[6*i+3] = obj_model->normals[3*(i+1)];
		vtData[6*i+4] = obj_model->normals[3*(i+1)+1];
		vtData[6*i+5] = obj_model->normals[3*(i+1)+2];
	}

	this->triNum = obj_model->numtriangles;
	this->triangles = new int[triNum*3];
	for( int i=0; i<triNum; i++)
	{
		//note that OBJ file counts from 1, instead of 0
		this->triangles[3*i] = obj_model->triangles[i].vindices[0] - 1;
		this->triangles[3*i+1] = obj_model->triangles[i].vindices[1] - 1;
		this->triangles[3*i+2] = obj_model->triangles[i].vindices[2] - 1;
	}

	return 1;
}

void CSurface::writeTOFileOBJ(const char* filename)
{
	GLMmodel *obj_model = convertToObj();
	glmWriteOBJ(obj_model, (char*)filename, GLM_SMOOTH);

	glmDelete(obj_model);
}


GLMmodel* CSurface::convertToObj()
{
	GLMmodel* obj_model = new GLMmodel();
	obj_model->pathname = NULL;
	obj_model->mtllibname = NULL;
	
	obj_model->numvertices = this->vtNum;
	float* vert = new float[this->vtNum*3+3];
	for(int i=1; i<=this->vtNum; i++)
	{
		vert[3*i] = this->vtData[(i-1)*vtDim];//waterSurface->m_ppt3dVertices[i-1][0];
		vert[3*i+1] = this->vtData[(i-1)*vtDim+1];//GRIDHEIGHT-1-waterSurface->m_ppt3dVertices[i-1][1];
		vert[3*i+2] = this->vtData[(i-1)*vtDim+2];//-waterSurface->m_ppt3dVertices[i-1][2];
	}
	obj_model->vertices = vert;

	if(haveNormalInfo())
	{
		obj_model->numnormals = this->vtNum;
		float* norm = new float[this->vtNum*3+3];
		for(int i=1; i<=this->vtNum; i++)
		{
			norm[3*i] = this->vtData[(i-1)*vtDim+3];//waterSurface->m_pvec3dNormals[i-1][0];
			norm[3*i+1] = this->vtData[(i-1)*vtDim+4];//waterSurface->m_pvec3dNormals[i-1][1];
			norm[3*i+2] = this->vtData[(i-1)*vtDim+5];//-waterSurface->m_pvec3dNormals[i-1][2];
		}
		obj_model->normals = norm;
	}
	else
	{
		obj_model->numnormals = 0;
		obj_model->normals = NULL;
	}

	obj_model->numtexcoords = 0;
	obj_model->texcoords = NULL;

	obj_model->numfacetnorms = 0;
	obj_model->facetnorms = NULL;

	obj_model->numtriangles = this->triNum;
	GLMtriangle* tri = new GLMtriangle[this->triNum];
	for( int i=0; i<this->triNum; i++)
	{
		tri[i].vindices[0] = this->triangles[3*i]+1;
		tri[i].vindices[1] = this->triangles[3*i+1]+1;
		tri[i].vindices[2] = this->triangles[3*i+2]+1;

		if(haveNormalInfo())
		{
			tri[i].nindices[0] = this->triangles[3*i]+1;
			tri[i].nindices[1] = this->triangles[3*i+1]+1;
			tri[i].nindices[2] = this->triangles[3*i+2]+1;
		}
	}
	obj_model->triangles = tri;

	obj_model->nummaterials = 0;
	obj_model->materials = NULL;

	obj_model->numgroups = 1;
	GLMgroup* m_group = new GLMgroup();
	m_group->name = __glmStrdup("surface");
	m_group->numtriangles = this->triNum;
	GLuint* triIdx = new GLuint[this->triNum];
	for( int i=0; i<this->triNum; i++)
	{
		triIdx[i] = i;
	}
	m_group->triangles = triIdx;
	m_group->next = NULL;
	m_group->material = 0;
	obj_model->groups = m_group;

	obj_model->position[0] = 0;
	obj_model->position[1] = 0;
	obj_model->position[2] = 0;

	return obj_model;
}
#endif


int CSurface::readFromIsoSurface(CIsoSurface<float> *surface_model)
{
	if( surface_model->m_nVertices != surface_model->m_nNormals )
	{
		printf("The vertex num does not equal to normal num! The program needs to be changed a little bit\n");
		return -1;
	}
	this->freeMemory();

	this->color = false;
	this->normal = true;
	this->vtDim = 6;
	this->vtNum = surface_model->m_nVertices;
	this->vtData = new float[vtNum*6];
	for( int i=0; i<vtNum; i++ )
	{
		vtData[6*i] = surface_model->m_ppt3dVertices[i][0];
		vtData[6*i+1] = surface_model->m_ppt3dVertices[i][1];
		vtData[6*i+2] = surface_model->m_ppt3dVertices[i][2];

		vtData[6*i+3] = surface_model->m_pvec3dNormals[i][0];
		vtData[6*i+4] = surface_model->m_pvec3dNormals[i][1];
		vtData[6*i+5] = surface_model->m_pvec3dNormals[i][2];
	}

	this->triNum = surface_model->m_nTriangles;
	this->triangles = new int[triNum*3];
	for( int i=0; i<triNum; i++ )
	{
		this->triangles[3*i] = surface_model->m_piTriangleIndices[3*i];
		this->triangles[3*i+1] = surface_model->m_piTriangleIndices[3*i+1];
		this->triangles[3*i+2] = surface_model->m_piTriangleIndices[3*i+2];
	}
	return 1;
}

CSurface* cloneCSurface(CSurface* surf)
{
	if(surf == NULL)
		return NULL;

	CSurface *ret = new CSurface();
	int *triangles = new int[surf->triNum*3];
	memcpy(triangles, surf->triangles, surf->triNum*3*sizeof(int));
	float *vtData = new float[surf->vtNum*surf->vtDim];
	memcpy(vtData, surf->vtData, surf->vtNum*surf->vtDim*sizeof(float));
	ret->triNum = surf->triNum;
	ret->vtNum = surf->vtNum;
	ret->vtDim = surf->vtDim;
	ret->color = surf->color;
	ret->normal = surf->normal;
	ret->triangles = triangles;
	ret->vtData = vtData;
	return ret;
}
