#include "DFSPH_rendering_cuda.h"

#include "DFSPH_define_cuda.h"
#include "DFSPH_macro_cuda.h"

#include "DFSPH_c_arrays_structure.h"
#include "SPH_other_systems_cuda.h"

#include <iostream>

namespace RenderingCuda
{
	__global__ void init_buffer_kernel(Vector3d* buff, unsigned int size,Vector3d val) {
		int i = blockIdx.x * blockDim.x + threadIdx.x;
		if (i >= size) { return; }

		buff[i] = (i>(size/2))?val+val:val;

	}
}





void cuda_opengl_initParticleRendering(ParticleSetRenderingData& renderingData, unsigned int numParticles,
	Vector3d** pos, Vector3d** vel, bool need_color_buffer, Vector3d** color) {

	//read_last_error_cuda("before alloc rendering on gpu: ");

	glGenVertexArrays(1, &renderingData.vao); // Cr�er le VAO
	glBindVertexArray(renderingData.vao); // Lier le VAO pour l'utiliser


	glGenBuffers(1, &renderingData.pos_buffer);
	// selectionne le buffer pour l'initialiser
	glBindBuffer(GL_ARRAY_BUFFER, renderingData.pos_buffer);
	// dimensionne le buffer actif sur array_buffer, l'alloue et l'initialise avec les positions des sommets de l'objet
	glBufferData(GL_ARRAY_BUFFER,
		/* length */	numParticles * sizeof(Vector3d),
		/* data */      NULL,
		/* usage */     GL_DYNAMIC_DRAW);
	//set it to the attribute
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FORMAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &renderingData.vel_buffer);
	// selectionne le buffer pour l'initialiser
	glBindBuffer(GL_ARRAY_BUFFER, renderingData.vel_buffer);
	// dimensionne le buffer actif sur array_buffer, l'alloue et l'initialise avec les positions des sommets de l'objet
	glBufferData(GL_ARRAY_BUFFER,
		/* length */	numParticles * sizeof(Vector3d),
		/* data */      NULL,
		/* usage */     GL_DYNAMIC_DRAW);
	//set it to the attribute
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FORMAT, GL_FALSE, 0, 0);

	if (need_color_buffer) {
		glGenBuffers(1, &renderingData.color_buffer);
		// selectionne le buffer pour l'initialiser
		glBindBuffer(GL_ARRAY_BUFFER, renderingData.color_buffer);
		// dimensionne le buffer actif sur array_buffer, l'alloue et l'initialise avec les positions des sommets de l'objet
		glBufferData(GL_ARRAY_BUFFER,
			/* length */	(numParticles) * sizeof(Vector3d),
			/* data */      NULL,
			/* usage */     GL_DYNAMIC_DRAW);
		//set it to the attribute
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 3, GL_FORMAT, GL_FALSE, 0, 0);
	}

	// nettoyage
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Registration with CUDA.
	gpuErrchk(cudaGraphicsGLRegisterBuffer(&renderingData.pos, renderingData.pos_buffer, cudaGraphicsRegisterFlagsNone));
	gpuErrchk(cudaGraphicsGLRegisterBuffer(&renderingData.vel, renderingData.vel_buffer, cudaGraphicsRegisterFlagsNone));
	if (need_color_buffer) {
		gpuErrchk(cudaGraphicsGLRegisterBuffer(&renderingData.color, renderingData.color_buffer, cudaGraphicsRegisterFlagsNone));
	}

	//link the pos and vel buffer to cuda
	gpuErrchk(cudaGraphicsMapResources(1, &renderingData.pos, 0));
	gpuErrchk(cudaGraphicsMapResources(1, &renderingData.vel, 0));
	if (need_color_buffer) {
		gpuErrchk(cudaGraphicsMapResources(1, &renderingData.color, 0));
	}

	//set the openglbuffer for direct use in cuda
	Vector3d* vboPtr = NULL;
	size_t size = 0;

	// pos
	gpuErrchk(cudaGraphicsResourceGetMappedPointer((void**)&vboPtr, &size, renderingData.pos));//get cuda ptr
	*pos = vboPtr;

	// vel
	gpuErrchk(cudaGraphicsResourceGetMappedPointer((void**)&vboPtr, &size, renderingData.vel));//get cuda ptr
	*vel = vboPtr;

	if (need_color_buffer) {
		gpuErrchk(cudaGraphicsResourceGetMappedPointer((void**)&vboPtr, &size, renderingData.color));//get cuda ptr
		*color = vboPtr;
	}
}

void cuda_opengl_releaseParticleRendering(ParticleSetRenderingData& renderingData) {
	//unlink the pos and vel buffer from cuda
	gpuErrchk(cudaGraphicsUnmapResources(1, &(renderingData.pos), 0));
	gpuErrchk(cudaGraphicsUnmapResources(1, &(renderingData.vel), 0));
	if (renderingData.color_buffer <= 100000) {
		gpuErrchk(cudaGraphicsUnmapResources(1, &renderingData.color, 0));
	}

	//delete the opengl buffers
	glDeleteBuffers(1, &renderingData.vel_buffer);
	glDeleteBuffers(1, &renderingData.pos_buffer);
	if (renderingData.color_buffer <= 100000) {
		glDeleteBuffers(1, &renderingData.color_buffer);
	}
	glDeleteVertexArrays(1, &renderingData.vao);
}

void cuda_opengl_renderParticleSet(ParticleSetRenderingData& renderingData, unsigned int numParticles) {


	//unlink the pos and vel buffer from cuda
	gpuErrchk(cudaGraphicsUnmapResources(1, &(renderingData.pos), 0));
	gpuErrchk(cudaGraphicsUnmapResources(1, &(renderingData.vel), 0));
	if (renderingData.color_buffer<=100000) {
		gpuErrchk(cudaGraphicsUnmapResources(1, &(renderingData.color), 0));
	}


	//Actual opengl rendering
	// link the vao
	glBindVertexArray(renderingData.vao);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//show it
	glDrawArrays(GL_POINTS, 0, numParticles);

	// unlink the vao
	glBindVertexArray(0);

	//link the pos and vel buffer to cuda
	gpuErrchk(cudaGraphicsMapResources(1, &renderingData.pos, 0));
	gpuErrchk(cudaGraphicsMapResources(1, &renderingData.vel, 0));
	if (renderingData.color_buffer <= 100000) {
		gpuErrchk(cudaGraphicsMapResources(1, &renderingData.color, 0));
	}

}

void cuda_renderFluid(SPH::DFSPHCData* data) {
	cuda_opengl_renderParticleSet(*data->fluid_data->renderingData, data->fluid_data[0].numParticles);
}



void cuda_renderBoundaries(SPH::DFSPHCData* data, bool renderWalls) {
	if (renderWalls) {
		cuda_opengl_renderParticleSet(*(data->boundaries_data->renderingData), data->boundaries_data->numParticles);
	}

	for (int i = 0; i < data->numDynamicBodies; ++i) {
		SPH::UnifiedParticleSet& body = data->vector_dynamic_bodies_data[i];
		cuda_opengl_renderParticleSet(*body.renderingData, body.numParticles);
	}
}



void cuda_reset_color(SPH::UnifiedParticleSet* particleSet) {
	if (particleSet->has_color_buffer) {
		int numBlocks = calculateNumBlocks(particleSet->numParticles);
		RenderingCuda::init_buffer_kernel << <numBlocks, BLOCKSIZE >> > (particleSet->color, particleSet->numParticles, Vector3d(-1));
		gpuErrchk(cudaDeviceSynchronize());
	}
}