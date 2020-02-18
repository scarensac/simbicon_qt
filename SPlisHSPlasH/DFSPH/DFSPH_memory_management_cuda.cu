#include "DFSPH_memory_management_cuda.h"

#include <stdio.h>
#include <chrono>
#include <iostream>
#include <thread>

#include "DFSPH_define_cuda.h"
#include "DFSPH_macro_cuda.h"
#include "DFSPH_static_variables_structure_cuda.h"


#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "DFSPH_c_arrays_structure.h"
#include "cub.cuh"



#include <curand.h>
#include <curand_kernel.h>


#include "basic_kernels_cuda.cuh"
#include "SPH_other_systems_cuda.h"

namespace MemoryManagementCuda
{
	__global__ void init_buffer_kernel(Vector3d* buff, unsigned int size, Vector3d val) {
		int i = blockIdx.x * blockDim.x + threadIdx.x;
		if (i >= size) { return; }

		buff[i] = val;
	}
}


void allocate_DFSPHCData_base_cuda(SPH::DFSPHCData& data) {
	if (data.damp_planes == NULL) {
		cudaMallocManaged(&(data.damp_planes), sizeof(Vector3d) * 10);
	}
	if (data.cancel_wave_planes == NULL) {
		cudaMallocManaged(&(data.cancel_wave_planes), sizeof(Vector3i) * 2);
	}

	if (data.bmin == NULL) {

		cudaMallocManaged(&(data.bmin), sizeof(Vector3d));
		cudaMallocManaged(&(data.bmax), sizeof(Vector3d));
	}

	//alloc static variables
	SVS_CU::get();
}

void free_DFSPHCData_base_cuda(SPH::DFSPHCData& data) {
	CUDA_FREE_PTR(data.damp_planes);
	CUDA_FREE_PTR(data.cancel_wave_planes);


	CUDA_FREE_PTR(data.bmin);
	CUDA_FREE_PTR(data.bmax);

	//free static variables
	SVS_CU::get(true);
}



void allocate_UnifiedParticleSet_cuda(SPH::UnifiedParticleSet& container) {

	read_last_error_cuda("before alloc unified on gpu: ");

	//cudaMalloc(&(container.pos), container.numParticles * sizeof(Vector3d)); //use opengl buffer with cuda interop
	//cudaMalloc(&(container.vel), container.numParticles * sizeof(Vector3d)); //use opengl buffer with cuda interop
    cudaMallocManaged(&(container.mass), container.numParticlesMax * sizeof(RealCuda));


	if (container.has_factor_computation) {
		//*
		cudaMallocManaged(&(container.numberOfNeighbourgs), container.numParticlesMax * 3 * sizeof(int));
		cudaMallocManaged(&(container.neighbourgs), container.numParticlesMax * MAX_NEIGHBOURS * sizeof(int));

		cudaMallocManaged(&(container.density), container.numParticlesMax * sizeof(RealCuda));
		cudaMalloc(&(container.factor), container.numParticlesMax * sizeof(RealCuda));
		cudaMallocManaged(&(container.densityAdv), container.numParticlesMax * sizeof(RealCuda));

		cudaMalloc(&(container.kappa), container.numParticlesMax * sizeof(RealCuda));
		cudaMalloc(&(container.kappaV), container.numParticlesMax * sizeof(RealCuda));

		if (container.velocity_impacted_by_fluid_solver) {
            cudaMallocManaged(&(container.acc), container.numParticlesMax * sizeof(Vector3d));

#ifdef BENDER2019_BOUNDARIES
			cudaMallocManaged(&(container.X_rigids), container.numParticlesMax * sizeof(Vector3d));
			cudaMallocManaged(&(container.V_rigids), container.numParticlesMax * sizeof(RealCuda));
#endif

			//I need the allocate the memory cub need to compute the reduction
			//I need the avg pointer because cub require it (but i'll clear after the cub call)
			RealCuda* avg_density_err = SVS_CU::get()->avg_density_err;

			container.d_temp_storage = NULL;
			container.temp_storage_bytes = 0;
			cub::DeviceReduce::Sum(container.d_temp_storage, container.temp_storage_bytes,
				container.densityAdv, avg_density_err, container.numParticlesMax);
			// Allocate temporary storage
			cudaMalloc(&(container.d_temp_storage), container.temp_storage_bytes);

		}
		//*/

	}

	if (container.is_dynamic_object) {
		cudaMalloc(&(container.pos0), container.numParticlesMax * sizeof(Vector3d));
		cudaMalloc(&(container.F), container.numParticlesMax * sizeof(Vector3d));
	}

	

	gpuErrchk(cudaDeviceSynchronize());
}

void release_UnifiedParticleSet_cuda(SPH::UnifiedParticleSet& container) {
	CUDA_FREE_PTR(container.mass);

	if (container.has_factor_computation) {
		//*
		CUDA_FREE_PTR(container.numberOfNeighbourgs);
		CUDA_FREE_PTR(container.neighbourgs);

		CUDA_FREE_PTR(container.density);
		CUDA_FREE_PTR(container.factor);
		CUDA_FREE_PTR(container.densityAdv);

		CUDA_FREE_PTR(container.kappa);
		CUDA_FREE_PTR(container.kappaV);
		if (container.velocity_impacted_by_fluid_solver) {
			CUDA_FREE_PTR(container.acc);

#ifdef BENDER2019_BOUNDARIES
			CUDA_FREE_PTR(container.V_rigids);
			CUDA_FREE_PTR(container.X_rigids);
#endif

			CUDA_FREE_PTR(container.d_temp_storage);
			container.temp_storage_bytes = 0;
		}
		//*/

	}

	if (container.is_dynamic_object) {
		CUDA_FREE_PTR(container.F);
	}

	//delete the cpu buffers if there are some
	if (container.is_dynamic_object) {
		FREE_PTR(container.F_cpu);
	}
}




void load_UnifiedParticleSet_cuda(SPH::UnifiedParticleSet& container, Vector3d* pos, Vector3d* vel, RealCuda* mass) {

	gpuErrchk(cudaMemcpy(container.pos, pos, container.numParticles * sizeof(Vector3d), cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(container.vel, vel, container.numParticles * sizeof(Vector3d), cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(container.mass, mass, container.numParticles * sizeof(RealCuda), cudaMemcpyHostToDevice));

	if (container.is_dynamic_object) {
		int numBlocks = calculateNumBlocks(container.numParticles);
		gpuErrchk(cudaMemcpy(container.pos0, pos, container.numParticles * sizeof(Vector3d), cudaMemcpyHostToDevice));
		DFSPH_setVector3dBufferToZero_kernel << <numBlocks, BLOCKSIZE >> > (container.F, container.numParticles);
	}

	if (container.has_factor_computation) {

		gpuErrchk(cudaMemset(container.kappa, 0, container.numParticles * sizeof(RealCuda)));
		gpuErrchk(cudaMemset(container.kappaV, 0, container.numParticles * sizeof(RealCuda)));
		if (container.velocity_impacted_by_fluid_solver) {
		}
	}


}

void read_UnifiedParticleSet_cuda(SPH::UnifiedParticleSet& container, Vector3d* pos, Vector3d* vel, RealCuda* mass, Vector3d* pos0) {
	if (pos != NULL) {
		gpuErrchk(cudaMemcpy(pos, container.pos, container.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToHost));
	}

	if (vel != NULL) {
		gpuErrchk(cudaMemcpy(vel, container.vel, container.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToHost));
	}

	if (mass != NULL) {
		gpuErrchk(cudaMemcpy(mass, container.mass, container.numParticles * sizeof(RealCuda), cudaMemcpyDeviceToHost));
	}

	if (container.is_dynamic_object&&pos0 != NULL) {
		gpuErrchk(cudaMemcpy(pos0, container.pos0, container.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToHost));
	}
}

void copy_UnifiedParticleSet_cuda(SPH::UnifiedParticleSet& dst, SPH::UnifiedParticleSet& src, bool copy_warmstart) {
	if (dst.numParticles != src.numParticles) {
		std::string err_mess("copy_UnifiedParticleSet_cuda: cannot copy data if the number is not the same in both structures");
		std::cout << err_mess << std::endl;
		throw(err_mess);
	}

	gpuErrchk(cudaMemcpy(dst.pos, src.pos, dst.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToDevice));
	
	gpuErrchk(cudaMemcpy(dst.vel, src.vel, dst.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToDevice));

	gpuErrchk(cudaMemcpy(dst.mass, src.mass, dst.numParticles * sizeof(RealCuda), cudaMemcpyDeviceToDevice));

	gpuErrchk(cudaMemcpy(dst.color, src.color, dst.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToDevice));	
}

void read_rigid_body_force_cuda(SPH::UnifiedParticleSet& container) {
	if (container.is_dynamic_object) {
		if (container.F_cpu == NULL) {
			container.F_cpu = new Vector3d[container.numParticles];
		}

		gpuErrchk(cudaMemcpy(container.F_cpu, container.F, container.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToHost));
	}
}



__global__ void compute_fluid_impact_on_dynamic_body_kernel(SPH::UnifiedParticleSet* container, Vector3d rb_position,
	Vector3d* force, Vector3d* moment, Vector3d reduction_factor) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= container->numParticles) { return; }

	Vector3d F, M;

	F = container->F[i];
	F *= reduction_factor;
	M = (container->pos[i] - rb_position).cross(F);

	atomicAdd(&(force->x), F.x);
	atomicAdd(&(force->y), F.y);
	atomicAdd(&(force->z), F.z);
	atomicAdd(&(moment->x), M.x);
	atomicAdd(&(moment->y), M.y);
	atomicAdd(&(moment->z), M.z);
}

void compute_fluid_impact_on_dynamic_body_cuda(SPH::UnifiedParticleSet& container, Vector3d& force, Vector3d& moment,
	const Vector3d& reduction_factor) {
	Vector3d* force_cuda = SVS_CU::get()->force_cuda;
	Vector3d* moment_cuda = SVS_CU::get()->moment_cuda;
	*force_cuda = Vector3d(0, 0, 0);
	*moment_cuda = Vector3d(0, 0, 0);


	int numBlocks = calculateNumBlocks(container.numParticles);
	compute_fluid_impact_on_dynamic_body_kernel << <numBlocks, BLOCKSIZE >> > (container.gpu_ptr,
		container.rigidBody_cpu->position, force_cuda,
		moment_cuda, reduction_factor);
	gpuErrchk(cudaDeviceSynchronize());

	force = *force_cuda;
	moment = *moment_cuda;
}





__global__ void compute_fluid_boyancy_on_dynamic_body_kernel(SPH::UnifiedParticleSet* container, Vector3d* force, Vector3d* pt_appli) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= container->numParticles) { return; }



	//I use the abs just in case for some reason the vertical force is negative ...
	//By this I mena that the y component also contains the y component of the drag. but there
	//is no way to extract the actual boyancy, soand approximation will have to do
	RealCuda boyancy = container->F[i].y;
	RealCuda boyancy_abs = abs(boyancy);
	if (boyancy_abs>0) {
		Vector3d pt = container->pos[i] * boyancy_abs;

		//in the x componant I'll store the total abs
		atomicAdd(&(force->x), boyancy_abs);
		atomicAdd(&(force->y), boyancy);
		atomicAdd(&(pt_appli->x), pt.x);
		atomicAdd(&(pt_appli->y), pt.y);
		atomicAdd(&(pt_appli->z), pt.z);
	}
}

void compute_fluid_Boyancy_on_dynamic_body_cuda(SPH::UnifiedParticleSet& container, Vector3d& force, Vector3d& pt_appli) {
	Vector3d* force_cuda = SVS_CU::get()->force_cuda;
	Vector3d* pt_cuda = SVS_CU::get()->pt_cuda;
	*force_cuda = Vector3d(0, 0, 0);
	*pt_cuda = Vector3d(0, 0, 0);

	int numBlocks = calculateNumBlocks(container.numParticles);
	compute_fluid_boyancy_on_dynamic_body_kernel << <numBlocks, BLOCKSIZE >> > (container.gpu_ptr, force_cuda, pt_cuda);
	gpuErrchk(cudaDeviceSynchronize());

	force = *force_cuda;
	//if the sum of the force is non zero
	if (abs(force.y)>0) {
		pt_appli = *pt_cuda;

		//now compute the avg to get the actual point
		pt_appli = pt_appli / force.x;
		//and clear the x component
		force.x = 0;
	}
	else {
		force = Vector3d(0, 0, 0);
		pt_appli = Vector3d(0, 0, 0);
	}
}




void allocate_and_copy_UnifiedParticleSet_vector_cuda(SPH::UnifiedParticleSet** out_vector, SPH::UnifiedParticleSet* in_vector, int numSets) {

	gpuErrchk(cudaMalloc(out_vector, numSets * sizeof(SPH::UnifiedParticleSet)));

	//now set the gpu_ptr in eahc object so that it points to the right place
	for (int i = 0; i < numSets; ++i) {
		in_vector[i].gpu_ptr = *out_vector + i;
	}

	//before being able to fill the gpu array we need to make a copy of the data structure since
	//we will have to change the neighborsdataset from the cpu to the gpu
	//*
	SPH::UnifiedParticleSet* temp;
	temp = new SPH::UnifiedParticleSet[numSets];
	std::copy(in_vector, in_vector + numSets, temp);

	for (int i = 0; i < numSets; ++i) {
		SPH::UnifiedParticleSet& body = temp[i];

		//we need to toggle the flag that prevent the destructor from beeing called on release
		//since it's the cpu version that clear the memory buffers that are common to the two structures
		body.releaseDataOnDestruction = false;

		//the gpu unified particle set has a irect pointer to the gpu neughbor dataset
		body.neighborsDataSet = body.neighborsDataSet->gpu_ptr;
	}
	//*/

	gpuErrchk(cudaMemcpy(*out_vector, temp, numSets * sizeof(SPH::UnifiedParticleSet), cudaMemcpyHostToDevice));

	//Now I have to update the pointer of the cpu set so that it point to the gpu structure
	delete[] temp;

}




void allocate_grouped_neighbors_struct_cuda(SPH::DFSPHCData& data) {
	std::cout << "initialising aggregated structure" << std::endl;

	if (data.numDynamicBodies < 1) {
		std::cout << "no dynamic bodies detected" << std::endl;

		//ok so I need the grouped buffer because I reuse it for external forces computation
		//note if I want to rmv that restriction I only have to use the one 
		//TODO apply that modification
		cudaMalloc(&(data.posBufferGroupedDynamicBodies), data.fluid_data->numParticlesMax * sizeof(Vector3d));

		return;
	}

	if (data.neighborsDataSetGroupedDynamicBodies != NULL || data.posBufferGroupedDynamicBodies != NULL) {
		throw("allocate_grouped_neighbors_struct_cuda already allocated");
	}

	int numParticles = 0;
	int numParticlesMax = 0;
	if (data.is_fluid_aggregated) {
		numParticles += data.fluid_data->numParticles;
		numParticlesMax += data.fluid_data->numParticlesMax;
	}

	for (int i = 0; i<data.numDynamicBodies; ++i) {
		numParticles += data.vector_dynamic_bodies_data[i].numParticles;
		numParticlesMax += data.vector_dynamic_bodies_data[i].numParticlesMax;
	}

	//allocate the dataset
	data.neighborsDataSetGroupedDynamicBodies = new SPH::NeighborsSearchDataSet(numParticles, numParticlesMax);

	//read gpu ptr
	data.neighborsDataSetGroupedDynamicBodies_cuda = data.neighborsDataSetGroupedDynamicBodies->gpu_ptr;


	//now it's like the normal neighbor search excapt that we have to iterate on all the solid particles
	//instead of just one buffer
	//the easiest way is to build a new pos array that contains all the dynamic particles
	cudaMalloc(&(data.posBufferGroupedDynamicBodies), numParticlesMax * sizeof(Vector3d));

}


void free_grouped_neighbors_struct_cuda(SPH::DFSPHCData& data) {
	FREE_PTR(data.neighborsDataSetGroupedDynamicBodies);

	CUDA_FREE_PTR(data.posBufferGroupedDynamicBodies);
}


void update_neighborsSearchBuffers_UnifiedParticleSet_vector_cuda(SPH::UnifiedParticleSet** out_vector, SPH::UnifiedParticleSet* in_vector, int numSets) {
	SPH::UnifiedParticleSet* temp;
	temp = new SPH::UnifiedParticleSet[numSets];

	gpuErrchk(cudaMemcpy(temp, *out_vector, numSets * sizeof(SPH::UnifiedParticleSet), cudaMemcpyDeviceToHost));

	for (int i = 0; i < numSets; ++i) {
		SPH::UnifiedParticleSet& body = temp[i];

		//we need to toggle the flag that prevent the destructor from beeing called on release
		//since it's the cpu version that clear the memory buffers that are common to the two structures
		body.releaseDataOnDestruction = false;

		//update the neighbor dataset to the cpu
		gpuErrchk(cudaMemcpy(body.neighborsDataSet, in_vector[i].neighborsDataSet,
			sizeof(SPH::NeighborsSearchDataSet), cudaMemcpyHostToDevice));

	}

	gpuErrchk(cudaMemcpy(*out_vector, temp, numSets * sizeof(SPH::UnifiedParticleSet), cudaMemcpyHostToDevice));


	delete[] temp;
}




void release_UnifiedParticleSet_vector_cuda(SPH::UnifiedParticleSet** vector, int numSets) {
	//each stucture properly clear itself currently so here i just need to destroy the high level array
	CUDA_FREE_PTR((*vector));

	cudaDeviceSynchronize();

}


template<class T> __global__ void cuda_updateParticleCount_kernel(T* container, unsigned int numParticles) {
	//that kernel wil only ever use one thread so I sould noteven need that
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= 1) { return; }

	container->numParticles = numParticles;
}


template<class T> void update_active_particle_number_cuda(T& container) {
	//And now I need to update the particle count in the gpu structures
	//the easiest way is to use a kernel with just one thread used
	//the other way would be to copy the data back to the cpu then update the value before sending it back to the cpu
	cuda_updateParticleCount_kernel<T> << <1, 1 >> > (container.gpu_ptr, container.numParticles);

	gpuErrchk(cudaDeviceSynchronize());
}
template void update_active_particle_number_cuda<SPH::UnifiedParticleSet>(SPH::UnifiedParticleSet& container);
template void update_active_particle_number_cuda<SPH::NeighborsSearchDataSet>(SPH::NeighborsSearchDataSet& container);

void change_fluid_max_particle_number(SPH::DFSPHCData& data, int numParticlesMax) {
	//update the fluid dataset
	data.fluid_data[0].changeMaxParticleNumber(numParticlesMax);

	//and update the aggregated neighbor search if it is used
#ifdef GROUP_DYNAMIC_BODIES_NEIGHBORS_SEARCH
	free_grouped_neighbors_struct_cuda(data);

	allocate_grouped_neighbors_struct_cuda(data);
#endif


}


void change_max_particle_number(SPH::UnifiedParticleSet& container, int numParticlesMax) {
	//we need to copy the existing data so let's start in order.
	//first disactivate the destructor
	bool old_release_on_destruct = container.releaseDataOnDestruction;
	container.releaseDataOnDestruction = false;

	//the easy way is to create a dummy, backup the existing buffers inside
	//alloc the new buffers
	//do the copy
	//delete the temps storage

	SPH::UnifiedParticleSet dummy;
	dummy = container;
	//remove the pointer on the neighbor search data since it is handled separately
	dummy.neighborsDataSet = NULL;
	dummy.gpu_ptr = NULL;


	//*
	//now change the number of particle
	container.numParticlesMax = numParticlesMax;

	//the rendering data

	//allocate new
	container.renderingData = new ParticleSetRenderingData();
	cuda_opengl_initParticleRendering(*container.renderingData, numParticlesMax, &container.pos, &container.vel, container.has_color_buffer, &container.color);

	//now we need to copy the data
	gpuErrchk(cudaMemcpy(container.pos, dummy.pos, dummy.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToDevice));
	gpuErrchk(cudaMemcpy(container.vel, dummy.vel, dummy.numParticles * sizeof(Vector3d), cudaMemcpyDeviceToDevice));

	//and the rest of the data
	allocate_UnifiedParticleSet_cuda(container);

	//and fill it with the old data for buffers that need to be kept
	gpuErrchk(cudaMemcpy(container.mass, dummy.mass, dummy.numParticles * sizeof(RealCuda), cudaMemcpyDeviceToDevice));
	gpuErrchk(cudaMemcpy(container.kappa, dummy.kappa, dummy.numParticles * sizeof(RealCuda), cudaMemcpyDeviceToDevice));
	gpuErrchk(cudaMemcpy(container.kappaV, dummy.kappaV, dummy.numParticles * sizeof(RealCuda), cudaMemcpyDeviceToDevice));

	//*/
	//finaly we need to update the data that is stored on the gpu
	//hopefully ne need for more allocation I can just copy the new ptr addresses
	SPH::NeighborsSearchDataSet* back_up = container.neighborsDataSet;
	container.neighborsDataSet = container.neighborsDataSet->gpu_ptr;
	gpuErrchk(cudaMemcpy(container.gpu_ptr, &container, sizeof(SPH::UnifiedParticleSet), cudaMemcpyHostToDevice));
	container.neighborsDataSet = back_up;
	//and free the data from the dummy
	dummy.clear();


	container.releaseDataOnDestruction = old_release_on_destruct;
}


void change_max_particle_number(SPH::NeighborsSearchDataSet& dataSet, int numParticlesMax) {
	if (!dataSet.internal_buffers_allocated) {
		throw("only consider fully allocated dataset for now");
	}


	release_neighbors_search_data_set(dataSet, false, true, true);

	//and allocate it back
	dataSet.numParticlesMax = numParticlesMax;

	allocate_neighbors_search_data_set(dataSet, false, true, false);
}



void add_particles_cuda(SPH::UnifiedParticleSet& container, int num_additional_particles, const Vector3d* pos, const Vector3d* vel) {
	//can't use memeset for the mass so I have to make a kernel for the set
	int numBlocks = calculateNumBlocks(num_additional_particles);
	cuda_setBufferToValue_kernel<RealCuda> << <numBlocks, BLOCKSIZE >> > (container.mass,
		container.m_V*container.density0, container.numParticles + num_additional_particles);



	gpuErrchk(cudaMemcpy(container.pos + container.numParticles, pos, num_additional_particles * sizeof(Vector3d), cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(container.vel + container.numParticles, vel, num_additional_particles * sizeof(Vector3d), cudaMemcpyHostToDevice));


	gpuErrchk(cudaMemset(container.kappa + container.numParticles, 0, num_additional_particles * sizeof(RealCuda)));
	gpuErrchk(cudaMemset(container.kappaV + container.numParticles, 0, num_additional_particles * sizeof(RealCuda)));

	//update the particle count
	container.updateActiveParticleNumber(container.numParticles + num_additional_particles);


	cudaError_t cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		std::cerr << "add_particles_cuda failed: " << (int)cudaStatus << std::endl;
		exit(1598);
	}


}

template<class T> void set_buffer_to_value(T* buff, T val, int size) {
	//can't use memeset for the mass so I have to make a kernel for the  set
	int numBlocks = calculateNumBlocks(size);
	cuda_setBufferToValue_kernel<T> << <numBlocks, BLOCKSIZE >> > (buff, val, size);

	cudaError_t cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		std::cerr << "set_buffer_to_value failed: " << (int)cudaStatus << std::endl;
		exit(1598);
	}
}
template void set_buffer_to_value<Vector3d>(Vector3d* buff, Vector3d val, int size);
template void set_buffer_to_value<int>(int* buff, int val, int size);


void allocate_precomputed_kernel_managed(SPH::PrecomputedCubicKernelPerso& kernel, bool minimize_managed) {

	if (minimize_managed) {
		cudaMalloc(&(kernel.m_W), kernel.m_resolution * sizeof(RealCuda));
		cudaMalloc(&(kernel.m_gradW), (kernel.m_resolution + 1) * sizeof(RealCuda));
	}
	else {
		fprintf(stderr, "trying to use managed buffers for the kernels\n");
		exit(1256);
		//cudaMallocManaged(&(kernel.m_W), kernel.m_resolution * sizeof(RealCuda));
		//cudaMallocManaged(&(kernel.m_gradW), (kernel.m_resolution + 1) * sizeof(RealCuda));
	}
}

void free_precomputed_kernel_managed(SPH::PrecomputedCubicKernelPerso& kernel) {
	CUDA_FREE_PTR(kernel.m_W);
	CUDA_FREE_PTR(kernel.m_gradW);
}


void init_precomputed_kernel_from_values(SPH::PrecomputedCubicKernelPerso& kernel, RealCuda* w, RealCuda* grad_W) {
	cudaError_t cudaStatus;
	//W
	cudaStatus = cudaMemcpy(kernel.m_W,
		w,
		kernel.m_resolution * sizeof(RealCuda),
		cudaMemcpyHostToDevice);

	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "precomputed initialization of W from data failed: %d\n", (int)cudaStatus);
		exit(1598);
	}

	//grad W
	cudaStatus = cudaMemcpy(kernel.m_gradW,
		grad_W,
		(kernel.m_resolution + 1) * sizeof(RealCuda),
		cudaMemcpyHostToDevice);

	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "precomputed initialization of grad W from data failed: %d\n", (int)cudaStatus);
		exit(1598);
	}

}




void allocate_neighbors_search_data_set(SPH::NeighborsSearchDataSet& dataSet, bool result_buffers_only, bool particle_related_only,
	bool allocate_gpu) {

	//result buffers
	cudaMalloc(&(dataSet.p_id_sorted), dataSet.numParticlesMax * sizeof(unsigned int));
	if (!particle_related_only) {
		cudaMalloc(&(dataSet.cell_start_end), (CELL_COUNT + 1) * sizeof(unsigned int));
	}

	//allocate the mem for fluid particles
	if (!result_buffers_only) {
		cudaMallocManaged(&(dataSet.cell_id), dataSet.numParticlesMax * sizeof(unsigned int));
		cudaMalloc(&(dataSet.cell_id_sorted), dataSet.numParticlesMax * sizeof(unsigned int));
		cudaMalloc(&(dataSet.local_id), dataSet.numParticlesMax * sizeof(unsigned int));
		cudaMalloc(&(dataSet.p_id), dataSet.numParticlesMax * sizeof(unsigned int));

		cudaMalloc(&(dataSet.intermediate_buffer_v3d), dataSet.numParticlesMax * sizeof(Vector3d));
		cudaMalloc(&(dataSet.intermediate_buffer_real), dataSet.numParticlesMax * sizeof(RealCuda));

		//reset the particle id
		int numBlocks = calculateNumBlocks(dataSet.numParticlesMax);
		DFSPH_setBufferValueToItself_kernel << <numBlocks, BLOCKSIZE >> > (dataSet.p_id, dataSet.numParticlesMax);
		DFSPH_setBufferValueToItself_kernel << <numBlocks, BLOCKSIZE >> > (dataSet.p_id_sorted, dataSet.numParticlesMax);
		gpuErrchk(cudaDeviceSynchronize());


		//cub pair sort
		dataSet.temp_storage_bytes_pair_sort = 0;
		dataSet.d_temp_storage_pair_sort = NULL;
		cub::DeviceRadixSort::SortPairs(dataSet.d_temp_storage_pair_sort, dataSet.temp_storage_bytes_pair_sort,
			dataSet.cell_id, dataSet.cell_id_sorted, dataSet.p_id, dataSet.p_id_sorted, dataSet.numParticlesMax);
		gpuErrchk(cudaDeviceSynchronize());
		cudaMalloc(&(dataSet.d_temp_storage_pair_sort), dataSet.temp_storage_bytes_pair_sort);


		if (!particle_related_only) {
			cudaMallocManaged(&(dataSet.hist), (CELL_COUNT + 1) * sizeof(unsigned int));

			//cub histogram
			dataSet.temp_storage_bytes_cumul_hist = 0;
			dataSet.d_temp_storage_cumul_hist = NULL;
			cub::DeviceScan::ExclusiveSum(dataSet.d_temp_storage_cumul_hist, dataSet.temp_storage_bytes_cumul_hist,
				dataSet.hist, dataSet.cell_start_end, (CELL_COUNT + 1));
			gpuErrchk(cudaDeviceSynchronize());
			cudaMalloc(&(dataSet.d_temp_storage_cumul_hist), dataSet.temp_storage_bytes_cumul_hist);
		}
	}


	/*
	std::cout << "neighbors struct num byte allocated cub (numParticlesMax pair_sort cumul_hist)" << dataSet.numParticlesMax << "  " <<
	dataSet.temp_storage_bytes_pair_sort << "  " << dataSet.temp_storage_bytes_cumul_hist << std::endl;
	//*/


	dataSet.internal_buffers_allocated = true;

	if (allocate_gpu) {
		//duplicate the neighbor dataset to the gpu
		gpuErrchk(cudaMalloc(&(dataSet.gpu_ptr), sizeof(SPH::NeighborsSearchDataSet)));
	}
	//and copy the gpu data if the buffer exists
	if (dataSet.gpu_ptr != NULL) {
		gpuErrchk(cudaMemcpy(dataSet.gpu_ptr, &dataSet,
			sizeof(SPH::NeighborsSearchDataSet), cudaMemcpyHostToDevice));
	}

	gpuErrchk(cudaDeviceSynchronize());

}



void release_neighbors_search_data_set(SPH::NeighborsSearchDataSet& dataSet, bool keep_result_buffers, bool keep_grid_related,
	bool keep_gpu) {

	//allocatethe mme for fluid particles
	CUDA_FREE_PTR(dataSet.cell_id);
	CUDA_FREE_PTR(dataSet.local_id);
	CUDA_FREE_PTR(dataSet.p_id);
	CUDA_FREE_PTR(dataSet.cell_id_sorted);

	CUDA_FREE_PTR(dataSet.d_temp_storage_pair_sort);
	dataSet.temp_storage_bytes_pair_sort = 0;

	CUDA_FREE_PTR(dataSet.intermediate_buffer_v3d);
	CUDA_FREE_PTR(dataSet.intermediate_buffer_real);

	if (!keep_grid_related) {
		CUDA_FREE_PTR(dataSet.hist);
		CUDA_FREE_PTR(dataSet.d_temp_storage_cumul_hist);
		dataSet.temp_storage_bytes_cumul_hist = 0;
	}

	dataSet.internal_buffers_allocated = false;

	if (!keep_result_buffers) {
		CUDA_FREE_PTR(dataSet.p_id_sorted);

		if (!keep_grid_related) {
			CUDA_FREE_PTR(dataSet.cell_start_end);
		}
	}

	if (!keep_gpu) {
		CUDA_FREE_PTR(dataSet.gpu_ptr);
	}
}



void load_bender2019_boundaries_from_cpu(SPH::UnifiedParticleSet& container, RealCuda* V_rigids_i, Vector3d* X_rigids_i) {
	
	//for some reason his returns invalid argument
	gpuErrchk(cudaMemcpy(container.X_rigids, X_rigids_i, container.numParticles * sizeof(Vector3d), cudaMemcpyHostToDevice));
	gpuErrchk(cudaMemcpy(container.V_rigids, V_rigids_i, container.numParticles * sizeof(RealCuda), cudaMemcpyHostToDevice));
	
}
