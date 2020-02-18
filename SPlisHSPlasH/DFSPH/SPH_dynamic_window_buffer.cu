#include "SPH_dynamic_window_buffer.h"
#include "DFSPH_core_cuda.h"

#include <stdio.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <sstream>
#include <fstream>

#include "DFSPH_define_cuda.h"
#include "DFSPH_macro_cuda.h"
#include "DFSPH_static_variables_structure_cuda.h"


#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "DFSPH_c_arrays_structure.h"
#include "cub.cuh"

#include "SPlisHSPlasH/Utilities/SegmentedTiming.h"
#include "SPH_other_systems_cuda.h"


#include <curand.h>
#include <curand_kernel.h>

#include "basic_kernels_cuda.cuh"



//NOTE1:	seems that virtual function can't be used with managed allocation
//			so I'll use a template to have an equivalent solution
//			the  template parameter allow the repr�sentation of various shapes
//NOTE2:	0 ==> plane: only use this for paralel planes (so max 2)
//			it's just a fast to compute solution if you need bands of fluid near the borders of the simulation
//			however since as soon as you get more than 2 planes the distance computation become pretty heavy
//			it's better to use another solution to represent the same surface (in particular in the case of boxes
//NOTE3:	1 ==> rectangular cuboid. 

template<int type>
class BufferFluidSurfaceBase
{
	//this is for the planes
	int count_planes;
	Vector3d* o;
	Vector3d* n;

	//this is for the cuboid
	Vector3d center;
	Vector3d halfLengths;
public:
	bool destructor_activated;

	inline BufferFluidSurfaceBase() {
		destructor_activated = false;
		switch (type) {
		case 0: {
			count_planes = 0;
			cudaMallocManaged(&(o), sizeof(Vector3d) * 2);
			cudaMallocManaged(&(n), sizeof(Vector3d) * 2);
			break;
		}
		case 1: {break; }
		default: {; }//it should NEVER reach here
		}
	}


	inline ~BufferFluidSurfaceBase() {
		if (destructor_activated) {
			switch (type) {
			case 0: {
				CUDA_FREE_PTR(o);
				CUDA_FREE_PTR(n);
				break;
			}
			case 1: {break; }
			default: {; }//it should NEVER reach here
			};
		}
	}


	inline int addPlane(Vector3d o_i, Vector3d n_i) {
		if (type != 0) {
			return -1;
		}
		if (count_planes >= 2) {
			return 1;
		}
		o[count_planes] = o_i;
		n[count_planes] = n_i;
		count_planes++;
		return 0;
	}

	inline int setCuboid(Vector3d c_i, Vector3d hl_i) {
		if (type != 1) {
			return -1;
		}
		center = c_i;
		halfLengths = hl_i;
		return 0;
	}

	inline void copy (const BufferFluidSurfaceBase& o) {
		switch (type) {
		case 0: {
			count_planes = 0;
			for (int i = 0; i < o.count_planes; ++i) {
				addPlane(o.o[i], o.n[i]);
			}
			break;
		}
		case 1: {
			center = o.center;
			halfLengths = o.halfLengths;
			break; 
		}
		default: {; }//it should NEVER reach here
		};
	}

	inline void move(const Vector3d& d) {
		switch (type) {
		case 0: {
			for (int i = 0; i < count_planes; ++i) {
				o[i]+=d;
			}
			break;
		}
		case 1: {
			center += d;
			break; 
		}
		default: {; }//it should NEVER reach here
		};
	}

	std::string toString() {
		std::ostringstream oss;

		switch (type) {
		case 0: {
			if (count_planes > 0) {
				for (int i = 0; i < count_planes; ++i) {
					oss << "plane " << i << " (o,n)  " << o[i].x << "   " << o[i].y << "   " << o[i].z << 
						"   " << n[i].x << "   " << n[i].y << "   " << n[i].z<< std::endl;
				}
			}
			else {
				oss << "No planes" << std::endl;
			}
			break;
		}
		case 1: {
			oss << "Cuboid center: " <<center.toString()<<"   halfLengths: "<<	halfLengths.toString()	<< std::endl;
			break; 
		}
		default: {; }//it should NEVER reach here
		};

		return oss.str();
	}



	//to know if we are on the inside of each plane we can simply use the dot product*
	FUNCTION inline bool isInsideFluid(Vector3d p) {
		switch (type) {
		case 0: {
			for (int i = 0; i < count_planes; ++i) {
				Vector3d v = p - o[i];
				if (v.dot(n[i]) < 0) {
					return false;
				}
			}
			break;
		}
		case 1: {
			Vector3d v = p - center ;
			v.toAbs();
			return (v.x<halfLengths.x) && (v.y < halfLengths.y) && (v.z < halfLengths.z);
			break; 
		}
		default: {; }//it should NEVER reach here
		}
		return true;
	}

	FUNCTION inline RealCuda distanceToSurface(Vector3d p) {
		RealCuda dist;

		switch (type) {
		case 0: {
			dist = abs((p - o[0]).dot(n[0]));
			for (int i = 1; i < count_planes; ++i) {
				Vector3d v = p - o[i];
				RealCuda l = abs(v.dot(n[i]));
				dist = MIN_MACRO_CUDA(dist, l);
			}
			break;
		}
		case 1: {
			
			Vector3d v = p - center;
			Vector3d v2 = v;
			if (v.x < -halfLengths.x) { v.x = -halfLengths.x; }
			else if (v.x > halfLengths.x) { v.x = halfLengths.x; }
			if (v.y < -halfLengths.y) { v.y = -halfLengths.y; }
			else if (v.y > halfLengths.y) { v.y = halfLengths.y; }
			if (v.z < -halfLengths.z) { v.z = -halfLengths.z; }
			else if (v.z > halfLengths.z) { v.z = halfLengths.z; }
			dist = (v - v2).norm();
			
			break; 
		}
		default: {; }//it should NEVER reach here
		}

		return dist;
	}

	FUNCTION inline RealCuda distanceToSurfaceSigned(Vector3d p) {
		RealCuda dist;
		switch (type) {
		case 0: {
			int plane_id = 0;
			dist = abs((p - o[0]).dot(n[0]));
			for (int i = 1; i < count_planes; ++i) {
				Vector3d v = p - o[i];
				RealCuda l = abs(v.dot(n[i]));
				if (l < dist) {
					dist = 0;
					plane_id = i;
				}
			}
			dist = (p - o[plane_id]).dot(n[plane_id]);
			break;
		}
		case 1: {
			dist = distanceToSurface(p) * ((isInsideFluid(p))?1:-1);
			break;
		}
		default: {; }//it should NEVER reach here
		}

		return dist;
	}

};

constexpr int surfaceType = 0;
using BufferFluidSurface = BufferFluidSurfaceBase<surfaceType>;


//this macro is juste so that the expression get optimized at the compilation 
//x_motion should be a bollean comming from a template configuration of the function where this macro is used
#define VECTOR_X_MOTION(pos,x_motion) ((x_motion)?pos.x:pos.z)

namespace DynamicWindowBuffer
{
	__global__ void init_buffer_kernel(Vector3d* buff, unsigned int size, Vector3d val) {
		int i = blockIdx.x * blockDim.x + threadIdx.x;
		if (i >= size) { return; }

		buff[i] = val;
	}
}


#define GAP_PLANE_POS 3


__global__ void DFSPH_init_buffer_velocity_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, 
													Vector3d* pos, Vector3d* vel, int numParticles) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numParticles) { return; }

	Vector3d pos_i = pos[i];

	//let's brute force it for now, I technicaly should be able to use the neighbor structure to accelerate it
#define num_neighbors 3
	Vector3d vel_neighbors[num_neighbors];
	RealCuda dist_neighbors[num_neighbors];

	for (int j = 0; j < num_neighbors; ++j) {
		dist_neighbors[j] = 1000000;
	}

	//we save the velocities and distance of the n closests
	for (int j = 0; j < particleSet->numParticles; ++j) {
		RealCuda cur_dist = (pos_i - particleSet->pos[j]).norm();
		

		int k = num_neighbors-1;
		while ((k > 0) && (cur_dist < dist_neighbors[k-1])) {
			dist_neighbors[k] = dist_neighbors[k - 1];
			vel_neighbors[k] = vel_neighbors[k - 1];
			k--;
		}
		if (cur_dist < dist_neighbors[k]) {
			dist_neighbors[k] = cur_dist;
			vel_neighbors[k] = particleSet->vel[j];
		}
	}

	//and now we can set the velocity to the averare of the closests
	RealCuda sum_dist = 0;
	Vector3d weighted_vel(0, 0, 0);
	for (int j = 0; j < num_neighbors; ++j) {
		sum_dist += dist_neighbors[j];
		weighted_vel += dist_neighbors[j]* vel_neighbors[j];
	}
	vel[i] = weighted_vel / sum_dist;

#undef num_neighbors
}

__global__ void DFSPH_reset_fluid_boundaries_remove_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, int* countRmv,
	BufferFluidSurface S) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= particleSet->numParticles) { return; }

	//*

	if (!S.isInsideFluid(particleSet->pos[i])) {
		atomicAdd(countRmv, 1);
		particleSet->neighborsDataSet->cell_id[i] = 25000000;
	}
	//*/

}

__global__ void DFSPH_reset_fluid_boundaries_add_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, SPH::UnifiedParticleSet* fluidBufferSet){

	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= fluidBufferSet->numParticles) { return; }

	particleSet->pos[particleSet->numParticles + i] = fluidBufferSet->pos[i];
	particleSet->vel[particleSet->numParticles + i] = fluidBufferSet->vel[i];
	particleSet->mass[particleSet->numParticles + i] = fluidBufferSet->mass[i];
	particleSet->color[particleSet->numParticles + i] = fluidBufferSet->color[i];

}

__device__ void atomicToMin(float* addr, float value)
{
	float old = *addr;
	if (old <= value) return;
	for (;;) {
		old = atomicExch(addr, value);
		if (old < value) {
			value = old;
		}
		else {
			return;
		}
	}
}

__device__ void atomicToMax(float* addr, float value)
{
	float old = *addr;
	if (old >= value) return;
	for (;;) {
		old = atomicExch(addr, value);
		if (old > value) {
			value = old;
		}
		else {
			return;
		}
	}
}

__global__ void DFSPH_compute_gap_length_fluid_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, RealCuda* gap) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= particleSet->numParticles) { return; }

	RealCuda gap_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius());
	if (particleSet->pos[i].x >= gap_pos) {
		//fluid side
		RealCuda dist = abs(particleSet->pos[i].x - gap_pos);
		atomicToMin(gap, dist);
	}
}

__global__ void DFSPH_compute_gap_length_buffer_kernel(SPH::DFSPHCData data, Vector3d* pos, int numParticles, RealCuda* gap) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numParticles) { return; }
	
	RealCuda gap_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius());
	RealCuda dist = abs(pos[i].x - gap_pos);
	atomicToMin(gap, dist);
	
}

__global__ void DFSPH_reduce_gap_length_kernel(SPH::DFSPHCData data, Vector3d* pos, int numParticles,
	RealCuda gap_length, RealCuda buffer_closest, RealCuda buffer_furthest) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numParticles) { return; }

	RealCuda gap_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius());
	
	//do a linear displacment to distribute the particles
	RealCuda dist = abs(pos[i].x - gap_pos);
	RealCuda displacement = (buffer_furthest - dist) / (buffer_furthest - buffer_closest) * gap_length;
	pos[i].x += displacement;
	
}

__global__ void DFSPH_get_fluid_particles_near_plane_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet,
	int* count_particles_fluid, int* ids_near_plane_fluid) {

	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= particleSet->numParticles) { return; }
	RealCuda gap_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius());
	if (particleSet->pos[i].x >= gap_pos) {
		if (particleSet->pos[i].x < (gap_pos + 0.5 * data.getKernelRadius())) {
			int id = atomicAdd(count_particles_fluid, 1);
			ids_near_plane_fluid[id] = i;
			particleSet->color[i] = Vector3d(1, 0, 0);
		}
	}
}

__global__ void DFSPH_get_buffer_particles_near_plane_kernel(SPH::DFSPHCData data, Vector3d* pos, int numParticles,
	int* count_particles_buffer, int* ids_near_plane_buffer) {

	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numParticles) { return; }

	RealCuda gap_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius());
	if (pos[i].x > (gap_pos - data.getKernelRadius())) {
		int id = atomicAdd(count_particles_buffer, 1);
		ids_near_plane_buffer[id] = i;
	}
	
}

__global__ void DFSPH_fit_particles_simple_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, Vector3d* pos_buffer,
	int count_particles_buffer, int count_particles_fluid, int* ids_near_plane_buffer, int* ids_near_plane_fluid,
	int count_buffer_displaced, int* ids_buffer_displaced,
	int* nbr_displaced, RealCuda* amount_displaced, int iter_nb, Vector3d* color) {

	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= (count_particles_buffer)) { return; }
	Vector3d pos_i = pos_buffer[ids_near_plane_buffer[i]];
	color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 0); 
	
	//first I'll only treat the rightmost particles so for anyother I just end the function
	RealCuda dist_limit = data.particleRadius * 2 * data.particleRadius * 2;
	for (int k = 0; k < count_particles_buffer; ++k) {
		if (k != i) {
			Vector3d pos_k = pos_buffer[ids_near_plane_buffer[k]];
			if (pos_i.x < pos_k.x) {
				Vector3d d = pos_i - pos_k;
				d.x = 0;
				RealCuda dist = d.squaredNorm();
				if (dist < dist_limit) {
					return;
				}
			}
		}
	}


	color[ids_near_plane_buffer[i]] = Vector3d(0.5, 0.5, 0.5);


	//since we are he rightmost we can look for the closest particle in the fluid and put us tangent to it
	RealCuda dist = (pos_i - particleSet->pos[ids_near_plane_fluid[0]]).squaredNorm();
	int id_closest = 0;
	for (int k = 1; k < count_particles_fluid; ++k) {
		RealCuda d = (pos_i - particleSet->pos[ids_near_plane_fluid[k]]).squaredNorm();
		if (d < dist) {
			dist = d;
			id_closest = k;
		}
	}

	//also look in the particles that were previouslys displaced
	bool closest_was_displaced = false;
	for (int k = 0; k < count_buffer_displaced; ++k) {
		RealCuda d = (pos_i - pos_buffer[ids_buffer_displaced[k]]).squaredNorm();
		if (d < dist) {
			dist = d;
			id_closest = k;
			closest_was_displaced = true;
		}
	}

	//now that we have the closest fluid put ourselve tangent to it by X-axis movement
	if (dist > dist_limit) {
		Vector3d pos_closest = closest_was_displaced?pos_buffer[ids_buffer_displaced[id_closest]]:particleSet->pos[ids_near_plane_fluid[id_closest]];
		Vector3d d_pos = pos_closest - pos_i;

		//remove the case of particles of to far in the tangent plane
		Vector3d d_pos_temp = d_pos;
		d_pos_temp.x = 0;
		RealCuda sq_tangent_plane_dist = d_pos_temp.squaredNorm();
		if (sq_tangent_plane_dist > dist_limit) {
			return;
		}

		//particleSet->vel[ids_near_plane_buffer[i]] = Vector3d(100);
		//displace the particle
		RealCuda x_displaced = sqrtf(dist_limit - sq_tangent_plane_dist);
		RealCuda d_x = d_pos.x - x_displaced;
		d_x *= 0.75;
		//pos_buffer[ids_near_plane_buffer[i]].x += d_x;

		atomicAdd(amount_displaced, d_x);
		int id_displaced=atomicAdd(nbr_displaced, 1);

		ids_buffer_displaced[count_buffer_displaced + id_displaced] = i;

		color[ids_near_plane_buffer[i]] = Vector3d(0, 1, 0); 
			/*
		switch (iter_nb) {
		case 0: color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 1); break;
		case 1: color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 0); break;
		case 2: color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 0); break;
		case 3: color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 0); break;
		case 4: color[ids_near_plane_buffer[i]] = Vector3d(0, 0, 0); break;
		}
		//*/

	}

}


__global__ void DFSPH_move_to_displaced_kernel(int* ids_near_plane_buffer, int count_particles_buffer, int* ids_buffer_displaced, 
	int count_buffer_displaced,	int nbr_to_move) {
	int k = blockIdx.x * blockDim.x + threadIdx.x;
	if (k >= 1) { return; }
	

	for (int i = 0; i < nbr_to_move; ++i) {
		int id_in_buffer = ids_buffer_displaced[count_buffer_displaced + i];

		//store the actual particle id in the displaced buffer
		ids_buffer_displaced[count_buffer_displaced + i] = ids_near_plane_buffer[id_in_buffer];

		//and now we need to sort it to remove it so I switch it with the last particle
		int last_particle_id = ids_near_plane_buffer[count_particles_buffer-1];
		ids_near_plane_buffer[id_in_buffer] = last_particle_id;
		

		for (int k = i + 1; k < nbr_to_move; ++k) {
			if (ids_buffer_displaced[count_buffer_displaced + k] == count_particles_buffer - 1) {
				ids_buffer_displaced[count_buffer_displaced + k] = id_in_buffer;
				break;
			}
		}

		count_particles_buffer--;

	}
}

__global__ void DFSPH_find_min_dist_near_plane_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* particleSet, Vector3d* pos_buffer, 
	int* ids_near_plane_buffer, int count_particles_buffer, int* ids_buffer_displaced,
	int count_buffer_displaced, int* ids_near_plane_fluid, int count_particles_fluid) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= (count_particles_buffer + count_buffer_displaced)) { return; }

	RealCuda min_dist = 10000000;
	Vector3d pos_i;
	if (i < count_particles_buffer) {
		pos_i = pos_buffer[ids_near_plane_buffer[i]];
		particleSet->densityAdv[i] = min_dist; return;
	}
	else {
		pos_i = pos_buffer[ids_buffer_displaced[i]];
	}

	//handle the buffer side
	/*
	for (int j = i + 1; j < (count_particles_buffer+count_buffer_displaced); ++j) {
		Vector3d pos_j;
		if (j < count_particles_buffer) {
			pos_j = pos_buffer[ids_near_plane_buffer[j]];
		}
		else {
			pos_j = pos_buffer[ids_buffer_displaced[j]];
		}

		RealCuda dist = (pos_j - pos_i).norm();
		if (dist < min_dist) {
			min_dist = dist;
		}
	}
	//*/

	//handle the fluid side
	//*
	for (int j = 0; j < count_particles_fluid; ++j) {
		Vector3d pos_j= particleSet->pos[ids_near_plane_fluid[j]];
		
		RealCuda dist = (pos_j - pos_i).norm();
		if (min_dist > dist ) {
			min_dist = dist;
		}
	}
	//*/

	//I'll just use a curerntly unused buffer to get the valueback
	particleSet->densityAdv[i] = min_dist;
}

__global__ void get_buffer_min_kernel(RealCuda* buffer, RealCuda* result, int nb_elem) {
	int k = blockIdx.x * blockDim.x + threadIdx.x;
	if (k >= 1) { return; }

	RealCuda min = buffer[0];
	//*
	for (int i = 1; i < nb_elem; ++i) {
		if (min > buffer[i]) {
			min = buffer[i];
		}
	}
	//*/
	*result = min;
}

template<int nbr_layers, int nbr_layers_in_buffer>
__global__ void DFSPH_evaluate_density_field_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet,
	SPH::UnifiedParticleSet* bufferSet, Vector3d min, Vector3d max,
	Vector3i vec_count_samples, RealCuda* samples, RealCuda* samples_after_buffer,
	int count_samples, RealCuda sampling_spacing, Vector3d* sample_pos) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= count_samples) { return; }

	//find the layer
	int nb_particle_in_layer = vec_count_samples.y * vec_count_samples.z;
	int layer_id = 0;
	int i_local = i;
	while (i_local >= nb_particle_in_layer) {
		i_local -= nb_particle_in_layer;
		layer_id++;
	}

	//it's a fail safe but should never be triggered unless something realy bad happened
	if (layer_id >= nbr_layers) {
		return;
	}


	//let's have 3 layers in in the buffer
	layer_id -= nbr_layers_in_buffer;


	Vector3d sampling_point;
	sampling_point.x = 0;
	sampling_point.y = (int)(i_local / vec_count_samples.z);
	sampling_point.z = (int)(i_local - (sampling_point.y) * vec_count_samples.z);
	sampling_point *= sampling_spacing;
	//put the coordinate to absolute
	sampling_point.y += min.y - sampling_spacing * 5;
	sampling_point.z += min.z - sampling_spacing * 5;

	//add the gap plane position
	RealCuda plane_pos = (-2.0 + GAP_PLANE_POS * data.getKernelRadius()) + sampling_spacing;
	sampling_point.x = plane_pos + layer_id * sampling_spacing;
	sample_pos[i] = sampling_point;//Vector3d(i_local, min.y, min.z);

	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(sampling_point, data.getKernelRadius(), data.gridOffset);


	RealCuda density = 0;
	RealCuda density_after_buffer = 0;



	bool near_fluid = false;
	bool near_buffer = false;

	//*
	//compute the fluid contribution
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
		RealCuda density_delta = fluidSet->mass[j] * KERNEL_W(data, sampling_point - fluidSet->pos[j]);
		if (density_delta>0){
			if (fluidSet->pos[j].x > plane_pos) {
				density_after_buffer += density_delta;
				if (sampling_point.x > plane_pos) {
					if (fluidSet->pos[j].y > (sampling_point.y)){
						near_fluid = true;
					}
				}
				else {
					if (fluidSet->pos[j].y > (sampling_point.y - sampling_spacing)) {
						near_fluid = true;
					}
				}
			}
			density += density_delta;
		}
	);

	//*
	//compute the buffer contribution
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(bufferSet->neighborsDataSet, bufferSet->pos,
		RealCuda density_delta = bufferSet->mass[j] * KERNEL_W(data, sampling_point - bufferSet->pos[j]);
		if (density_delta > 0) {
			density_after_buffer += density_delta;
			near_buffer = true;
		}

	);
	//*/

	//compute the boundaries contribution only if there is a fluid particle anywhere near
	//*
	if ((density > 100) || (density_after_buffer > 100)) {
		ITER_NEIGHBORS_FROM_STRUCTURE_BASE(data.boundaries_data_cuda->neighborsDataSet, data.boundaries_data_cuda->pos,
			RealCuda density_delta = data.boundaries_data_cuda->mass[j] * KERNEL_W(data, sampling_point - data.boundaries_data_cuda->pos[j]);
			density_after_buffer += density_delta;
			density += density_delta;
		);
	}
	//*/

	if (near_buffer && near_fluid) {
		samples[i] = density;
		samples_after_buffer[i] = density_after_buffer ;
		sample_pos[i] = sampling_point;

		//that line is just an easy way to recognise the plane
		//samples_after_buffer[i] *= (((layer_id == 0)&&(density_after_buffer>500)) ? -1 : 1);
	}
	else {
		samples[i] = 0;
		samples_after_buffer[i] = 0;
	}
}

//technically only the particles aorund the plane have any importance
//so no need to compute the density for any particles far from the plances
//also unless I want to save it I don't actually need to outpu the density
// I set those optimisations statically so that the related ifs get optimized
__global__ void DFSPH_evaluate_density_in_buffer_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet,
	SPH::UnifiedParticleSet* backgroundBufferSet, SPH::UnifiedParticleSet* bufferSet, int* countRmv, 
	BufferFluidSurface S, int iter=0) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= bufferSet->numParticles) { return; }

	Vector3d p_i=bufferSet->pos[i];

	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(p_i, data.getKernelRadius(), data.gridOffset);


	RealCuda density_after_buffer = bufferSet->mass[i] * data.W_zero;
	RealCuda density = 0;
	
	int count_neighbors = 0;
	bool keep_particle = true;

	//*
	keep_particle = false;
	

	//compute the fluid contribution
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
		RealCuda density_delta = fluidSet->mass[j] * KERNEL_W(data, p_i - fluidSet->pos[j]);
	if (density_delta > 0) {
		if (S.isInsideFluid(fluidSet->pos[j])) {
			density += density_delta;
			count_neighbors++;
			if (fluidSet->pos[j].y > (p_i.y)) {
				keep_particle = true;
			}
		}
	}
	);

	keep_particle = keep_particle || (!S.isInsideFluid(p_i));
	

	//keep_particle = true;
	if (keep_particle) {
		//*
		//compute the buffer contribution
		//if (iter == 0)
		if (false){
			//Note: the strange if is just here to check if p_i!=p_j but since doing this kind of operations on float
			//		that are read from 2 files might not be that smat I did some bastard check verifying if the particle are pretty close
			//		since the buffer we are evaluating is a subset of the background buffer, the only possibility to have particle 
			//		that close is that they are the same particle.
			//*
			//no need for that since I lighten the buffers before now
			float limit = data.particleRadius / 10.0f;
			limit *= limit;
			ITER_NEIGHBORS_FROM_STRUCTURE_BASE(backgroundBufferSet->neighborsDataSet, backgroundBufferSet->pos,
				if ((p_i - backgroundBufferSet->pos[j]).squaredNorm()> (limit)) {
					RealCuda density_delta = backgroundBufferSet->mass[j] * KERNEL_W(data, p_i - backgroundBufferSet->pos[j]);
					density_after_buffer += density_delta;
					count_neighbors++;
				}
			);
			//*/
		}
		else {
			//on the following passes I do the computations using the neighbors from the buffer
			ITER_NEIGHBORS_FROM_STRUCTURE_BASE(bufferSet->neighborsDataSet, bufferSet->pos,
				if (i!=j) {
					RealCuda density_delta = bufferSet->mass[j] * KERNEL_W(data, p_i - bufferSet->pos[j]);
					density_after_buffer += density_delta;
					count_neighbors++;
				}
			);

			//also has to iterate over the background buffer that now represent the air
			ITER_NEIGHBORS_FROM_STRUCTURE_BASE(backgroundBufferSet->neighborsDataSet, backgroundBufferSet->pos,
				RealCuda density_delta = backgroundBufferSet->mass[j] * KERNEL_W(data, p_i - backgroundBufferSet->pos[j]);
				density_after_buffer += density_delta;
				count_neighbors++;
			);
		}
		//*/

		//compute the boundaries contribution only if there is a fluid particle anywhere near
		//*
		if ((density > 100) || (density_after_buffer > 100)) {
			ITER_NEIGHBORS_FROM_STRUCTURE_BASE(data.boundaries_data_cuda->neighborsDataSet, data.boundaries_data_cuda->pos,
				RealCuda density_delta = data.boundaries_data_cuda->mass[j] * KERNEL_W(data, p_i - data.boundaries_data_cuda->pos[j]);
			density += density_delta;
			);
		}
		//*/
	}

	if (keep_particle) {
		density_after_buffer += density;
		if (true) {
			bufferSet->density[i] = density_after_buffer;
			bufferSet->densityAdv[i] = density;
			bufferSet->color[i] = Vector3d(1, 0, 0);//Vector3d((count_neighbors * 3)/255.0f, 0, 0);
		}
		
		//that line is just an easy way to recognise the plane
		//samples_after_buffer[i] *= (((layer_id == 0)&&(density_after_buffer>500)) ? -1 : 1);

		int limit_density = 1500 -50 * iter;


		keep_particle = (density_after_buffer) < limit_density;
		
		
		if (!keep_particle) {
			atomicAdd(countRmv, 1);
			bufferSet->neighborsDataSet->cell_id[i] = 25000000;
		}
		else {
			bufferSet->neighborsDataSet->cell_id[i] = 0;
		}
	}
	else {
		//here do a complete tag of the particles to remove them in the future
		if (true) {
			bufferSet->density[i] = 100000;
			bufferSet->densityAdv[i] = 100000;
		}

		atomicAdd(countRmv, 1);
		bufferSet->neighborsDataSet->cell_id[i] = 25000000;
	}
}

template<bool x_motion>
__global__ void DFSPH_compress_fluid_buffer_kernel(SPH::UnifiedParticleSet* particleSet, float compression_coefficient,
	Vector3d min, Vector3d max, RealCuda plane_inf, RealCuda plane_sup) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= particleSet->numParticles) { return; }

	float pos = VECTOR_X_MOTION(particleSet->pos[i], x_motion);
	//I must compre each side superior/inferior toward it's border
	float extremity = VECTOR_X_MOTION(((pos < 0) ? min : max), x_motion);

	float plane_pos = ((extremity < 0) ? plane_inf : plane_sup);


	pos -= extremity;
	if (abs(pos) > (abs(plane_pos - extremity) / 2)) {
		pos *= compression_coefficient;
		pos += extremity;
		VECTOR_X_MOTION(particleSet->pos[i], x_motion) = pos;
	}

}

__global__ void DFSPH_generate_buffer_from_surface_count_particles_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* backgroundSet,
	BufferFluidSurface S, int* count) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= backgroundSet->numParticles) { return; }

	//ok so I'll explain that
	//the idea is that I want the position of the particle from inside the back ground and inside the buffer to be the same
	//for an easy handling when applying the buffer as a mask
	//BUT I also would like that the particles stay relatively ordoned to still have pretty good memory coherency 
	//for faster access
	//So I manipulate the cell_id (which here is only an index for sorting the particles) to put every particle that are not
	//inside the buffer at the end of the buffer but with a linear addition so that the relative order of the particle
	//in each suggroups stays the same
	//also we need to do the height with a height map but for now it wil just be a fixe height
	//*
	RealCuda dist = S.distanceToSurfaceSigned(backgroundSet->pos[i]);
    if ((dist<data.particleRadius)&&(backgroundSet->pos[i].y<1.0)) {
		atomicAdd(count, 1);
	}
	else {
		backgroundSet->neighborsDataSet->cell_id[i] += CELL_COUNT;
	}
}



//I want to only keep particles that are above the fluid or above the buffer
//also remove the buffer particles that are above the fluid
//WARNING:	this function make full use of the fact that the fluid buffer is a subset of the background
//			Specificaly it needs to be composed of the first particles of the background and the orders of the particles must be the same 
__global__ void DFSPH_lighten_buffers_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet,
	SPH::UnifiedParticleSet* backgroundBufferSet, SPH::UnifiedParticleSet* bufferSet, int* countRmvBackground,
	int* countRmvBuffer, BufferFluidSurface S) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= backgroundBufferSet->numParticles) { return; }


	Vector3d p_i = backgroundBufferSet->pos[i];


	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(p_i, data.getKernelRadius(), data.gridOffset);


	bool keep_particle_background = true;
	bool keep_particle_buffer = (i<bufferSet->numParticles);//since the buffer is subset of the background
	//*
	RealCuda dist = S.distanceToSurfaceSigned(p_i);
	
	/*
	//quick code that verify that the buffer is a correct subset of the background
	dist = -1;
	if (keep_particle_buffer) {
		Vector3d v = p_i - bufferSet->pos[i];
		if (v.norm() > (data.particleRadius / 10.0)) {
			keep_particle_buffer = false;
		}
	}
	//*/

	if (dist>0) {
		//if the particle is too far inside the fluid we can discard it
		//the bufffer currently expends 1 kernel radius inside the fluid so anything further than 2 kernel radius
		//can be removed from the background
		if (dist>data.getKernelRadius()*3) {
			keep_particle_background = false;
		}

		bool is_buffer_particle_under_fluid = false;
		
		if (keep_particle_background) {
			//for the buffer we want the particle to be under the fluid
			//for the back ground we want only above the fluid (or at least realyc close from the fluid surface
			//note:	if it is part of the buffer and under the fluid we can stop the computation because it mean we have to discard
			//		it from the background
			ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
				if (!is_buffer_particle_under_fluid) {
					if (S.isInsideFluid(fluidSet->pos[j])) {
						if (keep_particle_buffer) {
							if (fluidSet->pos[j].y > (p_i.y)) {
								is_buffer_particle_under_fluid = true;
							}
						}
					
						//*
						if (keep_particle_background) {
							int nb_neighbors = fluidSet->getNumberOfNeighbourgs(0) + fluidSet->getNumberOfNeighbourgs(1);
							if ((nb_neighbors > 15)) {
								Vector3d delta = fluidSet->pos[j] - p_i;
								RealCuda dh = delta.y;
								delta.y = 0;
								if (dh > (2 * data.particleRadius)) {
								//if ((delta.norm() < 2 * data.particleRadius) && (dh > (2 * data.particleRadius))) {
										keep_particle_background = false;
								}
							}
						}
						//*/
					}
				}
			);

			//if it's a buffer particle only keep it if it  is under the fluid
			keep_particle_buffer &= is_buffer_particle_under_fluid;

		}

	}

	//if we keep it in the buffer it must be removed from the background
	keep_particle_background &= (!keep_particle_buffer);

	if (!keep_particle_buffer) {
		if (i < bufferSet->numParticles) {
			atomicAdd(countRmvBuffer, 1);
			bufferSet->neighborsDataSet->cell_id[i] = 25000000;
		}
	}
	else {
		bufferSet->neighborsDataSet->cell_id[i] = 0;
	}

	if (!keep_particle_background) {
		atomicAdd(countRmvBackground, 1);
		backgroundBufferSet->neighborsDataSet->cell_id[i] = 25000000;
	}
	else {
		backgroundBufferSet->neighborsDataSet->cell_id[i] = 0;
	}
}


 
__global__ void DFSPH_evaluate_density_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet, BufferFluidSurface S) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= fluidSet->numParticles) { return; }


	Vector3d p_i = fluidSet->pos[i];


	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(p_i, data.getKernelRadius(), data.gridOffset);


	bool keep_particle = true;
	//*
	RealCuda sq_diameter = data.particleRadius * 2;
	sq_diameter *= sq_diameter;
	int count_neighbors = 0;
	RealCuda density = fluidSet->mass[i] * data.W_zero;

	//check if there is any fluid particle above us
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
		if (i != j) {
			//RealCuda density_delta = (fluidSet->pos[j]-p_i).norm();
			RealCuda density_delta = fluidSet->mass[j] * KERNEL_W(data, p_i - fluidSet->pos[j]);
			density += density_delta;
			count_neighbors++;
		}
	);

	//*
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(data.boundaries_data_cuda->neighborsDataSet, data.boundaries_data_cuda->pos,
		RealCuda density_delta = data.boundaries_data_cuda->mass[j] * KERNEL_W(data, p_i - data.boundaries_data_cuda->pos[j]);
		density += density_delta;
		count_neighbors++;
	);
	//*/

	fluidSet->density[i] = density;
}


__global__ void DFSPH_evaluate_particle_concentration_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet,
	BufferFluidSurface S) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= fluidSet->numParticles) { return; }


	Vector3d p_i = fluidSet->pos[i];


	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(p_i, data.getKernelRadius(), data.gridOffset);


	bool keep_particle = true;
	//*
	RealCuda sq_diameter = data.particleRadius * 2;
	sq_diameter *= sq_diameter;
	int count_neighbors = 0;
	//we cna start at 0 and ignire the i contribution because we will do a sustracction when computing the concentration gradiant
	RealCuda concentration = 0;

	//check if there is any fluid particle above us
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
		if (i != j) {
			RealCuda concentration_delta = (fluidSet->mass[j]/ fluidSet->density[j])* KERNEL_W(data, p_i - fluidSet->pos[j]);
			concentration += concentration_delta;
			count_neighbors++;
		}
	);

	//supose that the density of boundaries particles is the rest density
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(data.boundaries_data_cuda->neighborsDataSet, data.boundaries_data_cuda->pos,
		RealCuda concentration_delta = (data.boundaries_data_cuda->mass[j] / data.density0) * KERNEL_W(data, p_i - data.boundaries_data_cuda->pos[j]);
	concentration += concentration_delta;
	count_neighbors++;
	);


	fluidSet->densityAdv[i] = concentration;
}

__global__ void DFSPH_particle_shifting_base_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet, RealCuda displacement_coefficient,
	BufferFluidSurface S, int* count_affected, RealCuda* total_abs_displacement) {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= fluidSet->numParticles) { return; }

	bool x_motion = true;
	Vector3d p_i = fluidSet->pos[i];

	//only move particles that are close to the planes
	RealCuda affected_range = data.getKernelRadius();
	RealCuda dist_to_surface = S.distanceToSurface(p_i);
	if (dist_to_surface>affected_range) {
		return;
	}

	ITER_NEIGHBORS_INIT_CELL_COMPUTATION(p_i, data.getKernelRadius(), data.gridOffset);

	RealCuda scaling = 1 - dist_to_surface / affected_range;

	bool keep_particle = true;
	//*
	RealCuda sq_diameter = data.particleRadius * 2;
	sq_diameter *= sq_diameter;
	int count_neighbors = 0;
	//we cna start at 0 and ignire the i contribution because we will do a sustracction when computing the concentration gradiant
	Vector3d displacement = Vector3d(0,0,0);

	//check if there is any fluid particle above us
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(fluidSet->neighborsDataSet, fluidSet->pos,
		if (i != j) {
			Vector3d displacement_delta = (fluidSet->densityAdv[j]- fluidSet->densityAdv[i])* (fluidSet->mass[j] / fluidSet->density[j]) * KERNEL_GRAD_W(data, p_i - fluidSet->pos[j]);
			/*
			Vector3d nj = fluidSet->pos[j] - p_i;
			nj.toUnit();
			Vector3d displacement_delta = fluidSet->density[j]*KERNEL_W(data, p_i - fluidSet->pos[j])*nj;
			//*/
			displacement += displacement_delta;
			count_neighbors++;
		}
	);

	//as long as I make so that the surface is more than 1 kernel radius from the boundaries those computation are fine no need to iterate on the boundaries
	//so I'll prevent shifting the particles that are even remotely clse from the boundary
	int count_neighbors_b = 0;
	ITER_NEIGHBORS_FROM_STRUCTURE_BASE(data.boundaries_data_cuda->neighborsDataSet, data.boundaries_data_cuda->pos,
		count_neighbors_b++;
	);

	displacement *= -displacement_coefficient;
	//we cap the displacement like in the papers
	RealCuda disp_norm = displacement.norm();
	if (disp_norm > (0.2 * data.getKernelRadius())) {
		displacement *= (0.2 * data.getKernelRadius()) / disp_norm;
	}

	displacement *= scaling * scaling;
	
	atomicAdd(count_affected, 1);
	atomicAdd(total_abs_displacement, displacement.norm());

	if (count_neighbors_b ==0) {
		fluidSet->pos[i]+=displacement;
		//fluidSet->color[i] = Vector3d(1, 1, 0.2);
	}
}


void handle_fluid_boundries_cuda(SPH::DFSPHCData& data, bool loading, Vector3d movement) {
	SPH::UnifiedParticleSet* particleSet = data.fluid_data;

	Vector3d min_fluid_buffer, max_fluid_buffer;


	//this buffer contains a set a particle corresponding to a fluid at rest covering the whole simulation space
	static SPH::UnifiedParticleSet* backgroundFluidBufferSet = NULL;
	static Vector3d* pos_background = NULL;
	static int numParticles_background = 0;

	//the object for the surface
	static BufferFluidSurface S_initial;
	static BufferFluidSurface S;
	static SPH::UnifiedParticleSet* fluidBufferSetFromSurface = NULL;
	static Vector3d* pos_base_from_surface = NULL;
	static int numParticles_base_from_surface = 0;

	//some variable defining the desired fluid buffer (axis and motion)
	//note those thing should be obtained from arguments at some point
    bool displace_windows = (movement.norm()>0.5);
	bool x_motion = (movement.x > 0.01) ? true : false;

    if ((abs(movement.z) > 0.5) && (movement.x > 0.5)) {
        throw("handle_fluid_boundries_cuda:: for now just use one direction :)");
	}





	//compute the movement on the position and the axis
	Vector3d mov_pos = movement * data.getKernelRadius();
	Vector3d mov_axis = (movement.abs()) / movement.norm();

	if (displace_windows&&(!loading)) {
		//update the displacement offset
		data.dynamicWindowTotalDisplacement += mov_pos;
		data.gridOffset -= movement;
	}


	//define the jonction planes
	RealCuda plane_pos_inf = -2.0;
	RealCuda plane_pos_sup = 2.0;
    if (!x_motion) {
        //plane_pos_inf = -0.7;
        //plane_pos_sup = 0.7;
        plane_pos_inf = -2;
        plane_pos_sup = 2;
	}
	plane_pos_inf += (GAP_PLANE_POS)*data.getKernelRadius() + VECTOR_X_MOTION(data.dynamicWindowTotalDisplacement, x_motion);
	plane_pos_sup += -(GAP_PLANE_POS)*data.getKernelRadius() + VECTOR_X_MOTION(data.dynamicWindowTotalDisplacement, x_motion);




	if (loading) {
		if (backgroundFluidBufferSet) {
			std::string msg = "handle_fluid_boundries_cuda: trying to change the boundary buffer size";
			std::cout << msg << std::endl;
			throw(msg);
		}
		//just activate that to back the current fluid for buffer usage
		bool save_to_buffer = false;
		if (save_to_buffer) {
			particleSet->write_to_file(data.fluid_files_folder + "fluid_buffer_file.txt");
		}

		//define the surface
		if(surfaceType==0){
			if (x_motion) {
				S_initial.addPlane(Vector3d(plane_pos_inf, 0, 0), Vector3d(1, 0, 0));
				S_initial.addPlane(Vector3d(plane_pos_sup, 0, 0), Vector3d(-1, 0, 0));
			}
			else {
				S_initial.addPlane(Vector3d(0, 0, plane_pos_inf), Vector3d(0, 0, 1));
				S_initial.addPlane(Vector3d(0, 0, plane_pos_sup), Vector3d(0, 0, -1));
			}
		}
		else if (surfaceType == 1) {
			Vector3d center(0, 0, 0);
			Vector3d halfLength(100);
			if (x_motion) {
				halfLength.x = plane_pos_sup-0.1;
			}
			else {
				halfLength.z = plane_pos_sup-0.1;
			}
			//halfLength.z = 0.7 - 0.1;
			S_initial.setCuboid(center, halfLength);

		}else {
			throw("the surface type need to be defined for that test");
		}


		//load the backgroundset
		{
			SPH::UnifiedParticleSet* dummy = NULL;
			backgroundFluidBufferSet = new SPH::UnifiedParticleSet();
			backgroundFluidBufferSet->load_from_file(data.fluid_files_folder + "background_buffer_file.txt", false, &min_fluid_buffer, &max_fluid_buffer, false);
			//fluidBufferSet->write_to_file(data.fluid_files_folder + "fluid_buffer_file.txt");
			allocate_and_copy_UnifiedParticleSet_vector_cuda(&dummy, backgroundFluidBufferSet, 1);
			
			backgroundFluidBufferSet->initNeighborsSearchData(data, true);
			backgroundFluidBufferSet->resetColor();

			numParticles_background = backgroundFluidBufferSet->numParticles;
			cudaMallocManaged(&(pos_background), sizeof(Vector3d) * numParticles_background);
			gpuErrchk(cudaMemcpy(pos_background, backgroundFluidBufferSet->pos, numParticles_background * sizeof(Vector3d), cudaMemcpyDeviceToDevice));

			if (false){
				Vector3d* pos = pos_background;
				int numParticles = numParticles_background;
				std::ostringstream oss;
				int effective_count = 0;
				for (int i = 0; i < numParticles; ++i) {
					uint8_t density = 255;
					uint8_t alpha = 255;
					uint32_t txt = (((alpha << 8) + density << 8) + 0 << 8) + 0;
					

					oss << pos[i].x << " " << pos[i].y << " " << pos[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}

				std::ofstream myfile("densityCloud.pcd", std::ofstream::trunc);
				if (myfile.is_open())
				{
					myfile << "VERSION 0.7" << std::endl
						<< "FIELDS x y z rgb" << std::endl
						<< "SIZE 4 4 4 4" << std::endl
						<< "TYPE F F F U" << std::endl
						<< "COUNT 1 1 1 1" << std::endl
						<< "WIDTH " << effective_count << std::endl
						<< "HEIGHT " << 1 << std::endl
						<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
						<< "POINTS " << effective_count << std::endl
						<< "DATA ascii" << std::endl;
					myfile << oss.str();
					myfile.close();
				}
			}
		}

		
		//create the buffer from the background and the surface
		{
			//first we count the nbr of particles and attribute the index for sorting
			//als we will reorder the background buffer so we need to resave its state
			int* out_int = NULL;
			cudaMallocManaged(&(out_int), sizeof(int));
			*out_int = 0;
			{
				int numBlocks = calculateNumBlocks(backgroundFluidBufferSet->numParticles);
				DFSPH_generate_buffer_from_surface_count_particles_kernel << <numBlocks, BLOCKSIZE >> > (data, backgroundFluidBufferSet->gpu_ptr, S_initial, out_int);
				gpuErrchk(cudaDeviceSynchronize());
			}
			int count_inside_buffer = *out_int;
			CUDA_FREE_PTR(out_int);



			//sort the buffer
			cub::DeviceRadixSort::SortPairs(backgroundFluidBufferSet->neighborsDataSet->d_temp_storage_pair_sort, backgroundFluidBufferSet->neighborsDataSet->temp_storage_bytes_pair_sort,
				backgroundFluidBufferSet->neighborsDataSet->cell_id, backgroundFluidBufferSet->neighborsDataSet->cell_id_sorted,
				backgroundFluidBufferSet->neighborsDataSet->p_id, backgroundFluidBufferSet->neighborsDataSet->p_id_sorted, backgroundFluidBufferSet->numParticles);
			gpuErrchk(cudaDeviceSynchronize());

			cuda_sortData(*backgroundFluidBufferSet, backgroundFluidBufferSet->neighborsDataSet->p_id_sorted);
			gpuErrchk(cudaDeviceSynchronize());

			//resave the background state
			gpuErrchk(cudaMemcpy(pos_background, backgroundFluidBufferSet->pos, numParticles_background * sizeof(Vector3d), cudaMemcpyDeviceToDevice));



			//and now we can create the buffer and save the positions
			SPH::UnifiedParticleSet* dummy = NULL;
			fluidBufferSetFromSurface = new SPH::UnifiedParticleSet();
			fluidBufferSetFromSurface->init(count_inside_buffer, true, true, false, true);
			allocate_and_copy_UnifiedParticleSet_vector_cuda(&dummy, fluidBufferSetFromSurface, 1);


			numParticles_base_from_surface = fluidBufferSetFromSurface->numParticles;
			cudaMallocManaged(&(pos_base_from_surface), sizeof(Vector3d) * numParticles_base_from_surface);
			gpuErrchk(cudaMemcpy(pos_base_from_surface, pos_background, numParticles_base_from_surface * sizeof(Vector3d), cudaMemcpyDeviceToDevice));
			gpuErrchk(cudaMemcpy(fluidBufferSetFromSurface->mass, backgroundFluidBufferSet->mass, numParticles_base_from_surface * sizeof(RealCuda), cudaMemcpyDeviceToDevice));

			fluidBufferSetFromSurface->resetColor();




			if (false) {
				Vector3d* pos = pos_base_from_surface;
				int numParticles = numParticles_base_from_surface;
				std::ostringstream oss;
				int effective_count = 0;
				for (int i = 0; i < numParticles; ++i) {
					uint8_t density = 255;
					uint8_t alpha = 255;
					uint32_t txt = (((alpha << 8) + density << 8) + 0 << 8) + 0;


					oss << pos[i].x << " " << pos[i].y << " " << pos[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}

				std::ofstream myfile("densityCloud.pcd", std::ofstream::trunc);
				if (myfile.is_open())
				{
					myfile << "VERSION 0.7" << std::endl
						<< "FIELDS x y z rgb" << std::endl
						<< "SIZE 4 4 4 4" << std::endl
						<< "TYPE F F F U" << std::endl
						<< "COUNT 1 1 1 1" << std::endl
						<< "WIDTH " << effective_count << std::endl
						<< "HEIGHT " << 1 << std::endl
						<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
						<< "POINTS " << effective_count << std::endl
						<< "DATA ascii" << std::endl;
					myfile << oss.str();
					myfile.close();
				}

				std::cout << "writting buffer to file end" << std::endl;

			}
		}


		return;
	}

	std::vector<std::string> timing_names{"color_reset","reset pos","apply displacement to buffer","init neightbors","compute density",
		"reduce buffer","cpy_velocity","reduce fluid", "apply buffer"};
	static SPH::SegmentedTiming timings("handle_fluid_boundries_cuda",timing_names,false);
	timings.init_step();



	SPH::UnifiedParticleSet* fluidBufferSet = fluidBufferSetFromSurface;
	Vector3d* pos_base = pos_base_from_surface;
	int numParticles_base = numParticles_base_from_surface;
	


	if (!fluidBufferSet) {
		std::string msg = "handle_fluid_boundries_cuda: you have to load a buffer first";
		std::cout << msg << std::endl;
		throw(msg);
	}



	timings.time_next_point();

	{

		//we could use the neighbo structure to have a faster access but for now let's just brute force it	
		static int* countRmv = NULL;
		static int* countRmv2 = NULL;
		static RealCuda* realcuda_ptr = NULL;

		if (!countRmv) {
			cudaMallocManaged(&(countRmv), sizeof(int));
			cudaMallocManaged(&(countRmv2), sizeof(int));
			cudaMallocManaged(&(realcuda_ptr), sizeof(RealCuda));
		}


		//reset the buffers
		fluidBufferSet->updateActiveParticleNumber(numParticles_base);
		backgroundFluidBufferSet->updateActiveParticleNumber(numParticles_background);
		gpuErrchk(cudaMemcpy(fluidBufferSet->pos, pos_base, numParticles_base * sizeof(Vector3d), cudaMemcpyDeviceToDevice));
		gpuErrchk(cudaMemcpy(backgroundFluidBufferSet->pos, pos_background, numParticles_background * sizeof(Vector3d), cudaMemcpyDeviceToDevice));
		particleSet->resetColor();
		fluidBufferSet->resetColor();
		S.copy(S_initial);

		timings.time_next_point();

		//we need to move the positions of the particles inside the buffer if we want the window to be dynamic
		if (displace_windows) {

			//first displace the boundaries
			SPH::UnifiedParticleSet* particleSetMove = data.boundaries_data;
			unsigned int numParticles = particleSetMove->numParticles;
			int numBlocks = calculateNumBlocks(numParticles);
			apply_delta_to_buffer_kernel << <numBlocks, BLOCKSIZE >> > (particleSetMove->pos, mov_pos, numParticles);

			//and the buffers
			//carefull since they are reset you must displace them for the full displacment since the start of the simulation
			particleSetMove = backgroundFluidBufferSet;
			numParticles = particleSetMove->numParticles;
			numBlocks = calculateNumBlocks(numParticles);
            apply_delta_to_buffer_kernel << <numBlocks, BLOCKSIZE >> > (particleSetMove->pos, data.dynamicWindowTotalDisplacement, numParticles);


			particleSetMove = fluidBufferSet;
			numParticles = particleSetMove->numParticles;
			numBlocks = calculateNumBlocks(numParticles);
			apply_delta_to_buffer_kernel << <numBlocks, BLOCKSIZE >> > (particleSetMove->pos, data.dynamicWindowTotalDisplacement, numParticles);


			gpuErrchk(cudaDeviceSynchronize());

			//and move the surface
			//wtf for some reason I can't seem to execute that line before the synchronize
			S.move(data.dynamicWindowTotalDisplacement);
			std::cout<<S.toString()<<std::endl;

			//update the boundaries neighbors
			data.boundaries_data->initNeighborsSearchData(data, false);
		}



        std::cout<<"test achieved"<<std::endl;
        read_last_error_cuda("any errors?: ");
		//update the neighbors structures for the buffers
        backgroundFluidBufferSet->initNeighborsSearchData(data, false);
        std::cout<<"test achieved"<<std::endl;
        read_last_error_cuda("any errors?: ");
        fluidBufferSet->initNeighborsSearchData(data, false);

        std::cout<<"test achieved"<<std::endl;
        read_last_error_cuda("any errors?: ");
		//update the neighbor structure for the fluid
		particleSet->initNeighborsSearchData(data, false);

        std::cout<<"test achieved"<<std::endl;
		timings.time_next_point();

		//first let's lighten the buffers to reduce the computation times
		{
			*countRmv = 0;
			*countRmv2 = 0;

			int numBlocks = calculateNumBlocks(backgroundFluidBufferSet->numParticles);

			DFSPH_lighten_buffers_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, backgroundFluidBufferSet->gpu_ptr, fluidBufferSet->gpu_ptr,
				countRmv, countRmv2, S);
			
			gpuErrchk(cudaDeviceSynchronize());

			std::cout << "ligthen estimation (background // buffer): " << *countRmv << "  //  " << *countRmv2 << std::endl;

			if (false) {
				Vector3d* pos = pos_background;
				int numParticles = numParticles_background;
				std::ostringstream oss;
				int effective_count = 0;
				for (int i = 0; i < numParticles; ++i) {
					if (backgroundFluidBufferSet->neighborsDataSet->cell_id[i] != 0) {
						continue;
					}
					uint8_t density = 255;
					uint8_t alpha = 255;
					uint32_t txt = (((alpha << 8) + density << 8) + 0 << 8) + 0;


					oss << pos[i].x << " " << pos[i].y << " " << pos[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}
				std::cout << "effcount: " << effective_count << "from:  " << numParticles << std::endl;

				pos = pos_base;
				numParticles = numParticles_base;
				for (int i = 0; i < numParticles; ++i) {
					if (fluidBufferSet->neighborsDataSet->cell_id[i] != 0) {
						continue;
					}

					uint8_t density = 255;
					uint8_t alpha = 255;
					uint32_t txt = (((alpha << 8) + 0 << 8) + density << 8) + 0;


					oss << pos[i].x << " " << pos[i].y << " " << pos[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}

				Vector3d* pos_cpu = new Vector3d[particleSet->numParticles];
				read_UnifiedParticleSet_cuda(*particleSet, pos_cpu, NULL, NULL, NULL);
				pos = pos_cpu;
				numParticles = particleSet->numParticles;
				for (int i = 0; i < numParticles; ++i) {

					uint8_t density = 255;
					uint8_t alpha = 255;
					uint32_t txt = (((alpha << 8) + 0 << 8) + 0 << 8) + 8;


					oss << pos[i].x << " " << pos[i].y << " " << pos[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}
				delete[] pos_cpu;

				std::ofstream myfile("densityCloud.pcd", std::ofstream::trunc);
				if (myfile.is_open())
				{
					myfile << "VERSION 0.7" << std::endl
						<< "FIELDS x y z rgb" << std::endl
						<< "SIZE 4 4 4 4" << std::endl
						<< "TYPE F F F U" << std::endl
						<< "COUNT 1 1 1 1" << std::endl
						<< "WIDTH " << effective_count << std::endl
						<< "HEIGHT " << 1 << std::endl
						<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
						<< "POINTS " << effective_count << std::endl
						<< "DATA ascii" << std::endl;
					myfile << oss.str();
					myfile.close();
				}
			}

			//remove the tagged particle from the buffer and the background 
			// use the same process as when creating the neighbors structure to put the particles to be removed at the end
			//from the background
			{
				UnifiedParticleSet* pset = backgroundFluidBufferSet;
				int count_to_rmv = *countRmv;
				cub::DeviceRadixSort::SortPairs(pset->neighborsDataSet->d_temp_storage_pair_sort, pset->neighborsDataSet->temp_storage_bytes_pair_sort,
					pset->neighborsDataSet->cell_id, pset->neighborsDataSet->cell_id_sorted,
					pset->neighborsDataSet->p_id, pset->neighborsDataSet->p_id_sorted, pset->numParticles);
				gpuErrchk(cudaDeviceSynchronize());

				cuda_sortData(*pset, pset->neighborsDataSet->p_id_sorted);
				gpuErrchk(cudaDeviceSynchronize());

				int new_num_particles = pset->numParticles - count_to_rmv;
				std::cout << "handle_fluid_boundries_cuda: ligthening the background to: " << new_num_particles << "   nb removed : " << count_to_rmv << std::endl;
				pset->updateActiveParticleNumber(new_num_particles);

				//we need to reinit the neighbors struct for the fluidbuffer since we removed some particles
				pset->initNeighborsSearchData(data, false);
			}


			//now the buffer
			{
				UnifiedParticleSet* pset = backgroundFluidBufferSet;
				int count_to_rmv = *countRmv2;
				cub::DeviceRadixSort::SortPairs(pset->neighborsDataSet->d_temp_storage_pair_sort, pset->neighborsDataSet->temp_storage_bytes_pair_sort,
					pset->neighborsDataSet->cell_id, pset->neighborsDataSet->cell_id_sorted,
					pset->neighborsDataSet->p_id, pset->neighborsDataSet->p_id_sorted, pset->numParticles);
				gpuErrchk(cudaDeviceSynchronize());

				cuda_sortData(*pset, pset->neighborsDataSet->p_id_sorted);
				gpuErrchk(cudaDeviceSynchronize());

				int new_num_particles = pset->numParticles - count_to_rmv;
				std::cout << "handle_fluid_boundries_cuda: ligthening the background to: " << new_num_particles << "   nb removed : " << count_to_rmv << std::endl;
				pset->updateActiveParticleNumber(new_num_particles);

				//we need to reinit the neighbors struct for the fluidbuffer since we removed some particles
				pset->initNeighborsSearchData(data, false);
			}
			

		}

		//ok since sampling the space regularly with particles to close the gap between the fluid and the buffer is realy f-ing hard
		//let's go the oposite way
		//I'll use a buffer too large,  evaluate the density on the buffer particles and remove the particle with density too large
		//also no need to do it on all partilce only those close enught from the plane with the fluid end
		{

			*countRmv = 0;
			{
				int numBlocks = calculateNumBlocks(fluidBufferSet->numParticles);
				
				DFSPH_evaluate_density_in_buffer_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, backgroundFluidBufferSet->gpu_ptr, fluidBufferSet->gpu_ptr,
						countRmv, S);

				gpuErrchk(cudaDeviceSynchronize());
			}
			std::cout << "end density comp" << std::endl;
			int test_int = *countRmv;
			std::cout << "read count" << std::endl;
			std::cout << "nbr of particles t rmv: " << test_int << " from " << fluidBufferSet->numParticles << std::endl;


			timings.time_next_point();
			//write it forthe viewer
			if (false) {
				std::ostringstream oss;
				int effective_count = 0;
				for (int i = 0; i < fluidBufferSet->numParticles; ++i) {
					uint8_t density = fminf(1.0f, (fluidBufferSet->density[i] / 1500.0f)) * 255;
					uint8_t alpha = 255;
					if (fluidBufferSet->density[i] >= 5000) {
						//continue;

					}
					/*
					if (density_field_after_buffer[i] >= 0) {
						if (density == 0) {
							continue;
						}
						if (density > 245) {
							continue;
						}
					}
					//*/
					//uint8_t density = (density_field[i] > 500) ? 255 : 0;
					uint32_t txt = (((alpha << 8) + density << 8) + density << 8) + density;
					//*
					if (fluidBufferSet->density[i] < 1000) {
						txt = (((alpha << 8) + density << 8) + 0 << 8) + 0;
					}
					if (fluidBufferSet->density[i] > 1000) {
						txt = (((alpha << 8) + 0 << 8) + density << 8) + 0;
					}
					if (fluidBufferSet->density[i] > 1100) {
						txt = (((alpha << 8) + 0 << 8) + 0 << 8) + density;
					}
					if (fluidBufferSet->density[i] > 1150) {
						txt = (((alpha << 8) + 255 << 8) + 0 << 8) + 144;
					}
					//*/

					oss << pos_base[i].x << " " << pos_base[i].y << " " << pos_base[i].z << " "
						<< txt << std::endl;
					effective_count++;
				}

				std::ofstream myfile("densityCloud.pcd", std::ofstream::trunc);
				if (myfile.is_open())
				{
					myfile << "VERSION 0.7" << std::endl
						<< "FIELDS x y z rgb" << std::endl
						<< "SIZE 4 4 4 4" << std::endl
						<< "TYPE F F F U" << std::endl
						<< "COUNT 1 1 1 1" << std::endl
						<< "WIDTH " << effective_count << std::endl
						<< "HEIGHT " << 1 << std::endl
						<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
						<< "POINTS " << effective_count << std::endl
						<< "DATA ascii" << std::endl;
					myfile << oss.str();
					myfile.close();
				}
			}

			//remove the tagged particle from the buffer (all yhe ones that have a density that is too high)
			//now we can remove the partices from the simulation
			// use the same process as when creating the neighbors structure to put the particles to be removed at the end
			cub::DeviceRadixSort::SortPairs(fluidBufferSet->neighborsDataSet->d_temp_storage_pair_sort, fluidBufferSet->neighborsDataSet->temp_storage_bytes_pair_sort,
				fluidBufferSet->neighborsDataSet->cell_id, fluidBufferSet->neighborsDataSet->cell_id_sorted,
				fluidBufferSet->neighborsDataSet->p_id, fluidBufferSet->neighborsDataSet->p_id_sorted, fluidBufferSet->numParticles);
			gpuErrchk(cudaDeviceSynchronize());

			cuda_sortData(*fluidBufferSet, fluidBufferSet->neighborsDataSet->p_id_sorted);
			gpuErrchk(cudaDeviceSynchronize());

			int new_num_particles = fluidBufferSet->numParticles - *countRmv;
			std::cout << "handle_fluid_boundries_cuda: changing num particles in the buffer: " << new_num_particles << "   nb removed : " << *countRmv << std::endl;
			fluidBufferSet->updateActiveParticleNumber(new_num_particles);

			//we need to reinit the neighbors struct for the fluidbuffer since we removed some particles
			fluidBufferSet->initNeighborsSearchData(data, false);

			timings.time_next_point();

			for (int i = 4; i < 8;++i){
				//we need to reinit the neighbors struct for the fluidbuffer since we removed some particles
				fluidBufferSet->initNeighborsSearchData(data, false);

				*countRmv = 0;
				{
					int numBlocks = calculateNumBlocks(fluidBufferSet->numParticles);
					DFSPH_evaluate_density_in_buffer_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, backgroundFluidBufferSet->gpu_ptr, fluidBufferSet->gpu_ptr,
							countRmv, S, i);
					
					gpuErrchk(cudaDeviceSynchronize());
				}
			
				//remove the tagged particle from the buffer (all yhe ones that have a density that is too high)
				//now we can remove the partices from the simulation
				// use the same process as when creating the neighbors structure to put the particles to be removed at the end
				cub::DeviceRadixSort::SortPairs(fluidBufferSet->neighborsDataSet->d_temp_storage_pair_sort, fluidBufferSet->neighborsDataSet->temp_storage_bytes_pair_sort,
					fluidBufferSet->neighborsDataSet->cell_id, fluidBufferSet->neighborsDataSet->cell_id_sorted,
					fluidBufferSet->neighborsDataSet->p_id, fluidBufferSet->neighborsDataSet->p_id_sorted, fluidBufferSet->numParticles);
				gpuErrchk(cudaDeviceSynchronize());

				cuda_sortData(*fluidBufferSet, fluidBufferSet->neighborsDataSet->p_id_sorted);
				gpuErrchk(cudaDeviceSynchronize());

				int new_num_particles = fluidBufferSet->numParticles - *countRmv;
				std::cout << "handle_fluid_boundries_cuda: (iter: "<<i<<") changing num particles in the buffer: " << new_num_particles << "   nb removed : " << *countRmv << std::endl;
				fluidBufferSet->updateActiveParticleNumber(new_num_particles);

			}





		}


		//I'll save the velocity field by setting the velocity of each particle to the weighted average of the three nearest
		//or set it to 0, maybe I need to do smth intermediary
		{
			int numBlocks = calculateNumBlocks(fluidBufferSet->numParticles);
			//DFSPH_init_buffer_velocity_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, fluidBufferSet->pos, fluidBufferSet->vel, fluidBufferSet->numParticles);
			DynamicWindowBuffer::init_buffer_kernel << <numBlocks, BLOCKSIZE >> > (fluidBufferSet->vel, fluidBufferSet->numParticles, Vector3d(0, 0, 0));
			gpuErrchk(cudaDeviceSynchronize());
		}

		timings.time_next_point();

		//now we can remove the partices from the simulation
		{

			*countRmv = 0;
			int numBlocks = calculateNumBlocks(particleSet->numParticles);
			DFSPH_reset_fluid_boundaries_remove_kernel<< <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, countRmv, S);
			
			gpuErrchk(cudaDeviceSynchronize());


			timings.time_next_point();

			//*
			//now use the same process as when creating the neighbors structure to put the particles to be removed at the end
			cub::DeviceRadixSort::SortPairs(particleSet->neighborsDataSet->d_temp_storage_pair_sort, particleSet->neighborsDataSet->temp_storage_bytes_pair_sort,
				particleSet->neighborsDataSet->cell_id, particleSet->neighborsDataSet->cell_id_sorted,
				particleSet->neighborsDataSet->p_id, particleSet->neighborsDataSet->p_id_sorted, particleSet->numParticles);
			gpuErrchk(cudaDeviceSynchronize());

			cuda_sortData(*particleSet, particleSet->neighborsDataSet->p_id_sorted);
			gpuErrchk(cudaDeviceSynchronize());

			//and now you can update the number of particles


			int new_num_particles = particleSet->numParticles - *countRmv;
			std::cout << "handle_fluid_boundries_cuda: changing num particles: " << new_num_particles << "   nb removed : " << *countRmv << std::endl;
			particleSet->updateActiveParticleNumber(new_num_particles);
			//*/

			timings.time_next_point();
		}

		{
			//and now add the buffer back into the simulation
			//check there is enougth space for the particles and make some if necessary
			int new_num_particles = particleSet->numParticles + fluidBufferSet->numParticles;
			if (new_num_particles > particleSet->numParticlesMax) {
				particleSet->changeMaxParticleNumber(new_num_particles * 1.25);
			}

			//add the particle in the simulation
			int numBlocks = calculateNumBlocks(fluidBufferSet->numParticles);
			DFSPH_reset_fluid_boundaries_add_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, fluidBufferSet->gpu_ptr);
			gpuErrchk(cudaDeviceSynchronize());

			//and change the number
			std::cout << "handle_fluid_boundries_cuda: changing num particles: " << new_num_particles << "   nb added : " << fluidBufferSet->numParticles << std::endl;
			particleSet->updateActiveParticleNumber(new_num_particles);

		}

		//now we need to use a particle shifting to remove the density spikes and gap
		// particle shifting from particle concentration
		// Formula found in the fllowing paper (though it's when they explain the earlier works)
		// A multi - phase particle shifting algorithm for SPHsimulations of violent hydrodynamics with a largenumber of particles
		if(true){
			RealCuda diffusion_coefficient = 1000*0.5* data.getKernelRadius()* data.getKernelRadius()/data.density0;
			particleSet->initNeighborsSearchData(data, false);

			{
				int numBlocks = calculateNumBlocks(particleSet->numParticles);
				DFSPH_evaluate_density_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, S);
				gpuErrchk(cudaDeviceSynchronize());
			}

			{
				int numBlocks = calculateNumBlocks(particleSet->numParticles);
				DFSPH_evaluate_particle_concentration_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, S);
				gpuErrchk(cudaDeviceSynchronize());
			}

			{
				*countRmv = 0;
				*realcuda_ptr = 0;
				int numBlocks = calculateNumBlocks(particleSet->numParticles);
				DFSPH_particle_shifting_base_kernel << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, diffusion_coefficient, S, countRmv, realcuda_ptr);
				gpuErrchk(cudaDeviceSynchronize());

				std::cout << "count particles shifted: " << *countRmv << "   for a total displacement: " << *realcuda_ptr << std::endl;
			}


			//	__global__ void DFSPH_evaluate_particle_concentration_kernel(SPH::DFSPHCData data, SPH::UnifiedParticleSet* fluidSet,
			//DFSPH_particle_shifting_base_kernel
		}
        particleSet->initNeighborsSearchData(data,true);

		timings.time_next_point();
		timings.end_step();
		timings.recap_timings();

		//still need the damping near the borders as long as we don't implement the implicit borders
		//with paricle boundaries 3 is the lowest number of steps that absorb nearly any visible perturbations
		//*/
		add_border_to_damp_planes_cuda(data, x_motion, !x_motion);
        data.damp_borders_steps_count = 5;
		data.damp_borders = true;
		//*/
	}

	//here is a test to see what does the density filed looks like at the interface
		//first let's do a test
		//let's compute the density on the transition plane at regular interval in the fluid 
		//then compare with the values obtained when I add the buffer back in the simulation
	if (false) {

		//get the min for further calculations
		Vector3d min, max;
		get_UnifiedParticleSet_min_max_naive_cuda(*particleSet, min, max);
		//first define the structure that will hold the density values
#define NBR_LAYERS 10
#define NBR_LAYERS_IN_BUFFER 7
		RealCuda sampling_spacing = data.particleRadius / 2;
		Vector3i vec_count_samples(0, (max.y - min.y) / sampling_spacing + 1 + 10, (max.z - min.z) / sampling_spacing + 1 + 10);
		int count_samples = vec_count_samples.y * vec_count_samples.z;
		count_samples *= NBR_LAYERS;

		static RealCuda* density_field = NULL;
		static RealCuda* density_field_after_buffer = NULL;
		static Vector3d* sample_pos = NULL;
		static int density_field_size = 0;

		if (count_samples > density_field_size) {
			CUDA_FREE_PTR(density_field);

			density_field_size = count_samples * 1.5;
			cudaMallocManaged(&(density_field), density_field_size * sizeof(RealCuda));
			cudaMallocManaged(&(density_field_after_buffer), density_field_size * sizeof(RealCuda));
			cudaMallocManaged(&(sample_pos), density_field_size * sizeof(Vector3d));

		}

		//re-init the neighbor structure to be able to compute the density
		particleSet->initNeighborsSearchData(data, false);
		fluidBufferSet->initNeighborsSearchData(data, false);

		{
			int numBlocks = calculateNumBlocks(count_samples);
			DFSPH_evaluate_density_field_kernel<NBR_LAYERS, NBR_LAYERS_IN_BUFFER> << <numBlocks, BLOCKSIZE >> > (data, particleSet->gpu_ptr, fluidBufferSet->gpu_ptr,
				min, max, vec_count_samples, density_field, density_field_after_buffer, count_samples, sampling_spacing, sample_pos);
			gpuErrchk(cudaDeviceSynchronize());
		}

		//write it forthe viewer
		if (true) {
			std::ostringstream oss;
			int effective_count = 0;
			for (int i = 0; i < count_samples; ++i) {
				uint8_t density = fminf(1.0f, (density_field[i] / 1300.0f)) * 255;
				uint8_t alpha = 255;
				/*
				if (density_field[i] >= 0) {
					if (density == 0) {
						continue;
					}
					if (density > 245) {
						continue;
					}
				}
				//*/

				//uint8_t density = (density_field[i] > 500) ? 255 : 0;
				uint32_t txt = (((alpha << 8) + density << 8) + density << 8) + density;

				if (density_field_after_buffer[i] < 1000) {
					txt = (((alpha << 8) + density << 8) + 0 << 8) + 0;
				}
				if (density_field_after_buffer[i] < 800) {
					txt = (((alpha << 8) + 0 << 8) + density << 8) + 0;
				}
				if (density_field_after_buffer[i] < 600) {
					txt = (((alpha << 8) + 0 << 8) + 0 << 8) + density;
				}
				//if (density_field_after_buffer[i] > 950) {
				//	txt = (((alpha << 8) + 255 << 8) + 0 << 8) + 144;
				//}

				oss << sample_pos[i].x << " " << sample_pos[i].y << " " << sample_pos[i].z << " "
					<< txt << std::endl;
				effective_count++;
			}

			std::ofstream myfile("densityCloud.pcd", std::ofstream::trunc);
			if (myfile.is_open())
			{
				myfile << "VERSION 0.7" << std::endl
					<< "FIELDS x y z rgb" << std::endl
					<< "SIZE 4 4 4 4" << std::endl
					<< "TYPE F F F U" << std::endl
					<< "COUNT 1 1 1 1" << std::endl
					<< "WIDTH " << effective_count << std::endl
					<< "HEIGHT " << 1 << std::endl
					<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
					<< "POINTS " << effective_count << std::endl
					<< "DATA ascii" << std::endl;
				myfile << oss.str();
				myfile.close();
			}
		}
#undef NBR_LAYERS
#undef NBR_LAYERS_IN_BUFFER
	}

	std::cout << "handling lfuid boundaries finished" << std::endl;
	
}
