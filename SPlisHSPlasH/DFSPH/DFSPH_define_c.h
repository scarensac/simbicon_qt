#ifndef DFSPH_DEFINE_C
#define DFSPH_DEFINE_C

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif


#define GROUP_DYNAMIC_BODIES_NEIGHBORS_SEARCH


//this control if the neighbors for each particle are stored
//currently not using the stored particles only works when computing the porperties of the fluid with only one fluid
//see macro file for comment on why
#define STORE_PARTICLE_NEIGHBORS
#ifdef STORE_PARTICLE_NEIGHBORS
#define INTERLEAVE_NEIGHBORS
#endif //STORE_PARTICLE_NEIGHBORS

//the size is 75 because I checked and the max neighbours I reached was 58
//so I put some more to be sure. In the end those buffers will stay on the GPU memory
//so there will be no transfers.
#define MAX_NEIGHBOURS 90

//control the precomputation of the kernel
//#define PRECOMPUTED_KERNELS
#define PRECOMPUTED_KERNELS_SAMPLE_COUNT 1000
#ifdef PRECOMPUTED_KERNELS
#define PRECOMPUTED_KERNELS_USE_CONSTANT_MEMORY
#endif

//use the bender 2019 boundaries
//#define BENDER2019_BOUNDARIES

//juste a define to active the ocean boundaries prototype
#define OCEAN_BOUNDARIES_PROTOTYPE

#endif 
