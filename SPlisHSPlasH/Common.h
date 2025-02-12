#ifndef __Common_h__
#define __Common_h__

#include "BasicTypes.h"
#include "Vector.h"

#ifdef SPLISHSPLASH_FRAMEWORK
#include <Eigen/Dense>

#ifdef USE_DOUBLE
#define EINGEN_FLOATING_VECTOR Eigen::VectorXd
#else
#define EINGEN_FLOATING_VECTOR Eigen::VectorXf
#endif


namespace SPH
{
	

	using Vector2r = Eigen::Matrix<Real, 2, 1>;
	using Vector3r = Eigen::Matrix<Real, 3, 1>;
	using Vector4r = Eigen::Matrix<Real, 4, 1>;
	using Vector5r = Eigen::Matrix<Real, 5, 1>;
	using Vector6r = Eigen::Matrix<Real, 6, 1>;
	using Matrix2r = Eigen::Matrix<Real, 2, 2>;
	using Matrix3r = Eigen::Matrix<Real, 3, 3>;
	using Matrix4r = Eigen::Matrix<Real, 4, 4>;
	using Matrix5r = Eigen::Matrix<Real, 5, 5>;
	using Matrix6r = Eigen::Matrix<Real, 6, 6>;
	using AlignedBox2r = Eigen::AlignedBox<Real, 2>;
	using AlignedBox3r = Eigen::AlignedBox<Real, 3>;
	using AngleAxisr = Eigen::AngleAxis<Real>;
	using Quaternionr = Eigen::Quaternion<Real>;

	//allocators to be used in STL collections containing Eigen structures
	using Alloc_Vector2r = Eigen::aligned_allocator<Vector2r>;
	using Alloc_Vector3r = Eigen::aligned_allocator<Vector3r>;
	using Alloc_Vector4r = Eigen::aligned_allocator<Vector4r>;
	using Alloc_Matrix2r = Eigen::aligned_allocator<Matrix2r>;
	using Alloc_Matrix3r = Eigen::aligned_allocator<Matrix3r>;
	using Alloc_Matrix4r = Eigen::aligned_allocator<Matrix4r>;
	using Alloc_AlignedBox2r = Eigen::aligned_allocator<AlignedBox2r>;
	using Alloc_AlignedBox3r = Eigen::aligned_allocator<AlignedBox3r>;
	using Alloc_AngleAxisr = Eigen::aligned_allocator<AngleAxisr>;
	using Alloc_Quaternionr = Eigen::aligned_allocator<Quaternionr>;

	FORCE_INLINE Vector3d vector3rTo3d(Vector3r v) { return Vector3d(v[0], v[1], v[2]); }
	FORCE_INLINE Vector3r vector3dTo3r(Vector3d v) { return Vector3r(v.x, v.y, v.z); }
	//note the output have to be allocated before the call to the function
	FORCE_INLINE void Matrix3rToArray(Matrix3r input, RealCuda* output) { 
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				output[i*3+j]=input(i, j);
			}
		}
	}

#if EIGEN_ALIGN
	#define SPH_MAKE_ALIGNED_OPERATOR_NEW EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)	   
#ifdef _DEBUG
	// Enable memory leak detection for Eigen new
	#undef SPH_MAKE_ALIGNED_OPERATOR_NEW
	#define SPH_MAKE_ALIGNED_OPERATOR_NEW	EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
		void *operator new(size_t size, int const block_use, char const*  file_name, int const line_number) { \
		\
			return _aligned_malloc_dbg(size, 16, file_name, line_number); \
		} \
		void *operator new[](size_t size, int const block_use, char const*  file_name, int const line_number) { \
			return operator new(size, block_use, file_name, line_number); \
		}\
		void operator delete(void* block, int const block_use, char const*  file_name, int const line_number) noexcept { \
		\
			return _aligned_free_dbg(block); \
		} \
		void operator delete[](void* block, int const block_use, char const*  file_name, int const line_number) noexcept { \
			return operator delete(block, block_use, file_name, line_number); \
		}	
	#define REPORT_MEMORY_LEAKS
#else
	#define REPORT_MEMORY_LEAKS
#endif
#else
	#define REPORT_MEMORY_LEAKS
#endif
#else
	#define SPH_MAKE_ALIGNED_OPERATOR_NEW

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)	   
	// Enable memory leak detection
#ifdef _DEBUG
	  #define _CRTDBG_MAP_ALLOC 
	  #include <stdlib.h> 
	  #include <crtdbg.h> 
	  #define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__) 	
	  #define REPORT_MEMORY_LEAKS _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#else
	  #define REPORT_MEMORY_LEAKS
#endif
#else
  #define REPORT_MEMORY_LEAKS
#endif

#endif




}
#else
namespace SPH
{
    using Vector3r = Vector3d;

    FORCE_INLINE Vector3d vector3rTo3d(Vector3r v) { return v; }
    FORCE_INLINE Vector3r vector3dTo3r(Vector3d v) { return v; }

}
#endif //SPLISHSPLASH_FRAMEWORK

#endif
