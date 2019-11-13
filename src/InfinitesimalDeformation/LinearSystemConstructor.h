#ifndef _LINEAR_SYSTEM_CONSTRUCTOR
#define _LINEAR_SYSTEM_CONSTRUCTOR

#include "AlgorithmCommon.h"

namespace Rigidity {
	
	class HEMatrixConstructor
	{
	public:
		HEMatrixConstructor(Volume::MeshVolume &volume);
		void ConstructHEMatrix(Eigen::MatrixXd &HE, Eigen::MatrixXd &N);
	protected:
		Volume::MeshVolume* m_volume;
	};

	HEMatrixConstructor::HEMatrixConstructor(Volume::MeshVolume &volume) : m_volume(&volume)
	{

	}

	inline void HEMatrixConstructor::ConstructHEMatrix(Eigen::MatrixXd & HE, Eigen::MatrixXd & N)
	{

	}
} // end of namespace Rigidity

#endif // !_LINEAR_SYSTEM_CONSTRUCTOR
