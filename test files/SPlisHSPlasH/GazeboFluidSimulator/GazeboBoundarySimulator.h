#ifndef __GazeboBoundarySimulator_h__
#define __GazeboBoundarySimulator_h__

#include "Simulator/BoundarySimulator.h"
#include "GazeboRigidBody.h"
#include <fstream>
#include <iomanip>

#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"

using namespace gazebo;
namespace SPH
{
	class GazeboSimulatorBase;
	class TriangleMesh;

	class GazeboBoundarySimulator : public BoundarySimulator
	{
	protected:
		GazeboSimulatorBase *m_base;
		
		// CSV export members
		std::ofstream m_csvFile;
		bool m_csvHeaderWritten;
		unsigned int m_frameCounter;
		
		void loadObj(const std::string &filename, TriangleMesh &mesh, const Vector3r &scale);
		void exportForcesTorquesToCSV();

	public:
		GazeboBoundarySimulator(GazeboSimulatorBase *base);
		void updateBoundaryForces();
		void initBoundaryData();
		virtual void timeStep();
		virtual ~GazeboBoundarySimulator();
		virtual void init() {}
	};
}

#endif