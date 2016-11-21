#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <stdio.h>

namespace gazebo
{

	class ModelWheel : public ModelPlugin{

		// Pointer to the model
		private: physics::ModelPtr model;

		/// \brief Pointer to the right wheel joint
	    public: physics::JointPtr RJoint;

	    /// \brief Pointer to the left wheel joint
	    public: physics::JointPtr LJoint;

	    /// \brief Pointer to the update event connection
	    //public: event::ConnectionPtr updateConnection;

		/////////////////////////////////////////////////
		ModelWheel(){
			this->LJoint = NULL;
			this->RJoint = NULL;
		}

		/////////////////////////////////////////////////
		~ModelWheel(){
			//this->updateConnection.reset();

			this->RJoint = NULL;

			this->LJoint = NULL;
		}


		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
			// Store the pointer to the model
			this->model = _parent;

			this->RJoint = this->model->GetJoint("link_0_JOINT_1");
			this->LJoint = this->model->GetJoint("link_0_JOINT_0");

			/*std::string lJointName =
			this->dataPtr->sdf->Get<std::string>("link_0_JOINT_0");

			this->dataPtr->LJoint = this->dataPtr->model->GetJoint(lJointName);

			std::string rJointName =
			this->dataPtr->sdf->Get<std::string>("link_0_JOINT_1");

			this->dataPtr->RJoint = this->dataPtr->model->GetJoint(rJointName);
	*/
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			//this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				//	boost::bind(&ModelWheel::Update, this, _1));
			std::cout<<this->model->GetJointCount();
		}

		// Called by the world update start event
		void Update(const common::UpdateInfo & /*_i axis = nfo*/)
		{
			/*math::Vector3 *axis = new math::Vector3(0,0,1);
			this->RJoint->SetAxis(1, *axis);
			this->LJoint->SetAxis(1, *axis);*/
		
			// Apply a small force to the wheels.
			this->RJoint->SetParam("max_force", 1, 10000);
			this->LJoint->SetParam("max_force", 1, 10000);
			this->RJoint->SetParam("vel", 1, 10);
			this->LJoint->SetParam("vel", 1, 10);
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelWheel)

}