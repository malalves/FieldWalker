#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ignition/math/Vector2.hh>
#include <stdio.h>
#include <math.h>

namespace gazebo
{

	class ModelWheel : public ModelPlugin{

		public: math::Vector2d* points[2] = {new math::Vector2d(-3,3), new math::Vector2d(-3,3)};

		public: math::Pose pose;

		public: double carrotDist = 0.5;

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
		}

		math::Vector2d carrotChaser(){
			double yaw, module;
			math::Vector2d Cpos, line;

			//line direction normalized
			line = (this->points[1]-this->points[0]);
			line->normalize();

			pose = this->model->getDirtyPose();//pose of the model
			
			//relative position of the model in relation to the last waypoint
			Cpos->x = (pose->pos->x - this->points[0]->x);
			Cpos->y = (pose->pos->y - this->points[0]->y);

			//projects the model relative position in the line direction
			Cpos = Cpos->dot(line);

			//adds the carrotDist in the direction of the line 
			module = sqrt(Cpos->dot(Cpos));
			Cpos->normalize();
			Cpos = Cpos*(module+this->carrotDist)

			//gets the robot orientation
			yaw = pose->rot->getYaw();
		}

		// Called by the world update start event 
		void Update(const common::UpdateInfo & /*_i axis = nfo*/)
		{

			// Apply a small force to the wheels.
			this->RJoint->SetForce(1, .2);
			this->LJoint->SetForce(1, .2);
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelWheel)

}