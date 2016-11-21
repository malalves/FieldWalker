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

		public: const double BaseSpeed = -0.5;

		public: const double AngC = 0.3;

		public: math::Vector2d* points[2] = {new math::Vector2d(-3,3), new math::Vector2d(-3,3)};

		public: math::Pose pose;

		public: const double carrotDist = 0.5;

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

			//pointers to the joints
			this->RJoint = this->model->GetJoint("link_0_JOINT_1");
			this->LJoint = this->model->GetJoint("link_0_JOINT_0");
		}

		double normAng (double ang){
			while(ang/M_PI > 1){
				ang = ang - 2*M_PI;
			}
			while(ang/M_PI <= -1){
				ang = ang + 2*M_PI;
			}
			return ang;
		}

		math::Vector2d carrotChaser(){
			double yaw, module, DesiredYaw, angDiference;
			math::Vector2d Cpos, line, speed;

			//line direction normalized
			line = (this->points[1]-this->points[0]);
			line.Normalize();

			pose = this->model->GetDirtyPose();//pose of the model
			
			//relative position of the model in relation to the last waypoint
			Cpos.x = (pose.pos.x - this->points[0]->x);
			Cpos.y = (pose.pos.y - this->points[0]->y);

			//projects the model relative position in the line direction
			Cpos = Cpos.Dot(line);

			//adds the carrotDist in the direction of the line 
			module = sqrt(Cpos.Dot(Cpos));
			Cpos.Normalize();
			Cpos = Cpos*(module+this->carrotDist);

			//gets the robot orientation
			yaw = pose.rot.GetYaw();

			//gets the direction of the carrot in reference of the vehicle
			DesiredYaw = std::atan2((pose.pos.y - Cpos.y),(pose.pos.x - Cpos.x));

			//the angle between the vehicle orientation and the carrot direction
			angDiference = normAng(DesiredYaw - yaw);

			//seting the speed of each wheel on a vector2d
			speed.x = BaseSpeed + AngC*angDiference;//Right wheel
			speed.y = BaseSpeed - AngC*angDiference;//Left wheel

			return speed;
		}

		// Called by the world update start event 
		void Update(const common::UpdateInfo & /*_i axis = nfo*/)
		{
			math::Vector2d speed = carrotChaser();
			// Apply a small force to the wheels.
			this->RJoint->SetParam("max_force", 1, 10000);
			this->LJoint->SetParam("max_force", 1, 10000);
			this->RJoint->SetParam("vel", 1, speed.x);
			this->LJoint->SetParam("vel", 1, speed.y);
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelWheel)

}