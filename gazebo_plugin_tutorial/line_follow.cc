#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ignition/math/Vector2.hh>
#include <math.h>
#include <stdio.h>

namespace gazebo
{

	class ModelLine : public ModelPlugin{

		public: double BaseSpeed = 10;

		public: const double AngC = 0.01;

		public: math::Vector2d* points[2] = {new math::Vector2d(-3,3), new math::Vector2d(3,3)};

		public: math::Pose pose;

		public: const double carrotDist = 2;

		// Pointer to the model
		private: physics::ModelPtr model;

		/// \brief Pointer to the right wheel joint
	    public: physics::JointPtr RJoint;

	    /// \brief Pointer to the left wheel joint
	    public: physics::JointPtr LJoint;

		public: physics::JointControllerPtr Jcontrol;

	    /// \brief Pointer to the update event connection
	    public: event::ConnectionPtr updateConnection;

		/////////////////////////////////////////////////
		public: ModelLine(){
			this->LJoint = NULL;
			this->RJoint = NULL;
			this->model = NULL;
			this->Jcontrol = NULL;
			printf("olar\n");
		}

		/////////////////////////////////////////////////
		public: ~ModelLine(){
			this->updateConnection.reset();

			this->RJoint = NULL;
			this->LJoint = NULL;
			this->model = NULL;
			this->Jcontrol = NULL;
		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
			// Store the pointer to the model
			this->model = _parent;

			//pointers to the joints
			this->RJoint = this->model->GetJoint("Rjoint");
			this->LJoint = this->model->GetJoint("Ljoint");
			
			this->Jcontrol = this->model->GetJointController();
			
			while(this->Jcontrol == NULL){
				printf("sem ponteiro\n");
			}
			this->Jcontrol->AddJoint(this->RJoint);
			this->Jcontrol->AddJoint(this->LJoint);

			common::PID pid;
			pid.Init(10,0,0.2,0,0,100,-100);
			this->Jcontrol->SetVelocityPID(this->RJoint->GetScopedName(false),pid);
			this->Jcontrol->SetVelocityPID(this->LJoint->GetScopedName(false),pid);

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelLine::OnUpdate, this, _1));
		}

		public: double normAng (double ang){
			while(ang/M_PI > 1){
				ang = ang - 2*M_PI;
			}
			while(ang/M_PI <= -1){
				ang = ang + 2*M_PI;
			}
			//printf("%f\n",ang*180/M_PI );
			return ang;
		}

		public: math::Vector2d carrotChaser(){
			double yaw, module, DesiredYaw, angDiference;
			math::Vector2d Cpos, line, speed, relPos;

			//line direction normalized
			line = (this->points[1]-this->points[0]);
			line.Normalize();
			
			pose = this->model->GetWorldPose();//pose of the model
			
			//relative position of the model in relation to the last waypoint
			Cpos.x = (pose.pos.x - this->points[0]->x);
			Cpos.y = (pose.pos.y - this->points[0]->y);

			relPos = Cpos;

			//projects the model relative position in the line direction
			Cpos = Cpos.Dot(line);

			//adds the carrotDist in the direction of the line 
			module = sqrt(Cpos.Dot(Cpos));
			Cpos.Normalize();
			Cpos = Cpos*(module+this->carrotDist);

			//gets the robot orientation
			yaw = pose.rot.GetYaw();

			//gets the direction of the carrot in reference of the vehicle
			DesiredYaw = std::atan2((Cpos.y - relPos.y),(Cpos.x - relPos.x));

			//the angle between the vehicle orientation and the carrot direction
			angDiference = normAng(DesiredYaw - yaw);

			//seting the speed of each wheel on a vector2d
			speed.x = BaseSpeed + AngC*BaseSpeed*angDiference;//Right wheel
			speed.y = BaseSpeed - AngC*BaseSpeed*angDiference;//Left wheel
			//printf("ang:%f\nR:%f\nL:%f\n", angDiference, speed.x, speed.y);

			return speed;
		}

		// Called by the world update start event 
		public: void OnUpdate(const common::UpdateInfo & /*_i axis = nfo*/)
		{
			math::Vector2d speed = carrotChaser();
			// controls the wheels speed
			this->Jcontrol->SetVelocityTarget(this->RJoint->GetScopedName(false),speed.x);
			this->Jcontrol->SetVelocityTarget(this->LJoint->GetScopedName(false),speed.y);
			//printf("R:%f\nL:%f\n", speed.x, speed.y);
			printf("R:%f\nL:%f\n", this->RJoint->GetVelocity(0)-speed.x, this->LJoint->GetVelocity(0)-speed.y);
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelLine)

}