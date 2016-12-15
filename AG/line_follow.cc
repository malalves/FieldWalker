#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <ignition/math/Vector2.hh>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Publisher.hh>
#include <boost/shared_ptr.hpp>
#include "msgs/build/Evolve.pb.h"
#include "msgs/build/EvolveRequest.pb.h"

typedef const boost::shared_ptr<const evolve_robots_msgs::msgs::EvolveRequest> evReqPtr;
typedef const boost::shared_ptr<const evolve_robots_msgs::msgs::Evolve> EvPtr;

namespace gazebo
{

	class ModelLine : public ModelPlugin{

		public: const double timeout = 120;

		public: double timeInit = -1;

		public: bool end = 0;

		public: double BaseSpeed[10];

		public: const double AngC = 0.01;

		public: math::Vector2d* points[11] = {new math::Vector2d(0,0), new math::Vector2d(3,3)
			, new math::Vector2d(10,11), new math::Vector2d(-30,-50), new math::Vector2d(90,0)
			, new math::Vector2d(26,-4), new math::Vector2d(-70,69), new math::Vector2d(40,40)
			, new math::Vector2d(53,-14), new math::Vector2d(9,12), new math::Vector2d(0,0)};

		public: math::Pose pose;

		public: std::vector<int> tour;

		public: int wayCounter = 0;

		public: double carrotDist[10];

		// Pointer to the model
		private: physics::ModelPtr model;

		/// \brief Pointer to the right wheel joint
	    public: physics::JointPtr RJoint;

	    /// \brief Pointer to the left wheel joint
	    public: physics::JointPtr LJoint;

		public: physics::JointControllerPtr Jcontrol;

	    /// \brief Pointer to the update event connection
	    public: event::ConnectionPtr updateConnection;

	    public: gazebo::transport::NodePtr node;

	    public: gazebo::transport::SubscriberPtr subAG;

	    public: gazebo::transport::SubscriberPtr subTime;

	    public: gazebo::transport::PublisherPtr sender;

	    public: EvPtr data;

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

		public: void cbTime(ConstWorldStatisticsPtr &_msgs){
			if(end){
				data->set_time(_msgs->real_time().sec() - timeInit);
				sender->WaitForConnection();
        		sender->Publish(*data);
			}
			if(timeInit == -1){
				timeInit = _msgs->real_time().sec();
			}
			else if(_msgs->real_time().sec() - timeInit >= timeout){
				data->set_time(_msgs->real_time().sec() - timeInit);
				end = 1;
			}
		}

		public: void cbAG(evReqPtr &_msgs){
			data->set_index(_msgs->index());
			//colocar o indice da rota
			for (int i = 0; i < 10; ++i)
			{
				tour.push_back(_msgs->road(i));
				BaseSpeed[i] = _msgs->speeds(i);
				carrotDist[i] = _msgs->carrot(i);
			}
			tour.push_back(_msgs->road(0));

		}

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
			// Store the pointer to the model
			this->model = _parent;

			//pointers to the joints
			this->RJoint = this->model->GetJoint("Rjoint");
			this->LJoint = this->model->GetJoint("Ljoint");
			
			this->Jcontrol = this->model->GetJointController();
			
			this->Jcontrol->AddJoint(this->RJoint);
			this->Jcontrol->AddJoint(this->LJoint);

			common::PID pid;
			pid.Init(10,0,0.2,0,0,100,-100);
			this->Jcontrol->SetVelocityPID(this->RJoint->GetScopedName(false),pid);
			this->Jcontrol->SetVelocityPID(this->LJoint->GetScopedName(false),pid);

			gazebo::transport::NodePtr nodeTmp(new gazebo::transport::Node());
			node = nodeTmp;
    		node->Init();
    		subAG = node->Subscribe("~/EvolveRequest", &ModelLine::cbAG, this);
  			subTime = node->Subscribe("~/world_stats", &ModelLine::cbTime, this);
    		sender = node->Advertise<evolve_robots_msgs::msgs::Evolve>("~/Evolve");


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
			double yaw, module, DesiredYaw, angDiference, toNext;
			math::Vector2d Cpos, line, speed, relPos;

			//line direction normalized
			line.x = (this->points[tour[wayCounter+1]]->x-this->points[tour[wayCounter]]->x);
			line.y = (this->points[tour[wayCounter+1]]->y-this->points[tour[wayCounter]]->y);
			line.Normalize();
						
			pose = this->model->GetWorldPose();//pose of the model

			//relative position of the model in relation to the last waypoint
			Cpos.x = (pose.pos.x - this->points[tour[wayCounter]]->x);
			Cpos.y = (pose.pos.y - this->points[tour[wayCounter]]->y);

			relPos = Cpos;

			//projects the model relative position in the line direction
			Cpos.x = Cpos.Dot(line)*line.x;
			Cpos.y = Cpos.Dot(line)*line.y;
			printf("X:%f\nY:%f\n",Cpos.x,Cpos.y);

			//adds the carrotDist in the direction of the line 
			Cpos.x = Cpos.x + carrotDist[wayCounter]*line.x;
			Cpos.y = Cpos.y + carrotDist[wayCounter]*line.y;
			//printf("X:%f\nY:%f\n",Cpos.x,Cpos.y);

			//gets the robot orientation
			yaw = pose.rot.GetYaw();

			//gets the direction of the carrot in reference of the vehicle
			DesiredYaw = std::atan2((Cpos.y - relPos.y),(Cpos.x - relPos.x));

			//the angle between the vehicle orientation and the carrot direction
			angDiference = normAng(DesiredYaw - yaw);

			//seting the speed of each wheel on a vector2d
			speed.x = BaseSpeed[wayCounter] + AngC*BaseSpeed[wayCounter]*angDiference;//Right wheel
			speed.y = BaseSpeed[wayCounter] - AngC*BaseSpeed[wayCounter]*angDiference;//Left wheel
			//printf("ang:%f\nR:%f\nL:%f\n", angDiference, speed.x, speed.y);

			toNext = (pose.pos.x - this->points[tour[wayCounter+1]]->x)*(pose.pos.x - this->points[tour[wayCounter+1]]->x);
			toNext += (pose.pos.y - this->points[tour[wayCounter+1]]->y)*(pose.pos.y - this->points[tour[wayCounter+1]]->y);
			toNext = sqrt(toNext);
			if(toNext < 3){
				wayCounter++;
			}
			if(wayCounter == 11){
				end = 1;
			}

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
			//printf("R:%f\nL:%f\n", this->RJoint->GetVelocity(0), this->LJoint->GetVelocity(0));
		}
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelLine)

}