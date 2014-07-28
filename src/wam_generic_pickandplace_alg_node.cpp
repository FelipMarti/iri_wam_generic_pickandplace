#include "wam_generic_pickandplace_alg_node.h"

WamGenericPickandplaceAlgNode::WamGenericPickandplaceAlgNode
    (void):algorithm_base::IriBaseAlgorithm < WamGenericPickandplaceAlgorithm >
    (), pick_and_place_aserver_(public_node_handle_, "pick_and_place"),
pick_or_place_client_("iri_wam_generic_pickorplace/pick_or_place", true)
{
	//init class attributes if necessary
	Pick_and_Place_State = 0;
	Pick_and_Place_Result = 0;
	//this->loop_rate_ = 2;//in [Hz]

	// [init publishers]

	// [init subscribers]

	// [init services]

	// [init clients]

	// [init action servers]
	pick_and_place_aserver_.registerStartCallback(boost::bind
						      (&WamGenericPickandplaceAlgNode::
						       pick_and_placeStartCallback,
						       this, _1));
	pick_and_place_aserver_.
	    registerStopCallback(boost::
				 bind(&WamGenericPickandplaceAlgNode::
				      pick_and_placeStopCallback, this));
	pick_and_place_aserver_.
	    registerIsFinishedCallback(boost::
				       bind(&WamGenericPickandplaceAlgNode::
					    pick_and_placeIsFinishedCallback,
					    this));
	pick_and_place_aserver_.
	    registerHasSucceedCallback(boost::
				       bind(&WamGenericPickandplaceAlgNode::
					    pick_and_placeHasSucceedCallback,
					    this));
	pick_and_place_aserver_.
	    registerGetResultCallback(boost::
				      bind(&WamGenericPickandplaceAlgNode::
					   pick_and_placeGetResultCallback,
					   this, _1));
	pick_and_place_aserver_.
	    registerGetFeedbackCallback(boost::
					bind(&WamGenericPickandplaceAlgNode::
					     pick_and_placeGetFeedbackCallback,
					     this, _1));
	pick_and_place_aserver_.start();

	// [init action clients]
}

WamGenericPickandplaceAlgNode::~WamGenericPickandplaceAlgNode(void)
{
	// [free dynamic memory]
}

void WamGenericPickandplaceAlgNode::mainNodeThread(void)
{
	// [fill msg structures]

	// [fill srv structure and make request to the server]

	// [fill action structure and make request to the action server]

	// [publish messages]

	// STATE MACHINE
	// Pick_and_Place_State=0 => Robot ready for a new action 
	// Pick_and_Place_State=1 => Robot PICK action
	// Pick_and_Place_State=2 => WAIT, Robot is doing PICK action
	// Pick_and_Place_State=3 => Robot transition
	// Pick_and_Place_State=4 => WAIT, Robot is doing Transition
	// Pick_and_Place_State=5 => Robot PLACE action
	// Pick_and_Place_State=6 => WAIT, Robot is doing PLACE action
	// Pick_and_Place_State=8 => END OK
	// Pick_and_Place_State=9 => END NOK
	// Pick_and_Place_State=10 => STOP
	// Pick_and_Place_State=50 => Check place action is possible
	// Pick_and_Place_State=51 => WAIT 50

	if (this->Pick_and_Place_State == 1) {	//Pick

		//Filling action strucutre
		pick_or_place_goal_.ini_point[0] = pregrasp_point.X;
		pick_or_place_goal_.ini_point[1] = pregrasp_point.Y;
		pick_or_place_goal_.ini_point[2] = pregrasp_point.Z;

		pick_or_place_goal_.grasp_point[0] = grasp_point.X;
		pick_or_place_goal_.grasp_point[1] = grasp_point.Y;
		pick_or_place_goal_.grasp_point[2] = grasp_point.Z;

		pick_or_place_goal_.end_point[0] = postgrasp_point.X;
		pick_or_place_goal_.end_point[1] = postgrasp_point.Y;
		pick_or_place_goal_.end_point[2] = postgrasp_point.Z;

		pick_or_place_goal_.ini_EF_rpy[0] = ini_grasp_EF_rpy.X;
		pick_or_place_goal_.ini_EF_rpy[1] = ini_grasp_EF_rpy.Y;
		pick_or_place_goal_.ini_EF_rpy[2] = ini_grasp_EF_rpy.Z;

		pick_or_place_goal_.end_EF_rpy[0] = end_grasp_EF_rpy.X;
		pick_or_place_goal_.end_EF_rpy[1] = end_grasp_EF_rpy.Y;
		pick_or_place_goal_.end_EF_rpy[2] = end_grasp_EF_rpy.Z;

		pick_or_place_goal_.pick = true;
		pick_or_place_goal_.execute = true;

		//Start the action
		pick_or_placeMakeActionRequest();

		//Change State
		if (Pick_and_Place_State == 1)
			Pick_and_Place_State = 2;

	}
	else if (this->Pick_and_Place_State == 3) {	//Transition

		if (postgrasp_point.X == preungrasp_point.X
		    && postgrasp_point.Y == preungrasp_point.Y
		    && postgrasp_point.Z == preungrasp_point.Z) {
			//Transition is not needed
			if (Pick_and_Place_State == 3)
				Pick_and_Place_State = 5;
		}
		else {
			//Transition Stuff
			//TODO:Transition with obstacle avoiding
			if (Pick_and_Place_State == 3)
				Pick_and_Place_State = 5;
		}

	}
	else if (this->Pick_and_Place_State == 5) {	//Place

		//Filling action strucutre
		pick_or_place_goal_.ini_point[0] = preungrasp_point.X;
		pick_or_place_goal_.ini_point[1] = preungrasp_point.Y;
		pick_or_place_goal_.ini_point[2] = preungrasp_point.Z;

		pick_or_place_goal_.grasp_point[0] = ungrasp_point.X;
		pick_or_place_goal_.grasp_point[1] = ungrasp_point.Y;
		pick_or_place_goal_.grasp_point[2] = ungrasp_point.Z;

		pick_or_place_goal_.end_point[0] = postungrasp_point.X;
		pick_or_place_goal_.end_point[1] = postungrasp_point.Y;
		pick_or_place_goal_.end_point[2] = postungrasp_point.Z;

		pick_or_place_goal_.ini_EF_rpy[0] = ini_ungrasp_EF_rpy.X;
		pick_or_place_goal_.ini_EF_rpy[1] = ini_ungrasp_EF_rpy.Y;
		pick_or_place_goal_.ini_EF_rpy[2] = ini_ungrasp_EF_rpy.Z;

		pick_or_place_goal_.end_EF_rpy[0] = end_ungrasp_EF_rpy.X;
		pick_or_place_goal_.end_EF_rpy[1] = end_ungrasp_EF_rpy.Y;
		pick_or_place_goal_.end_EF_rpy[2] = end_ungrasp_EF_rpy.Z;

		pick_or_place_goal_.pick = false;
		pick_or_place_goal_.execute = true;

		//Start the action
		pick_or_placeMakeActionRequest();

		//Change State
		if (Pick_and_Place_State == 5)
			Pick_and_Place_State = 6;

	}
	else if (this->Pick_and_Place_State == 10) {

		pick_or_place_client_.cancelGoal();
		Pick_and_Place_State = 9;

	}
	else if (this->Pick_and_Place_State == 50) {
		/*  EXTRA STATE
		   The idea is to check if it is possible to perform the place action
		   because we don't want to pick and let the robot hanging 
		 */

		//Filling action strucutre
		pick_or_place_goal_.ini_point[0] = preungrasp_point.X;
		pick_or_place_goal_.ini_point[1] = preungrasp_point.Y;
		pick_or_place_goal_.ini_point[2] = preungrasp_point.Z;

		pick_or_place_goal_.grasp_point[0] = ungrasp_point.X;
		pick_or_place_goal_.grasp_point[1] = ungrasp_point.Y;
		pick_or_place_goal_.grasp_point[2] = ungrasp_point.Z;

		pick_or_place_goal_.end_point[0] = postungrasp_point.X;
		pick_or_place_goal_.end_point[1] = postungrasp_point.Y;
		pick_or_place_goal_.end_point[2] = postungrasp_point.Z;

		pick_or_place_goal_.ini_EF_rpy[0] = ini_ungrasp_EF_rpy.X;
		pick_or_place_goal_.ini_EF_rpy[1] = ini_ungrasp_EF_rpy.Y;
		pick_or_place_goal_.ini_EF_rpy[2] = ini_ungrasp_EF_rpy.Z;

		pick_or_place_goal_.end_EF_rpy[0] = end_ungrasp_EF_rpy.X;
		pick_or_place_goal_.end_EF_rpy[1] = end_ungrasp_EF_rpy.Y;
		pick_or_place_goal_.end_EF_rpy[2] = end_ungrasp_EF_rpy.Z;

		pick_or_place_goal_.pick = false;
		pick_or_place_goal_.execute = false;

		//Start the action
		pick_or_placeMakeActionRequest();

		//Change State
		if (Pick_and_Place_State == 50)
			Pick_and_Place_State = 51;

	}

}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */
void WamGenericPickandplaceAlgNode::
pick_and_placeStartCallback(const
			    iri_wam_generic_pickandplace::
			    PickAndPlaceGoalConstPtr & goal)
{
	alg_.lock();
	//check goal 
	//execute goal 
	if (Pick_and_Place_State == 0 || Pick_and_Place_State == 9) {

		//Init var result
		Pick_and_Place_Result = 0;

		//Get all action vars
		pregrasp_point.X = goal->pregrasp_point[0];
		pregrasp_point.Y = goal->pregrasp_point[1];
		pregrasp_point.Z = goal->pregrasp_point[2];

		grasp_point.X = goal->grasp_point[0];
		grasp_point.Y = goal->grasp_point[1];
		grasp_point.Z = goal->grasp_point[2];

		postgrasp_point.X = goal->postgrasp_point[0];
		postgrasp_point.Y = goal->postgrasp_point[1];
		postgrasp_point.Z = goal->postgrasp_point[2];

		ini_grasp_EF_rpy.X = goal->ini_grasp_EF_rpy[0];
		ini_grasp_EF_rpy.Y = goal->ini_grasp_EF_rpy[1];
		ini_grasp_EF_rpy.Z = goal->ini_grasp_EF_rpy[2];

		end_grasp_EF_rpy.X = goal->end_grasp_EF_rpy[0];
		end_grasp_EF_rpy.Y = goal->end_grasp_EF_rpy[1];
		end_grasp_EF_rpy.Z = goal->end_grasp_EF_rpy[2];

		preungrasp_point.X = goal->preungrasp_point[0];
		preungrasp_point.Y = goal->preungrasp_point[1];
		preungrasp_point.Z = goal->preungrasp_point[2];

		ungrasp_point.X = goal->ungrasp_point[0];
		ungrasp_point.Y = goal->ungrasp_point[1];
		ungrasp_point.Z = goal->ungrasp_point[2];

		postungrasp_point.X = goal->postungrasp_point[0];
		postungrasp_point.Y = goal->postungrasp_point[1];
		postungrasp_point.Z = goal->postungrasp_point[2];

		ini_ungrasp_EF_rpy.X = goal->ini_ungrasp_EF_rpy[0];
		ini_ungrasp_EF_rpy.Y = goal->ini_ungrasp_EF_rpy[1];
		ini_ungrasp_EF_rpy.Z = goal->ini_ungrasp_EF_rpy[2];

		end_ungrasp_EF_rpy.X = goal->end_ungrasp_EF_rpy[0];
		end_ungrasp_EF_rpy.Y = goal->end_ungrasp_EF_rpy[1];
		end_ungrasp_EF_rpy.Z = goal->end_ungrasp_EF_rpy[2];

		//Start Action
		//Before start check place action 
		Pick_and_Place_State = 50;

	}
	alg_.unlock();
}

void WamGenericPickandplaceAlgNode::pick_and_placeStopCallback(void)
{
	alg_.lock();
	//stop action 
	if (Pick_and_Place_State > 0 && Pick_and_Place_State < 7)
		Pick_and_Place_State = 10;
	alg_.unlock();
}

bool WamGenericPickandplaceAlgNode::pick_and_placeIsFinishedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if action has finish for any reason 
	if (Pick_and_Place_State == 8 || Pick_and_Place_State == 9) {
		Pick_and_Place_State = 0;
		ret = true;
	}
	alg_.unlock();

	return ret;
}

bool WamGenericPickandplaceAlgNode::pick_and_placeHasSucceedCallback(void)
{
	bool ret = false;

	alg_.lock();
	//if goal was accomplished 
	ret = Pick_and_Place_Result;
	alg_.unlock();

	return ret;
}

void WamGenericPickandplaceAlgNode::
pick_and_placeGetResultCallback(iri_wam_generic_pickandplace::
				PickAndPlaceResultPtr & result)
{
	alg_.lock();
	//update result data to be sent to client 
	result->successful = Pick_and_Place_Result;
	alg_.unlock();
}

void WamGenericPickandplaceAlgNode::
pick_and_placeGetFeedbackCallback(iri_wam_generic_pickandplace::
				  PickAndPlaceFeedbackPtr & feedback)
{
	alg_.lock();
	//keep track of feedback 
	feedback->PandP_state = Pick_and_Place_State;
	feedback->PorP_state = Pick_or_Place_Feedback;
	//ROS_INFO("feedback: %s", feedback->data.c_str()); 
	alg_.unlock();
}

void WamGenericPickandplaceAlgNode::
pick_or_placeDone(const actionlib::SimpleClientGoalState & state,
		  const iri_wam_generic_pickorplace::PickOrPlaceResultConstPtr &
		  result)
{
	alg_.lock();
	if (state.toString().compare("SUCCEEDED") == 0) {
		ROS_INFO
		    ("WamGenericPickandplaceAlgNode::pick_or_placeDone: Goal Achieved!");
		//Change State
		if (Pick_and_Place_State == 2)
			Pick_and_Place_State = 3;
		if (Pick_and_Place_State == 4)
			Pick_and_Place_State = 5;
		if (Pick_and_Place_State == 6) {
			Pick_and_Place_State = 8;
			Pick_and_Place_Result = 1;
		}
		if (Pick_and_Place_State == 51)
			Pick_and_Place_State = 1;

	}
	else {
		ROS_INFO("WamGenericPickandplaceAlgNode::pick_or_placeDone: %s",
			 state.toString().c_str());
		Pick_and_Place_State = 9;
	}
	//copy & work with requested result 
	alg_.unlock();
}

void WamGenericPickandplaceAlgNode::pick_or_placeActive()
{
	alg_.lock();
	//ROS_INFO("WamGenericPickandplaceAlgNode::pick_or_placeActive: Goal just went active!"); 
	alg_.unlock();
}

void WamGenericPickandplaceAlgNode::
pick_or_placeFeedback(const
		      iri_wam_generic_pickorplace::PickOrPlaceFeedbackConstPtr &
		      feedback)
{
	alg_.lock();
	//ROS_INFO("WamGenericPickandplaceAlgNode::pick_or_placeFeedback: Got Feedback!"); 

	bool feedback_is_ok = true;

	//analyze feedback 
	//my_var = feedback->var; 
	Pick_or_Place_Feedback = feedback->PorP_state;

	//if feedback is not what expected, cancel requested goal 
	if (!feedback_is_ok) {
		pick_or_place_client_.cancelGoal();
		//ROS_INFO("WamGenericPickandplaceAlgNode::pick_or_placeFeedback: Cancelling Action!"); 
	}
	alg_.unlock();
}

/*  [action requests] */
void WamGenericPickandplaceAlgNode::pick_or_placeMakeActionRequest()
{
	ROS_INFO
	    ("WamGenericPickandplaceAlgNode::pick_or_placeMakeActionRequest: Starting New Request!");

	//wait for the action server to start 
	//will wait for infinite time 
	pick_or_place_client_.waitForServer();
	ROS_INFO
	    ("WamGenericPickandplaceAlgNode::pick_or_placeMakeActionRequest: Server is Available!");

	//send a goal to the action 
	//pick_or_place_goal_.data = my_desired_goal; 
	pick_or_place_client_.sendGoal(pick_or_place_goal_,
				       boost::bind
				       (&WamGenericPickandplaceAlgNode::
					pick_or_placeDone, this, _1, _2),
				       boost::
				       bind(&WamGenericPickandplaceAlgNode::
					    pick_or_placeActive, this),
				       boost::
				       bind(&WamGenericPickandplaceAlgNode::
					    pick_or_placeFeedback, this, _1));
	ROS_INFO
	    ("WamGenericPickandplaceAlgNode::pick_or_placeMakeActionRequest: Goal Sent. Wait for Result!");
}

void WamGenericPickandplaceAlgNode::node_config_update(Config & config,
						       uint32_t level)
{
	this->alg_.lock();

	this->alg_.unlock();
}

void WamGenericPickandplaceAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
	return algorithm_base::main < WamGenericPickandplaceAlgNode > (argc,
								       argv,
								       "wam_generic_pickandplace_alg_node");
}
