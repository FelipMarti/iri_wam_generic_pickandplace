// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _wam_generic_pickandplace_alg_node_h_
#define _wam_generic_pickandplace_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "wam_generic_pickandplace_alg.h"

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]
#include <iri_action_server/iri_action_server.h>
#include <iri_wam_generic_pickandplace/PickAndPlaceAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iri_wam_generic_pickorplace/PickOrPlaceAction.h>

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class WamGenericPickandplaceAlgNode:public algorithm_base::IriBaseAlgorithm <
    WamGenericPickandplaceAlgorithm > {
 private:
	// [publisher attributes]

	// [subscriber attributes]

	// [service attributes]

	// [client attributes]

	// [action server attributes]
	IriActionServer < iri_wam_generic_pickandplace::PickAndPlaceAction >
	    pick_and_place_aserver_;
	void pick_and_placeStartCallback(const iri_wam_generic_pickandplace::
					 PickAndPlaceGoalConstPtr & goal);
	void pick_and_placeStopCallback(void);
	bool pick_and_placeIsFinishedCallback(void);
	bool pick_and_placeHasSucceedCallback(void);
	void pick_and_placeGetResultCallback(iri_wam_generic_pickandplace::
					     PickAndPlaceResultPtr & result);
	void pick_and_placeGetFeedbackCallback(iri_wam_generic_pickandplace::
					       PickAndPlaceFeedbackPtr &
					       feedback);

	// [action client attributes]
	actionlib::SimpleActionClient <
	    iri_wam_generic_pickorplace::PickOrPlaceAction >
	    pick_or_place_client_;
	iri_wam_generic_pickorplace::PickOrPlaceGoal pick_or_place_goal_;
	void pick_or_placeMakeActionRequest();
	void pick_or_placeDone(const actionlib::SimpleClientGoalState & state,
			       const iri_wam_generic_pickorplace::
			       PickOrPlaceResultConstPtr & result);
	void pick_or_placeActive();
	void pick_or_placeFeedback(const iri_wam_generic_pickorplace::
				   PickOrPlaceFeedbackConstPtr & feedback);

	// MY CLASS VARS
	int Pick_and_Place_State;
	int Pick_and_Place_Result;
	int Pick_or_Place_Feedback;

	// Pick and Place Action variables 
	// 3D position   
	struct point_XYZ {	//or point_RPY
		float X;	//or R
		float Y;	//or P
		float Z;	//or Y
	};

	point_XYZ pregrasp_point;
	point_XYZ grasp_point;
	point_XYZ postgrasp_point;
	point_XYZ ini_grasp_EF_rpy;
	point_XYZ end_grasp_EF_rpy;
	point_XYZ preungrasp_point;
	point_XYZ ungrasp_point;
	point_XYZ postungrasp_point;
	point_XYZ ini_ungrasp_EF_rpy;
	point_XYZ end_ungrasp_EF_rpy;

 public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	WamGenericPickandplaceAlgNode(void);

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
	~WamGenericPickandplaceAlgNode(void);

 protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
	void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
	void node_config_update(Config & config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
	void addNodeDiagnostics(void);

	// [diagnostic functions]

	// [test functions]
};

#endif
