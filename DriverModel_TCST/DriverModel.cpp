/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that does nothing (uses Vissim's internal model).         */
/*                                                                          */
/*  Version of 2017-11-02                                   Lukas Kautzsch  */
/*==========================================================================*/

/*This is the edited version by Behdad to include scheduling and optimal control*/

#include "DriverModel.h"

#include <stdio.h>
#include <iostream> 
#include <fstream>
#include <stdio.h>
#include <wingdi.h>
#include <iostream> 
#include <fstream>
#include <list>
#include <math.h>
#include <complex>
#include <iostream>
#include <ctime>
#include <map>
#include <string> 
#include <vector>
#include <algorithm>
#include <functional>
#include <utility>
#include <set>
#include "CAV.h"
#include <Eigen\Dense>
#include <ctime>
#include <chrono>
#include <iomanip>




/*==========================================================================*/


double  desired_acceleration = 0.0;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_velocity     = 0.0;
long    turning_indicator    = 0;
long	vehicle_color        = RGB(0,0,0);
long switch_to_car_following = 0;


/*==========================================================================*/
std::ofstream log_file; 
std::ofstream log_profile; 
std::ofstream log_debug;
std::stringstream data_time;
std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
std::string path("C:/Users/bchalaki/Desktop/IEEETCST/Data/");

std::string fullName;
std::string fullName1;
std::string fullName2;


/*Variables for CAVs */

double current_time;
double old_time;
long   current_vehicle_id;
double current_velocity=0;
double current_acceleration=0;
double dist_traveled =0; 
double rel_cur_pos=0;

long lead_id;
long right_lead_id;
long left_lead_id;
long current_link;
long current_lane;
long num_lanes;
double car_length;
double dist_to_lead;
double dist_to_left_lead;
double dist_to_right_lead;
long vehicle_type_number; //100 Baseline, 621 CAV
bool baseline = false;
double l_c = 15;//lane change length

std::map <long, std::pair<double, std::string>> entryInfo;

//TCST
std::map <long, CAV*> vehicles_in_cz; // CAVs are added to this after entring the control zone 
std::map <long, std::vector<std::string>> vehicles_links; //for each vehicle id, it stores links that CAV crosses in CZ

std::map<std::string, long> colors = { { "blue", RGB(255, 0, 0) }, { "green", RGB(0, 255, 0) }, { "red", RGB(0, 0, 255) },
							 { "black", RGB(0, 0, 0) }, {"white", RGB(255, 255, 255) },
							 { "yellow", RGB(0, 255, 255) }, { "magenta", RGB(255, 0, 255) }, { "teal", RGB(0, 128, 128) },
							 { "cyan", RGB(255, 255, 0) }, { "purple", RGB(128, 0, 128) }, { "olive", RGB(128, 128, 0) }
};


double	v_min = 5; //[m/s]
double	v_max = 20;
double des_spd_urban = 15;
double safety_gap_dist = 5;

long	egoVehicle_color = RGB(255, 0, 0);
long	color_space = 256 * 256 * 256;
std::string link_seq;
std::string path_name;
std::map<std::string, std::vector<long>> mzs_on_path = { {"NB1",{1}}, {"NB2",{2}}, {"NB3",{3}},
{"SB1",{1}}, {"SB2",{2}}, {"SB3",{3}}, {"WB",{1,2,3}}, {"EB",{3,2,1}} };
												


//Functions 
std::vector<std::string> ReadStringStream(std::string line_string);
std::map<long, double> FindArrivalTime(CAV* car, long lane);
std::map<long, std::vector<double>> SolveUnconEnergyOptimalInterior(CAV* car);
void LaneChange(CAV* car);
bool inside_cz = 0;


/*===========================================================================*/


BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}


DRIVERMODEL_API  int  DriverModelSetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
    case DRIVER_DATA_TIME                   :
        current_time = double_value;
        return 1;
    case DRIVER_DATA_USE_UDA                :
      return 1; /* doesn't use any UDAs */
                /* must return 1 for desired values of index1 if UDA values are to be sent from/to Vissim */
    case DRIVER_DATA_VEH_ID                 :
        current_vehicle_id = long_value;
        return 1;
    case DRIVER_DATA_VEH_LANE               :
		current_lane = long_value;
		return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
		dist_traveled = double_value;
		return 1; 
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
    case DRIVER_DATA_VEH_VELOCITY           :
        current_velocity = double_value;
        return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
        current_acceleration = double_value;
        return 1;
    case DRIVER_DATA_VEH_LENGTH             :
		car_length = double_value;
		return 1;
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
    case DRIVER_DATA_VEH_TYPE               :
		vehicle_type_number = long_value;
		if (vehicle_type_number == 100) {
			//this is the baseline; 
			baseline = true;
			switch_to_car_following = 1;
			
		}
		else {
			baseline = false;
			switch_to_car_following = 0;
		}
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
		current_link = long_value; 
      return 1; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_VEH_INTAC_STATE        :
    case DRIVER_DATA_VEH_INTAC_TARGET_TYPE  :
    case DRIVER_DATA_VEH_INTAC_TARGET_ID    :
    case DRIVER_DATA_VEH_INTAC_HEADWAY      :
    case DRIVER_DATA_VEH_UDA                :
		if (index1 == 3) {
			//sequence of links in cz on cav's path
			link_seq = string_value;
		}
		if (index1 == 4) {
			path_name = string_value; 
		}
		return 1;
    case DRIVER_DATA_NVEH_ID                :
		if (index1 == 0 && index2 == 1) {
			//get id of current_veh in the same lane and link
			lead_id = long_value;
		}
		else if (index1 == 1 && index2 == 1) {
			// leftlane current link
			left_lead_id = long_value;
		}
		else if (index1 == -1 && index2 == 1) {
			// rightlane cuurent link 
			right_lead_id = long_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
    case DRIVER_DATA_NVEH_DISTANCE          :
		if (index1 == 0 && index2 == 1) {
			dist_to_lead = double_value;
		}
		else if (index1 == 1 && index2 == 1) {
			// leftlane current link
			dist_to_left_lead= double_value;
		}
		else if (index1 == -1 && index2 == 1) {
			// rightlane cuurent link 
			dist_to_right_lead = double_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
    case DRIVER_DATA_NVEH_ACCELERATION      :
    case DRIVER_DATA_NVEH_LENGTH            :
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NVEH_TYPE              :
    case DRIVER_DATA_NVEH_UDA               :
    case DRIVER_DATA_NVEH_X_COORDINATE      :
    case DRIVER_DATA_NVEH_Y_COORDINATE      :
    case DRIVER_DATA_NVEH_Z_COORDINATE      :
    case DRIVER_DATA_NVEH_REAR_X_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Y_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Z_COORDINATE :
    case DRIVER_DATA_NO_OF_LANES            :
		num_lanes = long_value;
		return 1;
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
    case DRIVER_DATA_CURRENT_LANE_POLY_N    :
    case DRIVER_DATA_CURRENT_LANE_POLY_X    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Y    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Z    :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      desired_lane_angle = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      active_lane_change = long_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      rel_target_lane = long_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      *double_value = desired_velocity;
      return 1;
    case DRIVER_DATA_VEH_COLOR :
      *long_value = vehicle_color;
      return 1;
    case DRIVER_DATA_VEH_UDA :
      return 0; /* doesn't set any UDA values */
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 1;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      *double_value = desired_acceleration;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      *long_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *long_value = 0;//lane change going to be handled by dll
      return 1;
    case DRIVER_DATA_USE_INTERNAL_MODEL:
      *long_value = switch_to_car_following; /* must be set to 0 if external model is to be applied */
      return 1;
    case DRIVER_DATA_WANTS_ALL_NVEHS:
      *long_value = 0; /* must be set to 1 if data for more than 2 nearby vehicles per lane and upstream/downstream is to be passed from Vissim */
      return 1;
    case DRIVER_DATA_ALLOW_MULTITHREADING:
      *long_value = 0; /* must be set to 1 to allow a simulation run to be started with multiple cores used in the simulation parameters */
      return 1;
    case DRIVER_DATA_VEH_ACCELERATION:
        *double_value = current_acceleration;
        return 1;
    default:
      return 0;
  }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */

  switch (number) {
    case DRIVER_COMMAND_INIT :
		data_time << std::put_time(std::localtime(&t), "%d_%m_%y_%H_%M_%S");

		//get the full file name
	

		fullName = path + "log_file" + "_" + data_time.str() + ".txt";
		fullName1 = path + "log_debug" + "_" + data_time.str() + ".txt";
		fullName2 = path + "profile" + "_" + data_time.str() + ".csv";

		log_file.open(fullName);
		log_debug.open(fullName1);
		log_profile.open(fullName2);


        log_profile << "Time,insidCZ,pathID,CAV_ID,Position,Velocity,Acceleration\n";

      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
        //TODO: Remove the car object, from the coordinator's information, whenever it exits the network 
		//log_file <<entryInfo[current_vehicle_id].first << "," << current_vehicle_id << "," << entryInfo[current_vehicle_id].second << "\n";
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :

		vehicle_color = colors["white"] - color_space;
		inside_cz = 0;

		
        //LogEntryDataOfCars
			if (entryInfo.count(current_vehicle_id) == 0 && !link_seq.empty()) {
				//add current vehicle_id to the entry info map
				std::pair <double, std::string> info(current_time, link_seq);

				//log_file << "++++++++++++++" << link_seq << "++++++++++++\n";
				entryInfo[current_vehicle_id] = info;
				vehicles_links[current_vehicle_id] = ReadStringStream(link_seq);
			}

			if (dist_traveled > 50 ) {
				//this_CAV just entered the control zone;
				vehicle_color = baseline?(colors["purple"] - color_space):(colors["blue"] - color_space);

				inside_cz = 1;

 
				if (vehicles_in_cz.count(current_vehicle_id) == 0) {
					// the CAV  object has not been created yet. 
					std::vector <std::string> conflict_paths;
					CAV* this_car = new CAV(current_vehicle_id, car_length, current_link, current_lane, vehicles_links[current_vehicle_id], current_time, dist_traveled, current_velocity, current_acceleration, lead_id);

					log_debug << "CAV " << this_car->cav_id << " entered the control zone at time: " << current_time;
					log_debug << " from  " << path_name << " and Lane "<<current_lane<<"__"<<dist_traveled<<std::endl;
					this_car->mzs = mzs_on_path[path_name];
					this_car->path_name = path_name; 

				//Find cavs that have lateral potential  with this_CAV 

					if (path_name.find("N") != std::string::npos || path_name.find("S") != std::string::npos) {
						conflict_paths = { "EB","WB" };
					}
					else {
						conflict_paths = { "NB1","SB1","NB2","SB2","NB3","SB3" };
					}

					this_car->conflict_paths = conflict_paths;

					std::map <long, std::vector<CAV*>> laterals; //key is the merging zone, and value returns vector of CAVs that have lateral potential in that zones

					//log_debug << "this CAV: " << this_car->cav_id << std::endl;
					for (auto mz : this_car->mzs) {

						//log_debug << "In mz: " << mz << std::endl;
						std::vector<CAV*> cars; 
						for (auto& key_val : vehicles_in_cz) {
							std::string name = key_val.second->path_name;
							if (std::find(conflict_paths.begin(), conflict_paths.end(), name) == conflict_paths.end()) {
								// it is not in conflict_path
								continue;
							}
							std::vector<long> other_mzs = key_val.second->mzs;
							if (std::find(other_mzs.begin(), other_mzs.end(), mz) != other_mzs.end()) {
								// the mz is common between other CAV and this CAV
								cars.push_back(key_val.second);
								//log_debug << "has lateral conflict with " << key_val.second->cav_id << std::endl;

							}
						}
						laterals[mz] = cars;
					}

					this_car->lateral_vehicles = laterals;

					//Find CAVs that are in front of us

					std::map <long, CAV*> leading_vehicles; 
					if (current_lane == 1) {
						//this_car is in Rightmost lane
						if (lead_id != -1 && vehicles_in_cz.count(lead_id)!=0) {
							leading_vehicles[current_lane] = vehicles_in_cz[lead_id];
							//log_debug << "vehicle leading in currrent lane is " << lead_id << std::endl;
						}
						if (left_lead_id != -1 && vehicles_in_cz.count(left_lead_id) != 0) {
							leading_vehicles[current_lane+1] = vehicles_in_cz[left_lead_id];
							//log_debug << "vehicle leading in left lane is " << left_lead_id << std::endl;
						}
					}
					else if(current_lane == 2){
						if (lead_id != -1 && vehicles_in_cz.count(lead_id) != 0) {
							leading_vehicles[current_lane] = vehicles_in_cz[lead_id];
							//log_debug << "vehicle leading in currrent lane is " << lead_id << std::endl;
						}
						if (right_lead_id != -1 && vehicles_in_cz.count(right_lead_id) != 0) {
							leading_vehicles[current_lane-1] = vehicles_in_cz[right_lead_id];
							//log_debug << "vehicle leading in right lane is " << right_lead_id << std::endl;
						}
					}

					this_car->leading_vehicles = leading_vehicles;
					//Find lane-change 

					if (current_lane == 1) {
						if (dist_to_lead > l_c && dist_to_left_lead >l_c) {
							//lane-change is feasible
							this_car->lane_change_feasible = 1;
						}
					}
					else if (current_lane == 2) {
						if (dist_to_lead > l_c && dist_to_right_lead > l_c) {
							//lane change is feasible
							this_car->lane_change_feasible = -1;

						}

					}

					LaneChange(this_car);

					//Find arrival time at each intersection 
					std::map <long, double> arrival_time_mzs;
					long lane = this_car->cur_lane + this_car->active_Ln_chng;
					arrival_time_mzs = FindArrivalTime(this_car,lane);
					this_car->arrival_time = arrival_time_mzs;

					//Solve the unconstrained optimal control problem 
					std::map <long, std::vector<double>> constants;
					constants = SolveUnconEnergyOptimalInterior(this_car);
					//log_debug << "SolveUnconEnergyOptimalInterior Done! for " << this_car->cav_id << std::endl;
					this_car->ctrl_constants = constants;

					//Check if rear-end safety becomes active 
					
					if (this_car->leading_vehicles.count(current_lane) != 0) {
						double t = this_car->init_time;
						double dt = 0.1;
						//there is a lead vehicle
						CAV* lead = this_car->leading_vehicles[current_lane];

						std::vector<double> pos_this = this_car->pos_conditions;
						std::vector<double>::iterator low_this; //an iterator to the first element which does not compare less than val
						long index_this;

						std::vector<double> pos_lead = lead->pos_conditions;
						std::vector<double>::iterator low_lead; //an iterator to the first element which does not compare less than val
						long index_lead;

						double pos_this_t = 0.01;//position of this car at time t;
						double pos_lead_t = 0.01;//position of this car at time t;

						std::map <long, std::vector<double>> ctrl_constants_this = this_car->ctrl_constants;
						std::map <long, std::vector<double>> ctrl_constants_lead = lead->ctrl_constants;

						double a_i, b_i, c_i, d_i;
						double a_k, b_k, c_k, d_k;

						
						while (t < this_car->time_conditions.back()) {
							t += dt;

							low_this = std::lower_bound(pos_this.begin(), pos_this.end(), pos_this_t);
							index_this = low_this - pos_this.begin();

							low_lead = std::lower_bound(pos_lead.begin(), pos_lead.end(), pos_lead_t);
							index_lead = low_lead - pos_lead.begin();
							if (ctrl_constants_lead.count(index_lead)==0) {
								//log_debug << "ERROR: CAV: " <<this_car->cav_id<<","<<index_this<<"<<<"<<lead->cav_id<<","<<index_lead<< std::endl;
								//log_debug << pos_this_t << "," << pos_this.front() << "," << std::endl;
								break;
								//CAV lead exited 
							}
							else {
								a_i = this_car->ctrl_constants[index_this].at(0);
								b_i = this_car->ctrl_constants[index_this].at(1);
								c_i = this_car->ctrl_constants[index_this].at(2);
								d_i = this_car->ctrl_constants[index_this].at(3);

								a_k = ctrl_constants_lead[index_lead].at(0);
								b_k = ctrl_constants_lead[index_lead].at(1);
								c_k = ctrl_constants_lead[index_lead].at(2);
								d_k = ctrl_constants_lead[index_lead].at(3);

								pos_this_t = a_i * pow(t, 3) / 6 + b_i * pow(t, 2) / 2 + c_i * t + d_i;
								pos_lead_t = a_k * pow(t, 3) / 6 + b_k * pow(t, 2) / 2 + c_k * t + d_k;

								//log_file << "i: "<< pos_this_t << ", K: " << pos_lead_t << std::endl;
								if (pos_lead_t-lead->cav_length - pos_this_t < safety_gap_dist) {
									//log_debug << "SIM:Pos Diff for CAV " << this_car->cav_id << " , " << lead->cav_id << " is " << pos_lead_t - lead->cav_length - pos_this_t <<"at time "<<t<< std::endl;
									
								}
							}
						}
					} 
		
				
					//DEBUG::: 


					log_file << "________________________________" << std::endl;
					log_file << "CAV: " << this_car->cav_id << std::endl;
					for (auto p : this_car->pos_conditions) {
						log_file <<p<< ", " ;
					}
					log_file << std::endl;
					for (auto t : this_car->time_conditions) {
						log_file << t << ", ";
					}
					log_file << std::endl;
					log_file << "________________________________" << std::endl;


					vehicles_in_cz[current_vehicle_id] = this_car;
					rel_cur_pos = dist_traveled - this_car->init_pos;

				}
				else {
					CAV* this_cav = vehicles_in_cz[current_vehicle_id];
					rel_cur_pos = dist_traveled - this_cav->init_pos;

					if (rel_cur_pos > this_cav->total_length + car_length){
						//CAV exits the control zone 
						vehicle_color = baseline ? (colors["white"] - color_space) : (colors["red"] - color_space);

						desired_velocity =  des_spd_urban;
						inside_cz = 0;



					}else{
					//the vehicle has not exited the cz yet

					//Lane change derive for a safety distance, then change lane 
						if (this_cav->active_Ln_chng != 0) {
							this_cav->FindlnChangeParam(rel_cur_pos, l_c, safety_gap_dist);
							active_lane_change = this_cav->active_Ln_chng;
							desired_lane_angle = this_cav->desired_lane_angle;//in radian
							rel_target_lane = this_cav->rel_target_lane;

							vehicle_color = (active_lane_change == 0) ? (colors["blue"] - color_space) : (colors["green"] - color_space);
						}


						//

						if (dist_to_lead < safety_gap_dist && lead_id != -1) {
							vehicle_color = baseline ? (colors["purple"] - color_space) : (colors["black"] - color_space);
							log_debug << "Pos Diff for CAV " << this_cav->cav_id << ", " << lead_id<< " is " <<dist_to_lead<< "at time " << current_time << std::endl;

						}
						std::vector<double> pos = this_cav->pos_conditions;
						std::vector<double>::iterator low; //an iterator to the first element which does not compare less than val
						low = std::lower_bound(pos.begin(), pos.end(),rel_cur_pos);
						long index = low - pos.begin();

						if (this_cav->ctrl_constants.count(index) == 0) {
							log_debug << "ERROR: CAV: " <<this_cav->cav_id<<" rel_pos: "<<rel_cur_pos <<"index: "<<index<< std::endl;
						}
						else {
							std::vector<double> consts = this_cav->ctrl_constants[index];
							if (consts.size() != 0) {
								desired_acceleration = consts.at(0)*current_time + consts.at(1);
							}
							else {
								log_debug << "ERROR IN DESIRED ACCELERATION" << std::endl;
							}
						}
					}
				}		
			}
        //LogData

        log_profile << current_time << "," <<inside_cz <<","<< path_name<<","<<current_vehicle_id << "," << rel_cur_pos <<","<< current_velocity << "," << current_acceleration << "\n";

      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/
// Function create a vector of segments 
std::vector<std::string> ReadStringStream(std::string line_string) {
	std::vector<std::string> string_vector;
	std::string value; 
	std::stringstream stream(line_string);

	while (getline(stream,value,',')) {
		string_vector.push_back(value);
	}

	return string_vector; 
}

std::map<long, double> FindArrivalTime(CAV* car,long lane){
	std::map<long, double> optimal_arrivals;
	//long lane = current_lane;
	CAV* leadCAV=NULL ;
	double t_bar;
	double t_rear; 
	if (car->leading_vehicles.count(lane) != 0) {
		//There is a leading vehicle in lane
		leadCAV= car->leading_vehicles[lane];
		if (lead_id == -1) {
			leadCAV = NULL;
		}
		else {
			log_file << "++++++++++++++++++++++" << leadCAV->cav_id << std::endl;
		}
	

	}

	for (int i = 0; i < car->total_mzs; i++) {
		std::vector<double> other_schedules;
		double t_optimal;
		
		if (i==0) {
			//mz is the first mz that it crosses
			t_bar = car->L / car->init_vel + car->init_time;
		}
		else {
			t_bar = optimal_arrivals[car->mzs.at(i - 1)] + (car->delta_t) + (car->D / car->init_vel);
		}

		t_rear = (leadCAV == NULL) ? (0) : (leadCAV->arrival_time[car->mzs.at(i)] + car->t_safety_rear);
		t_optimal = (t_bar > t_rear) ? (t_bar) : (t_rear);
		log_file << "++++++++++++++++++++++" << std::endl;
		log_file << "CAV: " << car->cav_id << "lead_id: " << lead_id << "t_rear: " << t_rear << std::endl;

		log_file << "CAV: " << car->cav_id <<"t_optimal: "<<t_optimal << "t_bar: " << t_bar << std::endl;

		std::map<double, CAV*> time_TO_CAV;
		for (auto other : car->lateral_vehicles[car->mzs.at(i)]) {
			double time = other->arrival_time[car->mzs.at(i)]; //arrival time of others at mz 
			other_schedules.push_back(time);
			time_TO_CAV[time] = other;
		}
		std::sort(other_schedules.begin(), other_schedules.end());

		for (auto t_other : other_schedules) {

			double crossing_time = time_TO_CAV[t_other]->delta_t;
			if (t_other + crossing_time < t_optimal) {
				continue; 
			}
			//log_debug << "t_optimal: " << t_optimal << "T_other: "<<t_other<< std::endl;
			if ((t_optimal + car->delta_t) <= t_other) {
				//it can exit before tOther
				//log_debug << "Optimal time that satisfies lateral safety is:  " << t_optimal << std::endl;
				break;
			}
			else {
				t_optimal = t_other + crossing_time;
			}
		}
		optimal_arrivals[car->mzs.at(i)] = t_optimal;
	}
	return optimal_arrivals;
}

std::map<long, std::vector<double>> SolveUnconEnergyOptimalInterior(CAV* car){
	std::map<long, std::vector<double>> ctrl_constants;
	std::vector<long> mzs = car->mzs;
	long num_mzs = car->total_mzs;
	long total_zones = 2 * num_mzs;
	long n = 4 * total_zones + (total_zones - 1); // 4 constant for each zone and 1 pi for each interior constraint(1 position);
	std::vector<double> time; 
	time.push_back(car->init_time);
	long count = 0;
	for (int i = 1; i <= 2*num_mzs; i++) {
		if (i % 2 != 0) {
			time.push_back(car->arrival_time[mzs.at(count)]);
			count++;
			continue;
		}
		else {
			time.push_back(time.back() + car->delta_t);
		}
	}
	car->SetTimeConditions(time);
	
	Eigen::MatrixXd A(n,n);
	Eigen::VectorXd Y(n);
	Eigen::VectorXd X(n);

	A = Eigen::MatrixXd::Zero(n, n);
	Y = Eigen::VectorXd::Zero(n);
	X = Eigen::VectorXd::Zero(n);
	
	//X = [constants for the first arc, constant for the second arc, ..., constants for the last arc, (Pis first and second arc), ..., (Pi last - 1 and last arc)]
	// p at each time instant(last time u is == 0)
	for (int i = 0; i < time.size(); i++) {
		if (i == time.size()-1) {
			A.block(i, 4 * i - 4, 1, 4) << pow(time.at(i), 3) / 6.0, pow(time.at(i), 2) / 2.0, time.at(i), 1;
			Y(i) = car->pos_conditions.at(i);
			break;
		} 
		A.block(i, 4 * i, 1, 4) << pow(time.at(i), 3) / 6.0, pow(time.at(i), 2) / 2.0, time.at(i), 1; // position at each initial of each arc time
		Y(i) = car->pos_conditions.at(i);
		
	}

	//Initial Speed 1 eq
	A.block(time.size(),0,1,4) << pow(time.front(),2)/2, time.front(),1, 0;
	Y(time.size()) = car->init_vel;
	//Final acceleration 1 eq
	A.block(time.size() + 1, 4 * time.size() - 8, 1, 4) << time.back(), 1, 0, 0; //acceleration zero at final time
	Y(time.size() + 1) = 0;
	//size(A)

	long cond = time.size() + 2;

	// 2 * (length(t) - 2) e.g.length = 5 -> 6 continuity
	for (int i = 0; i < time.size() - 2; i++) {
		A.block(cond + 2 * i, 4 * i,1,8) << pow(time.at(i + 1),3)/ 6, pow(time.at(i + 1),2) / 2, time.at(i + 1), 1, -pow(time.at(i + 1), 3) / 6, -pow(time.at(i + 1), 2)/ 2, -time.at(i + 1), - 1;
		Y(cond + 2 * i) = 0;
		A.block(cond + 2 * i + 1, 4 * i, 1, 8) << pow(time.at(i + 1), 2) / 2, time.at(i + 1), 1, 0, -pow(time.at(i + 1), 2) / 2, -time.at(i + 1), -1, 0;
		Y(cond + 2 * i + 1) = 0;
	}


	cond = cond + (time.size() - 2) * 2;

	for (int i = 0; i < time.size() - 2; i++) {

		// e.g. 2 * (length(t) - 2) e.g length = 5 -> 6 jump conditions
		//lambda p
		A(cond + 2 * i , 4 * i ) = 1;
		A(cond + 2 * i , 4 * i + 4) = -1;
		A(cond + 2 * i , (time.size() - 1) * 4 + i) = -1;
		Y(cond + 2 * i ) = 0;

		//lambda v
		A.block(cond + 2 * i + 1, 4*i,1,8) << -time.at(i+1), - 1, 0, 0, time.at(i + 1), 1, 0, 0;
		Y(cond + 2 * i+1) = 0;
	}



	//log_debug << A << std::endl;
	//log_debug << Y << std::endl;

	X = A.colPivHouseholderQr().solve(Y);
	//For x merging zone we have 2*x arc 
	for (int i = 0; i < total_zones ; i++) {
		std::vector<double> cntrls;


		cntrls.push_back(X(4*i));
		cntrls.push_back(X(4 * i + 1));
		cntrls.push_back(X(4 * i + 2));
		cntrls.push_back(X(4 * i + 3));
		

		ctrl_constants[i + 1] = cntrls;//key of the map is the arc number e.g. cntr_constants[1] is for the first arc
	}

	
	std::vector<double> lgrng_mltplrs;//lagrange multipliers \pi
	for (int i = 0; i < total_zones - 1; i++) {
		lgrng_mltplrs.push_back(X(4 * total_zones + i));
	}
	car->SetPi(lgrng_mltplrs);
	
	return ctrl_constants;
}

void LaneChange(CAV* car) {
	//test it for the west bound
	active_lane_change = 0;
	desired_lane_angle = 0;

	if (car->lane_change_feasible == 0) {
		//lane change is not feasible 
		active_lane_change = 0;
		desired_lane_angle = 0;
	}
	else{

		std::map<long, double> arrivaltime = FindArrivalTime(car, car->cur_lane);
		std::map<long, double> arrivaltimeNew = FindArrivalTime(car, car->cur_lane + car->lane_change_feasible);
		long last_mz = car->mzs.back();

		if (arrivaltime.count(last_mz) == 0 || arrivaltimeNew.count(last_mz) == 0) {
			log_debug << "ERROR IN LANE CHANGE" << std::endl;
		}
		else {
			double t1, t2; 
			t1 = arrivaltime[last_mz];
			t2 = arrivaltimeNew[last_mz];

			if (t2 < t1) {
				active_lane_change = car->lane_change_feasible;
				car->active_Ln_chng = active_lane_change;
				desired_lane_angle = 0;
				log_debug << car->cav_id << " : will change lane" << std::endl;
			}
			else {
				active_lane_change = 0;
				desired_lane_angle = 0;
			}
		}

	}


}