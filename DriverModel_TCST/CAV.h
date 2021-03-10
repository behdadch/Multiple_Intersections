#ifndef __CAVCLASS
#define __CAVCLASS

#include <string> 
#include <vector>
#include <map>


class CAV {
public:

	long cav_id, cur_lane, cur_link; 
	double cav_length;
	double init_time, init_pos, init_vel, init_acc; 
	double cur_time, cur_pos, cur_vel, cur_acc; 
	double total_length; 
	double D = 75.0; //distance between two intersection
	double L = 150.0; // from enty of the cz to the first mz
	double S = 15.0; // The merging zone length;
	double w = 3.5; //lane_width
	double delta_t = 2; //[s] time to spend inside intersection
	double t_safety_rear = 2; 
	std::vector<std::string> links_in_cz;
	std::vector <long> mzs;
	long total_mzs; 
	std::vector <std::string> conflict_paths;
	std::string path_name; 
	std::map <long, std::vector<CAV*>> lateral_vehicles;
	std::map <long, CAV*> leading_vehicles;
	std::map <long, double> arrival_time;
	std::vector<double> pos_conditions; //this is the position at initial, interior, and exit of czs 
	std::vector<double> time_conditions; //this is the position at initial, interior, and exit of czs 

	std::vector<double> lagrange_mltplrs;
	std::map <long, std::vector<double>> ctrl_constants;

	long active_Ln_chng = 0;
	double desired_lane_angle=0;//in radian
	long rel_target_lane = 0;
	long lane_change_feasible = 0;//+1 to left -1 torigth 0 not feasible


	//Lead_vehicle information 

	long lead_veh_id; 


	CAV(long id, double length, long lnk, long lane, std::vector<std::string> links_seq, double time, double pos, double vel, double acc, long lead_id);
	~CAV() {
	//Remove from the control zone 
	};

	void FindTotalLength();
	void FindPositionAtMzs();
	void SetPi(std::vector<double> p_mltprs);
	void SetTimeConditions(std::vector<double> times) {
		time_conditions = times;
	};
	void FindlnChangeParam(double pos, double laneChangeZone_length, double safety_dist) {
		
		if (pos < laneChangeZone_length) {

			if (pos < safety_dist) {
				active_Ln_chng = lane_change_feasible;
				desired_lane_angle = 0;//in radian
				rel_target_lane = lane_change_feasible;
			}
			else {

				active_Ln_chng = lane_change_feasible;
				double y = lane_change_feasible * w;
				double x = laneChangeZone_length - safety_dist;
				desired_lane_angle = atan2(y,x);//in radian
				rel_target_lane = lane_change_feasible;
			}

		}
		else {
			//cav is outside the lane change zone

			 active_Ln_chng = 0;
			 desired_lane_angle=0;//in radian
			 rel_target_lane = 0;
		}


	}

private:

};

#endif
