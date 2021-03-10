#include "CAV.h"
CAV::CAV(long id, double length, long lnk, long lane, std::vector<std::string> links_seq, double time, double pos, double vel, double acc, long lead_id) {
	cav_id = id;
	cav_length = length;

	cur_lane = lane;
	cur_link = lnk;

	links_in_cz = links_seq;

	init_time = time;
	cur_time = time;

	init_pos = pos;
	cur_pos = pos;

	init_vel = vel; 
	cur_vel = vel; 

	init_acc = acc; 
	cur_acc = acc; 

	lead_veh_id = lead_id; 


	FindTotalLength();
	FindPositionAtMzs();
	delta_t = (S + cav_length) / init_vel;
	
}
void CAV::FindTotalLength() {

	std::vector <long> segs;

	for (auto l : links_in_cz) {
		if (std::stol(l) < 10000) {
			segs.push_back(std::stol(l));
		}
	}
	int size = segs.size();
	if (size == 2) {
		total_length = L + S;
		total_mzs = 1;

	}else if (size==6) {
		total_length = L + 3 * S + 2 * D;
		total_mzs = 3;
	}

}
void CAV::FindPositionAtMzs() {
	pos_conditions.push_back(0);
	double l_1 = 0;

	for (int i = 0; i < 2*total_mzs; i++) {\
		if (i == 0) {
			l_1 += L;
			pos_conditions.push_back(l_1);
			continue;
		}
		l_1 = (i % 2 == 0) ? (l_1 + D - cav_length) : (l_1 + S + cav_length);
		pos_conditions.push_back(l_1);
	}
	//pos_conditions.back() += cav_length;

}
void CAV::SetPi(std::vector<double> p_mltprs) {
	lagrange_mltplrs = p_mltprs;
}


