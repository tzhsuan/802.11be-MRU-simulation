#include "packet.h"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;
class Station{
public:
	vector<Packet>packets;
	
	int cur_suc_packet = 0;
	int cur_data_size = 0;
	int cur_expired_size = 0;
	int n_expired_packet = 0;
	double n_suc_packet_chA = 0.0;
	double n_suc_packet_chB = 0.0;
	
	vector<vector<double>> requiredDRs = {{0.0,0.0},{0.0,0.0}};//(2,vector<double> (2,0.0))
	vector<vector<double>> allocDRs = {{0.0,0.0},{0.0,0.0}};
	
	vector<int> success_trans_nth = {0,0};
	vector<double> delays = {0.0,0.0}; //目前考慮1d 
	
	//根據2個MLO 
	vector<int> cur_data_sizes = {0,0};
	vector<int> startIdxs = {0,0};
	
	vector<int> last_packet_sizes = {0,0};
	
	vector<int> MCS(144,0); //MCS index
	
	string device;//STR NSTR SL
	double required_dr_A = 0.0;
	double required_dr_B = 0.0;
	//double alpha = 1.3;
	int STA_ID;
	int startIdx = 0;
	int priority;//0~3
	double required_dr = 0.0;
	double data_rate = 0.0;
	int success_trans = 0;//成功傳輸 
	int sim_time = 0;
	int packet_size = 0;
	
	double total_dealy_time = 0;
	
	double ana_TH = 0.0;
	double ana_remainData = 0.0;
	
	double ana_D = 0.0;
	
	double ana_avgDR = 0.0;
	
	
	int last_expired_size = 0;
	int last_expired2_size = 0;
	int last_expired3_size = 0;
	
	vector<double> ana_RFs; //Record remain packet flow in every tranmission
	
	int ana_RF_idx = 0;
	double ana_SN = 0.0;//successful trans count
	double ana_SD = 0.0;//(sum)a unit means one ms.

	
	
	int last_begin_trans_ChA = -1;
	int last_end_trans_ChA = -1;
	int last_begin_trans_ChB = -1;
	int last_end_trans_ChB = -1;
	
	
	Station(string,int,int,int,int);
	Station();
	void updateRD(int,int,bool,bool,int,double);
	void updateExpired(int);
	void transmit(int);//int duration
	
	static bool compareBySTAID(const Station&, const Station&);
	
	
	//opt
	int MRU_idx = 0;
	//double trans_rate = 0.0;
	double cur_sum_DR = 0.0;
	//void updateTR();
	void updateRMRU(int,int,double,int);
 	static bool compareByRD(const Station&, const Station&);
	
private:
	vector<int> upperBoundMRU = {1,2,3,4,5,9,18,27,36,54,72,90,108,126,144};//直到90 
	

};
