#ifndef _AP
#define _AP

#include <cmath>
#include "station.h"
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <exception> 
class AP{
public:
	//MCS index is 11
	//spatial stream is 1
	//modulation is 1024-QAM
	//coding is 5/6 
	//guard intervals is 0.8μs
	vector<int> traffic_arrival_rates;
	vector<vector<Station>> station_list;
	
	const int OFDMA_DURATION = 32768;
	const int Bandwidth_A_26 = 148;//320 MHz 148
	const int Bandwidth_B_26 = 74;//160 MHz
	string alloc_result = "";
	string fin_alloc_resultA = "";
	string fin_alloc_resultB = "";
	
	double ana_TH = 0.0;
	vector<int> ana_STPs; //Record tranmission time periods
	vector<int> ana_TPs;
	
	
	int sumT = 0;
	int sumT1 = 0;
	int sumT2 = 0;
	
	int last_T = 5000;
	int last2_T = 0;
	
	
	
	
	
	//int transTime = 0; 
	AP(int);
	void updateSTAs(int,int,bool,bool,int,double);
	//int knaspack_sra(int,int,int);
	int knaspack_sra(int,int,bool,int,int,int);
	int allocDR(bool,int,int,int);
	int find_avg_length(int);
	int find_avg_len4MLO0(int);
	int find_avg_len4MLO1(bool,int,int,int,int);
	double evalEBR(int,int,int);
	void sim_transmit2STAs(int,int,int,int);
	void transmit2STAs(int,int);
	void twoChUsersAlloc();
	void print_info();
	void sortSTAs(int);
	void cal_STAs_ana(int,int);
	//opt
	void opt_updateSTAs(int,int,int,int);
	int opt_RCL(int,bool,bool,bool);//rough classification
	int opt_FGC(int,bool,bool,bool);//fine-grained classification
	void opt_filter();
	//void opt_updateSTAs(int,int,int);
private:
	double EDR_A = 2.0/3;
	double EDR_B = 1.0/3;
	int sim_time = 0;
	const int MRUs[16] = {0,26,52,78,106,132,242,484,726,996,1480,1992,2476,2988,3472,3984};
	const double MRUs_dr[16] = {0.0,14.7,29.4,44.1,62.5,77.2,143.4,286.8,430.2,600.5, 887.3, 1201.0, 1487.8, 1801.5, 2088.3, 2402.0};//996 bit/mus mcs idx = 11
	//const double MRUs_dr[16] = {0.0,14.7,29.4,44.1,62.5,77.2,143.4,286.8,430.2,600.5, 1030.7, 1201.0, 1487.8, 1801.5, 2088.3, 2402.0};//996 bit/mus mcs idx = 11
	//const double MRUs_dr[16] = {0.0,13.2,26.5,39.7,56.3,69.5,129.0,258.1,387.1,540.4, 798.5, 1080.8, 1338.9, 1621.2, 1879.3, 2161.6};// mcs idx = 10
	//const double MRUs_dr[16] = {0.0,10.6,21.2,31.8,45.0,55.6,103.2,206.5,309.7,432.4, 638.9, 864.8, 1071.3, 1297.2, 1503.7, 1729.6};//mcs idx = 8
	//const double MRUs_dr[16] = {0.0,7.9,15.9,23.8,33.8,41.7,77.4,154.9,232.3,324.3, 479.2, 648.6, 803.5, 972.9, 1127.8, 1297.2};//mcs idx = 6
	//const double MRUs_dr[16] = {0.0,8.8,17.6,26.4,37.5,46.3,86.0,172.1,258.1,360.3, 532.4, 720.6,892.7, 1080.9, 1253.0, 1441.2};//mcs idx = 7
	const int idxToMRUs[149] = {0, 26, 52, 78, 106, 132, 158, 184, 210, 242, 268, 294, 320, 348, 374, 400, 426, 452, 484, 510, 536, 562, 590, 616, 642, 668, 694,726 , 752, 778, 804, 832, 858, 884, 910, 936, 962,996,
	1022, 1048, 1074, 1102, 1128, 1154, 1180, 1206, 1238, 1264, 1290, 1316, 1344, 1370, 1396, 1422, 1448, 1480, 1506, 1532, 1558, 1586, 1612, 1638, 1664, 1690, 1722, 1748, 1774, 1800, 1828, 1854, 1880, 1906, 1932, 1958, 
	1992, 2018, 2044, 2070, 2098, 2124, 2150, 2176, 2202, 2234, 2260, 2286, 2312, 2340, 2366, 2392, 2418, 2444, 2476, 2502, 2528, 2554, 2582, 2608, 2634, 2660, 2686, 2718, 2744, 2770, 2796, 2824, 2850, 2876, 2902, 2928, 2954, 2988, 
	3014, 3040, 3066, 3094, 3120, 3146, 3172, 3198, 3230, 3256, 3282, 3308, 3336, 3362, 3388, 3414, 3440, 3472, 3498, 3524, 3550, 3578, 3604, 3630, 3656, 3682, 3714, 3740, 3766, 3792, 3820, 3846, 3872, 3898, 3924, 3950, 3984};//高估j可以兌換的MRU，導致其可以切成很大的合法MRU，或是低估合法MRU的代價?
	map<int,int> MRUsToIdx; 
	
	void reOrderSTAs(int);
	double d_min(double,double);
	double getEDR4STA(double,double,double,double,double);
	
	const vector<int> MPDU_LENS = vector<int>{200*8,1000*8,500*8,500*8}; //Byte
	
	
	
	// opt function
	const vector<double> PRI_FACTOR = {2.0,1.5,1.0,0.5};
};

#endif
