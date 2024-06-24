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
	vector<vector<int>> allocation_table;
	
	const int OFDMA_DURATION = 32768;
	const int Bandwidth_A_26 = 148;//320 MHz 148 
	const int Bandwidth_B_26 = 74;//160 MHz
	string alloc_result = "";
	string fin_alloc_resultA = "";
	string fin_alloc_resultB = "";
	
	double ana_TH = 0.0;
	vector<int> ana_STPs; //Record tranmission time periods
	vector<int> ana_TPs;
	
	const int priority_num = 4;
	int sumT = 0;
	int sumT1 = 0;
	int sumT2 = 0;
	
	int last_T = 5000;
	int last2_T = 0;
	
	
	int remainRU_B[11] = {72,32,16,16,8,8,4,8,2,4,1};	
    int remainRU_A[15] = {144,64,32,32,16,16,8,16,4,8,2,12,4,8,1};
	int allocatin_table_B[72] ={0};
	int allocatin_table_A[144] ={0};  
	
	
	//int transTime = 0; 
	AP(int);
	void updateSTAs(int,int,bool,bool,bool,int,double);
	//int knaspack_sra(int,int,int);
	int knaspack_sra(int,int,bool,int,int,int);
	int Tzu(int,int,bool,int,int,int);
	vector<int> &MRU_map_26(int,int);
	void renew_allocation_table(int,int,vector<int>);
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
//	int MRUtable_52_26[32][3] = {{2,3,4},{5,6,7},{10,11,12},{11,12,13},{20,21,22},{23,24,25},{28,29,30},{29,30,31},
//									{38,39,40},{41,42,43},{46,47,48},{47,48,49},{56,57,58},{59,60,61},{64,65,66},{65,66,67},
//									{74,75,76},{77,78,79},{82,83,84},{83,84,85},{92,93,94},{95,96,97},{100,101,102},{101,102,103},
//									{110,111,112},{113,114,115},{118,119,120},{119,120,121},{128,129,130},{131,132,133},{136,137,138},{137,138,139}};
	const int MRUtable_52_26[32][2] = {{1,4},{2,7},{5,10},{5,13},{9,22},{10,25},{13,28},{13,31},
								{17,40},{18,43},{21,46},{21,49},{25,58},{26,61},{29,64},{29,67},
								{33,76},{34,79},{37,82},{37,85},{41,94},{42,97},{45,100},{45,103},
								{49,112},{50,115},{53,118},{53,121},{57,130},{58,133},{61,136},{61,139}};
	const int MRUtable_106_26[16][2] = {{0,4},{3,13},{4,22},{7,31},{8,40},{11,49},{12,58},{15,67},{16,76},{19,85},{20,94},{23,103},{24,112},{27,121},{28,130},{31,139}};
	
    const int MRUtable_484_242[16][2] = {{1,1},{1,0},{0,3},{0,2},{3,5},{3,4},{2,7},{2,6},{5,9},{5,8},{4,11},{4,10},{7,13},{7,12},{6,15},{6,14}};
    const int MRUtable_996_484[8][2] = {{1,1},{1,0},{0,3},{0,2},{3,5},{3,4},{2,7},{2,6}};
    const int MRUtable_2_996_484[12][3] = {{1,2,1},{1,2,0},{0,2,3},{0,2,2},{0,1,5},{0,1,4},{2,3,3},{2,3,2},{1,3,5},{1,3,4},{1,2,7},{1,2,6}};
    const int MRUtable_3_996[4][3] = {{1,2,3},{0,2,3},{0,1,3},{0,1,2}}; 
    const int MRUtable_3_996_484[8][4] = {{1,2,3,1},{1,2,3,0},{0,2,3,3},{0,2,3,2},{0,1,3,5},{0,1,3,4},{0,1,2,7},{0,1,2,6}}; 
	
	void reOrderSTAs(int);
	double d_min(double,double);
	double getEDR4STA(double,double,double,double,double);
	
	const vector<int> MPDU_LENS = vector<int>{500*8,200*8,1000*8,500*8,500*8}; //Byte
//	struct allocation_table
//	{
//		int MRU_26_A[144] = {0}; 
//		int MRU_52_A[64] = {0};
//		int MRU_52_26_A[32] = {0};
//		int MRU_106_A[32] = {0};
//		int MRU_106_26_A[16] = {0};
//		int MRU_242_A[16] = {0};
//		int MRU_484_A[8] = {0};
//		int MRU_484_242_A[16] = {0};
//		int MRU_996_A[4] = {0};
//		int MRU_996_484_A[8] = {0};
//		int MRU_2_996_A[2] = {0};
//		int MRU_2_996_484_A[12] = {0};
//		int MRU_3_996_A[4] = {0};
//		int MRU_3_996_484_A[8] = {0};
//		int MRU_4_996_A[1] = {0};
//		
//		int MRU_26_B[72] = {0}; 
//		int MRU_52_B[32] = {0};
//		int MRU_52_26_B[16] = {0};
//		int MRU_106_B[16] = {0};
//		int MRU_106_26_B[8] = {0};
//		int MRU_242_B[8] = {0};
//		int MRU_484_B[4] = {0};
//		int MRU_484_242_B[8] = {0};
//		int MRU_996_B[2] = {0};
//		int MRU_996_484_B[4] = {0};
//		int MRU_2_996_B[1] = {0};
//		
//	};
//	vector<vector<int>> MRU_allocation_A = {{144,&allocation_table->MRU_26_A},{64,&allocation_table->MRU_52_A},{32,&allocation_table->MRU_52_26_A},{32,&allocation_table->MRU_106_A},{16,&allocation_table->MRU_106_26_A},{16,&allocation_table->MRU_242_A},
//	{8,&MRU_484_A},{16,&allocation_table->MRU_484_242_A},{4,&allocation_table->MRU_996_A},{8,&allocation_table->MRU_996_484_A},{2,&allocation_table->MRU_2_996_A},{12,&allocation_table->MRU_2_996_484_A},{4,&allocation_table->MRU_3_996_A},{8,&allocation_table->MRU_3_996_484_A},{1,&allocation_table->MRU_4_996_A}};
	
	// opt function
	const vector<double> PRI_FACTOR = {2.5,2.0,1.5,1.0,0.5};
};

#endif
