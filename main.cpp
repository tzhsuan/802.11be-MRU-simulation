#include <iostream>
#include <fstream>
#include <vector>
#include "ap.h"
#include "scheduler.h"
#include <exception> 
using namespace std;
int D = 5;
int MILLION = 1000000;
int T = 1; //原測資為5 
const vector<double> SL_STR_NSTR_RATIO = {0.2,0.4,0.4}; 
const vector<int> PRI_PEOPLE = vector<int>{5,5,5,5};// 4*7的情況下 頻寬3600 ARR = 4060 會得到  3220  //更動為5個 
const vector<int> TRAFFIC_ARRIVAL_RATES = vector<int>{20,160,300,100};// Mbit 20 160 300 100 => 2250  //更動為五個 
const vector<int> MPDU_LENS = vector<int>{200,1000,500,500}; //Byte  //更動為五個  
const vector<int> DEADLINES = vector<int>{150000,200000,1000000,2000000}; // mus => 46.72w mus  //更動為五個  
const vector<string> Method_NAME = {"Joe_1CH_","Opt_1CH_","Joe_2CH_","Opt_2CH_"};   
/* run this program using the console pauser or add your own getch, system("pause") or input loop */

void writeData(int Method,string _fn,vector<vector<double>> Datas)
{
	string filen = "實驗結果1ch/" + Method_NAME[Method];
	string fn = filen + _fn + ".csv";
	ofstream outfile(fn);	
	for(int i = 0; i < Datas.size(); i++)
	{
		for(int j = 0; j < Datas[i].size(); j++)
		{
			outfile << Datas[i][j] << ",";		
		}
		outfile<<endl;		
	}
	outfile.close();
}

vector<double> get_anaErrRate(vector<vector<double>> THs,vector<vector<double>> MTHs) //分析數學分析與模擬誤差 
{
	double avg_Err = 0.0;
	vector<double> avg_Errs(4,0.0);
	int n = THs.size();
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			avg_Errs[j]+=abs(THs[i][j]-MTHs[i][j]) / THs[i][j] / n;
		}
	}
	
	return avg_Errs;
}



int main(int argc, char** argv) {
	int Method = 1;  //自己調整方法 
	double alpha = 2.75;  //對手自身參數設置 
	double sim_time = MILLION * D;
	
	vector<vector<double>> STAs_TH(4);
	vector<vector<double>> STAs_D(4);
	vector<double> P_TH(4);
	vector<double> P_D(4);
	
	//vector<vector<double>> alpha_TH(20,vector<double>(4,0.0));
	//vector<vector<double>> alpha_D(20,vector<double>(4,0.0));
	
	vector<int> VAR_PRI_PEOPLE = vector<int>{5,5,5,5};
	//vector<int> VAR_TRAFFIC_ARRIVAL_RATES = vector<int>{10,80,150,50};
	
	int initPN = 4;
	int finPN = 60;
	int initAlpha = alpha;
	int finAlpha = alpha + 4;
	//	int round4VarP = (finPN - initPN)/4 + 1;//finPN - initPN)/20 + 1
	//	int round4VarAlpha = (finAlpha - initAlpha)*4 + 1;
	int round4VarP = 1; 
	
	vector<vector<double>> VAR_STA_TH(round4VarP,vector<double>(4,0.0));//20~100共9個
	vector<vector<double>> VAR_STA_MTH(round4VarP,vector<double>(4,0.0));//Math  
	vector<vector<double>> VAR_STA_MD(round4VarP,vector<double>(4,0.0));//Math 
	vector<vector<double>> VAR_STA_D(round4VarP,vector<double>(4,0.0));
	
	vector<vector<double>> VAR_STA_packet_lr(round4VarP,vector<double>(4,0.0));
	vector<vector<double>> VAR_STA_chs_util(round4VarP,vector<double>(2,0.0));
	
	
	vector<vector<double>> DEV_TH(4,vector<double>(3,0.0)); // 4個pri, 3個裝置類型 
	vector<vector<double>> DEV_D(4,vector<double>(3,0.0)); // 4個pri, 3個裝置類型
	
	
		
	
	double total_avg_th = 0.0;
	for(int num = 0; num < round4VarP; num++)
	{
		for(int t = 0; t < T; t++)
		{
			cout << "alpha = " << alpha << endl;
			Scheduler scheduler = Scheduler(sim_time); //進入ap與各種方法比較 
			scheduler.generate_traffic(SL_STR_NSTR_RATIO,VAR_PRI_PEOPLE, TRAFFIC_ARRIVAL_RATES, MPDU_LENS, DEADLINES);
			
			if(Method < 2) scheduler.schedule_access(Method,alpha);
			else scheduler.schedule_access2CH(Method,alpha);		
			AP* ap = scheduler.ap;		
			
			if(t == 0)// init
			{
				for(int p = 0; p < PRI_PEOPLE.size(); p++)
				{
					STAs_TH[p] = vector<double>(ap->station_list[p].size(),0.0);
					STAs_D[p] = vector<double>(ap->station_list[p].size(),0.0);
				}
			} 
			double total_th = 0.0;
			for(int p = 0; p < 4; p++)
			{
				double p_pl = 0.0; // packet loss rate
				double p_dealy = 0.0;
				double p_math_delay = 0.0;
				double p_throughput = 0.0;
				double p_Mthroughput = 0.0;
				//sort(ap->station_list[p].begin(),ap->station_list[p].end(),Station::compareBySTAID);
				
				int c_SL = 0, c_NSTR = 0, c_STR = 0;
				
				for(int i = 0; i < ap->station_list[p].size(); i++)
				{
					Station* STA = &ap->station_list[p][i];
					double STA_dealy = STA->total_dealy_time / (STA->success_trans * 1000); //從mus to ms 
					double STA_TH = double(STA->success_trans)*8*MPDU_LENS[p] / sim_time;
					double STA_ana_D = STA->ana_SD / STA->ana_SN;
					p_pl+= (double)STA->n_expired_packet / STA->packets.size() ; 
					
					
					STAs_TH[p][i]+=STA_TH/T;
					STAs_D[p][i]+=STA_dealy/T;
					
					p_dealy+= STA_dealy;
					p_math_delay+=STA_ana_D;
					if(Method < 2){
						VAR_STA_chs_util[num][0]+= STA_TH / 2402 / T;
					}
					else{
						VAR_STA_chs_util[num][0]+= STA->n_suc_packet_chA * 8 * MPDU_LENS[p] / (sim_time * 2402 * T);
						VAR_STA_chs_util[num][1]+= STA->n_suc_packet_chB * 8 * MPDU_LENS[p] / (sim_time * 1201 * T);	
					}

					
					cout << "Method= " << Method << endl;
					std::cout <<"minMCS = "<< STA->minMCS_B << endl;
					cout <<"優先級別 = "<< 4-p << endl;
					cout << "STA ID:"<< STA->STA_ID <<", 封包總數 = " << STA->packets.size() << ", success transmission = " << STA->success_trans<<", 吞吐量 = "<< STA_TH  <<", 延遲 = "<< STA_dealy << endl;
					cout << "封包遺失率 = " << (double)STA->n_expired_packet / STA->packets.size() << ", 在A頻道傳輸數量 = " << STA->n_suc_packet_chA <<", 在B頻道傳輸數量 = "<< STA->n_suc_packet_chB  << endl;
					cout << "Device type = " << STA->device << endl;
					if(Method == 0) //cout << "數學分析, 吞吐量 = " << STA->ana_TH << ", 平均延遲 = " << STA_ana_D  <<", success transmission = " <<STA->ana_SN<<endl;
					if(Method > 1)
					{
						int dt = -1;
						if(STA->device == "SL"){
							c_SL+=1;
							dt = 0;
						}
						else if (STA->device == "NSTR") {
							c_NSTR+=1;
							dt =1;
						}
						else if (STA->device == "STR"){
							c_STR+=1;
							dt = 2;	
						} 
						
						DEV_TH[p][dt]+= STA_TH/T;
						DEV_D[p][dt]+= STA_dealy/T;
					}

					p_throughput+=STA_TH;
					p_Mthroughput+=STA->ana_TH;
				}				
				p_pl/=ap->station_list[p].size();
				p_dealy/=ap->station_list[p].size();
				p_math_delay/=ap->station_list[p].size();
				
				VAR_STA_packet_lr[num][p]+=p_pl / T;			
				VAR_STA_TH[num][p]+=p_throughput / T;
				VAR_STA_MTH[num][p]+=p_Mthroughput / T;
				VAR_STA_D[num][p]+=p_dealy / T;
				VAR_STA_MD[num][p]+=p_math_delay / T;
							
				cout <<"優先級別 = "<< 4-p <<", 總吞吐量/Mbits = "<<p_throughput << endl;
				cout <<"優先級別 = "<< 4-p << ", 平均用戶延遲/ms = " << p_dealy << endl;
				total_avg_th+=p_throughput/T;
				total_th+=p_throughput;
			}
			
			cout << "當前回合 總吞吐量 = " <<  total_th << endl;  
		}

		cout <<  "平均320 MHz頻道利用率 = " <<  VAR_STA_chs_util[num][0] << endl;
		cout <<  "平均160 MHz頻道利用率 = " <<  VAR_STA_chs_util[num][1] << endl;
		
		
		 
		 
		for(int p = 0; p < 4; p++)
		{
			VAR_PRI_PEOPLE[p]+=1;
		}
		//alpha+=0.25;
		//VAR_PRI_PEOPLE[num%4]+=5;
	}
	
	if(Method > 1){
		for(int p = 0; p < 4; p++)
		{
			int SL = SL_STR_NSTR_RATIO[0] * VAR_PRI_PEOPLE[p];
			int STR = SL_STR_NSTR_RATIO[1] * VAR_PRI_PEOPLE[p];
			int NSTR = SL_STR_NSTR_RATIO[2] * VAR_PRI_PEOPLE[p];
			
			if(SL > 0){
				DEV_TH[p][0]/=SL;
				DEV_D[p][0]/=SL;
			}
			if(NSTR > 0){
				DEV_TH[p][1]/=NSTR;
				DEV_D[p][1]/=NSTR;
			}
			if(STR > 0){
				DEV_TH[p][2]/=STR;
				DEV_D[p][2]/=STR;
			}					
		}
	}
	/*
	if(Method == 0)
	{
		//數學分析 誤差計算 
		vector<double> avg_Errs = get_anaErrRate(VAR_STA_D,VAR_STA_MD);
		
		for(int p = 0; p < 4; p++)
		{
			cout <<"優先級別 = "<< 4-p <<", 平均誤差 = "<<avg_Errs[p]*100 << "%" << endl;
		}
	}*/
	//writeData(Method,"VAR_STA_TH",VAR_STA_packet_lr);
	//writeData(Method,"VAR_STA_TH",VAR_STA_packet_lr);

	//writeData(Method,"pad_VAR_STA_pl",VAR_STA_packet_lr);
	//writeData(Method,"VAR_STA_cu",VAR_STA_chs_util);
	

	return 0;
}
