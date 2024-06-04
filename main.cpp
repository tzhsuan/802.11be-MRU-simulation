#include <iostream>
#include <fstream>
#include <vector>
#include "ap.h"
#include "scheduler.h"
#include <exception> 
using namespace std;
int D = 5;//實驗時間 單位 秒 
int MILLION = 1000000;
int T = 1;//每個實驗做幾次 
const vector<double> SL_STR_NSTR_RATIO = {0.2,0.4,0.4}; //各裝置比例 
const vector<int> TRAFFIC_ARRIVAL_RATES = vector<int>{20,160,300,100};// 各優先級單個用戶的封包流量 
const vector<int> MPDU_LENS = vector<int>{200,1000,500,500}; // 各優先級封包長度單位 
const vector<int> DEADLINES = vector<int>{150000,200000,1000000,2000000}; // mus
const vector<string> Method_NAME = {"Joe_1CH_","Opt_1CH_","Joe_2CH_","Opt_2CH_"};
/* run this program using the console pauser or add your own getch, system("pause") or input loop */

void writeData(int Method, string _fn0,string _fn,vector<vector<double>> Datas)
{
	string filen = _fn0 + Method_NAME[Method];
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





int main(int argc, char** argv) {
	int Method = 0;//方法選擇  0:MRU SRAs, 1:OPT, 2:TMS, 3:EOASYNC + OPT 
	double alpha = 2.75;
	double sim_time = MILLION * D; //模擬時間 單位mu s (百萬分之一) 
	
	vector<vector<double>> STAs_TH(4);//所有裝置的吞吐量紀錄 
	vector<vector<double>> STAs_D(4);//所有裝置的延遲紀錄
	vector<double> P_TH(4);
	vector<double> P_D(4);
	
	//vector<vector<double>> alpha_TH(20,vector<double>(4,0.0));
	//vector<vector<double>> alpha_D(20,vector<double>(4,0.0));
	
	vector<int> VAR_PRI_PEOPLE = vector<int>{1,1,1,1};//起始各優先級用戶數量 
	//vector<int> VAR_TRAFFIC_ARRIVAL_RATES = vector<int>{10,80,150,50};
	
	int initPN = 4;//起始用戶數量 
	int finPN = 60;//最終用戶數量 
	int initAlpha = alpha;
	int finAlpha = alpha + 4;
	int round4VarP = (finPN - initPN)/4 + 1;//finPN - initPN)/20 + 1 <- 用於雙頻道 
	int round4VarAlpha = (finAlpha - initAlpha)*4 + 1;//用於變動alpha 
	vector<vector<double>> VAR_STA_TH(round4VarP,vector<double>(4,0.0));//20~100共9個 吞吐量 
	vector<vector<double>> VAR_STA_MTH(round4VarP,vector<double>(4,0.0));//Math  
	vector<vector<double>> VAR_STA_MD(round4VarP,vector<double>(4,0.0));//Math 
	vector<vector<double>> VAR_STA_D(round4VarP,vector<double>(4,0.0));//延遲 
	
	vector<vector<double>> VAR_STA_packet_lr(round4VarP,vector<double>(4,0.0));//封包遺失率 
	vector<vector<double>> VAR_STA_chs_util(round4VarP,vector<double>(2,0.0));//頻道利用率 
	
	
	vector<vector<double>> DEV_TH(4,vector<double>(3,0.0)); // 4個pri, 3個裝置類型 
	vector<vector<double>> DEV_D(4,vector<double>(3,0.0)); // 4個pri, 3個裝置類型
	
	
		
	
	double total_avg_th = 0.0;
	for(int num = 0; num < round4VarP; num++)//實驗進行次數 
	{
		for(int t = 0; t < T; t++)//每個實驗重複T次取平均 
		{
			cout << "alpha = " << alpha << endl;
			Scheduler scheduler = Scheduler(sim_time);
			scheduler.generate_traffic(SL_STR_NSTR_RATIO,VAR_PRI_PEOPLE, TRAFFIC_ARRIVAL_RATES, MPDU_LENS, DEADLINES);//生成封包 
			//開始排程 
			if(Method < 2) scheduler.schedule_access(Method,alpha);
			else scheduler.schedule_access2CH(Method,alpha);
			//排程結束		
			AP* ap = scheduler.ap;		
			
			if(t == 0)// init
			{
				for(int p = 0; p < VAR_PRI_PEOPLE.size(); p++)
				{
					STAs_TH[p] = vector<double>(ap->station_list[p].size(),0.0);
					STAs_D[p] = vector<double>(ap->station_list[p].size(),0.0);
				}
			} 
			double total_th = 0.0;
			//疊代所有用戶，計算排程結束後的資訊 如:吞吐量、延遲等 
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

					
					
					cout << "STA ID:"<< STA->STA_ID <<", 封包總數 = " << STA->packets.size() << ", success transmission = " << STA->success_trans<<", 吞吐量 = "<< STA_TH  <<", 延遲 = "<< STA_dealy  << endl;
					cout << "封包遺失率 = " << (double)STA->n_expired_packet / STA->packets.size() << ", 在A頻道傳輸數量 = " << STA->n_suc_packet_chA <<", 在B頻道傳輸數量 = "<< STA->n_suc_packet_chB  << endl;
					cout << "Device type = " << STA->device << endl;
					if(Method == 0) cout << "數學分析, 吞吐量 = " << STA->ana_TH << ", 平均延遲 = " << STA_ana_D  <<", success transmission = " <<STA->ana_SN<<endl;
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
			VAR_PRI_PEOPLE[p]+=1;//若使用單頻道 ，若使用雙頻道 則+5 
		}
		//alpha+=0.25; 
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
	//所有資料都會寫檔在這裡 
	if(Method < 2){
		string file = "實驗結果1ch/";

		writeData(Method,file,"VAR_STA_TH", VAR_STA_TH);
		writeData(Method,file,"VAR_STA_D", VAR_STA_D);
		writeData(Method,file,"VAR_STA_packet_lr",VAR_STA_packet_lr);
		writeData(Method,file,"VAR_STA_chs_util",VAR_STA_chs_util);
	}
	else{
		string file = "實驗結果2ch/";
		writeData(Method,file,"VAR_STA_TH", VAR_STA_TH);
		writeData(Method,file,"VAR_STA_D", VAR_STA_D);
		writeData(Method,file,"DEV_AVG_TH", DEV_TH);
		writeData(Method,file,"DEV_AVG_D", DEV_D);
		writeData(Method,file,"VAR_STA_packet_lr",VAR_STA_packet_lr);
		writeData(Method,file,"VAR_STA_chs_util",VAR_STA_chs_util);
	}
	

	return 0;
}
