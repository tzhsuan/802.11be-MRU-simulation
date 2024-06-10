#include "scheduler.h"

//用來當作排程的接口，可以根據ap選擇不同排程算法，跑sim time 
using namespace std;

Scheduler::Scheduler(int sim_time)
{
	this->sim_time = sim_time;
	ap = new AP(sim_time);
}
Scheduler::~Scheduler()
{
	delete ap;
}

void Scheduler::generate_traffic(vector<double> SL_STR_NSTR_RATIO,vector<int> pri_people_count, vector<int> traffic_arrival_rates, vector<int> packet_sizes, vector<int> DEADLINES)
{
	
	int STA_ID = 0;
	ap->traffic_arrival_rates = traffic_arrival_rates;
	for(int i = 0; i < pri_people_count.size(); i++) //不同priority 
	{
		double lamda = double(traffic_arrival_rates[i]) / (packet_sizes[i] * 8);
		ap->station_list.push_back(vector<Station>(pri_people_count[i]));
		int SL = SL_STR_NSTR_RATIO[0] * pri_people_count[i];
		int STR = SL + SL_STR_NSTR_RATIO[1] * pri_people_count[i];
		int NSTR = STR + SL_STR_NSTR_RATIO[2] * pri_people_count[i];
		
		for(int j = 0; j < pri_people_count[i]; j++) //依據不同priority比例決定裝置個數 
		{
			string device_type = ""; 
			if(j < SL) device_type = "SL";
			else if(j < STR) device_type = "STR";
			else device_type = "NSTR";
			Station station = Station(device_type, STA_ID,i,sim_time,packet_sizes[i]*8);
			//Station station;
			unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();  //gets the current system time as an unsigned integer
		    std::mt19937 gen(seed1); //random number generator class
		    //每個mus 要來多少封包
		    // 20Mb / (200 * 8) = 12500 個封包 / s. => 0.0125個封包 / mus 
		    std::exponential_distribution<> distrib(lamda); //generate random numbers follow the exponential distribution with the lamda value=> represent waiting times between events
		    double last_arrival = 0;
		    double inter_arrival;
		    int packetCount = 0;
		    while (last_arrival < sim_time)  //計算單次sim週期會累積多少封包總數 
		    {
		    	//cout<< packetCount<<endl;
		        inter_arrival = distrib(gen);  //distrib recall exponential_distribution with lambda and gen recall mt19937 gen(seed1)
		        //cout<<"封包抵達時間 : "<< last_arrival << endl; 
		        station.packets.push_back(Packet(packet_sizes[i]*8, last_arrival,last_arrival+DEADLINES[i]));//packet 的基本單位轉為bit 
		        
		        last_arrival += inter_arrival;
		        packetCount+=1;
		    }
		    cout <<"優先級:"<< i  << ", STA ID:"<< STA_ID <<", 封包總數: " << packetCount << endl;
		    ap->station_list[i][j] = station;
		    STA_ID+=1;
		}
		
	}

	

}



void Scheduler::schedule_access(int method, double alpha)
{
	int curTime4A = 0;
	int c = 0;
	
	while(curTime4A < sim_time)
	{
		int transTime = 0;
		int BandwidthA = ap->Bandwidth_A_26;
		
		if(method == 0)
		{
								
			ap->updateSTAs(curTime4A,-1,true,false,BandwidthA,alpha);
			for(int p = 0; BandwidthA > 0 && p < 4; p++)
			{
				BandwidthA = ap->knaspack_sra(curTime4A,BandwidthA,false,-1,p,-1);
				//cout <<"優先級別 "<< 4-p<<"使用後剩餘頻寬 = " <<  BandwidthA << endl;
			}
			//ap->print_info();
			//cout << "剩餘頻寬 = " <<  BandwidthA << endl;

		}
		else if(method == 1)
		{
			ap->updateSTAs(curTime4A,-1,false,false,BandwidthA,1.0);
			ap->sortSTAs(1);
			BandwidthA = ap->opt_RCL(BandwidthA,true,false,false);	// 
			BandwidthA = ap->opt_FGC(BandwidthA,true,false,false);
			cout << "剩餘頻寬 = " <<  BandwidthA << endl; 
			//ap->print_info();
		}
		transTime = ap->find_avg_length(curTime4A);//
		ap->transmit2STAs(curTime4A,transTime);//要改32768 

		

		int duration = SIFS + transTime + SIFS + ACK;
		curTime4A+=duration;
		if(method == 0)
		{
			ap->ana_STPs.push_back(curTime4A);
			ap->ana_TPs.push_back(duration);
			//if(ap->ana_TPs.size() > 1) ap->ana_TPs[ap->ana_TPs.size()-1]+=ap->ana_TPs[ap->ana_TPs.size()-2];
			ap->cal_STAs_ana(duration,sim_time);
		}
		
		 //延遲傳輸的話 傳輸時間可以拉高很多 
		cout << "當前時間 = " <<  curTime4A << endl; 
		c+=1;
	}
	ap->sortSTAs(0);
	cout << "跑圈" << c<< endl; 
}
void Scheduler::schedule_access2CH(int method,double alpha)
{
	int curTime4A = 0;
	int curTime4B = 0;
	int c = 0;
	bool flagA = curTime4A < sim_time;
	bool flagB = curTime4B < sim_time;
	while(flagA || flagB)
	{
		bool alignRule1 = (curTime4A >= curTime4B) && (curTime4B + PIFS >= curTime4A);
		bool alignRule2 = (curTime4B >= curTime4A) && (curTime4A + PIFS >= curTime4B);
		
		//int tmpCurTime4A0 = curTime4A, tmpCurTime4B0 = curTime4B;
		//int tmpCurTime4A1 = curTime4A, tmpCurTime4B1 = curTime4B;
		
		int BandwidthA0 = ap->Bandwidth_A_26, BandwidthB0 = ap->Bandwidth_B_26;
		int BandwidthA1 = ap->Bandwidth_A_26, BandwidthB1 = ap->Bandwidth_B_26;
		if(alignRule1 || alignRule2)
		{
			if(alignRule1) curTime4B = curTime4A;
			else curTime4A = curTime4B; 	
			int BandwidthA = ap->Bandwidth_A_26, BandwidthB = ap->Bandwidth_B_26;
			int mlo0_trans_time = 0;
			//if my func
			if(method == 2)
			{
				ap->updateSTAs(curTime4A,-1,true,true,BandwidthA+BandwidthB,alpha);
				ap->twoChUsersAlloc();
				
				for(int p = 0; BandwidthA > 0 && p < 4; p++)
				{
					BandwidthA = ap->knaspack_sra(curTime4A,BandwidthA,true,0,p,0);
				}
				for(int p = 0; BandwidthB > 0 && p < 4; p++)
				{
					BandwidthB = ap->knaspack_sra(curTime4B,BandwidthB,true,0,p,1);
				}
				
				BandwidthA = ap->Bandwidth_A_26, BandwidthB = ap->Bandwidth_B_26;
				for(int p = 0; BandwidthA > 0 && p < 4; p++)
				{
					BandwidthA = ap->knaspack_sra(curTime4A,BandwidthA,true,1,p,0);
				}
				for(int p = 0; BandwidthB > 0 && p < 4; p++)
				{
					BandwidthB = ap->knaspack_sra(curTime4B,BandwidthB,true,1,p,1);
				}
				mlo0_trans_time = ap->find_avg_len4MLO0(curTime4A);	
			}
			else if(method == 3)
			{
				ap->updateSTAs(curTime4A,-1,false,true,BandwidthA,1.0);
				ap->sortSTAs(1);
				BandwidthA = ap->opt_RCL(BandwidthA,true,true,true);
				BandwidthA = ap->opt_FGC(BandwidthA,true,true,true);
				ap->opt_filter();
				BandwidthB = ap->opt_RCL(BandwidthB,false,true,true);
				BandwidthB = ap->opt_FGC(BandwidthB,false,true,true);
			}

			
			
			//end
			//else OPT 使用update RD後，兩個頻道各自配MRU策略 要思考 
			int mlo1_trans_timeA = ap->find_avg_len4MLO1(true,curTime4A,-1,-1,0);
			int mlo1_trans_timeB = ap->find_avg_len4MLO1(true,curTime4B,-1,-1,1);
			
			if(method == 2) ap->sim_transmit2STAs(curTime4A,mlo0_trans_time,mlo0_trans_time,0);
			ap->sim_transmit2STAs(curTime4B,mlo1_trans_timeA,mlo1_trans_timeB,1);
			
			double ebr_mlo0 = method == 2?ap->evalEBR(0,mlo0_trans_time,mlo0_trans_time):-1;
			double ebr_mlo1 = ap->evalEBR(1,mlo1_trans_timeA,mlo1_trans_timeB);
			cout << "EBR MLO 0 = " << ebr_mlo0 <<", EBR MLO 1 = "<<ebr_mlo1<<endl;
			int durationA = SIFS + 0 + SIFS + ACK;
			int durationB = durationA;
			bool sync_mode = false;
			if(ebr_mlo0 >= ebr_mlo1)
			{
				cout << "使用 EOSYNC Model" << endl;
				sync_mode = true;
				durationA+=mlo0_trans_time;
				durationB+=mlo0_trans_time;
			}
			else
			{
				cout << "使用 EOASYNC Model"<<endl;
				durationA+=mlo1_trans_timeA;
				durationB+=mlo1_trans_timeB;
			}
			int tA =  durationA - 2 * SIFS - ACK;
			int tB = durationB - 2 * SIFS - ACK;
			// 指派success trans, start idx, delay
			for(int p = 0; p < 4; p++)
			{
				for(int i = 0; i < ap->station_list[p].size(); i++)
				{
					Station* STA = &ap->station_list[p][i];
					int m = sync_mode?0:1;
					STA->startIdx = STA->startIdxs[m];
					STA->total_dealy_time+=STA->delays[m];
					
					double drA = !isnan(STA->allocDRs[m][0])? STA->allocDRs[m][0]:0.0;
					double drB = !isnan(STA->allocDRs[m][1])? STA->allocDRs[m][1]:0.0;
					double Tdr = drA + drB;
					
					//cout <<STA->STA_ID<<", 優先級別 = "<< p <<", success trans number = " << STA->success_trans_nth[m] << endl;
					if(Tdr != 0.0){
						double tmpA = STA->success_trans_nth[m] * tA * drA / (tA * drA + tB * drB);
						double tmpB = STA->success_trans_nth[m] * tB * drB / (tA * drA + tB * drB);
						//cout << "在頻道 A 傳輸的封包數量 = " << tmpA << endl;
						//cout << "在頻道 B 傳輸的封包數量 = " << tmpB << endl;

						
						STA->n_suc_packet_chA+=tmpA;
						STA->n_suc_packet_chB+=tmpB;
					}
					
					STA->success_trans+= STA->success_trans_nth[m];
					
					STA->last_begin_trans_ChA = curTime4A;
					STA->last_end_trans_ChA = curTime4A + durationA;
					STA->last_begin_trans_ChB = curTime4B;
					STA->last_end_trans_ChB = curTime4B + durationB;
					

				}	
			}	
			curTime4A+=durationA, curTime4B+=durationB;
		}
		else
		{
			int Time = 0, Time2 = 0,Bandwidth = 0, ch = -1;
			if(curTime4A < curTime4B)
			{
				Time = curTime4A;
				Time2 = curTime4B;
				Bandwidth = ap->Bandwidth_A_26;
				ch = 0;
			}
			else
			{
				Time = curTime4B;
				Time2 = curTime4A;
				Bandwidth = ap->Bandwidth_B_26;
				ch = 1;
			}
			int transTime = 0;
			if(method == 2){
				ap->updateSTAs(Time,ch,true,false,Bandwidth,alpha);		
				for(int p = 0; Bandwidth > 0 && p < 4; p++)
				{
					Bandwidth = ap->knaspack_sra(Time,Bandwidth,false,-1,p,-1);
				}
				
				transTime = ap->find_avg_len4MLO1(false,Time, Time2 - SIFS*2 - ACK , 5000,-1);
			}
			else{
				ap->updateSTAs(curTime4A,-1,false,false,Bandwidth,1.0);
				ap->sortSTAs(1);
				Bandwidth = ap->opt_RCL(Bandwidth,ch == 0,false,true);	//決定分配大小 
				Bandwidth = ap->opt_FGC(Bandwidth,ch == 0,false,true);	//決定分配速率 
				transTime = ap->find_avg_length(Time);
			}
			//int transTime = ap->find_avg_len4MLO1(false,Time, Time2 - SIFS*2 - ACK , 5000,-1);
			ap->transmit2STAs(Time,transTime);
			int duration = SIFS + transTime + SIFS + ACK;
			
			for(int p = 0; p < 4; p++)
			{
				for(int i = 0; i < ap->station_list[p].size(); i++)
				{
					Station* STA = &ap->station_list[p][i];
					if(curTime4A < curTime4B){
						STA->n_suc_packet_chA+=STA->cur_suc_packet;
						STA->last_begin_trans_ChA = curTime4A;
						STA->last_end_trans_ChA = curTime4A + duration;
						
					} 
					else{
						STA->n_suc_packet_chB+=STA->cur_suc_packet;
						STA->last_begin_trans_ChB = curTime4B;
						STA->last_end_trans_ChB = curTime4B + duration;
						
					} 
				}
			}
			if(curTime4A < curTime4B){
				curTime4A+=duration;
			}
			else{
				curTime4B+=duration;
			}
			
			
		}
		 //延遲傳輸的話 傳輸時間可以拉高很多 
		cout << "當前時間 在頻道A = " <<  curTime4A<<", 在頻道B = "<< curTime4B << endl; 
		c+=1;
		flagA = curTime4A < sim_time;
		flagB = curTime4B < sim_time;
	}
	ap->sortSTAs(0);
	cout << "跑圈" << c<< endl; 
}
