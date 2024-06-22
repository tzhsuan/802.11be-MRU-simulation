#include "station.h"
using namespace std;
Station::Station()
{
	
}

bool Station::compareBySTAID(const Station& s1, const Station& s2){
	return s1.STA_ID < s2.STA_ID;
}

bool Station::compareByRD(const Station& s1, const Station& s2){
	return s1.required_dr < s2.required_dr;
}

Station::Station(string device, int STA_ID,int priority, int sim_time, int packet_size)
{
	this->device = device;
	this->STA_ID = STA_ID;
	this->priority = priority;
	this->sim_time = sim_time;
	this->packet_size = packet_size;
	last_packet_sizes[0] = last_packet_sizes[1] = packet_size;
	
	//暫定選用rand選取MCS index，之後可能選用10%-50%頻道受干擾做實驗 
	for(int i=0; i<144; i++)
	{
		MCS_A[i] = rand()%12;
		if(minMCS_A > MCS_A[i]) minMCS_A = MCS_A[i];
	}
	for(int i=0; i<72; i++)
	{
		MCS_B[i] = rand()%12;
		if(minMCS_B > MCS_B[i]) minMCS_B = MCS_B[i];
	}
	minMCS_A = minMCS_B = 11;
}

void Station::updateRD(int curTime, int ch, bool isJoeFunc, bool isTzuFunc, bool two_ch_mode,int Bandwidth,double alpha)//required data rate
{
	// ch = 0 <- A, 1 <- B, -1 <- dont care, 可能有兩個頻道的傳輸權或是只跑A頻道實驗(不介意裝置類型) 
	required_dr = 0.0;
	cur_data_size = 0;
	data_rate = 0.0;
	int MAX_DR = -1;
	
	//前人設定 
//	if(Bandwidth == 148+74) MAX_DR = 3603;
//	else if(Bandwidth == 148) MAX_DR = 2402;
//	else MAX_DR = 1201;
    if(Bandwidth == 148+74)	MAX_DR = 4*966*MCS_R[minMCS_B][0]*MCS_R[minMCS_B][1]/(12.8+0.8) + 2*966*MCS_R[minMCS_A][0]*MCS_R[minMCS_A][1]/(12.8+0.8);
	else if(Bandwidth == 148) MAX_DR = (4*966)*MCS_R[minMCS_B][0]*MCS_R[minMCS_B][1]/(12.8+0.8);
	else MAX_DR = (2*996)*MCS_R[minMCS_A][0]*MCS_R[minMCS_A][1]/(12.8+0.8);

	if(device == "SL" && ch == 0){
		return;
	}
	else if(device == "NSTR" && ch != -1)
	{
		if(ch == 0 && curTime < last_end_trans_ChB){
			return;
		}
		else if (ch == 1 && curTime < last_end_trans_ChA){
			return;
		}
	}
	is_timecritical = false;
	
	for(int i = startIdx; i < packets.size(); i++)
	{
		//packets[i].canTrans = false;
		if(packets[i].arrival_time <= curTime)
		{
			
			if(isJoeFunc)
			{
				double tmp = double(packets[i].packetSize)/(min(sim_time,packets[i].deadline) - curTime);
				if(!isinf(tmp) && tmp > 0 && tmp < MAX_DR) required_dr+=tmp;
				else{
					//startIdx+=1;
					continue;
				}
				
			}
			else if(isTzuFunc)
			{
				if(Bandwidth == 148+74) MAX_DR = 4*966*MCS_R[11][0]*MCS_R[11][1]/(12.8+0.8) + 2*966*MCS_R[11][0]*MCS_R[11][1]/(12.8+0.8);
				else if(Bandwidth == 148) MAX_DR = 4*966*MCS_R[11][0]*MCS_R[11][1]/(12.8+0.8);
				else MAX_DR = 2*966*MCS_R[11][0]*MCS_R[11][1]/(12.8+0.8);
				
				if (sim_time > packets[i].deadline)
				{
					is_timecritical = true;
				}
				
				double tmp = double(packets[i].packetSize)/(min(sim_time,packets[i].deadline) - curTime);
				if(!isinf(tmp) && tmp > 0 && tmp < MAX_DR) required_dr+=tmp;
				else{
					//startIdx+=1;
					continue;
				}
				
			}	
			else required_dr+= double(packets[i].packetSize);			
			cur_data_size+=packets[i].packetSize;
											
			//這裡也許要去掉極端值 
		}
		else break;
	}
	if(!isJoeFunc && !isTzuFunc){
		required_dr/= double(sim_time)-curTime;
	}
	if(two_ch_mode)
	{
		//assign startIdx to vectors
		cur_data_sizes[0] = cur_data_sizes[1] = cur_data_size;
		startIdxs[0] = startIdxs[1] = startIdx;
	}
	if(!isTzuFunc) required_dr*=alpha;//alpha 
	
	//cout << STA_ID<<" 所需資料速率更新完畢 = "  << required_dr << endl; 
}

void Station::updateExpired(int curTime)
{
	cur_expired_size = 0;
	last_expired2_size = last_expired_size;
	last_expired_size = cur_expired_size;
	while(packets.size() > startIdx)
	{
		if(curTime > packets[startIdx].deadline) {
			//在這裡計算過期封包 
			n_expired_packet+=1;
			cur_expired_size+=packets[startIdx].packetSize;
			startIdx+=1;
		}		
		else break;		
	}
	
	//cout << STA_ID << " 過期封包更新完畢" << endl; 
}
void Station::updateRMRU(int curTime, int sim_time, double MaxDR, int Bandwidth)
{
	double trans_rate = (required_dr * sim_time - cur_sum_DR) / (sim_time - curTime);
	double In = required_dr / MaxDR * Bandwidth;
	MRU_idx = 0;
	for(int i = 0; i < upperBoundMRU.size(); i++)
	{
		if(In < upperBoundMRU[i]){
			MRU_idx = i;
			break;
		}
	}
}


