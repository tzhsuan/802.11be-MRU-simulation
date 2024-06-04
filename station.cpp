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
}

void Station::updateRD(int curTime, int ch, bool isJoeFunc, bool two_ch_mode,int Bandwidth,double alpha)//required data rate
{
	// ch = 0 <- A, 1 <- B, -1 <- dont care, �i�঳����W�D���ǿ��v�άO�u�]A�W�D����(�����N�˸m����) 
	required_dr = 0.0;
	cur_data_size = 0;
	data_rate = 0.0;
	int MAX_DR = -1;

	
	
	if(Bandwidth == 148+74) MAX_DR = 3603;
	else if(Bandwidth == 148) MAX_DR = 2402;
	else MAX_DR = 1201;

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
			else required_dr+= double(packets[i].packetSize);			
			cur_data_size+=packets[i].packetSize;
											
			//�o�̤]�\�n�h�����ݭ� 
		}
		else break;
	}
	if(!isJoeFunc){
		required_dr/= double(sim_time)-curTime;
	}
	if(two_ch_mode)
	{
		//assign startIdx to vectors
		cur_data_sizes[0] = cur_data_sizes[1] = cur_data_size;
		startIdxs[0] = startIdxs[1] = startIdx;
	}
	required_dr*=alpha;//alpha 
	//cout << STA_ID<<" �һݸ�Ƴt�v��s���� = "  << required_dr << endl; 
}

void Station::updateExpired(int curTime)
{
	cur_expired_size = 0;
	last_expired2_size = last_expired_size;
	last_expired_size = cur_expired_size;
	while(packets.size() > startIdx)
	{
		if(curTime > packets[startIdx].deadline) {
			//�b�o�̭p��L���ʥ] 
			n_expired_packet+=1;
			cur_expired_size+=packets[startIdx].packetSize;
			startIdx+=1;
		}		
		else break;		
	}
	
	//cout << STA_ID << " �L���ʥ]��s����" << endl; 
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



