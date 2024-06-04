#include "ap.h"
using namespace std;

//�ΨӼg�ڸ��L�פ�Ƶ{���t��k 
//station���޲z�� 
using namespace std;
AP::AP(int sim_time)
{
	this->sim_time = sim_time;
	//cout << MRUs_dr[13] << endl;
	for(int i = 0; i < 149; i++)
	{
		MRUsToIdx.insert(make_pair(idxToMRUs[i],i));
	}
	
}
//�ѤUNSTR�˸m��������u 
void AP::updateSTAs(int curTime, int ch, bool isJoeFunc,bool two_ch_mode,int Bandwidth, double alpha) 
{
	for(int p = 0; p < 4; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station *STA = &station_list[p][i];
			STA->updateExpired(curTime);
			if(two_ch_mode)
			{			
				// if use two channel
				STA->requiredDRs = {{0.0,0.0},{0.0,0.0}};
				STA->allocDRs = {{0.0,0.0},{0.0,0.0}};
				STA->startIdxs = {0,0};
				STA->success_trans_nth = {0,0};
				STA->delays = {0.0,0.0};
				STA->cur_data_sizes = {0,0};
			}
			STA->updateRD(curTime,ch,isJoeFunc,two_ch_mode,Bandwidth,alpha);
			if(!isJoeFunc){
				int MAX_DR = -1;
				if(Bandwidth == 148+74) MAX_DR = 3603;
				else if(Bandwidth == 148) MAX_DR = 2402;
				else MAX_DR = 1201;
				STA->updateRMRU(curTime,sim_time,MAX_DR,Bandwidth);
			}
			

			//cout << <<station_list[i].required_dr << endl;
			//cout << station_list[p][i].STA_ID<<" �һݸ�Ƴt�v��s���� = "  << station_list[p][i].required_dr << ", ��e�ݶǫʥ]�j�p/bit = "<< station_list[p][i].cur_data_size << endl; 
		}
	}

}

void AP::opt_updateSTAs(int curTime, int ch,int sim_time, int Bandwidth)
{
	for(int p = 0; p < 4; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station *STA = &station_list[p][i];
			STA->updateExpired(curTime);
			STA->updateRD(curTime,ch,false,false,Bandwidth,1.0);
			int MAX_DR = -1;
			if(Bandwidth == 148+74) MAX_DR = 3603;
			else if(Bandwidth == 148) MAX_DR = 2402;
			else MAX_DR = 1201;
			STA->updateRMRU(curTime,sim_time,MAX_DR,Bandwidth);
			//cout << STA->STA_ID<<"�ݭn"<<MRUs[STA->MRU_idx]<<"-tone MRU" << endl; 
		}
	}
}

int AP::allocDR(bool two_ch_mode, int m, int p, int ch)
{
	stringstream ss(alloc_result);
	string item;
	int count_26 = 0;
	double total_DR = 0.0;
	while (std::getline(ss, item, ',')) 
	{
		std::stringstream ss2(item);
		std::string sta, data_rate_idx;
		std::getline(ss2, sta, ':');
		std::getline(ss2, data_rate_idx, ':');
		if(two_ch_mode) station_list[p][stoi(sta)].allocDRs[m][ch] = MRUs_dr[stoi(data_rate_idx)];
		else station_list[p][stoi(sta)].data_rate = MRUs_dr[stoi(data_rate_idx)];
		total_DR+=MRUs_dr[stoi(data_rate_idx)];
		count_26+=MRUsToIdx[MRUs[stoi(data_rate_idx)]];
		int idx = stoi(data_rate_idx);
		//cout << station_list[p][stoi(sta)].STA_ID<<" �ϥ�26-tone �ƶq = "  << MRUsToIdx[MRUs[idx]] << ", ����MRU = "<< MRUs[idx] << ", ��Ƴt�v = "<< MRUs_dr[stoi(data_rate_idx)] << endl; 
	}
	//cout << "�`��Ƴt�v = " <<  total_DR << ", �ϥΤF�h��26-tone = "<<count_26 <<endl; 
	return count_26;

}

/*
	double tmp = (arr + STA->ana_remainData) / STA->data_rate;
			if(STA->data_rate != 0.0){
				double lam = (arr)/MPDU_LENS[p];// T/1000 ������30�L��Ӧh�֭ӫʥ] 
				double mu = STA->data_rate/MPDU_LENS[p];
				double k = th_now * sim_time/MPDU_LENS[p]/T;
				cout <<"lam = " <<lam << ", mu = " << mu << endl;
				cout <<"k = " <<k << endl;
				double raw = lam / mu;
				if(isnan(raw)) raw = 0.0;
				cout << "raw = " << raw << std::endl;
				double kp_raw = pow(raw, k);
				if(isnan(kp_raw)) kp_raw = 0.0;
			    double kpp1_raw = pow(raw, k + 1);
			    if(isnan(kpp1_raw)) kpp1_raw = 0.0;
			    double p_k = (1 - raw) / (1 - kpp1_raw) * kp_raw;
			    if(isnan(p_k)) p_k = 0.0;
			    //cout << "p^k = " << p_k << std::endl;
			    double lam_eff = lam * (1 - p_k);
			    cout << "lam_eff = " << lam_eff << std::endl;
			    double L = raw * (k * kpp1_raw - (k + 1) * kp_raw + 1) / ((1 - kpp1_raw) * (1 - raw));
			    //if(isnan(L)) L = 0.0;
			    cout << "L = " << L << std::endl;
			    double W = L / lam_eff;
			    if(W!=isnan(W*k))
					STA->ana_D+=(T*k)*W;						
			}
*/
void AP::cal_STAs_ana(int T, int sim_time)
{
	//�p��X1 mus�����|���h�֧]�R�q *10^6 => �Y�i���o�����C��]�R�q 
	double usedTDR = 0.0;
	for(int p = 0; p < 4; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			double padding = 0.0;
			Station *STA = &station_list[p][i];
			double arr = double(traffic_arrival_rates[p]);
			double th_avg_now = d_min(double(T) / sim_time *  STA->data_rate, double(last_T) / sim_time * arr + STA->ana_remainData); // average th in a mus.
			STA->ana_TH+=th_avg_now;
			
			STA->ana_RFs.push_back(double(last_T)  * arr);
			double tmp_th_now = th_avg_now * sim_time;
			double tmp_es = STA->cur_expired_size;
			while(STA->ana_RF_idx < STA->ana_RFs.size() && tmp_es > 0.0)
			{
				if(STA->ana_RFs[STA->ana_RF_idx] > tmp_es)
				{
					STA->ana_RFs[STA->ana_RF_idx]-=tmp_es;
					tmp_es = 0.0;
					//break;
				}
				else
				{
					tmp_es-=STA->ana_RFs[STA->ana_RF_idx];
					STA->ana_RFs[STA->ana_RF_idx] = 0.0;
					STA->ana_RF_idx+=1;
				}
			}
			while(STA->ana_RF_idx < STA->ana_RFs.size() && tmp_th_now > 0.0)
			{
				double curFlow = 0.0;
				//�ݭn�Ҽ{�Y�ɬq���ʥ]�L��
				 
				if(STA->ana_RFs[STA->ana_RF_idx] > tmp_th_now)
				{
					curFlow = tmp_th_now;
					STA->ana_RFs[STA->ana_RF_idx]-=tmp_th_now;
					tmp_th_now = 0.0;
				}
				else
				{
					curFlow = STA->ana_RFs[STA->ana_RF_idx];
					tmp_th_now-=STA->ana_RFs[STA->ana_RF_idx];
					STA->ana_RFs[STA->ana_RF_idx] = 0.0;
					//STA->ana_RF_idx+=1;
				}
				//�p��delay
				double curN = curFlow / MPDU_LENS[p];
				STA->ana_SN+=curN;
				 
				if(STA->data_rate > 0.0) STA->ana_SD+= (curN * (curN + 1) / 2 * (MPDU_LENS[p] / STA->data_rate) + curN * 32860 * (STA->ana_RFs.size() - STA->ana_RF_idx - 1 + 0.6))/ 1000; //mus to ms
				if(STA->ana_RFs[STA->ana_RF_idx] == 0.0) STA->ana_RF_idx+=1;
			}
			
			
			STA->ana_avgDR+=double(T) / sim_time *  STA->data_rate;
			STA->ana_remainData+=(arr - STA->data_rate) * T / sim_time;
			STA->ana_remainData-=double(STA->cur_expired_size)/sim_time;//���F�P�W�@�楿�W�ơA�ݭn���WT/sim_time�A�P�ɤ]�n���WT�~�ા�D����mus���h�ֹL���ʥ]bit�M 
			STA->ana_remainData = STA->ana_remainData < 0.0?0.0:STA->ana_remainData;			
			STA->last_expired_size = STA->cur_expired_size;
			
			usedTDR+= STA->data_rate;
		}
	}
	last_T = T;
}


int AP::find_avg_length(int curTime)//�o�̦����D 
{
    double l = 0.0;
    int c = 0;
    for(int p = 0; p < 4; p++)
    {
    	for (int i = 0; i < station_list[p].size(); i++)
    	{
    		Station* STA = &station_list[p][i];
    		double tmp =  double(STA->cur_data_size)/STA->data_rate;
    		if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
        	l += tmp;
        	c+=1;
    	}
	}
	
    //cout << l << "<->" << c << endl;
    int t = round(l/c);
    //cout << t << endl;
    //return 1000;
    if(t>32768) t = 32768;
    if(t < 5000) t = 5000;
    if(curTime + t > sim_time) t = sim_time - curTime;
    cout << "�ǿ�ɶ� = " << t << endl; 
    return t;
}

int AP::find_avg_len4MLO0(int curTime)
{
	double l = 0.0;
	int c = 0;
	for(int p = 0; p < 2; p++)
    {
    	for (size_t i = 0; i < station_list[p].size(); i++)
    	{
    		Station* STA = &station_list[p][i];
    		double tmp = double(STA->cur_data_size)/(STA->allocDRs[0][0] + STA->allocDRs[0][1]); //���n��cur_data_size�A�_�X�n��� EX: drA/(drA+drB) * cur_data_size
    		//cout << station_list[i].STA_ID<<" �z�Q�ǿ�ɶ�/mus = "  << tmp << endl; 
    		if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
        	l += tmp;
        	c+=1;
    	}
	}

    cout << l << " " << c << endl;
    int t = round(l/c);
    //cout << t << endl;
    if(t>32768) t = 32768;
    if(t < 5000) t = 5000;
    if(curTime + t > sim_time) t = sim_time - curTime;
    //cout << "�ǿ�ɶ� = " << t << endl; 
    return t;
}


int AP::find_avg_len4MLO1(bool two_ch_mode,int curTime, int targetTime, int toleranceThs, int ch)// target time �O�o�Ҽ{sifs + ack 
{
	double l = 0.0;
	int c = 0;
	for(int p = 0; p < 2; p++)
    {
    	for (size_t i = 0; i < station_list[p].size(); i++)
    	{
    		double tmp = double(station_list[p][i].cur_data_size);
    		if(two_ch_mode){
    			double total_DR = station_list[p][i].allocDRs[1][0] + station_list[p][i].allocDRs[1][1];
    			tmp = tmp*station_list[p][i].allocDRs[1][ch]/total_DR/station_list[p][i].allocDRs[1][1];
			} 
    		else tmp/=station_list[p][i].data_rate;
    		if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
        	l += tmp;
        	c+=1;
    	}
	}

    cout << l << " " << c << endl;
    int t = round(l/c);
    //cout << t << endl;
    if(t>32768) t = 32768;
    if(t < 5000) t = 5000;
    if(targetTime!=-1 && curTime + t + toleranceThs >= targetTime) t = targetTime - curTime;
    if(curTime + t > sim_time) t = sim_time - curTime;
    //cout <<"�W�D = "<< ch << ", �ǿ�ɶ� = " << t << endl;
    return t;
}
void AP::print_info()
{
	double sum_dr = 0.0;
	cout << "�C�LSTAs��T"<<endl; 
	for(int p = 0; p < 4; p++)
	{
		for(int i = 0; i <station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			//cout <<STA->STA_ID<<", "<<STA->allocDRs[0][0] << ", "<< STA->allocDRs[0][1] <<", "<<STA->allocDRs[1][0] << ", "<< STA->allocDRs[1][1]<< endl;
			cout << STA->STA_ID<<", "<<STA->data_rate << endl;
			sum_dr+=STA->data_rate;
		}
	}
	cout << "�������t�`�@ DR = " << sum_dr << endl;
}
 

//���� OFDMA��duration �쩳�n���n�g��
// 

int AP::knaspack_sra(int curTime, int Bandwidth, bool two_ch_mode,int m,int p, int ch)
{
	vector<pair<double, string>> dp(Bandwidth+1, {0.0, ""});
	for (int i = 1; i <= station_list[p].size(); i++)
	{
	    for (int j = Bandwidth; j >= 1; j--)
	    {
	    	
	    	Station *STA = &station_list[p][i-1];
	    	if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    	if(!two_ch_mode && STA->required_dr == 0.0) continue;
	        int c = 0;
	        while (idxToMRUs[j] >= MRUs[c])
	        {
	        	 
	        	double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
	        	
	            double tmp_dr = min(MRUs_dr[c], RD);
	            if (dp[j].first < dp[j - MRUsToIdx[MRUs[c]]].first + tmp_dr)
	            {
	                dp[j].first = dp[j - MRUsToIdx[MRUs[c]]].first + tmp_dr;
	                stringstream ss;
	                if (dp[j - MRUsToIdx[MRUs[c]]].second.empty())
	                {
	                    ss << i-1 << ":" << c;
	                }
	                else
	                {
	                    ss << dp[j - MRUsToIdx[MRUs[c]]].second << "," << i-1 << ":" << c;
	                }
	                dp[j].second = ss.str();
	            }
	            c++;
	        }
	    }
	}
	alloc_result = dp[Bandwidth].second;
	//cout <<"�Ƶ{���G = "<< alloc_result<<endl;
	int remain_BW = two_ch_mode?Bandwidth - allocDR(true,m,p,ch):Bandwidth - allocDR(false,-1,p,-1); // allocDR����n�NDR ASSIGN�� STA��allocDRs
	//int remain_BW = Bandwidth - allocDR(false,-1,p,-1);
	reOrderSTAs(p);
	return remain_BW;
}


double AP::d_min(double a, double b) {
    if (std::abs(a - b) < 1e-9) {
        return a < b ? a : b;
    } else {
        return a < b ? a : b;
    }
}

double AP::getEDR4STA(double EDRch, double usedRD_A, double usedRD_B, double RD, double usedRD_ch)
{ 
	//cout <<"here~" <<EDRch*(usedRD_A + usedRD_B + RD) - usedRD_ch << ", RD = "<< RD << endl;
	double result = EDRch*(usedRD_A + usedRD_B + RD) - usedRD_ch;
	if(result <= 0.0) return 0.0;
	return result;
}


void AP::twoChUsersAlloc()
{
	//�إ�2*4*2��vec�A�Ω�έp��e���tData Rate;2 mlo, 4 pri, 2 ch
	vector<vector<vector<double>>> usedRDs(2, vector<vector<double>>(4, vector<double>(2, 0.0)));
	for(int p = 0; p < 4; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(STA->device != "SL")// MLD
			{				
				STA->requiredDRs[0][0] = getEDR4STA(EDR_A, usedRDs[0][p][0],usedRDs[0][p][1], STA->required_dr,usedRDs[0][p][0]);// 0 = sync, 0 = ch A 
				STA->requiredDRs[0][1] = getEDR4STA(EDR_B, usedRDs[0][p][0],usedRDs[0][p][1], STA->required_dr,usedRDs[0][p][1]);// 0 = sync, 1 = ch B
				usedRDs[0][p][0]+=STA->requiredDRs[0][0];
				usedRDs[0][p][1]+=STA->requiredDRs[0][1];
				if(STA->device == "STR")
				{
					STA->requiredDRs[1][0] = getEDR4STA(EDR_A, usedRDs[1][p][0],usedRDs[1][p][1], STA->required_dr,usedRDs[1][p][0]);// 0 = sync, 0 = ch A 
					STA->requiredDRs[1][1] = getEDR4STA(EDR_B, usedRDs[1][p][0],usedRDs[1][p][1], STA->required_dr,usedRDs[1][p][1]);// 0 = sync, 1 = ch B
					usedRDs[1][p][0]+=STA->requiredDRs[1][0];
					usedRDs[1][p][1]+=STA->requiredDRs[1][1];
				}
				else if(STA->device == "NSTR")
				{
					double totalRD = usedRDs[1][p][0] + usedRDs[1][p][1] + STA->required_dr;
					double errGoChA = abs((usedRDs[1][p][0] + STA->required_dr)/totalRD - EDR_A) + abs(usedRDs[1][p][1] / totalRD - EDR_B);
					double errGoChB = abs((usedRDs[1][p][0])/totalRD - EDR_A) + abs((usedRDs[1][p][1] + STA->required_dr) / totalRD - EDR_B);
					if(STA->requiredDRs[1][0]==0.0 && STA->requiredDRs[1][1]!=0.0)
					{
						STA->requiredDRs[1][0] = STA->required_dr;
						usedRDs[1][p][0]+=STA->requiredDRs[1][0];
					}
					else if(STA->requiredDRs[1][0]!=0.0 && STA->requiredDRs[1][1]==0.0)
					{
						STA->requiredDRs[1][1] = STA->required_dr;
						usedRDs[1][p][1]+=STA->requiredDRs[1][1];
					}				
					else if(errGoChA < errGoChB)
					{
						STA->requiredDRs[1][0] = STA->required_dr;
						usedRDs[1][p][0]+=STA->requiredDRs[1][0];
					}
					else if(errGoChA > errGoChB) 
					{
						STA->requiredDRs[1][1] = STA->required_dr;
						usedRDs[1][p][1]+=STA->requiredDRs[1][1];
					}					
				}
			}
			else if(STA->device == "SL")
			{
				STA->requiredDRs[0][0] = STA->required_dr;
				usedRDs[0][p][0]+=STA->requiredDRs[0][0];
				STA->requiredDRs[1][0] = STA->required_dr;
				usedRDs[1][p][0]+=STA->requiredDRs[1][0];										
			}
			//cout <<"MLO = 0, ch A,B = "<<STA->requiredDRs[0][0] << " ," <<STA->requiredDRs[0][1]<<endl;
			//cout <<"MLO = 1, ch A,B = "<<STA->requiredDRs[1][0] << " ," <<STA->requiredDRs[1][1]<<endl;  
						
		}
	}
	 
}
/*
int AP::knaspack_sra(int curTime, int Bandwidth, int p) //2d DP 
{
	//pair <int, map<int, double>> element; ���q,MRU�t�m  
	pair<double, string> dp[station_list[p].size()+1][Bandwidth+1];
	//dp to all 0
	alloc_result = "";
	
	for(int i = 0; i <=station_list[p].size();i++){
		dp[i][0] = {0.0,""};
	}
	for(int j = 0; j <= Bandwidth;j++){
		dp[0][j] = {0.0,""};
	}
	
	for(int i = 1; i <= station_list[p].size(); i++)
	{
		for(int j = 1; j <= Bandwidth; j++)
		{
			int c = 0;
			dp[i][j] = {0.0,""};
			while(idxToMRUs[j] >= MRUs[c])
			{
				double tmp_dr = min(MRUs_dr[c], station_list[p][i-1].required_dr);
				//cout << "j = " << j << ", ���t���ei-1�ӥΤ� = "<<  j - MRUsToIdx[MRUs[c]] <<", ���t��i�ӥΤ� = "<<MRUsToIdx[MRUs[c]]<<endl; 
				if (dp[i][j].first < dp[i-1][j - MRUsToIdx[MRUs[c]]].first + tmp_dr)
				{
					dp[i][j].first = dp[i-1][j - MRUsToIdx[MRUs[c]]].first + tmp_dr; //trouble here out of mem
					stringstream ss;
					if(dp[i-1][j - MRUsToIdx[MRUs[c]]].second.empty())
					{
						ss  << i-1 << ":" << c;
					}
					else
					{
						ss << dp[i-1][j - MRUsToIdx[MRUs[c]]].second << "," << i-1 << ":" << c;
					}
					
					dp[i][j].second = ss.str();
				}
				c+=1;
			} 
			
		}
		
	}

	alloc_result = dp[station_list[p].size()][Bandwidth].second;
	cout <<"�Ƶ{���G = "<< alloc_result<<endl;
	int remain_BW = Bandwidth - allocDR(p);
	return remain_BW;
	
}*/

void AP::transmit2STAs(int curTime, int transTime)
{
	for(int p = 0; p < 4; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			STA->cur_suc_packet = 0;
			if(fabs(STA->data_rate) < 1e-15) continue;
			int throughput = min(static_cast<int>(round(transTime * STA->data_rate)), STA->cur_data_size);
			STA->cur_sum_DR+=throughput;
			//cout << "�i�ǧ]�R�q = " << throughput<<", idx = "<< station_list[p][i].startIdx << endl;
			int* startIdx = &STA->startIdx;
			double sum_trans_dealy = 0.0;
			while(throughput > 0 && *startIdx < STA->packets.size())
			{
				
				if(throughput >= STA->packets[*startIdx].packetSize)
				{
					throughput-=STA->packets[*startIdx].packetSize;
					STA->cur_data_size-=STA->packets[*startIdx].packetSize;
					sum_trans_dealy+=double(STA->packets[*startIdx].packetSize) / STA->data_rate;
					STA->total_dealy_time+= curTime + sum_trans_dealy - STA->packets[*startIdx].arrival_time;//�̲ץت��A�p������ʥ]���`����ɶ� �q�L���H���\�ǿ骺����success_trans
					*startIdx+=1;//���ʥ]�ǿ駹��
					STA->success_trans+=1;
					STA->cur_suc_packet+=1;
				}
				else
				{
					STA->packets[*startIdx].packetSize-=throughput;
					throughput = STA->cur_data_size =  0;
				}
			}
			//cout <<"after idx = "<< station_list[p][i].startIdx << endl; 
		}
		
	}
}



void AP::sim_transmit2STAs(int curTime,int transTimeA, int transTimeB, int m)
{
	//dealy, success_trans_nth �אּ�ھ�m��1d vec 
	for(int p = 0; p < 4; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			int commonTime = -1, extraTime = -1, extraCh = -1;
			double sum_trans_dealy = 0.0;
			if(transTimeA > transTimeB)
			{
				commonTime = transTimeB;
				extraTime = transTimeA - commonTime;
				extraCh = 0;
			}
			else
			{
				commonTime = transTimeA;
				extraTime = transTimeB - commonTime;
				extraCh = 1;
			}
			if(fabs(STA->allocDRs[m][0]) < 1e-15 && fabs(STA->allocDRs[m][1]) < 1e-15) continue;
			int throughput = min(static_cast<int>(round(commonTime * (STA->allocDRs[m][0]+STA->allocDRs[m][1]))), STA->cur_data_sizes[m]);
			int* startIdx = &STA->startIdxs[m];
			int* curPS = &STA->last_packet_sizes[m];
			//�ثe�ݤU�� startIdxs,cur_data_sizes, delays, success_trans_nth ���u�ݭn�ھ� mlo�Y�i 
			//cout << "�i�ǧ]�R�q = " << throughput<<", idx = "<< station_list[p][i].startIdx << endl; 
			while(throughput > 0 && *startIdx < STA->packets.size())
			{
				
				if(throughput >= *curPS)//�i�H�ǧ���ӫʥ] 
				{
					throughput-=*curPS;
					STA->cur_data_sizes[m]-=*curPS;
					sum_trans_dealy+= double(*curPS) / (STA->allocDRs[m][0] + STA->allocDRs[m][1]);
					STA->delays[m]+= curTime + sum_trans_dealy \
					- STA->packets[*startIdx].arrival_time;//�̲ץت��A�p������ʥ]���`����ɶ� �q�L���H���\�ǿ骺����success_trans
					*startIdx+=1;
					STA->success_trans_nth[m]+=1;
					*curPS = STA->packet_size;
				}
				else
				{
					*curPS-=throughput;
					throughput = STA->cur_data_sizes[m] =  0;
				}
			}
			throughput = min(static_cast<int>(round(extraTime * STA->allocDRs[m][extraCh])), STA->cur_data_sizes[m]);
			sum_trans_dealy = 0;
			while(throughput > 0 && *startIdx < STA->packets.size())
			{
				if(throughput >= *curPS)//�i�H�ǧ���ӫʥ] 
				{
					throughput-=*curPS;
					STA->cur_data_sizes[m]-=*curPS;
					sum_trans_dealy+=double(*curPS) / STA->allocDRs[m][extraCh];
					STA->delays[m]+= curTime + commonTime + sum_trans_dealy\
					- STA->packets[*startIdx].arrival_time;//�̲ץت��A�p������ʥ]���`����ɶ� �q�L���H���\�ǿ骺����success_trans
					*startIdx+=1;
					STA->success_trans_nth[m]+=1;
					*curPS = STA->packet_size;
				}
				else
				{
					*curPS-=throughput;
					throughput = STA->cur_data_sizes[m] =  0;
				}
			}
			//cout <<"after idx = "<< station_list[p][i].startIdx << endl; 
		}	
	}	
}
double AP::evalEBR(int m, int transTimeA, int transTimeB)
{
	double avg_throughput = 0.0;
	for(int p = 0; p < 4; p++)
	{	
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			double total_DR = STA->allocDRs[m][0] + STA->allocDRs[m][1];
			double tmp = double(STA->success_trans_nth[m])*STA->allocDRs[m][0] * STA->packet_size / total_DR  / transTimeA;
			if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
			avg_throughput+= tmp;
			tmp = double(STA->success_trans_nth[m])*STA->allocDRs[m][1] * STA->packet_size / total_DR  / transTimeB;
			if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
			avg_throughput+= tmp;
		}
	}
	return avg_throughput;
}

void AP::reOrderSTAs(int p)
{
	Station tmp = station_list[p][0];
	for (int i = 0; i < station_list[p].size() - 1; i++)
	{
		station_list[p][i] = station_list[p][(i+1)%station_list[p].size()];
	}
	station_list[p][station_list[p].size() - 1] = tmp;
		
}

void AP::sortSTAs(int method)
{
	if(method == 0)
	{
		for(int p = 0; p < 4; p++)
		{
			sort(station_list[p].begin(),station_list[p].end(),Station::compareBySTAID);
		}
	}
	else if(method == 1)
	{
		for(int p = 0; p < 4; p++)
		{
			sort(station_list[p].begin(),station_list[p].end(),Station::compareByRD);
		}
	}
	
}
//�n�g��function alloc dr - rd 
void AP::opt_filter()
{
	for(int p = 0; p < 4; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(STA->data_rate > 0.0)
			{
				STA->MRU_idx = 0;
			}
		}
	}
}
int AP::opt_RCL(int Bandwidth, bool isCHA, bool two_ch_mode, bool two_ch)//two_ch�����O�����W�D������A two_ch_mode�h�O����P�ɨ���W�D 
{
	int ch = isCHA? 0:1;
	for(int p = 0; p < 4; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(two_ch && ch == 0 && STA->device == "SL") continue; 
			if(!isCHA && MRUs[STA->MRU_idx] > 2*996) continue;
			int tmp = Bandwidth - MRUsToIdx[MRUs[STA->MRU_idx]];
			if(tmp < 0) return Bandwidth;
			if(two_ch_mode){
				STA->allocDRs[1][ch] = MRUs_dr[STA->MRU_idx];
			}
			else{
				STA->data_rate = MRUs_dr[STA->MRU_idx];
			}
			
			Bandwidth = tmp;
			if(tmp == 0) return Bandwidth;
			
		}
	}
	return Bandwidth;
}

int AP::opt_FGC(int Bandwidth, bool isCHA, bool two_ch_mode, bool two_ch)
{
	int ch = isCHA? 0:1;
	for(int p = 0; p < 4; p++)
	{
		//if(p==2) continue;
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(two_ch && ch == 0 && STA->device == "SL") continue; 
			if(MRUs[STA->MRU_idx] <= 106)// small size mru
			{
				int compensate = min(5 - STA->MRU_idx,Bandwidth);
				Bandwidth-=compensate;
				if(Bandwidth == 0) return 0;
				STA->MRU_idx+=compensate;
				STA->data_rate = MRUs_dr[STA->MRU_idx];
				if(two_ch_mode){
					STA->allocDRs[1][ch] = MRUs_dr[STA->MRU_idx];
				}
				else{
					STA->data_rate = MRUs_dr[STA->MRU_idx];
				}
			}
		}
	}
	return Bandwidth;
}
