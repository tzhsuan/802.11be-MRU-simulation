#include "ap.h"
#include "hungarian_optimizer.h"
using namespace std;

//用來寫我跟其他論文排程的演算法 
//station的管理者 
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
//剩下NSTR裝置不能全雙工 
void AP::updateSTAs(int curTime, int ch, bool isJoeFunc, bool isTzuFunc,bool two_ch_mode,int Bandwidth, double alpha) 
{
	for(int p = 0; p < priority_num; p++)
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
			STA->updateRD(curTime,ch,isJoeFunc,isTzuFunc,two_ch_mode,Bandwidth,alpha);
			if(!isJoeFunc && !isTzuFunc){
				int MAX_DR = -1;
//				if(Bandwidth == 148+74) MAX_DR = 3603;
//				else if(Bandwidth == 148) MAX_DR = 2402;
//				else MAX_DR = 1201;
    			if(Bandwidth == 148+74)	MAX_DR = 4*966*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8) + 2*966*STA->MCS_R[STA->minMCS_B][0]*STA->MCS_R[STA->minMCS_B][1]/(12.8+0.8);
				else if(Bandwidth == 148) MAX_DR = (4*966)*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8);
				else MAX_DR = (2*996)*STA->MCS_R[STA->minMCS_B][0]*STA->MCS_R[STA->minMCS_B][1]/(12.8+0.8);
				STA->updateRMRU(curTime,sim_time,MAX_DR,Bandwidth);
			}
			

			//cout << <<station_list[i].required_dr << endl;
			//cout << station_list[p][i].STA_ID<<" 所需資料速率更新完畢 = "  << station_list[p][i].required_dr << ", 當前待傳封包大小/bit = "<< station_list[p][i].cur_data_size << endl; 
		}
	}

}

void AP::updateEND(int curTime, int ch, bool isJoeFunc, bool isTzuFunc,bool two_ch_mode,int Bandwidth, double alpha) 
{
	for(int p = 0; p < priority_num; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station *STA = &station_list[p][i];
			STA->ENDupdateExpired(curTime);
		}
	}

}

void AP::opt_updateSTAs(int curTime, int ch,int sim_time, int Bandwidth)
{
	for(int p = 0; p < priority_num; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station *STA = &station_list[p][i];
			STA->updateExpired(curTime);
			STA->updateRD(curTime,ch,false,false,false,Bandwidth,1.0);
			int MAX_DR = -1;
//			if(Bandwidth == 148+74) MAX_DR = 3603;
//			else if(Bandwidth == 148) MAX_DR = 2402;
//			else MAX_DR = 1201;
            if(Bandwidth == 148+74)	MAX_DR = 4*966*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8) + 2*966*STA->MCS_R[STA->minMCS_B][0]*STA->MCS_R[STA->minMCS_B][1]/(12.8+0.8);
			else if(Bandwidth == 148) MAX_DR = (4*966)*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8);
			else MAX_DR = (2*996)*STA->MCS_R[STA->minMCS_B][0]*STA->MCS_R[STA->minMCS_B][1]/(12.8+0.8);
			STA->updateRMRU(curTime,sim_time,MAX_DR,Bandwidth);
			//cout << STA->STA_ID<<"需要"<<MRUs[STA->MRU_idx]<<"-tone MRU" << endl; 
		}
	}
}

int AP::allocDR(vector<vector<int>>& allocation_table,bool two_ch_mode, int m, int p, int ch)
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
//		if(two_ch_mode) station_list[p][stoi(sta)].allocDRs[m][ch] = MRUs_dr[stoi(data_rate_idx)];
//		else station_list[p][stoi(sta)].data_rate = MRUs_dr[stoi(data_rate_idx)];
		Station *STA = &station_list[p][stoi(sta)];
		double En =0.0;
		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
		int Canallocate = 0;
		for(int i=0; i<allocation_table[stoi(data_rate_idx)-1].size();i++)
		{
			if(allocation_table[stoi(data_rate_idx)-1][i] == 0)
			{
				int min_mcs =12;
				vector<int> map_26;
				MRU_map_26(map_26,allocation_table,stoi(data_rate_idx)-1,i);
				for(int index = 0; index < map_26.size();index++)
  				{
  					if (ch ==1)
  					{
						if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
					}	  
  					else
					{
						if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
					}
					//cout <<"index = "<< index <<endl;	  
  				    //cout <<"min_mcs = "<< min_mcs <<endl;
				}
				//cout <<"min_mcs = "<< min_mcs <<endl;
				En = MRUs[stoi(data_rate_idx)]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);				
				renew_allocation_table(allocation_table,ch,stoi(data_rate_idx)-1,map_26);	
				map_26.clear(); 
				vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
				Canallocate = 1;
				break;
			} 		
		}

		if(Canallocate == 0) continue;
		if(two_ch_mode)
		{
			if(ch == 0)	station_list[p][stoi(sta)].allocDRs[m][ch] = min(En,RD);
			else station_list[p][stoi(sta)].allocDRs[m][ch] = min(En,RD);
			total_DR+= station_list[p][stoi(sta)].allocDRs[m][ch];
		} 
		else
		{
			station_list[p][stoi(sta)].data_rate = min(En,RD);
			total_DR+=station_list[p][stoi(sta)].data_rate;
		} 
		//total_DR+=MRUs_dr[stoi(data_rate_idx)];
		count_26+=MRUsToIdx[MRUs[stoi(data_rate_idx)]];
		int idx = stoi(data_rate_idx);
		//cout << station_list[p][stoi(sta)].STA_ID<<", RD= "<< RD<<", 使用26-tone 數量 = "  << MRUsToIdx[MRUs[idx]] << ", 換成MRU = "<< MRUs[idx] << ", 資料速率 = "<< min(En,RD) << endl; 
	}

	cout << "總資料速率 = " <<  total_DR << ", 使用了多少26-tone = "<<count_26 <<endl; 
	return count_26;

}

/*
	double tmp = (arr + STA->ana_remainData) / STA->data_rate;
			if(STA->data_rate != 0.0){
				double lam = (arr)/MPDU_LENS[p];// T/1000 約等於30微秒來多少個封包 
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
void AP::cal_STAs_ana(vector<vector<int>>& allocation_table, int T, int sim_time,int c, int curtime)
{
	//計算出1 mus平均會有多少吞吐量 *10^6 => 即可推得平均每秒吞吐量 
	int ch = 0;
	double usedTDR = 0.0;
	for(int p = 0; p < priority_num; p++)
	{
		vector<int> non_critical;
		for(int i = 0; i < station_list[p].size(); i++)
		{
			double padding = 0.0;
			Station *STA = &station_list[p][i];
			if(STA->is_timecritical == true)
			{
				if(STA->required_dr == 0.0) continue;
				int MRUtype = 0; 
				int allocated =0;
				double RD = STA->required_dr;
				for (MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
				{
					if(RD <= MRUs_dr[MRUtype+1]) break;
				}
				for (int j = MRUtype; j < allocation_table.size(); j++)
				{
					
					for	 (int l = 0;l < allocation_table[j].size(); l++)
					{
						if (allocation_table[j][l] == 1) continue;
  						double En = 0.0; 
  						int min_mcs = 12;
  						vector<int> map_26;
						MRU_map_26(map_26,allocation_table,j,l);
						//cout <<"l = "<< l <<endl;
//						for (int num : map_26) {
//						   std::cout << num << " ";
//  						}
//  						std::cout << std::endl;	
  						for(int index = 0; index < map_26.size();index++)
  						{
							if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
//							cout <<"index = "<< index <<endl;	  
//  						cout <<"min_mcs = "<< min_mcs <<endl;
						}
						En = MRUs[j+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);	
//						cout <<"En = "<< En <<endl;
//						cout <<"RD = "<< RD <<endl;
						if (En >= RD)
						{
							STA->ana_tempTH = min(RD,En);
							//total_DR+=station_list[p][i].ana_tempTH;
							//cout << station_list[p][i-1].data_rate; 
							allocated = 1;					
							renew_allocation_table(allocation_table,ch,j,map_26);	
							map_26.clear(); 
							vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
							break;
						} 
						map_26.clear();
						vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
					}
					if(allocated ==1) break;
				}
				if(allocated != 1) non_critical.push_back(i);
			}
			else
			{
				non_critical.push_back(i);
			}
		}
		//second 
		int minMRUtype = 12;
		for (int i = 1; i <= non_critical.size(); i++)
		{
			Station *STA = &station_list[p][non_critical[i-1]];
    		if(STA->required_dr == 0.0) continue;
			double RD = STA->required_dr;	
			//cout <<"排程結果 = "<< RD <<endl;
			for (int MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
			{
				if(RD <= MRUs_dr[MRUtype+1])
				{
					if(minMRUtype > MRUtype)
					{
						minMRUtype = MRUtype;
					}
					break;
				}
			}
		}
		vector<int>	KMindex; //MRU location index 
		vector<int>	remainRU(allocation_table.size(),0);
		for (int i=0;i<allocation_table.size();i++)
		{
			for(int j=0;j<allocation_table[i].size();j++)
			{
				if(allocation_table[i][j] == 0)	remainRU[i]+=1;
			}
			//cout <<"MRUtype = "<< MRUs[i+1] <<", remain_RU = "<< remainRU[i] <<endl;
		}
		//cout <<"remain_RU = "<< remainRU[0] <<endl;
		int MRUtype;
		for (MRUtype = minMRUtype; MRUtype >0; MRUtype--)
		{
			if(non_critical.size()<= remainRU[MRUtype])	break;
		}		
		if(MRUtype == 12 || non_critical.size() == 0)
		{
			int RemainRU_26 = remainRU[0];
			remainRU.clear();
			vector<int>().swap(remainRU); 
			//return RemainRU_26;
		}
		for(int j=0;j<allocation_table[MRUtype].size();j++)
		{
			if(allocation_table[MRUtype][j] != 1) {
				KMindex.push_back(j);
			}
		}	
		vector<vector<float>> association_mat(non_critical.size());
		for (int i = 1; i <= non_critical.size(); i++)
		{
			Station *STA = &station_list[p][non_critical[i-1]];
    		if(STA->required_dr == 0.0) continue;
			double RD = STA->required_dr;
    		//cout << RD << endl; 
			for(int j=0;j<allocation_table[MRUtype].size();j++)
			{
				if(allocation_table[MRUtype][j] == 0)
				{
					int min_mcs = 12;
					vector<int> map_26;
					MRU_map_26(map_26,allocation_table,MRUtype,j);
					for(int index = 0; index < map_26.size();index++)
					{
						if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
						//cout <<"index = "<< index <<endl;	  
					}
					//cout <<"min_mcs = "<< min_mcs <<", MRUtype = "<< MRUtype <<", indexj = "<< j <<", ch = "<< ch <<", allocation_table[MRUtype].size() = "<< allocation_table[MRUtype].size() <<endl;
					double En = MRUs[MRUtype+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
					association_mat[i-1].push_back(min(RD,En));
					map_26.clear(); 
					vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
				}
			}
		}
		HungarianOptimizer<float> optimizer;
		std::vector<std::pair<size_t, size_t>> assignments;
		//cout <<"association_mat[0].size() = "<< association_mat[0].size() <<", association_mat[association_mat.size()-1].size() = "<< association_mat[association_mat.size()-1].size() <<endl;
		UpdateCosts(association_mat, optimizer.costs());
		// entry of hungarian optimizer maximize-weighted matching
		optimizer.Maximize(&assignments);
		for(int i=0;i<assignments.size();i++)
		{
	
			Station *STA = &station_list[p][non_critical[assignments[i].first]];

			station_list[p][non_critical[assignments[i].first]].ana_tempTH = association_mat[assignments[i].first][assignments[i].second];
			//total_DR+=station_list[p][non_critical[assignments[i].first]].ana_tempTH;
			cout << station_list[p][assignments[i].first].ana_tempTH << endl; 
			//cout <<"KMindex[assignments[i].first] = "<< KMindex[assignments[i].first] <<endl;
			//cout <<"KMindex[assignments[i].second] = "<< KMindex[assignments[i].second] <<endl;
			allocation_table[MRUtype][KMindex[assignments[i].second]] = 1;
			vector<int> map_26;
			MRU_map_26(map_26,allocation_table,MRUtype,KMindex[assignments[i].second]);
			renew_allocation_table(allocation_table,ch,MRUtype,map_26);	
			map_26.clear(); 
			vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
		} 		
		//Third
			
		for(int i=0;i<assignments.size();i++)
	  	{
	  		
	  		Station *STA = &station_list[p][non_critical[assignments[i].first]];
	  		double preEN = station_list[p][non_critical[assignments[i].first]].ana_tempTH;
	  		double RD = STA->required_dr;
	  		if(preEN>=RD) continue;
			//cout <<"preEN = "<< preEN <<endl;
			int CanReAllocate = 0;
			int index = KMindex[assignments[i].second];
			for(int type = MRUtype; type<allocation_table.size()-1; type++)
			{
				//cout <<"type = "<< type <<", index = "<< index <<", preEN = "<< preEN <<", RD = "<< RD <<endl;
				vector<int> map_26;
				MRU_map_26(map_26,allocation_table,type,index);
				sort(map_26.begin(),map_26.end());
	
				int check_ava =0;
					for(int location=0; location<allocation_table[type+1].size();location++)
					{
						vector<int> map_26_next;
						MRU_map_26(map_26_next,allocation_table,type+1,location);
						sort(map_26_next.begin(),map_26_next.end());
						vector<int> v_intersection;
						set_intersection(map_26.begin(), map_26.end(),map_26_next.begin(), map_26_next.end(),std::back_inserter(v_intersection));
						
	//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
	//					cout <<"map_26.size() = "<< v_intersection.size() <<endl;
	//					cout <<"map_26_next.size() = "<< v_intersection.size() <<endl;
						
						int check_index =0;
						int min_mcs=12;
						if(v_intersection.size() == 0 || v_intersection.empty()) continue;
						else check_ava = 1;
	
						vector<int> v_difference;
						set_difference(map_26_next.begin(), map_26_next.end(), v_intersection.begin(), v_intersection.end(), inserter(v_difference, v_difference.begin()));
						
	//					cout <<"check_index = "<< check_index <<endl;
	//					cout <<"map_26.size() = "<< map_26.size() <<endl;
	//					cout <<"map_26_next.size() = "<< map_26_next.size() <<endl;
	//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
	//					cout <<"v_difference.size() = "<< v_difference.size() <<endl;
						
						for(int l=0;l<v_difference.size();l++)
						{
							
							//cout <<"v_difference[l] = "<< v_difference[l] <<endl;
							if(allocation_table[0][v_difference[l]] == 1)
							{
								CanReAllocate =-1;
								break;
							}
							else
							{
								if(min_mcs > STA->MCS_A[v_difference[l]]) min_mcs = STA->MCS_A[v_difference[l]];	
								//cout <<"min_mcs?? = "<< min_mcs <<endl;
							}
						}
						if(CanReAllocate == -1) break;
						
						if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) > preEN)
						{
							station_list[p][non_critical[assignments[i].first]].ana_tempTH = min(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8),RD);
							renew_allocation_table(allocation_table,ch,type+1,map_26_next);	
							index = location;
							//cout <<"preEN = "<< preEN <<endl;
							preEN = MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
							if(preEN >=RD) CanReAllocate = 1;
							else CanReAllocate = 0;
							//cout <<"min_mcs = "<< min_mcs <<endl;
							//cout <<"MRUtype = "<< type+1 <<endl;
							//cout <<"preEN = "<< preEN <<endl;
							//cout <<"RD = "<< RD <<endl;
						}
						else if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) <= preEN) CanReAllocate == -1;
						map_26_next.clear(); 
						vector<int>().swap(map_26_next);
						v_intersection.clear(); 
						vector<int>().swap(v_intersection);
						v_difference.clear(); 
						vector<int>().swap(v_difference);
						break;
					}	
				//if(check_ava ==0) break;				
				map_26.clear(); 
				vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
				if(CanReAllocate == 0) continue;
				else if(CanReAllocate == 1 || CanReAllocate == -1) break;
			}
		}
		KMindex.clear();
		vector<int>().swap(KMindex);
		association_mat.clear();
		vector<vector<float>>().swap(association_mat);
		reOrderSTAs(p);
		
  		double sum_curflow = 0.0;
		for(int i = 0; i < station_list[p].size(); i++)
		{
			double padding = 0.0;
			Station *STA = &station_list[p][i];
			double th_avg_now =  STA->ana_tempTH; //STA->data_rate
			double arr = STA->required_dr;
			int tmp_th_now = min(static_cast<int>(round(5000 * STA->ana_tempTH)), STA->cur_data_size);//STA->data_rate
			double TH = tmp_th_now;
			int tempindex = STA->startIdx;
			
//			double curFlow = tmp_th_now/ MPDU_LENS[p];
//			int curN = curFlow / MPDU_LENS[p];
//			STA->ana_SN+=curN;
//			STA->ana_TH = curFlow;
			
			int sum_trans_dealy = 0.0;
			int total_dealy_time =0.0;
			while(tmp_th_now > 0 && tempindex < STA->packets.size())
			{
				
				if(tmp_th_now >= STA->packets[tempindex].packetSize)
				{
					tmp_th_now-=STA->packets[tempindex].packetSize;
					//STA->cur_data_size-=STA->packets[tempindex].packetSize;
					//STA->ana_SN += STA->packets[tempindex].packetSize;
					sum_trans_dealy+=double(STA->packets[tempindex].packetSize) / STA->ana_tempTH; //STA->data_rate
					//STA->total_dealy_time+= curTime + sum_trans_dealy - STA->packets[*startIdx].arrival_time;//最終目的，計算全部封包的總延遲時間 通過除以成功傳輸的次數success_trans
					STA->ana_SD += (curtime + sum_trans_dealy - STA->packets[tempindex].arrival_time)/1000;
					tempindex+=1;//此封包傳輸完成
					STA->ana_SN+=1;
					//STA->cur_suc_packet+=1;
				}
				else
				{
					TH-=tmp_th_now;
					tmp_th_now = 0;
				}
			}
			STA->ana_TH += TH/sim_time;
			
			
//			STA->ana_avgDR+=double(T) / sim_time *  STA->data_rate;
//			STA->ana_remainData+=(arr - STA->data_rate) * T / sim_time;
//			STA->ana_remainData-=double(STA->cur_expired_size)/sim_time;//為了與上一行正規化，需要乘上T/sim_time，同時也要除上T才能知道平均mus有多少過期封包bit和 
//			STA->ana_remainData = STA->ana_remainData < 0.0?0.0:STA->ana_remainData;			
//			STA->last_expired_size = STA->cur_expired_size;
			
			//STA->ana_avgDR+=double(T) / sim_time *  STA->data_rate;
			STA->ana_remainData+=(arr - STA->ana_TH) * T / sim_time;
			STA->ana_remainData-=double(STA->cur_expired_size)/sim_time;//為了與上一行正規化，需要乘上T/sim_time，同時也要除上T才能知道平均mus有多少過期封包bit和 
			STA->ana_remainData = STA->ana_remainData < 0.0?0.0:STA->ana_remainData;			
			STA->last_expired_size = STA->cur_expired_size;
			
			//usedTDR+= STA->data_rate;
			usedTDR+= STA->ana_TH;
			//cout << usedTDR << endl;
		}
	}
	last_T = T;
}


int AP::find_avg_length(int curTime)//這裡有問題 
{
    double l = 0.0;
    int c = 0;
    for(int p = 0; p < priority_num; p++)
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
    //cout << "傳輸時間 = " << t << endl; 
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
    		double tmp = double(STA->cur_data_size)/(STA->allocDRs[0][0] + STA->allocDRs[0][1]); //不要用cur_data_size，起碼要比例 EX: drA/(drA+drB) * cur_data_size
    		//cout << station_list[i].STA_ID<<" 理想傳輸時間/mus = "  << tmp << endl; 
    		if(isinf(tmp) || fabs(tmp) < 1e-15 || isnan(tmp)) continue;
        	l += tmp;
        	c+=1;
    	}
	}

    //cout << l << " " << c << endl;
    int t = round(l/c);
    //cout << t << endl;
    if(t>32768) t = 32768;
    if(t < 5000) t = 5000;
    if(curTime + t > sim_time) t = sim_time - curTime;
    //cout << "傳輸時間 = " << t << endl; 
    return t;
}


int AP::find_avg_len4MLO1(bool two_ch_mode,int curTime, int targetTime, int toleranceThs, int ch)// target time 記得考慮sifs + ack 
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

    //cout << l << " " << c << endl;
    int t = round(l/c);
    //cout << t << endl;
    if(t>32768) t = 32768;
    if(t < 5000) t = 5000;
    if(targetTime!=-1 && curTime + t + toleranceThs >= targetTime) t = targetTime - curTime;
    if(curTime + t > sim_time) t = sim_time - curTime;
    //cout <<"頻道 = "<< ch << ", 傳輸時間 = " << t << endl;
    return t;
}
void AP::print_info()
{
	double sum_dr = 0.0;
	cout << "列印STAs資訊"<<endl; 
	for(int p = 0; p < priority_num; p++)
	{
		for(int i = 0; i <station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			cout <<STA->STA_ID<<", "<<STA->allocDRs[0][0] << ", "<< STA->allocDRs[0][1] <<", "<<STA->allocDRs[1][0] << ", "<< STA->allocDRs[1][1]<< endl;
			cout << STA->STA_ID<<", "<<STA->data_rate << endl;
			sum_dr+=STA->data_rate;
		}
	}
	cout << "此次分配總共 DR = " << sum_dr << endl;
}
 

//實驗 OFDMA的duration 到底要不要寫死
// 

int AP::knaspack_sra(vector<vector<int>>& allocation_table,int curTime, int Bandwidth, bool two_ch_mode,int m,int p, int ch)
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
	        	double tmp_dr = min(MRUs[c]*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8), RD);
	        	if (ch == 0){
	        		double tmp_dr = min(MRUs[c]*STA->MCS_R[STA->minMCS_A][0]*STA->MCS_R[STA->minMCS_A][1]/(12.8+0.8), RD);
				}
				else{
	        		double tmp_dr = min(MRUs[c]*STA->MCS_R[STA->minMCS_B][0]*STA->MCS_R[STA->minMCS_B][1]/(12.8+0.8), RD);
				}
	            //double tmp_dr = min(MRUs_dr[c], RD);
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
	//cout <<"排程結果 = "<< alloc_result<<endl;
	int remain_BW = two_ch_mode?Bandwidth - allocDR(allocation_table,true,m,p,ch):Bandwidth - allocDR(allocation_table,false,-1,p,-1); // allocDR那邊要將DR ASSIGN給 STA的allocDRs
	//int remain_BW = Bandwidth - allocDR(false,-1,p,-1);
	reOrderSTAs(p);
	//cout <<"remain_BW = "<< remain_BW<<endl;
	return remain_BW;
}

void appendVector(std::vector<int>& target, std::vector<int>& source) {
  target.insert(target.end(), source.begin(), source.end());
}

void AP::MRU_map_26(vector<int>& map_26,vector<vector<int>>& allocation_table,int MRUtype, int location) {
	//cout << "RUtype = " << MRUtype <<endl;
	switch (MRUtype) {
  		case 0: //26
			map_26.push_back(location);
    	    break;
  		case 1: //52
    		if (location %4 >1)
    		{
    			location = location/4*9+location %4*2+1;
    			map_26.push_back(location);
    			map_26.push_back(location+1);
			}
			else
			{
				location = location/4*9+location %4*2;
				map_26.push_back(location);
    			map_26.push_back(location+1);
			}
    		break;
  		case 2: //52+26
  			MRU_map_26(map_26,allocation_table,1, MRUtable_52_26[location][0]);
  			MRU_map_26(map_26,allocation_table,0, MRUtable_52_26[location][1]);
    		break;
   		case 3: //106
    		if (location %2 >0)
    		{
    			location = location/2*9+location%2*4+1;
				for(int i=0;i<4;i++) map_26.push_back(location+i);
			}
			else
			{
    			location = location/2*9+location%2*4;
				for(int i=0;i<4;i++) map_26.push_back(location+i);
			}
    		break;
  		case 4: //106+26
  			MRU_map_26(map_26,allocation_table,3,MRUtable_106_26[location][0]);
  			MRU_map_26(map_26,allocation_table,0,MRUtable_106_26[location][1]);
		    break;	
  		case 5: //242
            location = location*9;
            for(int i=0;i<9;i++) map_26.push_back(location+i);
		    break;		
  		case 6: //484
            location = location*18;
            for(int i=0;i<18;i++) map_26.push_back(location+i);
		    break;		
  		case 7: //484+242
  			MRU_map_26(map_26,allocation_table,6,MRUtable_484_242[location][0]);
  			MRU_map_26(map_26,allocation_table,5,MRUtable_484_242[location][1]);	
		    break;		
  		case 8: //996
            location = location*36;
            for(int i=0;i<36;i++) map_26.push_back(location+i);
		    break;	
  		case 9: //996+484
  			MRU_map_26(map_26,allocation_table,8,MRUtable_996_484[location][0]);
  			MRU_map_26(map_26,allocation_table,6,MRUtable_996_484[location][1]);
			break;	
  		case 10: //2*996
            location = location*72;
            for(int i=0;i<72;i++) map_26.push_back(location+i);
		    break;		
  		case 11: //2*996+484
  			MRU_map_26(map_26,allocation_table,8,MRUtable_2_996_484[location][0]);
  			MRU_map_26(map_26,allocation_table,8,MRUtable_2_996_484[location][1]);
  			MRU_map_26(map_26,allocation_table,6,MRUtable_2_996_484[location][2]);		
			break;
	    case 12: //3*996
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996[location][0]);
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996[location][1]);
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996[location][2]);		
			break;		
	    case 13: //3*996+484
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996_484[location][0]);
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996_484[location][1]);
  			MRU_map_26(map_26,allocation_table,8,MRUtable_3_996_484[location][2]);
  			MRU_map_26(map_26,allocation_table,6,MRUtable_3_996_484[location][3]);
		
			break;		
  		case 14: //4*996
            location = location*144;
            for(int i=0;i<144;i++) map_26.push_back(location+i);
		    break;								   		
  		default:
   			// Code to execute if expression doesn't match any case
    		break;
	}
	//cout << "分配記憶體大小 = " <<  map_26.capacity() << endl;
//	for (int num : map_26) {
//		std::cout << num << " ";
//  	}
//  	std::cout << std::endl;	
//	cout << map_26.size() << endl;
//	cout << map_26[0] << endl;	
}

void AP::renew_allocation_table(vector<vector<int>>& allocation_table,int ch, int MRUtype, vector<int> map_26)
{
	for (int i=0;i<map_26.size();i++)
	{
		allocation_table[0][map_26[i]] = 1; //26
		if(map_26[i]%9<4) allocation_table[1][(map_26[i] - map_26[i]/9)/2] = 1; //52
		else if(map_26[i]%9>4) allocation_table[1][(map_26[i] - map_26[i]/9 -1)/2] = 1;

		
		if(map_26[i]%9<4) allocation_table[3][(map_26[i] - map_26[i]/9)/4] = 1; //106 
		else if(map_26[i]%9>4) allocation_table[3][(map_26[i] - map_26[i]/9 -1)/4] = 1;
		
		allocation_table[5][map_26[i]/9] = 1; //242
		allocation_table[6][map_26[i]/18] = 1; //484
		allocation_table[8][map_26[i]/36] = 1; //996
		allocation_table[10][map_26[i]/72] = 1; //2_996
		if (ch!=1)	allocation_table[14][0] = 1; //4_996
	}
	for(int j=0;j<allocation_table[2].size();j++) //52+26 
	{
		if (allocation_table[2][j]!=1)
		{
			for(int k=0;k<allocation_table[0].size();k++) 
			{
				if(allocation_table[0][k] == 1 && k == MRUtable_52_26[j][1])	allocation_table[2][j]=1;
			}
			for(int k=0;k<allocation_table[1].size();k++)
			{
				if(allocation_table[1][k] == 1 && k == MRUtable_52_26[j][0])	allocation_table[2][j]=1;
			}
		}
	} 
	for(int j=0;j<allocation_table[4].size();j++) //106+26 
	{
		if (allocation_table[4][j]!=1)
		{
			for(int k=0;k<allocation_table[0].size();k++)
			{
				if(allocation_table[0][k] == 1 && k == MRUtable_106_26[j][1])	allocation_table[4][j]=1;
			}
			for(int k=0;k<allocation_table[3].size();k++)
			{
				if(allocation_table[3][k] == 1 && k == MRUtable_106_26[j][0])	allocation_table[4][j]=1;
			}
		}
	} 
	for(int j=0;j<allocation_table[7].size();j++) //484_242 
	{
		if (allocation_table[7][j]!=1)
		{
			for(int k=0;k<allocation_table[5].size();k++)
			{
				if(allocation_table[5][k] == 1 && k == MRUtable_484_242[j][1])	allocation_table[7][j]=1;
			}
			for(int k=0;k<allocation_table[6].size();k++)
			{
				if(allocation_table[6][k] == 1 && k == MRUtable_484_242[j][0])	allocation_table[7][j]=1;
			}
		}
	} 
	for(int j=0;j<allocation_table[9].size();j++) //996_484 
	{
		if (allocation_table[9][j]!=1)
		{
			for(int k=0;k<allocation_table[6].size();k++)
			{
				if(allocation_table[6][k] == 1 && k == MRUtable_996_484[j][1])	allocation_table[9][j]=1;
			}
			for(int k=0;k<allocation_table[8].size();k++)
			{
				if(allocation_table[8][k] == 1 && k == MRUtable_996_484[j][0])	allocation_table[9][j]=1;
			}
		}
	} 
	if(ch!=1)
	{
		for(int j=0;j<allocation_table[11].size();j++) //2_996_484 
		{
			if (allocation_table[11][j]!=1)
			{
				for(int k=0;k<allocation_table[6].size();k++)
				{
					if(allocation_table[6][k] == 1 && k == MRUtable_2_996_484[j][2])	allocation_table[11][j]=1;
				}
				for(int k=0;k<allocation_table[8].size();k++)
				{
					if(allocation_table[8][k] == 1 && (k == MRUtable_2_996_484[j][0] || k == MRUtable_2_996_484[j][1]))	allocation_table[11][j]=1;
				}
			}
		} 
		for(int j=0;j<allocation_table[12].size();j++) //3_996
		{
			if (allocation_table[12][j]!=1)
			{
				for(int k=0;k<allocation_table[8].size();k++)
				{
					if(allocation_table[8][k] == 1 && (k == MRUtable_3_996[j][0] || k == MRUtable_3_996[j][1] || k == MRUtable_3_996[j][2]))	allocation_table[12][j]=1;
				}
			}
		} 
		for(int j=0;j<allocation_table[13].size();j++) //3_996_484
		{
			if (allocation_table[13][j]!=1)
			{
				for(int k=0;k<allocation_table[8].size();k++)
				{
					if(allocation_table[8][k] == 1 && (k == MRUtable_3_996_484[j][0] || k == MRUtable_3_996_484[j][1] || k == MRUtable_3_996_484[j][2]))	allocation_table[12][j]=1;
				}
				for(int k=0;k<allocation_table[6].size();k++)
				{
					if(allocation_table[6][k] == 1 && (k == MRUtable_3_996_484[j][3]))	allocation_table[13][j]=1;
				}
			}
		} 
	}
//	for (int num : map_26) {
//	   std::cout << num << " ";
// 	}
// 	std::cout << std::endl;	
}

void AP::UpdateCosts(const std::vector<std::vector<float>>& association_mat,
                 SecureMat<float>* costs) {
  size_t rows_size = association_mat.size();
  size_t cols_size = rows_size > 0 ? association_mat.at(0).size() : 0;

  costs->Resize(rows_size, cols_size);

  for (size_t row_idx = 0; row_idx < rows_size; ++row_idx) {
    for (size_t col_idx = 0; col_idx < cols_size; ++col_idx) {
      (*costs)(row_idx, col_idx) = association_mat.at(row_idx).at(col_idx);
    }
  }
}
int AP::Tzu(vector<vector<int>>& allocation_table,int curTime, int Bandwidth, bool two_ch_mode,int m,int p, int ch)
{
	vector<int> non_critical;
	double total_DR = 0.0;
	double sum = 0.0;
	for (int i = 1; i <= station_list[p].size(); i++)
	{
		//cout <<"allocation_table.size() = "<< allocation_table.size()  <<endl;
		Station *STA = &station_list[p][i-1];
		if(two_ch_mode && allocation_table.size() == 15 && STA->device == "SL") continue;
		if(two_ch_mode && station_list[p][i-1].allocDRs[m][ch] > 0.0) continue;

		if(STA->is_timecritical == true)
		{
			if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    	if(!two_ch_mode && STA->required_dr == 0.0) continue;
	    	
			int MRUtype = 0; 
			int allocated =0;
			double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
			cout << RD << endl;
			if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
			//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
			//cout <<"排程結果 = "<< RD <<endl;
			//cout <<"allocation_table.size() = "<< allocation_table.size() <<endl;
				for (MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
				{
					if(RD <= MRUs_dr[MRUtype+1]) break;
				}
				for (int j = MRUtype; j < allocation_table.size(); j++)
				{
					
					for	 (int l = 0;l < allocation_table[j].size(); l++)
					{
						if (allocation_table[j][l] == 1) continue;
  						double En = 0.0; 
  						int min_mcs = 12;
  						vector<int> map_26;
						MRU_map_26(map_26,allocation_table,j,l);
//						cout <<"l = "<< l <<endl;
//						for (int num : map_26) {
//						   std::cout << num << " ";
//  						}
//  						std::cout << std::endl;	
  						for(int index = 0; index < map_26.size();index++)
  						{
  							if (allocation_table.size() == 11) //ch==1
  							{
								if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
							}	  
  							else
							{
								if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
							}
							//cout <<"index = "<< index <<endl;	  
	 						//cout <<"min_mcs = "<< min_mcs <<endl;
						}
						En = MRUs[j+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);	
						//cout <<"En = "<< En <<endl;
						//cout <<"RD = "<< RD <<endl;
						if (En >= RD)
						{
							//station_list[p][i-1].allocDRs[m][ch] = min(RD,En);
							if(two_ch_mode)
							{
								if(allocation_table.size() == 11)	station_list[p][i-1].allocDRs[m][ch] =  min(RD,En);
								else station_list[p][i-1].allocDRs[m][ch] =  min(RD,En);
								//cout <<"station_list[p][i-1].allocDRs[m][ch]  = "<< station_list[p][i-1].allocDRs[m][ch]  <<endl;
								total_DR+= station_list[p][i-1].allocDRs[m][ch];
							} 
							else
							{
								station_list[p][i-1].data_rate = min(RD,En);
								total_DR+=station_list[p][i-1].data_rate;
								cout << station_list[p][i-1].data_rate << endl; 
							}
							allocated = 1;					
							renew_allocation_table(allocation_table,ch,j,map_26);	
							map_26.clear(); 
							vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
							break;
						} 
						map_26.clear();
						vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
					}
					if(allocated ==1) break;
				}
			if(allocated != 1) non_critical.push_back(i-1);
		}
		else if(STA->is_timecritical != true) 
		{
			non_critical.push_back(i-1);
		}
			
	}
	
	int minMRUtype = 12;
	for (int i = 1; i <= non_critical.size(); i++)
	{	
		Station *STA = &station_list[p][non_critical[i-1]];
		if(two_ch_mode && allocation_table.size() == 15 && STA->device == "SL") continue;
		if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    if(!two_ch_mode && STA->required_dr == 0.0) continue;
		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;	
		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
		//cout <<"排程結果 = "<< RD <<endl;
		for (int MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
		{
			if(RD <= MRUs_dr[MRUtype+1])
			{
				if(minMRUtype > MRUtype)
				{
					minMRUtype = MRUtype;
				}
				break;
			}
		}
	}
	//cout << "minMRUtype = "<< minMRUtype << endl;
	
	vector<int>	KMindex; //MRU location index 
	vector<int>	remainRU(allocation_table.size(),0);
	for (int i=0;i<allocation_table.size();i++)
	{
		for(int j=0;j<allocation_table[i].size();j++)
		{
			if(allocation_table[i][j] == 0)	remainRU[i]+=1;
		}
		//cout <<"MRUtype = "<< MRUs[i+1] <<", remain_RU = "<< remainRU[i] <<endl;
	}
	//cout <<"remain_RU = "<< remainRU[0] <<endl;
	int MRUtype;
	for (MRUtype = minMRUtype; MRUtype >0; MRUtype--)
	{
		if(non_critical.size()<= remainRU[MRUtype])	break;
	}
//	cout << "ch = "<< ch << endl;
//	cout << "station_list[p].size() = "<< station_list[p].size() << endl;
//	cout << "non_critical.size() = "<< non_critical.size() << endl;
//	cout << "MRUtype = "<< MRUtype << endl;
	
	if(MRUtype == 12 || non_critical.size() == 0)
	{
		int RemainRU_26 = remainRU[0];
		remainRU.clear();
		vector<int>().swap(remainRU); 
		return RemainRU_26;
	}
	
	for(int j=0;j<allocation_table[MRUtype].size();j++)
	{
		if(allocation_table[MRUtype][j] != 1) {
			KMindex.push_back(j);
		}
	}
	
	vector<vector<float>> association_mat(non_critical.size());
	
	for (int i = 1; i <= non_critical.size(); i++)
	{
		Station *STA = &station_list[p][non_critical[i-1]];
		if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    if(!two_ch_mode && STA->required_dr == 0.0) continue;
		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
	    cout << RD << endl; 
		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
		for(int j=0;j<allocation_table[MRUtype].size();j++)
		{
			if(allocation_table[MRUtype][j] == 0)
			{
				int min_mcs = 12;
				vector<int> map_26;
				MRU_map_26(map_26,allocation_table,MRUtype,j);
				for(int index = 0; index < map_26.size();index++)
  				{
  					if (allocation_table.size() == 11)
  					{
						if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
					}	  
  					else
					{
						if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
					}
					//cout <<"index = "<< index <<endl;	  
				}
				//cout <<"min_mcs = "<< min_mcs <<", MRUtype = "<< MRUtype <<", indexj = "<< j <<", ch = "<< ch <<", allocation_table[MRUtype].size() = "<< allocation_table[MRUtype].size() <<endl;
				double En = MRUs[MRUtype+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
				association_mat[i-1].push_back(min(RD,En));
				map_26.clear(); 
				vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
			}
		}
		//cout <<"i = "<< i <<endl;
	}
	//cout <<"index = "<< 1 <<endl;
	HungarianOptimizer<float> optimizer;
	std::vector<std::pair<size_t, size_t>> assignments;
	//cout <<"association_mat[0].size() = "<< association_mat[0].size() <<", association_mat[association_mat.size()-1].size() = "<< association_mat[association_mat.size()-1].size() <<endl;
	UpdateCosts(association_mat, optimizer.costs());
	// entry of hungarian optimizer maximize-weighted matching
  	optimizer.Maximize(&assignments);
  	
  	for (const auto& assignment : assignments) {
    	std::cout << "    (" << assignment.first << ", " << assignment.second << ")" << std::endl;
    	
  	}
	
  	for(int i=0;i<assignments.size();i++)
  	{
  		
  		Station *STA = &station_list[p][non_critical[assignments[i].first]];
  		if(two_ch_mode)
		{
			if(allocation_table.size() == 11)	station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] =  association_mat[assignments[i].first][assignments[i].second];
			else station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] =  association_mat[assignments[i].first][assignments[i].second];
			total_DR+= station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch];
		} 
		else
		{
			station_list[p][non_critical[assignments[i].first]].data_rate = association_mat[assignments[i].first][assignments[i].second];
			total_DR+=station_list[p][non_critical[assignments[i].first]].data_rate;
			cout << station_list[p][assignments[i].first].data_rate << endl; 
		}
		//cout <<"KMindex[assignments[i].first] = "<< KMindex[assignments[i].first] <<endl;
		//cout <<"KMindex[assignments[i].second] = "<< KMindex[assignments[i].second] <<endl;
		allocation_table[MRUtype][KMindex[assignments[i].second]] = 1;
		vector<int> map_26;
		MRU_map_26(map_26,allocation_table,MRUtype,KMindex[assignments[i].second]);
		renew_allocation_table(allocation_table,ch,MRUtype,map_26);	
		map_26.clear(); 
		vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
	}
	
	
	for(int i=0;i<assignments.size();i++)
  	{
  		
  		Station *STA = &station_list[p][non_critical[assignments[i].first]];
  		double preEN = two_ch_mode?station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch]:station_list[p][non_critical[assignments[i].first]].data_rate;
  		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
  		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
  		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
  		//cout <<"preEN = "<< preEN <<", RD = "<< RD <<endl;
  		if(preEN>=RD) continue;
  		
		//cout <<"preEN = "<< preEN <<endl;
		int CanReAllocate = 0;
		int index = KMindex[assignments[i].second];
		for(int type = MRUtype; type<allocation_table.size()-1; type++)
		{
			//cout <<"type = "<< type <<", index = "<< index <<", preEN = "<< preEN <<", RD = "<< RD <<endl;
			vector<int> map_26;
			MRU_map_26(map_26,allocation_table,type,index);
			sort(map_26.begin(),map_26.end());

			int check_ava =0;
				for(int location=0; location<allocation_table[type+1].size();location++)
				{
					vector<int> map_26_next;
					MRU_map_26(map_26_next,allocation_table,type+1,location);
					sort(map_26_next.begin(),map_26_next.end());
					vector<int> v_intersection;
					set_intersection(map_26.begin(), map_26.end(),map_26_next.begin(), map_26_next.end(),std::back_inserter(v_intersection));
					
//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
//					cout <<"map_26.size() = "<< v_intersection.size() <<endl;
//					cout <<"map_26_next.size() = "<< v_intersection.size() <<endl;
					
					int check_index =0;
					int min_mcs=12;
					if(v_intersection.size() == 0 || v_intersection.empty()) continue;
					else check_ava = 1;

					vector<int> v_difference;
					set_difference(map_26_next.begin(), map_26_next.end(), v_intersection.begin(), v_intersection.end(), inserter(v_difference, v_difference.begin()));
					
//					cout <<"check_index = "<< check_index <<endl;
//					cout <<"map_26.size() = "<< map_26.size() <<endl;
//					cout <<"map_26_next.size() = "<< map_26_next.size() <<endl;
//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
//					cout <<"v_difference.size() = "<< v_difference.size() <<endl;
					
					for(int l=0;l<v_difference.size();l++)
					{
						
						//cout <<"v_difference[l] = "<< v_difference[l] <<endl;
						if(allocation_table[0][v_difference[l]] == 1)
						{
							CanReAllocate =-1;
							break;
						}
						else
						{
							//cout <<"CanReAllocate = "<< CanReAllocate <<endl;
							if (allocation_table.size() == 11)
  							{
								if(min_mcs > STA->MCS_B[v_difference[l]])	min_mcs = STA->MCS_B[v_difference[l]];
							}	  
  							else
							{
								if(min_mcs > STA->MCS_A[v_difference[l]]) min_mcs = STA->MCS_A[v_difference[l]];	
								//cout <<"min_mcs?? = "<< min_mcs <<endl;
							}							
						}

						
					}
					if(CanReAllocate == -1) break;
					
					if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) > preEN)
					{
  						if(two_ch_mode)	station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] = min(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8),RD);
						else	station_list[p][non_critical[assignments[i].first]].data_rate = min(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8),RD);
						renew_allocation_table(allocation_table,ch,type+1,map_26_next);	
						index = location;
						//cout <<"preEN = "<< preEN <<endl;
						preEN = MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
						if(preEN >=RD) CanReAllocate = 1;
						else CanReAllocate = 0;
						//cout <<"min_mcs = "<< min_mcs <<endl;
						//cout <<"MRUtype = "<< type+1 <<endl;
						//cout <<"preEN = "<< preEN <<endl;
						//cout <<"RD = "<< RD <<endl;
					}
					else if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) <= preEN) CanReAllocate == -1;
					map_26_next.clear(); 
					vector<int>().swap(map_26_next);
					v_intersection.clear(); 
					vector<int>().swap(v_intersection);
					v_difference.clear(); 
					vector<int>().swap(v_difference);
					break;
				}	
			//if(check_ava ==0) break;				
			map_26.clear(); 
			vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
			if(CanReAllocate == 0) continue;
			else if(CanReAllocate == 1 || CanReAllocate == -1) break;
		}
	}
	
  	for (int i=0;i<allocation_table.size();i++)
	{
		remainRU[i]=0;
		for(int j=0;j<allocation_table[i].size();j++)
		{
			if(allocation_table[i][j] == 0)	remainRU[i]+=1;
		}
		//cout <<"MRUtype = "<< MRUs[i+1] <<", remain_RU = "<< remainRU[i] <<endl;
	}
	
  	
	
	int RemainRU_26 = remainRU[0];
	remainRU.clear();
	vector<int>().swap(remainRU); 
	KMindex.clear();
	vector<int>().swap(KMindex);
	association_mat.clear();
	vector<vector<float>>().swap(association_mat);
	reOrderSTAs(p);
	//cout <<"remain_RU_26 = "<< RemainRU_26 <<endl;
	return RemainRU_26;
}
int AP::TzuU(vector<vector<int>>& allocation_table,int curTime, int Bandwidth, bool two_ch_mode,int m,int p, int ch)
{
	vector<int> non_critical;
	double total_DR = 0.0;
	double sum = 0.0;
	for (int i = 1; i <= station_list[p].size(); i++)
	{
		//cout <<"allocation_table.size() = "<< allocation_table.size()  <<endl;
		Station *STA = &station_list[p][i-1];
		if(STA->device != "SL") continue;
		if(two_ch_mode && allocation_table.size() == 15 && STA->device == "SL") continue;
        
		if(STA->is_timecritical == true)
		{
			if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    	if(!two_ch_mode && STA->required_dr == 0.0) continue;
	    	
			int MRUtype = 0; 
			int allocated =0;
			double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
			if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
			//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
			//cout <<"排程結果 = "<< RD <<endl;
			//cout <<"allocation_table.size() = "<< allocation_table.size() <<endl;
				for (MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
				{
					if(RD <= MRUs_dr[MRUtype+1]) break;
				}
				for (int j = MRUtype; j < allocation_table.size(); j++)
				{
					
					for	 (int l = 0;l < allocation_table[j].size(); l++)
					{
						if (allocation_table[j][l] == 1) continue;
  						double En = 0.0; 
  						int min_mcs = 12;
  						vector<int> map_26;
						MRU_map_26(map_26,allocation_table,j,l);
//						cout <<"l = "<< l <<endl;
//						for (int num : map_26) {
//						   std::cout << num << " ";
//  						}
//  						std::cout << std::endl;	
  						for(int index = 0; index < map_26.size();index++)
  						{
  							if (allocation_table.size() == 11) //ch==1
  							{
								if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
							}	  
  							else
							{
								if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
							}
							//cout <<"index = "<< index <<endl;	  
	 						//cout <<"min_mcs = "<< min_mcs <<endl;
						}
						En = MRUs[j+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);	
						//cout <<"En = "<< En <<endl;
						//cout <<"RD = "<< RD <<endl;
						if (En >= RD)
						{
							//station_list[p][i-1].allocDRs[m][ch] = min(RD,En);
							if(two_ch_mode)
							{
								if(allocation_table.size() == 11)	station_list[p][i-1].allocDRs[m][ch] =  min(RD,En);
								else station_list[p][i-1].allocDRs[m][ch] =  min(RD,En);
								//cout <<"station_list[p][i-1].allocDRs[m][ch]  = "<< station_list[p][i-1].allocDRs[m][ch]  <<endl;
								total_DR+= station_list[p][i-1].allocDRs[m][ch];
							} 
							else
							{
								station_list[p][i-1].data_rate = min(RD,En);
								total_DR+=station_list[p][i-1].data_rate;
								//cout << station_list[p][i-1].data_rate << endl; 
							}
							allocated = 1;					
							renew_allocation_table(allocation_table,ch,j,map_26);	
							map_26.clear(); 
							vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
							break;
						} 
						map_26.clear();
						vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
					}
					if(allocated ==1) break;
				}
			if(allocated != 1 && STA->device == "SL") non_critical.push_back(i-1);
		}
		else
		{
			if(STA->device == "SL") non_critical.push_back(i-1);
		}
			
	}
	
	int minMRUtype = 12;
	for (int i = 1; i <= non_critical.size(); i++)
	{	
		Station *STA = &station_list[p][non_critical[i-1]];
		if(two_ch_mode && allocation_table.size() == 15 && STA->device == "SL") continue;
		if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    if(!two_ch_mode && STA->required_dr == 0.0) continue;
		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;	
		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
		//cout <<"排程結果 = "<< RD <<endl;
		for (int MRUtype = 0; MRUtype < allocation_table.size(); MRUtype++)
		{
			if(RD <= MRUs_dr[MRUtype+1])
			{
				if(minMRUtype > MRUtype)
				{
					minMRUtype = MRUtype;
				}
				break;
			}
		}
	}
	//cout << "minMRUtype = "<< minMRUtype << endl;
	
	vector<int>	KMindex; //MRU location index 
	vector<int>	remainRU(allocation_table.size(),0);
	for (int i=0;i<allocation_table.size();i++)
	{
		for(int j=0;j<allocation_table[i].size();j++)
		{
			if(allocation_table[i][j] == 0)	remainRU[i]+=1;
		}
		//cout <<"MRUtype = "<< MRUs[i+1] <<", remain_RU = "<< remainRU[i] <<endl;
	}
	//cout <<"remain_RU = "<< remainRU[0] <<endl;
	int MRUtype;
	for (MRUtype = minMRUtype; MRUtype >0; MRUtype--)
	{
		if(non_critical.size()<= remainRU[MRUtype])	break;
	}
//	cout << "ch = "<< ch << endl;
//	cout << "station_list[p].size() = "<< station_list[p].size() << endl;
//	cout << "non_critical.size() = "<< non_critical.size() << endl;
//	cout << "MRUtype = "<< MRUtype << endl;
	
	if(MRUtype == 12 || non_critical.size() == 0)
	{
		int RemainRU_26 = remainRU[0];
		remainRU.clear();
		vector<int>().swap(remainRU); 
		return RemainRU_26;
	}
	
	for(int j=0;j<allocation_table[MRUtype].size();j++)
	{
		if(allocation_table[MRUtype][j] != 1) {
			KMindex.push_back(j);
		}
	}
	
	vector<vector<float>> association_mat(non_critical.size());
	
	for (int i = 1; i <= non_critical.size(); i++)
	{
		Station *STA = &station_list[p][non_critical[i-1]];
		if(two_ch_mode && STA->requiredDRs[m][ch] == 0.0) continue;
	    if(!two_ch_mode && STA->required_dr == 0.0) continue;
		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;	
		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
		for(int j=0;j<allocation_table[MRUtype].size();j++)
		{
			if(allocation_table[MRUtype][j] == 0)
			{
				int min_mcs = 12;
				vector<int> map_26;
				MRU_map_26(map_26,allocation_table,MRUtype,j);
				for(int index = 0; index < map_26.size();index++)
  				{
  					if (allocation_table.size() == 11)
  					{
						if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
					}	  
  					else
					{
						if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
					}
					//cout <<"index = "<< index <<endl;	  
				}
				//cout <<"min_mcs = "<< min_mcs <<", MRUtype = "<< MRUtype <<", indexj = "<< j <<", ch = "<< ch <<", allocation_table[MRUtype].size() = "<< allocation_table[MRUtype].size() <<endl;
				double En = MRUs[MRUtype+1]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
				association_mat[i-1].push_back(min(RD,En));
				map_26.clear(); 
				vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
			}
		}
		//cout <<"i = "<< i <<endl;
	}
	//cout <<"index = "<< 1 <<endl;
	HungarianOptimizer<float> optimizer;
	std::vector<std::pair<size_t, size_t>> assignments;
	//cout <<"association_mat[0].size() = "<< association_mat[0].size() <<", association_mat[association_mat.size()-1].size() = "<< association_mat[association_mat.size()-1].size() <<endl;
	UpdateCosts(association_mat, optimizer.costs());
	// entry of hungarian optimizer maximize-weighted matching
  	optimizer.Maximize(&assignments);
  	
//  	for (const auto& assignment : assignments) {
//    	std::cout << "    (" << assignment.first << ", " << assignment.second << ")" << std::endl;
//    	
//  	}
	
  	for(int i=0;i<assignments.size();i++)
  	{
  		
  		Station *STA = &station_list[p][non_critical[assignments[i].first]];
  		if(two_ch_mode)
		{
			if(allocation_table.size() == 11)	station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] =  association_mat[assignments[i].first][assignments[i].second];
			else station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] =  association_mat[assignments[i].first][assignments[i].second];
			total_DR+= station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch];
		} 
		else
		{
			station_list[p][non_critical[assignments[i].first]].data_rate = association_mat[assignments[i].first][assignments[i].second];
			total_DR+=station_list[p][non_critical[assignments[i].first]].data_rate;
			//cout << station_list[p][assignments[i].first].data_rate << endl; 
		}
		//cout <<"KMindex[assignments[i].first] = "<< KMindex[assignments[i].first] <<endl;
		//cout <<"KMindex[assignments[i].second] = "<< KMindex[assignments[i].second] <<endl;
		allocation_table[MRUtype][KMindex[assignments[i].second]] = 1;
		vector<int> map_26;
		MRU_map_26(map_26,allocation_table,MRUtype,KMindex[assignments[i].second]);
		renew_allocation_table(allocation_table,ch,MRUtype,map_26);	
		map_26.clear(); 
		vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
	}
	
	
	for(int i=0;i<assignments.size();i++)
  	{
  		
  		Station *STA = &station_list[p][non_critical[assignments[i].first]];
  		double preEN = two_ch_mode?station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch]:station_list[p][non_critical[assignments[i].first]].data_rate;
  		double RD = two_ch_mode?STA->requiredDRs[m][ch]:STA->required_dr;
  		if(allocation_table.size() == 11 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][0];
  		//if(allocation_table.size() == 15 && two_ch_mode && STA->device != "SL") RD-=STA->allocDRs[m][1];
  		//cout <<"preEN = "<< preEN <<", RD = "<< RD <<endl;
  		if(preEN>=RD) continue;
  		
		//cout <<"preEN = "<< preEN <<endl;
		int CanReAllocate = 0;
		int index = KMindex[assignments[i].second];
		for(int type = MRUtype; type<allocation_table.size()-1; type++)
		{
			//cout <<"type = "<< type <<", index = "<< index <<", preEN = "<< preEN <<", RD = "<< RD <<endl;
			vector<int> map_26;
			MRU_map_26(map_26,allocation_table,type,index);
			sort(map_26.begin(),map_26.end());

			int check_ava =0;
				for(int location=0; location<allocation_table[type+1].size();location++)
				{
					vector<int> map_26_next;
					MRU_map_26(map_26_next,allocation_table,type+1,location);
					sort(map_26_next.begin(),map_26_next.end());
					vector<int> v_intersection;
					set_intersection(map_26.begin(), map_26.end(),map_26_next.begin(), map_26_next.end(),std::back_inserter(v_intersection));
					
//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
//					cout <<"map_26.size() = "<< v_intersection.size() <<endl;
//					cout <<"map_26_next.size() = "<< v_intersection.size() <<endl;
					
					int check_index =0;
					int min_mcs=12;
					if(v_intersection.size() == 0 || v_intersection.empty()) continue;
					else check_ava = 1;

					vector<int> v_difference;
					set_difference(map_26_next.begin(), map_26_next.end(), v_intersection.begin(), v_intersection.end(), inserter(v_difference, v_difference.begin()));
					
//					cout <<"check_index = "<< check_index <<endl;
//					cout <<"map_26.size() = "<< map_26.size() <<endl;
//					cout <<"map_26_next.size() = "<< map_26_next.size() <<endl;
//					cout <<"v_intersection.size() = "<< v_intersection.size() <<endl;
//					cout <<"v_difference.size() = "<< v_difference.size() <<endl;
					
					for(int l=0;l<v_difference.size();l++)
					{
						
						//cout <<"v_difference[l] = "<< v_difference[l] <<endl;
						if(allocation_table[0][v_difference[l]] == 1)
						{
							CanReAllocate =-1;
							break;
						}
						else
						{
							//cout <<"CanReAllocate = "<< CanReAllocate <<endl;
							if (allocation_table.size() == 11)
  							{
								if(min_mcs > STA->MCS_B[v_difference[l]])	min_mcs = STA->MCS_B[v_difference[l]];
							}	  
  							else
							{
								if(min_mcs > STA->MCS_A[v_difference[l]]) min_mcs = STA->MCS_A[v_difference[l]];	
								//cout <<"min_mcs?? = "<< min_mcs <<endl;
							}							
						}

						
					}
					if(CanReAllocate == -1) break;
					
					if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) > preEN)
					{
  						if(two_ch_mode)	station_list[p][non_critical[assignments[i].first]].allocDRs[m][ch] = min(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8),RD);
						else	station_list[p][non_critical[assignments[i].first]].data_rate = min(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8),RD);
						renew_allocation_table(allocation_table,ch,type+1,map_26_next);	
						index = location;
						//cout <<"preEN = "<< preEN <<endl;
						preEN = MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);
						if(preEN >=RD) CanReAllocate = 1;
						else CanReAllocate = 0;
						//cout <<"min_mcs = "<< min_mcs <<endl;
						//cout <<"MRUtype = "<< type+1 <<endl;
						//cout <<"preEN = "<< preEN <<endl;
						//cout <<"RD = "<< RD <<endl;
					}
					else if(MRUs[type+2]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8) <= preEN) CanReAllocate == -1;
					map_26_next.clear(); 
					vector<int>().swap(map_26_next);
					v_intersection.clear(); 
					vector<int>().swap(v_intersection);
					v_difference.clear(); 
					vector<int>().swap(v_difference);
					break;
				}	
			//if(check_ava ==0) break;				
			map_26.clear(); 
			vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
			if(CanReAllocate == 0) continue;
			else if(CanReAllocate == 1 || CanReAllocate == -1) break;
		}
	}
	
  	for (int i=0;i<allocation_table.size();i++)
	{
		remainRU[i]=0;
		for(int j=0;j<allocation_table[i].size();j++)
		{
			if(allocation_table[i][j] == 0)	remainRU[i]+=1;
		}
		//cout <<"MRUtype = "<< MRUs[i+1] <<", remain_RU = "<< remainRU[i] <<endl;
	}
  	
	
	int RemainRU_26 = remainRU[0];
	remainRU.clear();
	vector<int>().swap(remainRU); 
	KMindex.clear();
	vector<int>().swap(KMindex);
	association_mat.clear();
	vector<vector<float>>().swap(association_mat);
	
	reOrderSTAs(p);
	//cout <<"remain_RU_26 = "<< RemainRU_26 <<endl;
	return RemainRU_26;
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


void AP::twoChUsersAlloc(int priority_num, bool isTzuFunc)
{
	//建立2*4*2的vec，用於統計當前分配Data Rate;2 mlo, priority_num pri, 2 ch
	vector<vector<vector<double>>> usedRDs(2, vector<vector<double>>(priority_num, vector<double>(2, 0.0)));
	for(int p = 0; p < priority_num; p++)
	{
		for(int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(isTzuFunc)
			{
				if(STA->device == "SL")
				{
					//STA->requiredDRs[0][1] = STA->required_dr;
					STA->requiredDRs[1][1] = STA->required_dr;
				}
				else
				{
					//STA->requiredDRs[0][0] = STA->required_dr;
					//STA->requiredDRs[0][1] = STA->required_dr;
					STA->requiredDRs[1][0] = STA->required_dr;
					STA->requiredDRs[1][1] = STA->required_dr;
				}
			}

			if(STA->device != "SL" && !isTzuFunc)// MLD
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
			else if(STA->device == "SL" && !isTzuFunc)
			{
				STA->requiredDRs[0][1] = STA->required_dr;
				usedRDs[0][p][1]+=STA->requiredDRs[0][1];
				STA->requiredDRs[1][1] = STA->required_dr;
				usedRDs[1][p][1]+=STA->requiredDRs[1][1];										
			}
			//cout <<"MLO = 0, ch A,B = "<<STA->requiredDRs[0][0] << " ," <<STA->requiredDRs[0][1]<<endl;
			//cout <<"MLO = 1, ch A,B = "<<STA->requiredDRs[1][0] << " ," <<STA->requiredDRs[1][1]<<endl;  
						
		}
	}
	 
}
/*
int AP::knaspack_sra(int curTime, int Bandwidth, int p) //2d DP 
{
	//pair <int, map<int, double>> element; 收益,MRU配置  
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
				//cout << "j = " << j << ", 分配給前i-1個用戶 = "<<  j - MRUsToIdx[MRUs[c]] <<", 分配第i個用戶 = "<<MRUsToIdx[MRUs[c]]<<endl; 
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
	cout <<"排程結果 = "<< alloc_result<<endl;
	int remain_BW = Bandwidth - allocDR(p);
	return remain_BW;
	
}*/

void AP::transmit2STAs(int curTime, int transTime)
{
	for(int p = 0; p < priority_num; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			STA->cur_suc_packet = 0;
			if(fabs(STA->data_rate) < 1e-15) continue;
			int throughput = min(static_cast<int>(round(transTime * STA->data_rate)), STA->cur_data_size);
			STA->cur_sum_DR+=throughput;
			//cout << "可傳吞吐量 = " << throughput<<", idx = "<< station_list[p][i].startIdx << endl;
			int* startIdx = &STA->startIdx;
			double sum_trans_dealy = 0.0;
			while(throughput > 0 && *startIdx < STA->packets.size())
			{
				
				if(throughput >= STA->packets[*startIdx].packetSize)
				{
					throughput-=STA->packets[*startIdx].packetSize;
					STA->cur_data_size-=STA->packets[*startIdx].packetSize;
					sum_trans_dealy+=double(STA->packets[*startIdx].packetSize) / STA->data_rate;
					STA->total_dealy_time+= curTime + sum_trans_dealy - STA->packets[*startIdx].arrival_time;//最終目的，計算全部封包的總延遲時間 通過除以成功傳輸的次數success_trans
					*startIdx+=1;//此封包傳輸完成
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
	//dealy, success_trans_nth 改為根據m的1d vec 
	for(int p = 0; p < priority_num; p++)
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
			//目前看下來 startIdxs,cur_data_sizes, delays, success_trans_nth 都只需要根據 mlo即可 
			//cout << "可傳吞吐量 = " << throughput<<", idx = "<< station_list[p][i].startIdx << endl; 
			while(throughput > 0 && *startIdx < STA->packets.size())
			{
				
				if(throughput >= *curPS)//可以傳完整個封包 
				{
					throughput-=*curPS;
					STA->cur_data_sizes[m]-=*curPS;
					sum_trans_dealy+= double(*curPS) / (STA->allocDRs[m][0] + STA->allocDRs[m][1]);
					STA->delays[m]+= curTime + sum_trans_dealy \
					- STA->packets[*startIdx].arrival_time;//最終目的，計算全部封包的總延遲時間 通過除以成功傳輸的次數success_trans
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
				if(throughput >= *curPS)//可以傳完整個封包 
				{
					throughput-=*curPS;
					STA->cur_data_sizes[m]-=*curPS;
					sum_trans_dealy+=double(*curPS) / STA->allocDRs[m][extraCh];
					STA->delays[m]+= curTime + commonTime + sum_trans_dealy\
					- STA->packets[*startIdx].arrival_time;//最終目的，計算全部封包的總延遲時間 通過除以成功傳輸的次數success_trans
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
	for(int p = 0; p < priority_num; p++)
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
	if(method == 0 || method == 2)
	{
		for(int p = 0; p < priority_num; p++)
		{
			sort(station_list[p].begin(),station_list[p].end(),Station::compareBySTAID);
		}
	}
	else if(method == 1)
	{
		for(int p = 0; p < priority_num; p++)
		{
			sort(station_list[p].begin(),station_list[p].end(),Station::compareByRD);
		}
	}
	
}
//要寫個function alloc dr - rd 
void AP::opt_filter()
{
	for(int p = 0; p < priority_num; p++)
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

int AP::opt_RCL(vector<vector<int>>& allocation_table,int Bandwidth, bool isCHA, bool two_ch_mode, bool two_ch)//two_ch指的是做雙頻道的實驗， two_ch_mode則是獲取同時兩個頻道 
{
	int ch = isCHA? 0:1;
	int Remain_26 =0;
	for(int p = 0; p < priority_num; p++)
	{
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Remain_26 =0;
			Station* STA = &station_list[p][i];
			if(two_ch && ch == 0 && STA->device == "SL") continue; 
			if(!isCHA && MRUs[STA->MRU_idx] > 2*996) continue;
		
			double En =0.0;
			double RD = two_ch_mode?STA->requiredDRs[1][ch]:STA->required_dr;
			int Canallocate = 0;
			if(STA->MRU_idx == 0) continue;
			for(int j=0; j<allocation_table[(STA->MRU_idx)-1].size();j++)
			{
				if(allocation_table[(STA->MRU_idx)-1][j] == 0)
				{
					//cout <<"STA->MRU_idx = "<< STA->MRU_idx -1 <<endl;
					int min_mcs =12;
					vector<int> map_26;
					MRU_map_26(map_26,allocation_table,STA->MRU_idx-1,j);
					for(int index = 0; index < map_26.size();index++)
  					{
  						if (ch ==1)
  						{
							if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
						}	  
  						else
						{
							if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
						}
						//cout <<"index = "<< index <<endl;	  
  				    	//cout <<"min_mcs = "<< min_mcs <<endl;
					}
					//cout <<"min_mcs = "<< min_mcs <<endl;
					En = MRUs[STA->MRU_idx]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);				
					renew_allocation_table(allocation_table,ch,STA->MRU_idx-1,map_26);	
					map_26.clear(); 
					vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
					Canallocate = 1;
					break;
				} 		
			}
			if(Canallocate == 0) continue;
			
			if(two_ch_mode){
				//STA->allocDRs[1][ch] = MRUs_dr[STA->MRU_idx];
				if(ch == 1)	STA->allocDRs[1][ch] = min(RD,En);
				else STA->allocDRs[1][ch] = min(RD,En);
			}
			else{
				//STA->data_rate = MRUs_dr[STA->MRU_idx];
				STA->data_rate = min(RD,En);
			}
			cout << station_list[p][i].STA_ID<<", RD= "<< RD<<", 使用26-tone 數量 = "  << MRUsToIdx[MRUs[STA->MRU_idx]] << ", 換成MRU = "<< MRUs[STA->MRU_idx] << ", 資料速率 = "<< En << endl; 
			
			for(int j=0; j<allocation_table[0].size(); j++)
			{
				if(allocation_table[0][j] ==0) Remain_26++;
			}
			if(Remain_26 == 0) return Remain_26;
			
		}
	}
	return Remain_26;
}

int AP::opt_FGC(vector<vector<int>>& allocation_table,int Remain_26, bool isCHA, bool two_ch_mode, bool two_ch)
{
	int ch = isCHA? 0:1;
	for(int p = 0; p < priority_num; p++)
	{
		//if(p==2) continue;
		for (int i = 0; i < station_list[p].size(); i++)
		{
			Station* STA = &station_list[p][i];
			if(two_ch && ch == 0 && STA->device == "SL") continue; 
			if(MRUs[STA->MRU_idx] <= 106)// small size mru
			{
				int compensate = min(5 - STA->MRU_idx,Remain_26);
				Remain_26-=compensate;
				if(Remain_26 == 0) return 0;
				STA->MRU_idx+=compensate;
				//STA->data_rate = MRUs_dr[STA->MRU_idx];
				double En =0.0;
				double RD = two_ch_mode?STA->requiredDRs[1][ch]:STA->required_dr;
				int Canallocate = 0;
				if(STA->MRU_idx == 0) continue;
				for(int j=0; j<allocation_table[(STA->MRU_idx)-1].size();j++)
				{
					if(allocation_table[(STA->MRU_idx)-1][j] == 0)
					{
						cout <<"STA->MRU_idx = "<< STA->MRU_idx -1 <<endl;
						int min_mcs =12;
						vector<int> map_26;
						MRU_map_26(map_26,allocation_table,STA->MRU_idx-1,j);
						for(int index = 0; index < map_26.size();index++)
  						{
  							if (ch ==1)
  							{
								if(min_mcs > STA->MCS_B[index])	min_mcs = STA->MCS_B[index];
							}	  
  							else
							{
								if(min_mcs > STA->MCS_A[index]) min_mcs = STA->MCS_A[index];	
							}
							//cout <<"index = "<< index <<endl;	  
  				    		//cout <<"min_mcs = "<< min_mcs <<endl;
						}
						cout <<"min_mcs = "<< min_mcs <<endl;
						En = MRUs[STA->MRU_idx]*STA->MCS_R[min_mcs][0]*STA->MCS_R[min_mcs][1]/(12.8+0.8);				
						renew_allocation_table(allocation_table,ch,STA->MRU_idx-1,map_26);	
						map_26.clear(); 
						vector<int>().swap(map_26);//釋放記憶體空間，非常重要因為map會一直增長空間最後導致無法執行
						Canallocate = 1;
						break;
					} 		
				}
				if(Canallocate == 0) continue;
				if(two_ch_mode){
					//STA->allocDRs[1][ch] = MRUs_dr[STA->MRU_idx];
					if(ch == 1)	STA->allocDRs[1][ch] = min(En,RD);
					else STA->allocDRs[1][ch] =  min(En,RD);
				}
				else{
					//STA->data_rate = MRUs_dr[STA->MRU_idx];
					STA->data_rate =  min(En,RD);
				}
				cout << station_list[p][i].STA_ID<<", RD= "<< RD<<", 使用26-tone 數量 = "  << MRUsToIdx[MRUs[STA->MRU_idx]] << ", 換成MRU = "<< MRUs[STA->MRU_idx] << ", 資料速率 = "<< min(En,RD) << endl; 
			}
			Remain_26 =0;
			for(int j=0; j<allocation_table[0].size(); j++)
			{
				if(allocation_table[0][j] ==0) Remain_26++;
			}
			if(Remain_26 == 0) return Remain_26;
		}
	}
	return Remain_26;
}
