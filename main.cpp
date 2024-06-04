#include <iostream>
#include <fstream>
#include <vector>
#include "ap.h"
#include "scheduler.h"
#include <exception> 
using namespace std;
int D = 5;//����ɶ� ��� �� 
int MILLION = 1000000;
int T = 1;//�C�ӹ��簵�X�� 
const vector<double> SL_STR_NSTR_RATIO = {0.2,0.4,0.4}; //�U�˸m��� 
const vector<int> TRAFFIC_ARRIVAL_RATES = vector<int>{20,160,300,100};// �U�u���ų�ӥΤ᪺�ʥ]�y�q 
const vector<int> MPDU_LENS = vector<int>{200,1000,500,500}; // �U�u���ūʥ]���׳�� 
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
	int Method = 0;//��k���  0:MRU SRAs, 1:OPT, 2:TMS, 3:EOASYNC + OPT 
	double alpha = 2.75;
	double sim_time = MILLION * D; //�����ɶ� ���mu s (�ʸU�����@) 
	
	vector<vector<double>> STAs_TH(4);//�Ҧ��˸m���]�R�q���� 
	vector<vector<double>> STAs_D(4);//�Ҧ��˸m���������
	vector<double> P_TH(4);
	vector<double> P_D(4);
	
	//vector<vector<double>> alpha_TH(20,vector<double>(4,0.0));
	//vector<vector<double>> alpha_D(20,vector<double>(4,0.0));
	
	vector<int> VAR_PRI_PEOPLE = vector<int>{1,1,1,1};//�_�l�U�u���ťΤ�ƶq 
	//vector<int> VAR_TRAFFIC_ARRIVAL_RATES = vector<int>{10,80,150,50};
	
	int initPN = 4;//�_�l�Τ�ƶq 
	int finPN = 60;//�̲ץΤ�ƶq 
	int initAlpha = alpha;
	int finAlpha = alpha + 4;
	int round4VarP = (finPN - initPN)/4 + 1;//finPN - initPN)/20 + 1 <- �Ω����W�D 
	int round4VarAlpha = (finAlpha - initAlpha)*4 + 1;//�Ω��ܰ�alpha 
	vector<vector<double>> VAR_STA_TH(round4VarP,vector<double>(4,0.0));//20~100�@9�� �]�R�q 
	vector<vector<double>> VAR_STA_MTH(round4VarP,vector<double>(4,0.0));//Math  
	vector<vector<double>> VAR_STA_MD(round4VarP,vector<double>(4,0.0));//Math 
	vector<vector<double>> VAR_STA_D(round4VarP,vector<double>(4,0.0));//���� 
	
	vector<vector<double>> VAR_STA_packet_lr(round4VarP,vector<double>(4,0.0));//�ʥ]�򥢲v 
	vector<vector<double>> VAR_STA_chs_util(round4VarP,vector<double>(2,0.0));//�W�D�Q�βv 
	
	
	vector<vector<double>> DEV_TH(4,vector<double>(3,0.0)); // 4��pri, 3�Ӹ˸m���� 
	vector<vector<double>> DEV_D(4,vector<double>(3,0.0)); // 4��pri, 3�Ӹ˸m����
	
	
		
	
	double total_avg_th = 0.0;
	for(int num = 0; num < round4VarP; num++)//����i�榸�� 
	{
		for(int t = 0; t < T; t++)//�C�ӹ��筫��T�������� 
		{
			cout << "alpha = " << alpha << endl;
			Scheduler scheduler = Scheduler(sim_time);
			scheduler.generate_traffic(SL_STR_NSTR_RATIO,VAR_PRI_PEOPLE, TRAFFIC_ARRIVAL_RATES, MPDU_LENS, DEADLINES);//�ͦ��ʥ] 
			//�}�l�Ƶ{ 
			if(Method < 2) scheduler.schedule_access(Method,alpha);
			else scheduler.schedule_access2CH(Method,alpha);
			//�Ƶ{����		
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
			//�|�N�Ҧ��Τ�A�p��Ƶ{�����᪺��T �p:�]�R�q�B���� 
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
					double STA_dealy = STA->total_dealy_time / (STA->success_trans * 1000); //�qmus to ms 
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

					
					
					cout << "STA ID:"<< STA->STA_ID <<", �ʥ]�`�� = " << STA->packets.size() << ", success transmission = " << STA->success_trans<<", �]�R�q = "<< STA_TH  <<", ���� = "<< STA_dealy  << endl;
					cout << "�ʥ]�򥢲v = " << (double)STA->n_expired_packet / STA->packets.size() << ", �bA�W�D�ǿ�ƶq = " << STA->n_suc_packet_chA <<", �bB�W�D�ǿ�ƶq = "<< STA->n_suc_packet_chB  << endl;
					cout << "Device type = " << STA->device << endl;
					if(Method == 0) cout << "�ƾǤ��R, �]�R�q = " << STA->ana_TH << ", �������� = " << STA_ana_D  <<", success transmission = " <<STA->ana_SN<<endl;
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
							
				cout <<"�u���ŧO = "<< 4-p <<", �`�]�R�q/Mbits = "<<p_throughput << endl;
				cout <<"�u���ŧO = "<< 4-p << ", �����Τ᩵��/ms = " << p_dealy << endl;
				total_avg_th+=p_throughput/T;
				total_th+=p_throughput;
			}
			
			cout << "��e�^�X �`�]�R�q = " <<  total_th << endl;  
		}

		cout <<  "����320 MHz�W�D�Q�βv = " <<  VAR_STA_chs_util[num][0] << endl;
		cout <<  "����160 MHz�W�D�Q�βv = " <<  VAR_STA_chs_util[num][1] << endl;
		
		
		 
		 
		for(int p = 0; p < 4; p++)
		{
			VAR_PRI_PEOPLE[p]+=1;//�Y�ϥγ��W�D �A�Y�ϥ����W�D �h+5 
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
	//�Ҧ���Ƴ��|�g�ɦb�o�� 
	if(Method < 2){
		string file = "���絲�G1ch/";

		writeData(Method,file,"VAR_STA_TH", VAR_STA_TH);
		writeData(Method,file,"VAR_STA_D", VAR_STA_D);
		writeData(Method,file,"VAR_STA_packet_lr",VAR_STA_packet_lr);
		writeData(Method,file,"VAR_STA_chs_util",VAR_STA_chs_util);
	}
	else{
		string file = "���絲�G2ch/";
		writeData(Method,file,"VAR_STA_TH", VAR_STA_TH);
		writeData(Method,file,"VAR_STA_D", VAR_STA_D);
		writeData(Method,file,"DEV_AVG_TH", DEV_TH);
		writeData(Method,file,"DEV_AVG_D", DEV_D);
		writeData(Method,file,"VAR_STA_packet_lr",VAR_STA_packet_lr);
		writeData(Method,file,"VAR_STA_chs_util",VAR_STA_chs_util);
	}
	

	return 0;
}
