#include <vector>
#include <chrono>


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <random>

#include <map>

#include <queue>
#include "ap.h"

using namespace std;
class Scheduler{
public:
	AP* ap = nullptr;
	Scheduler(int);
	~Scheduler();
	void generate_traffic(vector<double>,vector<int>, vector<int>,  vector<int>,vector<int>);
	void schedule_access(int,double);
	void schedule_access2CH(int,double);
private:
	const int priority_num = 4;
	const int SIFS = 16;//mus
	const int PIFS = 25;
	const int ACK = 60;
	const int DIFS = 34;
	const int TXOP = 5000;//mus
	int sim_time = 0;
};
