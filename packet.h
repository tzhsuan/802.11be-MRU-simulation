#ifndef _PACKET_H_
#define _PACKET_H_
class Packet{
public:
	int packetSize;//bit
	int arrival_time;
	int deadline;//0~simtime
	bool canTrans = false;
	Packet(int,int,int);
};

#endif
