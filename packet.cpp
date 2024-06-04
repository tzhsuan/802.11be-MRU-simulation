#include "packet.h"
Packet::Packet(int packetSize, int arrival_time, int deadline)
{
	this->packetSize = packetSize;
	this->arrival_time = arrival_time;
	this->deadline = deadline;
}
