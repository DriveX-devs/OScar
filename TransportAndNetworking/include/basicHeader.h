//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OCABS_BASICHEADER_H
#define OCABS_BASICHEADER_H

#include <cstdint>
#include "packetBuffer.h"

class basicHeader {
public:
	basicHeader();
	~basicHeader();
	void serializeInto(packetBuffer &packet);

	//Getters
	[[nodiscard]] uint8_t GetVersion() const {return m_version;}
	[[nodiscard]] uint8_t GetNextHeader() const {return m_nextHeader;}
	[[nodiscard]] uint8_t GetLifeTime() const {return m_lifeTime;}
	[[nodiscard]] uint8_t GetRemainingHL() const{return m_remainingHopLimit;}

	//Setters
	void SetVersion(uint8_t version) {m_version = version;}
	void SetNextHeader(uint8_t NH) {m_nextHeader = NH;}
	void SetLifeTime(uint8_t LT) {m_lifeTime = LT;}
	void SetRemainingHL(uint8_t RHL){m_remainingHopLimit = RHL;}

	static void printBasicHeader(packetBuffer &packet,std::string filename);

private:
		uint8_t m_version: 4;
		uint8_t m_nextHeader: 4;
		uint8_t m_reserved;
		uint8_t m_lifeTime;
		uint8_t m_remainingHopLimit;
	};

#endif //S_LDM_BASICHEADER_H
