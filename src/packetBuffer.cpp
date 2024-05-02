#include <iostream>
#include "packetBuffer.h"

packetBuffer::packetBuffer() {
	m_it_idx=0;
}

packetBuffer::packetBuffer(unsigned int bufsize) {
	m_internal_buff.reserve(static_cast<size_t>(bufsize));
	m_it_idx=0;
}

packetBuffer::packetBuffer(const char *buffer,unsigned int bufsize) {
	m_internal_buff.reserve(static_cast<size_t>(bufsize));
	m_internal_buff.insert(m_internal_buff.end(), &buffer[0], &buffer[bufsize]);
	m_it_idx=0;
}

void 
packetBuffer::addU8(uint8_t val) {
	m_internal_buff.push_back(val);
	m_it_idx++;
}

void 
packetBuffer::addU16(uint16_t val) {
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
}

void 
packetBuffer::addU32(uint32_t val) {
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
}

void 
packetBuffer::addU64(uint64_t val) {
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
	val>>=8;
	addU8(val & 0xFF);
}

void 
packetBuffer::addHtonU16(uint16_t val) {
	addU8((val >> 8) & 0xFF);
	addU8((val >> 0) & 0xFF);
}

void 
packetBuffer::addHtonU32(uint32_t val) {
	addU8((val >> 24) & 0xFF);
	addU8((val >> 16) & 0xFF);
	addU8((val >> 8) & 0xFF);
	addU8((val >> 0) & 0xFF);
}

void 
packetBuffer::addGNAddress(uint8_t val[8]) {
	addU8(val[0]);
	addU8(val[1]);
	addU8(val[2]);
	addU8(val[3]);
	addU8(val[4]);
	addU8(val[5]);
	addU8(val[6]);
	addU8(val[7]);
}

const uint8_t *
packetBuffer::getBufferPointer() {
	return &m_internal_buff[0];
}

const uint8_t *
packetBuffer::getBufferAlloc() {
	uint8_t *copybuff = new uint8_t[m_internal_buff.size()];
	std::copy(m_internal_buff.begin(),m_internal_buff.end(),copybuff);

	return copybuff;
}

void 
packetBuffer::addHeader(packetBuffer &pktheader) {
 	std::vector <uint8_t> pktheaderVec = pktheader.getBufferVector();
	pktheaderVec.insert(pktheaderVec.end(),m_internal_buff.begin(),m_internal_buff.end());
	m_internal_buff=std::move(pktheaderVec);
}

size_t 
packetBuffer::getBufferSize() {
	return m_internal_buff.size();
}

size_t 
packetBuffer::getBufferCapacity() {
	return m_internal_buff.capacity();
}

void
packetBuffer::printContent() {
	for(size_t i=0;i<m_internal_buff.size();i++) {
		fprintf(stdout,"%02X",m_internal_buff[i]);
	}
	fprintf(stdout,"\n");
}

void 
packetBuffer::printContent(const std::string text) {
	std::cout << text << ": " << std::endl;
	printContent();
}