#ifndef PACKETBUFFER_H
#define PACKETBUFFER_H

#include <gps.h>
#include <string>
#include <vector>

class packetBuffer {
	public:
		packetBuffer();

		// More efficient constructor pre-allocating the needed memory for the buffer
		packetBuffer(unsigned int bufsize);
		packetBuffer(const char *buffer,unsigned int bufsize);

		void addU8(uint8_t val);
		void addU16(uint16_t val);
		void addU32(uint32_t val);
		void addU64(uint64_t val);

		void addHtonU16(uint16_t val);
		void addHtonU32(uint32_t val);
		void addGNAddress(uint8_t val[8]);

		// This method returns a const pointer to the internal buffer
		// The pointer is valid until no additional operation is performed on the packetBuffer
		// If any operation is performed, getBuffer() must be called again to avoid undefined behaviour
		const uint8_t *getBufferPointer();

		// This method returns a copy of the internal buffer, at the moment the method was called
		// The returned buffer is dynamically allocated, thus the user is responsible for cleaning
		// up the memory with "delete []", when the returned buffer is no longer used
		const uint8_t *getBufferAlloc();

		const std::vector<uint8_t> getBufferVector() {return m_internal_buff;};

		size_t getBufferSize();
		size_t getBufferCapacity();

		void addHeader(packetBuffer &pktheader);

		void printContent();
		void printContent(const std::string text);
	private:
		std::vector<uint8_t> m_internal_buff;
		int m_it_idx;
};

#endif // VDPGPSC_H
