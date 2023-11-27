//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OCABS_SHBHEADER_H
#define OCABS_SHBHEADER_H

#include <cstdint>
#include <cstring>
#include "utils.h"

class shbHeader {
	public:
		shbHeader();
		~shbHeader();
		void serializeInto(packetBuffer &packet);
		
		void removeHeader(unsigned char * buffer);

		//Getters
		[[nodiscard]] GNlpv_t GetLongPositionV() const {return m_sourcePV;}
		
		//Setters
		void SetLongPositionV(GNlpv_t longPositionVector) {m_sourcePV = longPositionVector;}

		static void printTSBPheader(packetBuffer &packet,std::string filename);
	private:
		GNlpv_t m_sourcePV;  //! Source long position vector
		uint8_t m_reserved; //! Aux variable for reserved fields
		uint32_t m_reserved32; //! Aux variable for reserved fields

};

#endif // OCABS_SHBHEADER_H
