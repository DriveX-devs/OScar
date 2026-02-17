//
// Created by Carlos Mateo Risma Carletti on 07/05/21.
// Updated by Francesco Raviglione in November 2021
//

#ifndef OSCAR_SHBHEADER_H
#define OSCAR_SHBHEADER_H

#include <cstdint>
#include <cstring>
#include "gn_utils.h"
#include "DCC.h"

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

        void setDCC(DCC* dcc) {m_dcc = dcc;};
        double GetCBRR0Hop() {return m_CBR_R0_Hop;};
        double GetCBRR1Hop() {return m_CBR_R1_Hop;};

		static void printTSBPheader(packetBuffer &packet,std::string filename);
	private:
		GNlpv_t m_sourcePV;  //! Source long position vector
		uint8_t m_reserved; //! Aux variable for reserved fields
		uint32_t m_reserved32; //! Aux variable for reserved fields
		uint8_t m_local_CBR;
		uint8_t m_max_CBR_neighbouring;
		uint8_t m_tx_power_reserved;
        DCC* m_dcc = nullptr;
        double m_CBR_R0_Hop;
        double m_CBR_R1_Hop;

};

#endif // OSCAR_SHBHEADER_H
