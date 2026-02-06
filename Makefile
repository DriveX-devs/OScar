EXECNAME=OScar

OPENWRT_CROSSCOMPILER_C=x86_64-openwrt-linux-musl-gcc
OPENWRT_CROSSCOMPILER_CXX=x86_64-openwrt-linux-musl-g++
OPENWRT_CROSSCOMPILER_LD=x86_64-openwrt-linux-musl-ld
OPENWRT_TARGET=target-x86_64_musl
OPENWRT_TOOLCHAIN=toolchain-x86_64_gcc-8.4.0_musl
OPENWRT_LIBGPS_VER=3.23

OPENWRT_INCLUDE_DIR=/mnt/xtra/OpenWrt-V2X/staging_dir/target-x86_64_musl/usr/include
OPENWRT_MAIN_DIR=/mnt/xtra/OpenWrt-V2X

SRC_DIR=src
OBJ_DIR=obj

SRC_GEOLIB_PORT_DIR=geographiclib-port
OBJ_GEOLIB_PORT_DIR=obj/geographiclib-port

SRC_VEHVIS_DIR=vehicle-visualizer/src
OBJ_VEHVIS_DIR=obj/vehicle-visualizer

SRC_ASN1_DIR=asn1/src
OBJ_ASN1_DIR=obj/asn1

SRC_ETSI_DIR=TransportAndNetworking/src
OBJ_ETSI_DIR=obj/TransportAndNetworking

SRC_JSON11_DIR=json11
OBJ_JSON11_DIR=obj/json11

SRC_GPSC_DIR=gnss
OBJ_GPSC_DIR=obj/gnss

SRC_ASN1CPP_DIR=asn1cpp
OBJ_ASN1CPP_DIR=obj/asn1cpp

SRC_CESERIAL_DIR=ceSerial
OBJ_CESERIAL_DIR=obj/ceSerial

SRC_INI_DIR=iniReader
OBJ_INI_DIR=obj/iniReader

SRC_INICLIB_DIR=iniLibraryC
OBJ_INICLIB_DIR=obj/iniLibraryC

SRC_PKIREQRES_DIR=pkiReqRes
OBJ_PKIREQRES_DIR=obj/pkiReqRes

SRC=$(wildcard $(SRC_DIR)/*.cpp)
SRC_GEOLIB_PORT=$(wildcard $(SRC_GEOLIB_PORT_DIR)/*.c)
SRC_VEHVIS=$(wildcard $(SRC_VEHVIS_DIR)/*.cc)
SRC_ASN1=$(wildcard $(SRC_ASN1_DIR)/*.c)
SRC_ETSI=$(wildcard $(SRC_ETSI_DIR)/*.cpp)
SRC_JSON11=$(wildcard $(SRC_JSON11_DIR)/*.cpp)
SRC_GPSC=$(wildcard $(SRC_GPSC_DIR)/*.cpp)
SRC_ASN1CPP=$(wildcard $(SRC_ASN1CPP_DIR)/*.cpp)
SRC_CESERIAL=$(wildcard $(SRC_CESERIAL_DIR)/*.cpp)
SRC_INI=$(wildcard $(SRC_INI_DIR)/*.cpp)
SRC_INICLIB=$(wildcard $(SRC_INICLIB_DIR)/*.c)
SRC_PKIREQRES=$(wildcard $(SRC_PKIREQRES_DIR)/*.cpp)

OBJ=$(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_GEOLIB_PORT=$(SRC_GEOLIB_PORT:$(SRC_GEOLIB_PORT_DIR)/%.c=$(OBJ_GEOLIB_PORT_DIR)/%.o)
OBJ_VEHVIS=$(SRC_VEHVIS:$(SRC_VEHVIS_DIR)/%.cc=$(OBJ_VEHVIS_DIR)/%.o)
OBJ_ASN1=$(SRC_ASN1:$(SRC_ASN1_DIR)/%.c=$(OBJ_ASN1_DIR)/%.o)
OBJ_ETSI=$(SRC_ETSI:$(SRC_ETSI_DIR)/%.c=$(OBJ_ETSI_DIR)/%.o)
OBJ_JSON11=$(SRC_JSON11:$(SRC_JSON11_DIR)/%.cpp=$(OBJ_JSON11_DIR)/%.o)
OBJ_GPSC=$(SRC_GPSC:$(SRC_GPSC_DIR)/%.cpp=$(OBJ_GPSC_DIR)/%.o)
OBJ_ASN1CPP=$(SRC_ASN1CPP:$(SRC_ASN1CPP_DIR)/%.cpp=$(OBJ_ASN1CPP_DIR)/%.o)
OBJ_CESERIAL=$(SRC_CESERIAL:$(SRC_CESERIAL_DIR)/%.cpp=$(OBJ_CESERIAL_DIR)/%.o)
OBJ_INI=$(SRC_INI:$(SRC_INI_DIR)/%.cpp=$(OBJ_INI_DIR)/%.o)
OBJ_INICLIB=$(SRC_INICLIB:$(SRC_INICLIB_DIR)/%.c=$(OBJ_INICLIB_DIR)/%.o)
OBJ_PKIREQRES=$(SRC_PKIREQRES:$(SRC_PKIREQRES_DIR)/%.cpp=$(OBJ_PKIREQRES_DIR)/%.o)

OBJ_CC=$(OBJ)
OBJ_CC+=$(OBJ_GEOLIB_PORT)
OBJ_CC+=$(OBJ_VEHVIS)
OBJ_CC+=$(OBJ_ASN1)
OBJ_CC+=$(OBJ_ETSI)
OBJ_CC+=$(OBJ_JSON11)
OBJ_CC+=$(OBJ_ASN1CPP)
OBJ_CC+=$(OBJ_CESERIAL)
OBJ_CC+=$(OBJ_INI)
OBJ_CC+=$(OBJ_INICLIB)
OBJ_CC+=$(OBJ_PKIREQRES)

CXXFLAGS += -Wall -O3 -Iinclude -std=c++17 -Ivehicle-visualizer/include -Igeographiclib-port -Iasn1/include -I. -ITransportAndNetworking/include -Ijson11 -Iasn1cpp -IceSerial -I/usr/include/openssl -I/usr/include/libnl3 -IiniReader -IiniLibraryC -IpkiReqRes -IhttpRest
CFLAGS += -Wall -O3 -Iinclude -Ioptions -Iasn1/include -Igeographiclib-port -I/usr/include/openssl -IpkiReqRes -IhttpRest
LDLIBS += -lpthread -lm -lgps -latomic -lssl -lcrypto -lnl-3 -lnl-genl-3

.PHONY: all clean

all: compilePC

compilePC: CXX = g++
compilePC: CC = gcc

compileAPU: CXX = $(OPENWRT_CROSSCOMPILER_CXX)
compileAPU: CC = $(OPENWRT_CROSSCOMPILER_C)
compileAPU: LD = $(OPENWRT_CROSSCOMPILER_LD)
compileAPU: CXXFLAGS += -I$(OPENWRT_INCLUDE_DIR)
compileAPU: LDLIBS += -L$(OPENWRT_MAIN_DIR)/staging_dir/$(OPENWRT_TARGET)/usr/lib -lgps -lssl -lcrypto

compilePCdebug: CXXFLAGS += -g
compilePCdebug: CFLAGS += -g
compilePCdebug: compilePC

compileAPUdebug: CXXFLAGS += -g
compileAPUdebug: CFLAGS += -g
compileAPUdebug: compileAPU

compilePC compilePCdebug: $(EXECNAME)
compileAPU compileAPUdebug: APUcopyfiles $(EXECNAME)

APUcopyfiles:
	# This is just a "trick" to be able to compile OCABS for OpenWrt, properly linking with libgps, without writing a specific package Makefile
	@ cp $(OPENWRT_INCLUDE_DIR)/../../../../build_dir/$(OPENWRT_TARGET)/gpsd-$(OPENWRT_LIBGPS_VER)/ipkg-install/usr/lib/*.so* $(OPENWRT_INCLUDE_DIR)/../../../$(OPENWRT_TOOLCHAIN)/lib

# Standard targets
$(EXECNAME): $(OBJ_CC)
	# @ cp $(OPENWRT_STAGING_DIR)/../../../../build_dir/target-x86_64_musl/gpsd-3.23/ipkg-install/usr/lib/*.so* $(OPENWRT_STAGING_DIR)/../../../toolchain-x86_64_gcc-8.4.0_musl/lib
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) $(CXXFLAGS) $(CFLAGS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@ mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_GEOLIB_PORT_DIR)/%.o: $(SRC_GEOLIB_PORT_DIR)/%.c
	@ mkdir -p $(OBJ_GEOLIB_PORT_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_VEHVIS_DIR)/%.o: $(SRC_VEHVIS_DIR)/%.cc
	@ mkdir -p $(OBJ_VEHVIS_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_ASN1_DIR)/%.o: $(SRC_ASN1_DIR)/%.c
	@ mkdir -p $(OBJ_ASN1_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_ETSI_DIR)/%.o: $(SRC_ETSI_DIR)/%.c
	@ mkdir -p $(OBJ_ETSI_DIR)
	$(CC) $(CFLAGS) -c $< -o $@
	
$(OBJ_JSON11_DIR)/%.o: $(SRC_JSON11_DIR)/%.cpp
	@ mkdir -p $(OBJ_JSON11_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_CESERIAL_DIR)/%.o: $(SRC_CESERIAL_DIR)/%.cpp
	@ mkdir -p $(OBJ_CESERIAL_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_INI_DIR)/%.o: $(SRC_INI_DIR)/%.cpp
	@ mkdir -p $(OBJ_INI_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_INICLIB_DIR)/%.o: $(SRC_INICLIB_DIR)/%.c
	@ mkdir -p $(OBJ_INICLIB_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_PKIREQRES_DIR)/%.o: $(SRC_PKIREQRES_DIR)/%.cpp
	@ mkdir -p $(OBJ_PKIREQRES_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ_DIR)/*.o $(OBJ_ASN1_DIR)/*.o
	-rm -rf $(OBJ_DIR)
	-rm -rf $(OBJ_ASN1_DIR)
	-rm -rf $(OBJ_ETSI_DIR)
	-rm -rf $(OBJ_JSON11_DIR)
	-rm -rf $(OBJ_GEOLIB_PORT_DIR)
	-rm -rf $(OBJ_VEHVIS_DIR)
	-rm -rf $(OBJ_GPSC_DIR)
	-rm -rf $(OBJ_ASN1CPP_DIR)
	-rm -rf $(OBJ_CESERIAL_DIR)
	-rm -rf $(OBJ_INI_DIR)
	-rm -rf $(OBJ_INICLIB_DIR)
	-rm -f cachefile.sldmc
	
fullclean: clean
	$(RM) $(EXECNAME)
