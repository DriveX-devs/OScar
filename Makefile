EXECNAME=OCABS

OPENWRT_STAGING_DIR=/home/phd/Desktop/Francesco51/OpenWrt-V2X/staging_dir/target-x86_64_musl/usr/include
OPENWRT_LIBGPS_VER=3.23

SRC_DIR=src
OBJ_DIR=obj

SRC_ASN1_DIR=asn1/src
OBJ_ASN1_DIR=obj/asn1

SRC_ETSI_DIR=TransportAndNetworking/src
OBJ_ETSI_DIR=obj/TransportAndNetworking

SRC=$(wildcard $(SRC_DIR)/*.cpp)
SRC_ASN1=$(wildcard $(SRC_ASN1_DIR)/*.c)
SRC_ETSI=$(wildcard $(SRC_ETSI_DIR)/*.cpp)

OBJ=$(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_ASN1=$(SRC_ASN1:$(SRC_ASN1_DIR)/%.c=$(OBJ_ASN1_DIR)/%.o)
OBJ_ETSI=$(SRC_ETSI:$(SRC_ETSI_DIR)/%.c=$(OBJ_ETSI_DIR)/%.o)

OBJ_CC=$(OBJ)
OBJ_CC+=$(OBJ_ASN1)
OBJ_CC+=$(OBJ_ETSI)

CXXFLAGS += -Wall -O3 -Iinclude -std=c++17 -Iasn1/include -I. -ITransportAndNetworking/include
CFLAGS += -Wall -O3 -Iinclude -Ioptions -Iasn1/include
LDLIBS += -lpthread -lm -lgps

.PHONY: all clean

all: compilePC

compilePC: CXX = g++
compilePC: CC = gcc

compileAPU: CXX = x86_64-openwrt-linux-musl-g++
compileAPU: CC = x86_64-openwrt-linux-musl-gcc
compileAPU: LD = x86_64-openwrt-linux-musl-ld
compileAPU: CFLAGS += -I$(OPENWRT_STAGING_DIR)
	
compilePCdebug: CXXFLAGS += -g
compilePCdebug: compilePC

compileAPUdebug: CXXFLAGS += -g
compileAPUdebug: compileAPU

compilePC compilePCdebug: mainBuild
compileAPU compileAPUdebug: APUcopyfiles mainBuild

mainBuild: $(EXECNAME)

APUcopyfiles:
	# This is just a "trick" to be able to compile OCABS for OpenWrt, properly linking with libgps, without writing a specific package Makefile
	@ cp $(OPENWRT_STAGING_DIR)/../../../../build_dir/target-x86_64_musl/gpsd-3.23/ipkg-install/usr/lib/*.so* $(OPENWRT_STAGING_DIR)/../../../toolchain-x86_64_gcc-8.4.0_musl/lib

# Standard targets
$(EXECNAME): $(OBJ_CC)
	# @ cp $(OPENWRT_STAGING_DIR)/../../../../build_dir/target-x86_64_musl/gpsd-3.23/ipkg-install/usr/lib/*.so* $(OPENWRT_STAGING_DIR)/../../../toolchain-x86_64_gcc-8.4.0_musl/lib
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) $(CXXFLAGS) $(CFLAGS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@ mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_ASN1_DIR)/%.o: $(SRC_ASN1_DIR)/%.c
	@ mkdir -p $(OBJ_ASN1_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_ETSI_DIR)/%.o: $(SRC_ETSI_DIR)/%.c
	@ mkdir -p $(OBJ_ETSI_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ_DIR)/*.o $(OBJ_ASN1_DIR)/*.o
	-rm -rf $(OBJ_DIR)
	-rm -rf $(OBJ_ASN1_DIR)
	-rm -rf $(OBJ_ETSI_DIR)
	-rm -f cachefile.sldmc
	
fullclean: clean
	$(RM) $(EXECNAME)
