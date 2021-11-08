EXECNAME=OCABS

SRC_DIR=src
OBJ_DIR=obj

SRC_ASN1_DIR=asn1/src
OBJ_ASN1_DIR=obj/asn1

SRC=$(wildcard $(SRC_DIR)/*.cpp)
SRC_ASN1=$(wildcard $(SRC_ASN1_DIR)/*.c)

OBJ=$(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_ASN1=$(SRC_ASN1:$(SRC_ASN1_DIR)/%.c=$(OBJ_ASN1_DIR)/%.o)

OBJ_CC=$(OBJ)
OBJ_CC+=$(OBJ_ASN1)

CXXFLAGS += -Wall -O3 -Iinclude -std=c++17 -Iasn1/include -I.
CFLAGS += -Wall -O3 -Iinclude -Ioptions -Iasn1/include
LDLIBS += -lpthread -lm

.PHONY: all clean

all: compilePC

compilePC: CXX = g++
compilePC: CC = gcc
	
compilePCdebug: CXXFLAGS += -g
compilePCdebug: compilePC

compilePC compilePCdebug: $(EXECNAME)

# Standard targets
$(EXECNAME): $(OBJ_CC)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) $(CXXFLAGS) $(CFLAGS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@ mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_ASN1_DIR)/%.o: $(SRC_ASN1_DIR)/%.c
	@ mkdir -p $(OBJ_ASN1_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ_DIR)/*.o $(OBJ_ASN1_DIR)/*.o
	-rm -rf $(OBJ_DIR)
	-rm -rf $(OBJ_ASN1_DIR)
	-rm -f cachefile.sldmc
	
fullclean: clean
	$(RM) $(EXECNAME)
