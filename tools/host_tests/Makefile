# Pruebas del firmware en la PC (no en el dispositivo).
#
#   make          -> compila y corre todo
#   make clean
#
# El stub/ reimplementa lo minimo de la API de Arduino/ESP-NOW para poder
# compilar el firmware con g++ y probar su logica sin subirlo al hardware.
CXX      ?= g++
CXXFLAGS ?= -O1 -Wall -Wextra -std=c++17
INC       = -Istub -I../../include
STUB      = stub/stub.cpp
STUB_ESP  = stub/stub.cpp stub/stub_esp.cpp

all: run

t_ctrl:     ; $(CXX) $(CXXFLAGS) -Wpedantic -I../../include -o $@ test_ctrl.cpp
t_ik:       ; $(CXX) $(CXXFLAGS) -Wpedantic -I../../include -o $@ test_ik.cpp
t_mega:     ; $(CXX) $(CXXFLAGS) $(INC) -o $@ test_mega.cpp $(STUB)
t_receiver: ; $(CXX) $(CXXFLAGS) $(INC) -o $@ test_receiver.cpp $(STUB_ESP)

run: t_ctrl t_ik t_mega t_receiver
	@echo "=== ctrl_mtds.h ==="      && ./t_ctrl
	@echo "=== ik_mtds.h ==="        && ./t_ik
	@echo "=== firmware del Mega ===" && ./t_mega
	@echo "=== firmware del receiver ===" && ./t_receiver

clean: ; rm -f t_ctrl t_ik t_mega t_receiver
.PHONY: all run clean
