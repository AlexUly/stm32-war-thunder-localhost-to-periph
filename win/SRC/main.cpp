#include "cpr/cpr.h"
#include "nlohmann/json.hpp"
#include "stmUsb.cpp"

#define _UNICODE


int main() {
    stm_Usb stmDev;
    stmDev.showConsole();
    stmDev.Init();
    stmDev.DCBInit();
    stmDev.StmHandshake();

    while(stmDev.isReady()) {
        stmDev.dataParse();
        stmDev.dataSend();
    }
    return 0;
}