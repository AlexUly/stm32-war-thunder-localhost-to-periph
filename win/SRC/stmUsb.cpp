#define UNICODE

#include <windows.h>
#include <iostream>
#include "tchar.h"


class stm_Usb{
private:
    HANDLE hPort;
    wchar_t portName[7];
    LPCTSTR ptrPortName;
    unsigned int portNum;
    DCB dcbParams;
    COMMTIMEOUTS ctParams;
    char data[8] = "STMINIT";
    std::string url = "localhost:8111/state";
    DWORD strSize;
    DWORD dataWritten;
    volatile char DATA[8];
    DWORD packageSize = sizeof(DATA);
    cpr::Response r;
    nlohmann::json stats;


    struct telem{
        bool valid = false;
        float Vy = 0;
        float AoA = 0;
        int efficiency = 0;
        int hp = 0;
    };

    telem Tele;
    char URC[1];
    bool rdy = true;
    bool console = false;
public:
    bool isReady() const{
        return rdy;
    }
    void showConsole(){
        console = true;
    }
    static int binToInt(char copy){
        int j = 1;
        int res = 0;
        char data = copy;
        for(int i = 0; i < 8; i++){
            if((data >> i) & 1){
                res += j;
            }
            else {
            }
            j *= 2;
        }
        char msh11[8];

        return res;
    }

    static char signedIntBin(int num, volatile char &res){
        int sign = 1;

        if(num < 0){
            num = std::abs(num);
            res = res | (1 << 7);
        }
        for(int i = 0; i < 7; i++){
            if(num & 1) {
                res = res | (1 << i);
            }else {
                res = res & ~(1 << i);
            }
            num = num >> 1;
        }
        return res;
    }

    char intBin(int num, volatile char &res){
        for(int i = 0; i < 8; i++){
            if(num & 1) {
                res = res | (1 << i);
            }else {
                res = res & ~(1 << i);
            }
            num = num >> 1;
        }
        return res;
    }

    int  Init(){
        if(console)
            std::cout << "INITIALIZING\n";
        hPort = ::CreateFile(_T("COM5"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        if (hPort != INVALID_HANDLE_VALUE) {
            if(console)
                std::cout << "PORT OK\n";
            return 0;
        } else {
            if(console)
                std::cout << "PORT DOES NOT EXIST\n";
            rdy = false;
            return 1;
        }
    }

    int DCBInit(){
        dcbParams.DCBlength = sizeof(dcbParams);
        if (!GetCommState(hPort, &dcbParams))
        {
            if(console)
                std::cout << "ERROR GCS\n";
            rdy = false;
            return 0;
        }
        return 1;
    }

    int StmHandshake(){

        DWORD dataSize = sizeof(data);
        if(console)
         std::cout << "WRITING\n";
        WriteFile(hPort, &data, dataSize, &dataWritten, NULL);
        char strReceived[9] = {0};
        memset(strReceived, 0, sizeof(strReceived));

        ReadFile(hPort, &strReceived, sizeof(strReceived), &strSize, 0);
        if(console)
            std::cout << "READING\n";
        if (strSize > 0) {
            if (strcmp(strReceived, "STMCOMR") == 0){
                strSize = 0;
                if(console)
                    std::cout << "CONNECT\n";
                return 0;
            }
            else {
                rdy = false;
                std::cout << strReceived << std::endl;
                if(console)
                    std::cout << "NOCONNECT\n";
                return 1;
            }
        }
        else {
            rdy = false;
            if(console)
              std::cout << "NO HANDSHAKE\n";
            return 1;
        }
        return 0;
    }
    void dataParse(){
        r = cpr::Get(cpr::Url(url));
        std::ofstream file("stats.json");
        file << r.text;
        file.close();
        std::ifstream fileIn("stats.json");

        fileIn >> stats;
        if( stats["valid"] == false)
            return;
        Tele.Vy = stats["Vy, m/s"];
        Tele.valid = stats["valid"];
        Tele.AoA = stats["AoA, deg"];
        Tele.efficiency = stats["efficiency 1, %"];
        Tele.hp = stats["power 1, hp"];


        int fractional;
        double integ;
        fractional = modf(Tele.Vy, &integ) * 10;
        DATA[0] = 0;


        DATA[1] = 0;
        DATA[2] = 0;
        DATA[3] = 0;
        DATA[4] = 0;
        DATA[5] = 0;
        DATA[6] = 0;
        DATA[7] = 0;

        Tele.valid == true ? DATA[0] = intBin(60,  DATA[0]) : DATA[0] = 0;
        signedIntBin((int) integ,  DATA[1]);
        intBin(fractional, (char &) DATA[2]);
        fractional = modf(Tele.AoA, &integ) * 10;
        DATA[3] = signedIntBin(integ, (char &) DATA[3]);
        DATA[4] = intBin(fractional, DATA[4]);
        DATA[5] = intBin(Tele.efficiency, DATA[5]);
        DATA[6] = intBin(Tele.hp / 100, DATA[6]);
        DATA[7] = intBin(Tele.hp % 100, DATA[7]);

    }

    void dataSend(){
        URC[0] = 0;
        ReadFile(hPort, &URC, sizeof(URC), &strSize, 0);
        char inChar = URC[0];
        if (inChar == 'R') {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if(console){
                std::cout << binToInt(DATA[0]) << std::endl;
                std::cout << "Vy: "<<binToInt(DATA[1]) << "." << binToInt(DATA[2])  << std::endl;
                std::cout << "AoA "<<binToInt(DATA[3]) << "." << binToInt(DATA[4])  << std::endl;
                std::cout << "EFF "<<binToInt(DATA[5])  << std::endl;
                std::cout << "HP "<<binToInt(DATA[6]) * 100 +  binToInt(DATA[7]) << std::endl;
            }
            WriteFile(hPort, (LPCVOID) &DATA, packageSize, &dataWritten, NULL);
        }
    }


};