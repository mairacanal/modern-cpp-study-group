/********************************************************************

Challenge 1 - Force/Torque sensor read
Modern C++ study grouṕ  at AeroTech Lab, School of Engineering of São Carlos - University of São Paulo - Brazil

Author: Maíra Canal 
Date: 01/2021

********************************************************************/


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>

#define READ_AXIS_PACKAGE_SIZE 1
#define CALIBRATION_MATRIX_SIZE 6

struct CANresponse {
    std::string messageId;
    int payloadSize;
    std::vector<std::string> bytes;
};

struct sensorStatus {
    int CpF;
    int CpT;
    std::string forceUnit;
    std::string torqueUnit;
    int SG[6];
    int actualReadingAxis;
    float calibrationMatrix[6][6];
};

union ulf {
    unsigned long ul;
    float f;
};

// HEADERS
void read_log();
void read_line(std::string line, sensorStatus& data);
std::string get_messageId(std::string line);
std::vector<std::string> get_bytes(std::string line, int payload);
void buffer(std::vector<std::string> bytes, sensorStatus& data, char messageId);
void read_0x0(std::vector<std::string> bytes, sensorStatus& data);
void read_0x1(std::vector<std::string> bytes, sensorStatus& data);
void read_0x2(std::vector<std::string> bytes, sensorStatus& data);
void read_0x3(std::vector<std::string> bytes, sensorStatus& data);
void read_0x4(std::vector<std::string> bytes, sensorStatus& data);
void setActive_0x6(std::vector<std::string> bytes);
void read_0x7(std::vector<std::string> bytes, sensorStatus& data);
void readUnit_0x8(std::vector<std::string> bytes, sensorStatus& data);
void resultAnswer(sensorStatus& data);
float fromHEXtofloat(std::string HEXvalue);

int main () {

    read_log();

    return 0;
}

void read_log() {

    std::fstream file;
    sensorStatus data;

    file.open("sensor-log.txt", std::ios::in);

    if (!file) {
        
        std::cout << "File not finded" << std::endl;

    } else {

		for (std::array<char, 43> line; file.getline(&line[0], 43, '\n'); ) {

            read_line(&line[0], data);
        
        }

        file.close();

    }

}

void read_line(std::string line, sensorStatus& data) {

    CANresponse response;

    response.messageId = get_messageId(line);
    response.payloadSize = line[15] -'0';
    response.bytes = get_bytes(line, response.payloadSize);

    if (response.payloadSize != 0 && response.messageId[0] == '7' && response.messageId[1] == 'F') {

        buffer(response.bytes, data, response.messageId[2]);

    }

}

std::string get_messageId(std::string line) {

    std::string messageId{line[8], line[9], line[10]};

    return messageId;

}

std::vector<std::string> get_bytes(std::string line, int payload) {

    auto it = line.begin() + 19;
    std::vector<std::string> bytes;

    for (int i = 0; i < payload; ++i) {

        std::string byte{*it, *(++it)};
        bytes.push_back(byte);
        it = it + 2;

    } 

    return bytes;

}

void buffer(std::vector<std::string> bytes, sensorStatus& data, char messageId) {

    switch (messageId) {
        case '0':
            read_0x0(bytes, data);
            break;
        case '1':
            read_0x1(bytes, data);
            break;
        case '2':
            read_0x2(bytes, data);
            break;
        case '3':
            read_0x3(bytes, data);
            break;
        case '4':
            read_0x4(bytes, data);
            break;
        case '6':
            setActive_0x6(bytes);
            break;
        case '7':
            read_0x7(bytes, data);
            break;
        case '8':
            readUnit_0x8(bytes, data);
            break;
        default:
            break;
    }

}

void read_0x0(std::vector<std::string> bytes, sensorStatus& data) {

    data.SG[0] = std::stoi("0x" + bytes[2] + bytes[3], nullptr, 16);
    data.SG[2] = std::stoi("0x" + bytes[4] + bytes[5], nullptr, 16);
    data.SG[4] = std::stoi("0x" + bytes[6] + bytes[7], nullptr, 16);

}

void read_0x1(std::vector<std::string> bytes, sensorStatus& data) {

    data.SG[1] = std::stoi("0x" + bytes[0] + bytes[1], nullptr, 16);
    data.SG[3] = std::stoi("0x" + bytes[2] + bytes[3], nullptr, 16);
    data.SG[5] = std::stoi("0x" + bytes[4] + bytes[5], nullptr, 16);

    resultAnswer(data);

}

void read_0x2(std::vector<std::string> bytes, sensorStatus& data) {

    if (bytes.size() == READ_AXIS_PACKAGE_SIZE) {

        data.actualReadingAxis = std::stoi("0x" + bytes[0], nullptr, 16);

    } else {

        float sg0 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]);
        float sg1 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

        if (data.actualReadingAxis == 0 || data.actualReadingAxis == 1 || data.actualReadingAxis == 2 ) {

            data.calibrationMatrix[data.actualReadingAxis][0] = sg0 * data.CpF;
            data.calibrationMatrix[data.actualReadingAxis][1] = sg1 * data.CpF;

        } else {

            data.calibrationMatrix[data.actualReadingAxis][0] = sg0 * data.CpT;
            data.calibrationMatrix[data.actualReadingAxis][1] = sg1 * data.CpT;

        }

    }

}

void read_0x3(std::vector<std::string> bytes, sensorStatus& data) {

    float sg2 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]); 
    float sg3 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

    if (data.actualReadingAxis == 0 || data.actualReadingAxis == 1 || data.actualReadingAxis == 2 ) {

        data.calibrationMatrix[data.actualReadingAxis][2] = sg2 * data.CpF;
        data.calibrationMatrix[data.actualReadingAxis][3] = sg3 * data.CpF;

    } else {

        data.calibrationMatrix[data.actualReadingAxis][2] = sg2 * data.CpT;
        data.calibrationMatrix[data.actualReadingAxis][3] = sg3 * data.CpT;

    }

}

void read_0x4(std::vector<std::string> bytes, sensorStatus& data) {

    float sg4 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]); 
    float sg5 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

    if (data.actualReadingAxis == 0 || data.actualReadingAxis == 1 || data.actualReadingAxis == 2 ) {

        data.calibrationMatrix[data.actualReadingAxis][4] = sg4 * data.CpF;
        data.calibrationMatrix[data.actualReadingAxis][5] = sg5 * data.CpF;

    } else {

        data.calibrationMatrix[data.actualReadingAxis][4] = sg4 * data.CpT;
        data.calibrationMatrix[data.actualReadingAxis][5] = sg5 * data.CpT;

    }

}

void setActive_0x6(std::vector<std::string> bytes) {

    if (std::stoi(bytes[0], nullptr, 10) == 0) {

        std::cout << "Watchdog Reset" << std::endl;

    }

}

void read_0x7(std::vector<std::string> bytes, sensorStatus& data) {

    std::string CpF{"0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]};
    std::string CpT{"0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]};

    data.CpF = std::stoi(CpF, nullptr, 16);
    data.CpT = std::stoi(CpT, nullptr, 16);

}

void readUnit_0x8(std::vector<std::string> bytes, sensorStatus& data) {

    std::string forceUnitCode[6]{"lbf", "N", "Klbf", "kN", "kgf", "gf"};
    std::string torqueUnitCode[6]{"lbf-in", "lbf-ft", "N-m", "N-mm", "kgf-cm", "kN-m"};

    data.forceUnit = forceUnitCode[std::stoi("0x" + bytes[0], nullptr, 16) - 1];
    data.torqueUnit = torqueUnitCode[std::stoi("0x" + bytes[1], nullptr, 16) -1 ];

}

void resultAnswer(sensorStatus& data) {

    float resultAnswer[6];
    std::string label[6]{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};

    for (int i = 0; i < CALIBRATION_MATRIX_SIZE; i++) {

        for (int j = 0; j < CALIBRATION_MATRIX_SIZE; j++) {
        
            resultAnswer[i] +=  data.calibrationMatrix[i][j] * data.SG[j]; 

        }

        if (i < 3) std::cout << label[i] << ": " << resultAnswer[i]/data.CpF << " " << data.forceUnit << " ";
        else std::cout << label[i] << ": " << resultAnswer[i]/data.CpT << " " << data.torqueUnit << " ";

    }

    std::cout << std::endl;

}

float fromHEXtofloat(std::string HEXvalue) {

    ulf u;
    std::stringstream buffer(HEXvalue);
    buffer >> std::hex >> u.ul;
    float floatValue = u.f;
    return floatValue;

}