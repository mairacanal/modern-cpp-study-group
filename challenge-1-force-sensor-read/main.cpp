/********************************************************************

Challenge 1 - Force/Torque sensor read

* @file main.cpp
* @brief Exercise for the modern C++ study group at AeroTech Lab, School of 
Engineering of São Carlos - University of São Paulo - Brazil
* @author Autor: Maíra Canal (@mairacanal)
* @date Jan 2021
* @version 0.1

********************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>

#define READ_AXIS_PACKAGE_SIZE 1
#define CALIBRATION_MATRIX_SIZE 6

/* Data structure defined to convert 4-bytes floating point 
in hexadecimal format to a 4-bytes floating point in decimal
format*/
union ulf {
    unsigned long ul;
    float f;
};

class CANresponse {

    public:
        std::string messageId;
        int payloadSize;
        std::vector<std::string> bytes;

        /*
         * @brief Defines all proprieties of the CANresponse class
         * @param Actual reading line
         */
        void get_data(const std::string line) {

            get_messageID(line);
            get_payload(line);
            get_bytes(line);

        }
        /*
         * @brief Defines the messageID propriety of the CANresponse class
         * @param Actual reading line
         */
        void get_messageID(std::string line) {

            std::string newMessageId{line[8], line[9], line[10]};

            messageId.swap(newMessageId);

        }
        /*
         * @brief Defines the payloadSize propriety of the CANresponse class
         * @param Actual reading line
         */
        void get_payload(std::string line) {

            payloadSize = line[15] -'0';    

        }
        /*
         * @brief Defines the bytes propriety of the CANresponse class
         * @param Actual reading line
         */
        void get_bytes(std::string line) {

            auto it = line.begin() + 19;
            bytes.clear();

            for (int i = 0; i < payloadSize; ++i) {

                std::string byte{*it, *(++it)};
                bytes.push_back(byte);
                it = it + 2;

            } 

        }

};

class Sensor {

    private:

        CANresponse data;

        /*
         * @brief Converts a 4-bytes floating point in hexadecimal format into a 
         * 4-bytes floating point in decimal format
         * @param 4-bytes floating point in hexadecimal format
         * @return 4-bytes floating point in decimal format
         */
        static float fromHEXtofloat(const std::string HEXvalue) {

            ulf u;
            std::stringstream buffer(HEXvalue);
            buffer >> std::hex >> u.ul;
            float floatValue = u.f;

            return floatValue;

        }

    public: 

        int CpF;
        int CpT;
        std::string forceUnit;
        std::string torqueUnit;
        int actualReadingAxis;
        int SG[CALIBRATION_MATRIX_SIZE];
        float calibrationMatrix[CALIBRATION_MATRIX_SIZE][CALIBRATION_MATRIX_SIZE];

        void setCANresponse(std::string line) {

            data.get_data(line);

        }

        /*
         * @brief Redirects the information to the specific function
         */
        void buffer() {

            if (data.payloadSize != 0 && data.messageId[0] == '7' && data.messageId[1] == 'F') {

                switch (data.messageId[2]) {
                    case '0':
                        read_0x0(data.bytes);
                        break;
                    case '1':
                        read_0x1(data.bytes);
                        resultAnswer();
                        break;
                    case '2':
                        read_0x2(data.bytes);
                        break;
                    case '3':
                        read_0x3(data.bytes);
                        break;
                    case '4':
                        read_0x4(data.bytes);
                        break;
                    case '6':
                        setActive_0x6(data.bytes);
                        break;
                    case '7':
                        read_0x7(data.bytes);
                        break;
                    case '8':
                        readUnit_0x8(data.bytes);
                        break;
                    default:
                        break;
                }

            }
        }

        /*
         * @brief Read SG Data => A packet with the opcode set to b0000, which
         * contains the two byte status code, followed by the two byte values 
         * for sg0, sg2, and sg4
         * @param 8-bytes vector in hexadecimal format
         */
        void read_0x0(std::vector<std::string> bytes) {

            SG[0] = std::stoi("0x" + bytes[2] + bytes[3], nullptr, 16);
            SG[2] = std::stoi("0x" + bytes[4] + bytes[5], nullptr, 16);
            SG[4] = std::stoi("0x" + bytes[6] + bytes[7], nullptr, 16);

        }

        /*
         * @brief Read SG Data => A packet with the opcode b0001, which contains
         * the three 2-byte values sg1, sg3, and sg5
         * @param 6-bytes vector in hexadecimal format
         */
        void read_0x1(std::vector<std::string> bytes) {

            SG[1] = std::stoi("0x" + bytes[0] + bytes[1], nullptr, 16);
            SG[3] = std::stoi("0x" + bytes[2] + bytes[3], nullptr, 16);
            SG[5] = std::stoi("0x" + bytes[4] + bytes[5], nullptr, 16);

        }

        /*
         * @brief Defines the actual reading axis in a 1-byte package
         * or defines the SG0 and SG1 matrix coefficients
         * @param 1-byte vector in hexadecimal format or 8-bytes vector
         */
        void read_0x2(std::vector<std::string> bytes) {

            if (bytes.size() == READ_AXIS_PACKAGE_SIZE) {

                actualReadingAxis = std::stoi("0x" + bytes[0], nullptr, 16);

            } else {

                float sg0 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]);
                float sg1 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

                if (actualReadingAxis == 0 || actualReadingAxis == 1 || actualReadingAxis == 2 ) {

                    calibrationMatrix[actualReadingAxis][0] = sg0 * CpF;
                    calibrationMatrix[actualReadingAxis][1] = sg1 * CpF;

                } else {

                    calibrationMatrix[actualReadingAxis][0] = sg0 * CpT;
                    calibrationMatrix[actualReadingAxis][1] = sg1 * CpT;

                }

            }

        }

        /*
         * @brief Defines the SG2 and SG3 matrix coefficients
         * @param 8-bytes vector in hexadecimal format
         */
        void read_0x3(std::vector<std::string> bytes) {

            float sg2 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]); 
            float sg3 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

            if (actualReadingAxis == 0 || actualReadingAxis == 1 || actualReadingAxis == 2 ) {

                calibrationMatrix[actualReadingAxis][2] = sg2 * CpF;
                calibrationMatrix[actualReadingAxis][3] = sg3 * CpF;

            } else {

                calibrationMatrix[actualReadingAxis][2] = sg2 * CpT;
                calibrationMatrix[actualReadingAxis][3] = sg3 * CpT;

            }

        }

        /*
         * @brief Defines the SG4 and SG5 matrix coefficients
         * @param 8-bytes vector in hexadecimal format
         */
        void read_0x4(std::vector<std::string> bytes) {

            float sg4 = fromHEXtofloat("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3]); 
            float sg5 = fromHEXtofloat("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7]);

            if (actualReadingAxis == 0 || actualReadingAxis == 1 || actualReadingAxis == 2 ) {

                calibrationMatrix[actualReadingAxis][4] = sg4 * CpF;
                calibrationMatrix[actualReadingAxis][5] = sg5 * CpF;

            } else {

                calibrationMatrix[actualReadingAxis][4] = sg4 * CpT;
                calibrationMatrix[actualReadingAxis][5] = sg5 * CpT;

            }

        }

        void setActive_0x6(std::vector<std::string> bytes) {

            if (std::stoi(bytes[0], nullptr, 10) == 0) {

                std::cout << "Watchdog Reset" << std::endl;

            }

        }
        
        /*
         * @brief Defines counts per force (CpF) and counts per torque (CpT) 
         * The first 4 bytes are the counts per force, followed by the 4 byte
         * counts per torque.
         * @param 8-bytes vector in hexadecimal format
         */
        void read_0x7(std::vector<std::string> bytes) {

            CpF = std::stoi("0x" + bytes[0] + bytes[1] + bytes[2] + bytes[3], nullptr, 16);
            CpT = std::stoi("0x" + bytes[4] + bytes[5] + bytes[6] + bytes[7], nullptr, 16);

        }

        /*
         * @brief Defines the force unit and the torque unit
         * @param 2-bytes vector in hexadecimal format
         */
        void readUnit_0x8(std::vector<std::string> bytes) {

            std::string forceUnitCode[6]{"lbf", "N", "Klbf", "kN", "kgf", "gf"};
            std::string torqueUnitCode[6]{"lbf-in", "lbf-ft", "N-m", "N-mm", "kgf-cm", "kN-m"};

            forceUnit = forceUnitCode[std::stoi("0x" + bytes[0], nullptr, 16) - 1];
            torqueUnit = torqueUnitCode[std::stoi("0x" + bytes[1], nullptr, 16) -1 ];

        }

        /*
         * @brief Multiply the calibration matrix for the SG vector, resulting in the
         * final answer
         */
        void resultAnswer() {

            float resultAnswer[6];
            std::string label[6]{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};

            for (int i = 0; i < CALIBRATION_MATRIX_SIZE; i++) {

                for (int j = 0; j < CALIBRATION_MATRIX_SIZE; j++) {
                
                    resultAnswer[i] +=  calibrationMatrix[i][j] * SG[j]; 

                }

                if (i < 3) std::cout << label[i] << ": " << resultAnswer[i]/CpF << " " << forceUnit << " ";
                else std::cout << label[i] << ": " << resultAnswer[i]/CpT << " " << torqueUnit << " ";

            }

            std::cout << std::endl;

        }

};

int main () {

    std::fstream file;
    Sensor sensor;

    file.open("sensor-log.txt", std::ios::in);

    if (!file) {
        
        std::cout << "File not finded" << std::endl;

    } else {

        // Reads each line of the file and makes the respective operation
        
		for (std::array<char, 43> line; file.getline(&line[0], 43, '\n'); ) {

            sensor.setCANresponse(&line[0]);
            sensor.buffer();

        }

        file.close();

    }

    return 0;
}
