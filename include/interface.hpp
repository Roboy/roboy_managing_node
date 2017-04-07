#pragma once

#include "myoMaster.hpp"

namespace ncursesInterface {

#include <ncurses.h>
#include <vector>
#include <cstring>
#include <unistd.h>

    using namespace std;

    class Interface {
    public:
        Interface(int argc, char *argv[]);

        ~Interface();

        void printMessage(uint row, uint col, char *msg);

        void printMessage(uint row, uint col, char *msg, uint color);

        void print(uint row, uint startcol, uint length, const char *s);

        void clearAll(uint row);

        void querySensoryData();

        void processing(char *msg1, char *what, char *msg2);

        void processing(char *msg1, char *msg2);

        void toggleSPI();

        void reset();

        void changeSetpoint();

        void setAllToDisplacement();

        MyoMaster *myoMaster;
        uint timeout_ms = 10;
    private:
        uint rows, cols;
        int pos;
        uint ganglion_id = 0;
        uint motor_id = 0;
        char inputstring[30];
        WINDOW *window;
    };
}