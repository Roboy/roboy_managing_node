#include "interface.hpp"
namespace ncursesInterface {
//! standard query messages
	char welcomestring[] = "commandline tool for controlling myode muscle via de0-nano setup";
	char commandstring[] = "[0]changeSetPoint, [1]setAllToDisplacement, [9]exit";
	char setpointstring[] = "set point (ticks) ?";
	char setvelstring[] = "set velocity (ticks/s) ?";
	char setdisplacementstring[] = "set displacement (ticks)?";
	char motorstring[] = "which motor?";
	char motorinfo[30];
	char runningstring[] = "running ";
	char recordingstring[] = "recording ";
	char donestring[] = "done ";
	char samplingtimestring[] = "samplingTime [milliseconds]: ";
	char recordtimestring[] = "recordTime [seconds]: ";
	char invalidstring[] = "invalid!";
	char quitstring[] = " [hit q to quit]";
	char averageconnectionspeedstring[] = "average connection speed: ";
	char logfilestring[] = "see logfile measureConnectionTime.log for details";
	char filenamestring[] = "enter filename to save recorded trajectories: ";
	char remotecontrolactivestring[] = "remote control active [hit q to quit]";
	char publishingmotorstring[] = "publishing motor status[hit q to quit]";
	char receivedupdatestring[] = "received update";
	char errormessage[] = "Error: received update for motor that is not connected";
	char byebyestring[] = "BYE BYE!";

	enum COLORS {
		CYAN = 1,
		RED,
		GREEN,
	};

	Interface::Interface(int argc, char *argv[]) {
		myoMaster = new MyoMaster(argc, argv);
		//! start ncurses mode
		initscr();
		//! Start color functionality
//        start_color();
		init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
		init_pair(RED, COLOR_RED, COLOR_BLACK);
		init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
		//! get the size of the terminal window
		getmaxyx(stdscr, rows, cols);

		print(0, 0, cols, "-");
		printMessage(1, 0, welcomestring);
		print(2, 0, cols, "-");
		print(6, 0, cols, "-");
		querySensoryData();
		printMessage(3, 0, commandstring);
	}

	Interface::~Interface() {
		delete myoMaster;
		clearAll(0);
		printMessage(rows / 2, cols / 2 - strlen(byebyestring) / 2, byebyestring);
		refresh();
		usleep(1000000);
		endwin();
	}

	void Interface::printMessage(uint row, uint col, char *msg) {
		mvprintw(row, col, "%s", msg);
		refresh();
	}

	void Interface::printMessage(uint row, uint col, char *msg, uint color) {
		mvprintw(row, col, "%s", msg);
		mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
		refresh();
	}

	void Interface::print(uint row, uint startcol, uint length, const char *s) {
		for (uint i = startcol; i < startcol + length; i++) {
			mvprintw(row, i, "%s", s);
		}
		refresh();
	}

	void Interface::clearAll(uint row) {
		for (uint i = row; i < rows; i++) {
			print(i, 0, cols, " ");
		}
		refresh();
	}

	void Interface::querySensoryData() {
//		sprintf(motorinfo, "motor %d   ", motor_id);
//		printMessage(7, 0, motorinfo, CYAN);
//		mvprintw(9, 0, "actuatorPos :        %d\t\t 0x%032x        ", pos, pos);
//		mvprintw(12, 0, "tendonDisplacement: %d\t\t 0x%032x        ", displacement, displacement);
		refresh();
	}

	void Interface::processing(char *msg1, char *what, char *msg2) {
		char cmd;
		uint a = strlen(msg1);
		uint b = strlen(what);
		uint c = strlen(msg2);

		print(5, 0, cols, " ");
		printMessage(5, 0, msg1);
		printMessage(5, a + 1, what);
		printMessage(5, a + 1 + b + 1, msg2);
		mvchgat(5, 0, a + 1 + b, A_BLINK, 2, NULL);
		mvchgat(5, a + 1 + b + 1, a + 1 + b + 1 + c, A_BLINK, 1, NULL);
		timeout(timeout_ms);
		do {
			querySensoryData();
			cmd = mvgetch(5, a + 1 + b + 1 + c);
		} while (cmd != 'q');
		timeout(-1);
	}

	void Interface::processing(char *msg1, char *msg2) {
		char cmd;
		uint a = strlen(msg1);
		uint c = strlen(msg2);

		print(5, 0, cols, " ");
		printMessage(5, 0, msg1);
		printMessage(5, a + 1, msg2);
		mvchgat(5, 0, a, A_BLINK, 2, NULL);
		mvchgat(5, a + 1, a + 1 + c, A_BLINK, 1, NULL);
		timeout(timeout_ms);
		do {
			querySensoryData();
			cmd = mvgetch(5, a + 1 + c);
		} while (cmd != 'q');
		timeout(-1);
	}

	void Interface::changeSetpoint() {
		timeout(-1);
		echo();
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		myoMaster->changeControl(motor_id, POSITION);
		printMessage(4, 0, setpointstring);
		mvchgat(4, 0, strlen(setpointstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5, 0, inputstring, 30);
		pos = atoi(inputstring);
		myoMaster->changeSetPoint(motor_id, pos);
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		noecho();
	}

	void Interface::setAllToDisplacement() {
		timeout(-1);
		echo();
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		printMessage(4, 0, setdisplacementstring);
		mvchgat(4, 0, strlen(setdisplacementstring), A_BOLD, 1, NULL);
		refresh();
		mvgetnstr(5, 0, inputstring, 30);
		pos = atoi(inputstring);
		myoMaster->changeControl(DISPLACEMENT);
		myoMaster->sendControllerConfig();
		myoMaster->changeSetPoint(pos);
		processing(runningstring, inputstring, quitstring);
		// set back to zero force
		myoMaster->changeSetPoint(0);
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		noecho();
	}
}
