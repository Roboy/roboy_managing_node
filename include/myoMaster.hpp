#pragma once

#include "xap.h"

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <system/system.h>
#include <obdcreate/obdcreate.h>
#include <console/console.h>
#include <eventlog/eventlog.h>

#if defined(CONFIG_USE_PCAP)
#include <pcap/pcap-console.h>
#endif

#include <stdio.h>
#include <limits.h>
#include <string.h>

#include <arpa/inet.h>
#include <user/sdoudp.h>
#include <oplk/frame.h>
#include <obdpi.h>

#define CYCLE_LEN           UINT_MAX
#define NODEID              0xF0                //=> MN
#define IP_ADDR             0xc0a86401          // 192.168.100.1
#define SUBNET_MASK         0xFFFFFF00          // 255.255.255.0
#define DEFAULT_GATEWAY     0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID

//#define RUN_IN_THREAD

#include "UDPSocket.hpp"
#include "myoFPGA.pb.h"

#include <thread>
#include <sys/stat.h>
#include <ros/ros.h>
#include <communication/MotorConfig.h>
#include <communication/MotorStatus.h>
#include <communication/MotorCommand.h>


typedef struct
{
    char            cdcFile[256];
    char*           pLogFile;
    tEventlogFormat logFormat;
    UINT32          logLevel;
    UINT32          logCategory;
    char   devName[128];
} tOptions;

class MyoMaster{
public:
    MyoMaster(int argc, char* argv[]);
    ~MyoMaster();
    /**
	 * This is the main loop, receiving commands and sending motor status via powerlink
	 */
    void mainLoop();
    /**
     * Changes the control mode of a motor
     * @param motor for this motor
     * @param mode POSITION, VECLOCITY, DISPLACEMENT
     * @return
     */
    void changeControl(int motor, int mode);
    /**
     * Changes the control mode of all motors
     * @param mode POSITION, VECLOCITY, DISPLACEMENT
     * @return
     */
    void changeControl(int mode);
    /**
     * Changes the setPoint of a motor
     * @param motor for this motor
     * @param setPoint to this setpoint
     * @return
     */
    void changeSetPoint(int motor, int setPoint);
    /**
     * Changes the setPoint of all motor
     * @param setPoint to this setpoint
     * @return
     */
    void changeSetPoint(int setPoint);
    /**
     * This actually updates the controller parameters by sending a UDP package with the config to the myoSlave
     */
    void sendControllerConfig();
private:
    /**
     * This is the callback for motor status
     * @param msg status message
     */
    void MotorStatus(const communication::MotorStatus::ConstPtr &msg);
    /**
     * This is the callback for motor commands
     * @param msg motor command message
     */
    void MotorCommand(const communication::MotorCommand::ConstPtr &msg);
    /**
     * This initializes the process image for openPowerLink
     * @return errorCode
     */
    tOplkError initProcessImage();
    /**
     * The function parses the supplied command line parameters and stores the
     * options at pOpts_p.
     * @param argc_p Argument count.
     * @param argv_p Pointer to arguments.
     * @param pOpts_p Pointer to store options
     * @return The function returns the parsing status.
     * @retval 0 Successfully parsed
     * @retval -1 Parsing error
    */
    int getOptions(int argc_p, char* const argv_p[], tOptions* pOpts_p);
    /**
     * The function initializes the openPOWERLINK stack.
     * @param cycleLen_p          Length of POWERLINK cycle.
     * @param devName_p           Device name string.
     * @param macAddr_p           MAC address to use for POWERLINK interface.
     * @param nodeId_p            POWERLINK node ID.
     * @return The function returns a tOplkError error code.
    */
    tOplkError initPowerlink(UINT32 cycleLen_p,
                             const char* cdcFileName_p,
                             const char* devName_p,
                             const UINT8* macAddr_p);
    /**
     * The function implements the synchronous data handler.
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processSync();
    /** Shuts down powerLink */
    void shutdownPowerlink();
    /**
     * The function processes error and warning events.
     * @param  EventType_p         Type of event
     * @param  pEventArg_p         Pointer to union which describes the event in detail
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processEvents(tOplkApiEventType EventType_p,
                                    const tOplkApiEventArg* pEventArg_p,
                                    void* pUserArg_p);
    /**
     * The function processes state change events.
     * @param  pNmtStateChange_p   Pointer to the state change structure
     * @param[in]      pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                              void* pUserArg_p);
    /**
     * The function processes error and warning events.
     * @param  pInternalError_p    Pointer to the error structure
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processErrorWarningEvent(const tEventError* pInternalError_p, void* pUserArg_p);
    /**
     * Process history events
     * @param  pHistoryEntry_p    Pointer to the history entry
     * @param  pUserArg_p         User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p, void* pUserArg_p);
    /**
     * The function processes node events.
     * @param  pNode_p             Pointer to the node event
     * @param  pUserArg_p         User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processNodeEvent(const tOplkApiEventNode* pNode_p, void* pUserArg_p);
    /**
     * The function processes PDO change events.
     * @param  pPdoChange_p        Pointer to the PDO change event structure
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p, void* pUserArg_p);
    /**
     * The function processes CFM progress events.
     * @param  pCfmProgress_p      Pointer to the CFM progress information
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p, void* pUserArg_p);
    /**
     * The function processes CFM result events.
     * @param  pCfmResult_p        Pointer to the CFM result information
     * @param  pUserArg_p          User specific argument
     * @return The function returns a tOplkError error code.
    */
    static tOplkError processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p, void* pUserArg_p);
    static UINT             cnt_l;
    static PI_IN*           pProcessImageIn_l;
    static const PI_OUT*    pProcessImageOut_l;
    static bool updateControllerConfig;
    BOOL         fGsOff_l;
    static UDPSocket *motorCommandSocket;
    static control_Parameters_t MotorConfig;
    static int32_t setPoints[14];
    std::thread *powerLinkThread;
    static bool fExit;
    ros::NodeHandlePtr nh;
    static ros::Publisher motorConfig;
    ros::Subscriber motorStatus, motorCommand;
};