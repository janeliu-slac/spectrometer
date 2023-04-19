#ifndef DRVSEABREEZEWRAPPER_H
#define DRVSEABREEZEWRAPPER_H

/*==============================================================
#  Abs: C++ header for Ocean Optics USB spectrometer driver, using the SeaBreezeWrapper API.
#
#  Name: drvSeaBreezeWrapper.h
#
#  Desc: This file should be included in drvSeaBreezeWrapper.cpp
#
#  Facility: LCLS
#
#  Auth: 07-Feb-2023, M. Dunning (mdunning)
#==============================================================*/

#include "asynPortDriver.h"

#define acquireString            "ACQUIRE"          /* asynInt32         r/w */
#define updateTimeString         "UPDATE_TIME"      /* asynFloat64       r/w */
#define updateTimeActString      "UPDATE_TIME_ACT"  /* asynFloat64       r/o */
#define serialNumString          "SERIAL_NUM"       /* asynOctet         r/o */
#define modelString              "MODEL"            /* asynOctet         r/o */
#define wavelengthsString        "WAVELENGTHS"      /* asynFloat64Array  r/o */
#define spectrumString           "SPECTRUM"         /* asynFloat64Array  r/o */
#define spectrumLengthString     "SPEC_LEN"         /* asynInt32         r/o */
#define minIntegrationTimeString "MIN_INT_TIME"     /* asynFloat64       r/o */
#define integrationTimeString    "INT_TIME"         /* asynFloat64       r/w */
#define shutterString            "SHUTTER"          /* asynInt32         r/w */
#define subtractBkgString        "BKG"              /* asynInt32         r/w */
#define getBkgString             "GET_BKG"          /* asynInt32         r/w */
#define clearBkgString           "CLEAR_BKG"        /* asynInt32         r/w */
#define apiVersionString         "API_VER"          /* asynOctet         r/o */
#define apiDebugString           "API_DBG"          /* asynInt32         r/w */
#define connStatusString         "CONN"             /* asynInt32         r/o */


class drvSeaBreezeWrapper : public asynPortDriver{
public:
    drvSeaBreezeWrapper(const char* port, const char* serialNum);
    virtual ~drvSeaBreezeWrapper();
    virtual void pollerThread();
    static void exitHandler(void*);

    /* These are the methods that we override from asynPortDriver */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

protected:
    int P_acquire;
    int P_updateTime;
    int P_updateTimeAct;
    int P_serialNum;
    int P_model;
    int P_wavelengths;
    int P_spectrum;
    int P_spectrumLength;
    int P_minIntegrationTime;
    int P_integrationTime;
    int P_shutter;
    int P_subtractBkg;
    int P_getBkg;
    int P_clearBkg;
    int P_apiVersion;
    int P_apiDebug;
    int P_conn;

private:
    bool _running;
    bool _exited;
    epicsEventId _eventId;
    double* _wavelengths;
    double* _spectrum;
    double* _background_spectrum;
    int _spectrum_length;
    double _min_update_time;
    std::string _serial_num;
    int _device_index;
    long _min_integration_time;
    bool _connected;
    bool _connect();
    bool _find_device_by_serial_number(int index);
    void _get_device_features();
    void _get_spectrum();
    void _get_api_version();
    int _check_error(int index, int error, std::string func);
};

#endif

