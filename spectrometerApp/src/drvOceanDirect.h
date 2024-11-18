#ifndef DRVOCEANDIRECTAPI_H
#define DRVOCEANDIRECTAPI_H

/*==============================================================
#  Abs: C++ header for Ocean Optics USB spectrometer driver, using the OceanDirect API.
#
#  Name: drvOceanDirect.h
#
#  Desc: This file should be included in drvOceanDirect.cpp
#
#  Facility: FACET
#
#  Auth: 18-Nov-2024, M. Dunning (mdunning)
#==============================================================*/

#include "asynPortDriver.h"

#define acquireString            "ACQUIRE"          /* asynInt32         r/w */
#define updateTimeString         "UPDATE_TIME"      /* asynFloat64       r/w */
#define updateRateActString      "UPDATE_RATE_ACT"  /* asynFloat64       r/o */
#define updateTimeActString      "UPDATE_TIME_ACT"  /* asynFloat64       r/o */
#define apiVersionString         "API_VER"          /* asynFloat64       r/o */
#define serialNumString          "SERIAL_NUM"       /* asynOctet         r/o */
#define modelString              "MODEL"            /* asynOctet         r/o */
#define wavelengthsString        "WAVELENGTHS"      /* asynFloat64Array  r/o */
#define spectrumString           "SPECTRUM"         /* asynFloat64Array  r/o */
#define spectrumLengthString     "SPEC_LEN"         /* asynInt32         r/o */
#define minIntegrationTimeString "MIN_INT_TIME"     /* asynFloat64       r/o */
#define integrationTimeString    "INT_TIME"         /* asynFloat64       r/w */
#define shutterString            "SHUTTER"          /* asynInt32         r/w */
#define numAveString             "NUM_AVE"          /* asynInt32         r/w */
#define smoothingWidthString     "SMO_WID"          /* asynInt32         r/w */
#define subtractBkgString        "BKG"              /* asynInt32         r/w */
#define getBkgString             "GET_BKG"          /* asynInt32         r/w */
#define clearBkgString           "CLEAR_BKG"        /* asynInt32         r/w */
#define connStatusString         "CONN"             /* asynInt32         r/o */
#define reconnectString          "RECONN"           /* asynInt32         r/w */
#define trigModeString           "TRIG_MODE"        /* asynInt32         r/w */
#define eventCodeString          "EVENT_CODE"       /* asynInt32         r/w */


class drvOceanDirect : public asynPortDriver {
public:
    drvOceanDirect(const char* port, const char* serialNum, int numPixels);
    virtual ~drvOceanDirect();
    virtual void pollerThread();
    static void exitHandler(void*);

    /* These are the methods that we override from asynPortDriver */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);

protected:
    int P_acquire;
    int P_updateTime;
    int P_updateTimeAct;
    int P_updateRateAct;
    int P_apiVersion;
    int P_serialNum;
    int P_model;
    int P_wavelengths;
    int P_spectrum;
    int P_spectrumLength;
    int P_minIntegrationTime;
    int P_integrationTime;
    int P_shutter;
    int P_numAve;
    int P_smoothingWidth;
    int P_subtractBkg;
    int P_getBkg;
    int P_clearBkg;
    int P_conn;
    int P_reconn;
    int P_trigMode;
    int P_eventCode;

private:
    bool _running;
    bool _exited;
    int _error_count;
    epicsEventId _event_id;
    std::vector<double> _wavelengths;
    std::vector<double> _spectrum;
    std::vector<double> _background_spectrum;
    int _spectrum_length;
    double _update_time;
    double _min_update_time;
    std::string _serial_num;
    long _device_id;
    long _min_integration_time;
    bool _connected;
    bool _connect();
    bool _find_device_by_serial_number(long device_id);
    bool _get_device_features();
    void _get_spectrum();
    void _get_int_time();
    void _set_int_time(epicsFloat64 value);
    void _get_shutter();
    void _get_trig_mode();
    void _get_num_ave();
    void _get_smoothing_width();
    int _check_error(long index, int error, const std::string& func);
    bool _has_shutter;
    bool _has_temperature;
    bool _has_nonlinear_cal;
    bool _has_autonull;
};

#endif

