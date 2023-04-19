
/*==============================================================
#  Abs: C++ source for Ocean Optics USB spectrometer driver, using the SeaBreezeWrapper API.
#
#  Name: drvSeaBreezeWrapper.cpp
#
#  Desc: This file should include drvSeaBreezeWrapper.h
#
#  Facility: LCLS
#
#  Auth: 07-Feb-2023, M. Dunning (mdunning)
===============================================================*/

#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>

#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsTime.h>
#include <iocsh.h>
#include <alarm.h>
#include <dbAccess.h>
#include <errlog.h>

#include "drvSeaBreezeWrapper.h"
#include "api/SeaBreezeWrapper.h"

static const std::string driverName = "drvSeaBreezeWrapper";

static void pollerThreadC(void * pPvt) {
    drvSeaBreezeWrapper *pdrvSeaBreezeWrapper = (drvSeaBreezeWrapper *)pPvt;
    pdrvSeaBreezeWrapper->pollerThread();
}


/* Constructor for the drvSeaBreezeWrapper class */
drvSeaBreezeWrapper::drvSeaBreezeWrapper(const char *port, const char* serialNum):
        asynPortDriver(port, 1,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. This driver blocks and it is not multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/
                    _running(true),
                    _exited(false),
                    _spectrum_length(0),
                    _min_update_time(0.01),
                    _serial_num(serialNum),
                    _device_index(-1),
                    _min_integration_time(0),
                    _connected(0)
{
    const std::string functionName = "drvSeaBreezeWrapper";
    
    // Asyn parameter table
    createParam(acquireString,             asynParamInt32,         &P_acquire);
    createParam(updateTimeString,          asynParamFloat64,       &P_updateTime);
    createParam(updateTimeActString,       asynParamFloat64,       &P_updateTimeAct);
    createParam(serialNumString,           asynParamOctet,         &P_serialNum);
    createParam(modelString,               asynParamOctet,         &P_model);
    createParam(wavelengthsString,         asynParamFloat64Array,  &P_wavelengths);
    createParam(spectrumString,            asynParamFloat64Array,  &P_spectrum);
    createParam(spectrumLengthString,      asynParamInt32,         &P_spectrumLength);
    createParam(minIntegrationTimeString,  asynParamFloat64,       &P_minIntegrationTime);
    createParam(integrationTimeString,     asynParamFloat64,       &P_integrationTime);
    createParam(shutterString,             asynParamInt32,         &P_shutter);
    createParam(subtractBkgString,         asynParamInt32,         &P_subtractBkg);
    createParam(getBkgString,              asynParamInt32,         &P_getBkg);
    createParam(clearBkgString,            asynParamInt32,         &P_clearBkg);
    createParam(apiVersionString,          asynParamOctet,         &P_apiVersion);
    createParam(apiDebugString,            asynParamInt32,         &P_apiDebug);
    createParam(connStatusString,          asynParamInt32,         &P_conn);

    // Start the main thread
    epicsThreadId tid = epicsThreadCreate("drvSeaBreezeWrapperMain",
                    epicsThreadPriorityMedium,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollerThreadC, this);

    if (!tid) {
        errlogPrintf("%s:%s: epicsThreadCreate failure\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

    // Create an EPICS exit handler
    epicsAtExit(exitHandler, (void*)this);
}

drvSeaBreezeWrapper::~drvSeaBreezeWrapper() {
/*-----------------------------------------------------------
    Destructor.
--------------------------------------------------------------- */
    const std::string functionName = "~drvSeaBreezeWrapper";
    int error;
  
    lock();
    _running = false;
    epicsEventSignal(_eventId);
    unlock();
    
    while(!_exited) {
        epicsThreadSleep(0.2);
    }

    if (_connected) {
        seabreeze_close_spectrometer(_device_index, &error);
        _check_error(_device_index, error, "seabreeze_close_spectrometer");
        seabreeze_shutdown();
    }

    // Free memory
    delete[] _wavelengths;
    delete[] _spectrum;
    delete[] _background_spectrum;
    
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Exiting...\n", driverName.c_str(), functionName.c_str());
}

void drvSeaBreezeWrapper::pollerThread() {
/*-------------------------------------------------------------------- 
    This function runs in a separate thread.  
----------------------------------------------------------------------*/
    std::string functionName = "pollerThread";
    int num_params = 0, acquire = 0;
    long poll_count = 0;
    epicsTimeStamp ts1, ts2;
    double tdiff = 0.0, update_time = 0.5;
    asynStatus status = asynSuccess;
    
    status = getNumParams(&num_params);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Failed to get number of parameters\n",
                driverName.c_str(), functionName.c_str());
    }

    // Set some initial conditions
    setIntegerParam(P_acquire, acquire);
    setDoubleParam(P_updateTime, update_time);
    setIntegerParam(P_subtractBkg, 0);
    callParamCallbacks();

    // Wait until iocInit is finished
    while (!interruptAccept) {
        epicsThreadSleep(0.2);
    }

    _eventId = epicsEventCreate(epicsEventEmpty);

    // Get API version
    _get_api_version();

    // Find and connect to spectrometer
    lock();
    _connected = _connect();
    setIntegerParam(P_conn, _connected);
    callParamCallbacks();
    unlock();

    // Create memory for waveforms
    _wavelengths = new double[_spectrum_length]();
    _spectrum = new double[_spectrum_length]();
    _background_spectrum = new double[_spectrum_length]();

    while(_running) {
        lock();

        // If not connected, set alarms and try to reconnect periodically
        if (!_connected) {
            for (int i=0; i<num_params; i++) {
                if (i == P_conn) continue;
                setParamAlarmStatus(i, COMM_ALARM);
                setParamAlarmSeverity(i, INVALID_ALARM);
            }
            if (poll_count % 100 == 0) {
                _connected = _connect();
                setIntegerParam(P_conn, _connected);
                callParamCallbacks();
                if (_connected) {
                    for (int i=0; i<num_params; i++) {
                        setParamAlarmStatus(i, NO_ALARM);
                        setParamAlarmSeverity(i, NO_ALARM);
                    }
                }
            }
        }

        // Update the poll delay time based on the actual time required to read data
        getDoubleParam(P_updateTime, &update_time);
        if (tdiff >= update_time) {
            update_time = 0.0;
        } else {
            update_time -= tdiff;
        }
        setDoubleParam(P_updateTimeAct, update_time + tdiff);
        callParamCallbacks();

        getIntegerParam(P_acquire, &acquire);
        // Release the lock while we wait for a command to start or wait for updateTime
        unlock();
        if (acquire) {
            epicsEventWaitWithTimeout(_eventId, update_time);
        } else {
            (void)epicsEventWait(_eventId);
        }
        if (!_running) break;
        // Take the lock again
        lock(); 
        /* acquire could have changed while we were waiting */
        getIntegerParam(P_acquire, &acquire);
        if (!acquire) {
            unlock();
            continue;
        }

        // Poll device and measure elapsed time
        epicsTimeGetCurrent(&ts1);
        // TODO: Find a way to check the device connection
        // Device needs to be connected otherwise API calls will crash the program
        if (_connected) {
            _get_spectrum();
        }
        epicsTimeGetCurrent(&ts2);
        tdiff = epicsTimeDiffInSeconds(&ts2, &ts1);
        
        unlock();
        poll_count++;
    } // End of while loop
    
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: Main thread exiting...\n",
            driverName.c_str(), functionName.c_str());
    _exited = true;
}


bool drvSeaBreezeWrapper::_connect() {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "connect";
    int rc;
    int error;
    int device_count = 0;
    bool found = false;
   
    for (int i=0; i<SEABREEZE_MAX_DEVICES; i++) {
        rc = seabreeze_open_spectrometer(i, &error);
        _check_error(_device_index, error, "seabreeze_open_spectrometer");
        if (rc == 0) {
            device_count++;
            found = _find_device_by_serial_number(i);
            if (found) break;
        }
    }

    if (device_count) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Found %d spectrometer(s)\n",
                driverName.c_str(), functionName.c_str(), device_count);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: No spectrometers found\n",
                driverName.c_str(), functionName.c_str());
    }

    if (found) {
        epicsPrintf("%s::%s: Found spectrometer with serial number %s\n",
                driverName.c_str(), functionName.c_str(), _serial_num.c_str());
        
        // Get features, e.g. spectrum length
        _get_device_features();

        return true;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Spectrometer with serial number %s not found\n",
                driverName.c_str(), functionName.c_str(), _serial_num.c_str());
        return false;
    }
}


bool drvSeaBreezeWrapper::_find_device_by_serial_number(int index) {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "_find_device_by_serial_number";
    char buf[32];
    int error;
   
    seabreeze_get_serial_number(index, &error, buf, sizeof(buf));
    _check_error(_device_index, error, "seabreeze_get_serial_number");
    std::string serial = buf;
    if (serial.find(_serial_num) != std::string::npos) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Matched serial number %s\n",
                driverName.c_str(), functionName.c_str(), _serial_num.c_str());
        _device_index = index;
        setStringParam(P_serialNum, buf);
        callParamCallbacks();
        return true;
    }
    return false;
}

void drvSeaBreezeWrapper::_get_api_version() {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "_get_api_version";
    char buf[32];
   
    seabreeze_get_api_version_string(buf, sizeof(buf));
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: SeaBreeze API version: %s\n",
            driverName.c_str(), functionName.c_str(), buf);
    setStringParam(P_apiVersion, buf);
        
    callParamCallbacks();
}



void drvSeaBreezeWrapper::_get_device_features() {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "_get_device_features";
    char buf[32];
    int error;
   
    seabreeze_get_model(_device_index, &error, buf, sizeof(buf));
    _check_error(_device_index, error, "seabreeze_get_model");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrometer model: %s\n",
            driverName.c_str(), functionName.c_str(), buf);
    setStringParam(P_model, buf);

    _min_integration_time = seabreeze_get_min_integration_time_microsec(_device_index, &error);
    _check_error(_device_index, error, "seabreeze_get_min_integration_time_microsec");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrometer min. integration time: %ld us\n",
            driverName.c_str(), functionName.c_str(), _min_integration_time);
    setDoubleParam(P_minIntegrationTime, _min_integration_time/1000.0);

    _spectrum_length = seabreeze_get_formatted_spectrum_length(_device_index, &error);
    _check_error(_device_index, error, "seabreeze_get_formatted_spectrum_length");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrum length: %d pixels\n",
            driverName.c_str(), functionName.c_str(), _spectrum_length);
    setIntegerParam(P_spectrumLength, _spectrum_length);
        
    callParamCallbacks();
}


void drvSeaBreezeWrapper::_get_spectrum() {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "_get_spectrum";
    int error;
    int subtract_background = 0;
    asynStatus status = asynSuccess;

    seabreeze_get_wavelengths(_device_index, &error, _wavelengths, _spectrum_length);
    _check_error(_device_index, error, "seabreeze_get_wavelengths");
    seabreeze_get_formatted_spectrum(_device_index, &error, _spectrum, _spectrum_length);
    _check_error(_device_index, error, "seabreeze_get_formatted_spectrum");

    // Simulated spectrum    
    //double integration_time;
    //for (int i=0; i<_spectrum_length; i++) {
    //    _spectrum[i] = rand() % 100 + 1000;
    //}
    //getDoubleParam(P_integrationTime, &integration_time);
    //epicsThreadSleep(integration_time/1000);

    //for (int i=0; i<_spectrum_length; i++) {
    //    printf("_wavelengths=%d, _spectrum=%f\n", _wavelengths[i], _spectrum[i]);
    //}

    getIntegerParam(P_subtractBkg, &subtract_background);
    if (subtract_background) {
        for (int i=0; i<_spectrum_length; i++) {
            _spectrum[i] -= _background_spectrum[i];
            if (_spectrum[i] < 0.0) {
                _spectrum[i] = 0.0;
            }
        }
    }

    status = doCallbacksFloat64Array(_wavelengths, _spectrum_length, P_wavelengths, 0);
    status = doCallbacksFloat64Array(_spectrum, _spectrum_length, P_spectrum, 0);

    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Error in spectrum callback\n",
                driverName.c_str(), functionName.c_str());
    }
        
}


int drvSeaBreezeWrapper::_check_error(int index, int error, std::string func) {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "_check_error";
    char err_buf[64];
    
    seabreeze_get_error_string(error, err_buf, sizeof(err_buf));
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: %s returned %d (%s) for device %d\n",
            driverName.c_str(), functionName.c_str(), func.c_str(), error, err_buf, index);
    return error;
}


asynStatus drvSeaBreezeWrapper::writeInt32(asynUser *pasynUser, epicsInt32 value) {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    int error;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);
    
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_acquire) {
        /* If acquire was set then wake up the acquisition task */
        if (value) epicsEventSignal(_eventId);
    } else if (function == P_shutter) {
        if (_connected) {
            seabreeze_set_shutter_open(_device_index, &error, (unsigned char)value);
            _check_error(_device_index, error, "seabreeze_set_shutter_open");
        }
    } else if (function == P_getBkg) {
        if (_connected) {
            seabreeze_get_formatted_spectrum(_device_index, &error, _background_spectrum, _spectrum_length);
            _check_error(_device_index, error, "seabreeze_get_formatted_spectrum");
           
            // Simulated spectrum 
            //for (int i=0; i<_spectrum_length; i++) {
            //    _background_spectrum[i] = rand() % 100 + 300;
            //}
        }
    } else if (function == P_clearBkg) {
        for (int i=0; i<_spectrum_length; i++) {
            _background_spectrum[i] = 0.0;
        }
    } else if (function == P_apiDebug) {
        if (_connected) {
            // Enable verbose API output
            seabreeze_set_verbose(value);
            _check_error(_device_index, error, "seabreeze_set_verbose");
        }
    } else {
        /* All other parameters just get set in parameter list, no need to
         * act on them here */
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%d", 
                  driverName.c_str(), functionName.c_str(), status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName.c_str(), functionName.c_str(), function, paramName, value);
    return status;
}


asynStatus drvSeaBreezeWrapper::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
/*-------------------------------------------------------------------- 
----------------------------------------------------------------------*/
    std::string functionName = "writeFloat64";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    epicsInt32 acquire;
    const char *paramName;
    int error;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setDoubleParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_updateTime) {
        /* Make sure the update time is valid. If not change it and put back in parameter library */
        if (value < _min_update_time) {
            asynPrint(pasynUser, ASYN_TRACE_WARNING,
                "%s:%s: warning, update time too small, changed from %f to %f\n", 
                driverName.c_str(), functionName.c_str(), value, _min_update_time);
            value = _min_update_time;
            setDoubleParam(P_updateTime, value);
        }
        /* If the update time has changed and we are acquiring then wake up the acquisition task */
        getIntegerParam(P_acquire, &acquire);
        if (acquire) epicsEventSignal(_eventId);
    } else if (function == P_integrationTime) {
        /* Make sure the integration time is valid. If not change it and put back in parameter library */
        if (value < _min_integration_time/1000.0) {
            asynPrint(pasynUser, ASYN_TRACE_WARNING,
                "%s:%s: warning, integration time too small, changed from %f to %f\n",
                driverName.c_str(), functionName.c_str(), value, _min_integration_time/1000.0);
            value = _min_integration_time/1000.0;
            setDoubleParam(P_integrationTime, value);
        } else {
            if (_connected) {
                seabreeze_set_integration_time_microsec(_device_index, &error, value*1000);
                _check_error(_device_index, error, "seabreeze_set_integration_time_microsec");
            }
        }
    } else {
        /* All other parameters just get set in parameter list, no need to
         * act on them here */
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();
    
    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%f", 
                  driverName.c_str(), functionName.c_str(), status, function, paramName, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%f\n", 
              driverName.c_str(), functionName.c_str(), function, paramName, value);
    return status;
}


void drvSeaBreezeWrapper::exitHandler(void *arg) {
/*---------------------------------------------------------
    Exit handler, delete the drvSeaBreezeWrapper object.
-----------------------------------------------------------*/
    const std::string functionName = "exitHandler";

    //printf("%s::%s\n", driverName.c_str(), functionName.c_str());
    drvSeaBreezeWrapper *pdrvSeaBreezeWrapper = (drvSeaBreezeWrapper *)arg;
    delete pdrvSeaBreezeWrapper;
}


// Configuration routine.  Called directly, or from the iocsh function below
extern "C" {
int drvSeaBreezeWrapperConfigure(const char* port, const char* serialNum) {
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvSeaBreezeWrapper class.
 *  port    The name of the asyn port driver to be created.
 *----------------------------------------------------------------------------*/
    new drvSeaBreezeWrapper(port, serialNum);
    return asynSuccess;
}

static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"serialNum", iocshArgString};
static const iocshArg* const initArgs[] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"drvSeaBreezeWrapperConfigure", 2, initArgs};

static void initCallFunc(const iocshArgBuf *args) {
    drvSeaBreezeWrapperConfigure(args[0].sval, args[1].sval);
}

void drvSeaBreezeWrapperRegister(void){
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvSeaBreezeWrapperRegister);
}

