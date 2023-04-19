
/*==============================================================
#  Abs: C++ source for Ocean Optics USB spectrometer driver, using the SeaBreeze API.
#
#  Name: drvSeaBreezeAPI.cpp
#
#  Desc: This file should include drvSeaBreezeAPI.h
#
#  Facility: FACET
#
#  Auth: 23-Feb-2023, M. Dunning (mdunning)
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
//#include <dbAccess.h>
#include <errlog.h>

#include "drvSeaBreezeAPI.h"
#include "api/seabreezeapi/SeaBreezeAPI.h"
#include "api/seabreezeapi/SeaBreezeAPIConstants.h"

static const std::string driverName = "drvSeaBreezeAPI";

static void pollerThreadC(void * pPvt) {
    drvSeaBreezeAPI *pdrvSeaBreezeAPI = (drvSeaBreezeAPI *)pPvt;
    pdrvSeaBreezeAPI->pollerThread();
}


/* Constructor for the drvSeaBreezeAPI class */
drvSeaBreezeAPI::drvSeaBreezeAPI(const char *port, const char* serialNum, int numPixels):
        asynPortDriver(port, 1,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. This driver blocks and it is not multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/
                    _running(true),
                    _exited(false),
                    _spectrum_length(numPixels),
                    _min_update_time(0.01),
                    _serial_num(serialNum),
                    _device_id(0),
                    _feature_id(0),
                    _min_integration_time(0),
                    _connected(0)
{
    const std::string functionName = "drvSeaBreezeAPI";
    
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
    createParam(connStatusString,          asynParamInt32,         &P_conn);
    createParam(reconnectString,           asynParamInt32,         &P_reconn);

    // Start the main thread
    epicsThreadId tid = epicsThreadCreate("drvSeaBreezeAPIMain",
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

drvSeaBreezeAPI::~drvSeaBreezeAPI() {
/*-----------------------------------------------------------
    Destructor.
--------------------------------------------------------------- */
    const std::string functionName = "~drvSeaBreezeAPI";
    int error;
  
    lock();
    _running = false;
    epicsEventSignal(_eventId);
    unlock();
    
    while(!_exited) {
        epicsThreadSleep(0.2);
    }

    // Close device connection & clean up
    lock();
    if (_connected && _device_id) {
        sbapi_close_device(_device_id, &error);
    }
    sbapi_shutdown();
    unlock();

    // Free memory
    delete[] _wavelengths;
    delete[] _spectrum;
    delete[] _background_spectrum;
    
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Exiting...\n", driverName.c_str(), functionName.c_str());
}

void drvSeaBreezeAPI::pollerThread() {
/*-------------------------------------------------------------------- 
    This function runs in a separate thread.  
----------------------------------------------------------------------*/
    std::string functionName = "pollerThread";
    int num_params = 0, acquire = 0;
    long poll_count = 0;
    epicsTimeStamp ts1, ts2;
    double tdiff = 0.0, update_time = 0.5;
    asynStatus status = asynSuccess;
    _eventId = epicsEventCreate(epicsEventEmpty);
    
    // Create memory for waveforms
    _wavelengths = new double[_spectrum_length]();
    _spectrum = new double[_spectrum_length]();
    _background_spectrum = new double[_spectrum_length]();

    lock();
    
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

    // Initialize the API
    sbapi_initialize();
    epicsThreadSleep(0.2);

    // Find and connect to spectrometer
    _connect();

    unlock();

    while(_running) {
        lock();

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
        // acquire could have changed while we were waiting
        getIntegerParam(P_acquire, &acquire);
        if (!acquire) {
            unlock();
            continue;
        }

        // If not connected, set alarms and try to reconnect periodically
        if (!_connected) {
            setIntegerParam(P_conn, _connected);
            callParamCallbacks();
            for (int i=0; i<num_params; i++) {
                if (i == P_conn) continue;
                setParamAlarmStatus(i, COMM_ALARM);
                setParamAlarmSeverity(i, INVALID_ALARM);
            }
            if (poll_count % 100 == 0) {
                _connect();
            }
        }

        // Poll device and measure elapsed time
        epicsTimeGetCurrent(&ts1);
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


bool drvSeaBreezeAPI::_connect() {
/*--------------------------------------------------------------------
Find and connect to spectrometer.
----------------------------------------------------------------------*/
    std::string functionName = "connect";
    int error;
    int num_probed_devices = 0, num_device_ids = 0, status = asynSuccess, num_params = 0;
    bool found = false;

    num_probed_devices = sbapi_probe_devices();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: sbapi_probe_devices() found %d devices\n",
            driverName.c_str(), functionName.c_str(), num_probed_devices);

    num_device_ids = sbapi_get_number_of_device_ids();
    if (num_device_ids <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: sbapi_get_number_of_device_ids found %d device IDs\n",
                driverName.c_str(), functionName.c_str(), num_device_ids);
        return false;
    }
    
    long* ids = new long[num_device_ids];
    num_device_ids = sbapi_get_device_ids(ids, num_device_ids);
    if (num_device_ids <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: sbapi_get_device_ids found %d device IDs\n",
                driverName.c_str(), functionName.c_str(), num_device_ids);
        delete[] ids;
        return false;
    }
    
    for (int i=0; i<num_device_ids; i++) {
        int rc = sbapi_open_device(ids[i], &error);
        _check_error(ids[i], error, "sbapi_open_device");
        if (rc == 0) {
            char* device_type_buf = new char[64];
            int flag = sbapi_get_device_type(ids[i], &error, device_type_buf, sizeof(device_type_buf));
            _check_error(ids[i], error, "sbapi_get_device_type");
            if (flag && (error == 0)) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: Opened device 0x%02lX (%s)\n",
                        driverName.c_str(), functionName.c_str(), ids[i], device_type_buf);
            }
            delete[] device_type_buf;
            found = _find_device_by_serial_number(ids[i]);
            if (found) break;
        }
    }

    delete[] ids;
    
    if (found) {
        epicsPrintf("%s::%s: Found spectrometer with serial number %s\n",
                driverName.c_str(), functionName.c_str(), _serial_num.c_str());
        _connected = true;
        status = getNumParams(&num_params);
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: Failed to get number of parameters\n",
                    driverName.c_str(), functionName.c_str());
        }
        for (int i=0; i<num_params; i++) {
            setParamAlarmStatus(i, NO_ALARM);
            setParamAlarmSeverity(i, NO_ALARM);
        }
        _get_device_features();
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Spectrometer with serial number %s not found\n",
                driverName.c_str(), functionName.c_str(), _serial_num.c_str());
        _connected = false;
    }
    
    setIntegerParam(P_conn, _connected);
    callParamCallbacks();
    return _connected;
}


bool drvSeaBreezeAPI::_find_device_by_serial_number(long device_id) {
/*-------------------------------------------------------------------- 
Find device given a serial number.
----------------------------------------------------------------------*/
    std::string functionName = "_find_device_by_serial_number";
    int error;
    int num_serial_numbers = 0;

    num_serial_numbers = sbapi_get_number_of_serial_number_features(device_id, &error);
    _check_error(device_id, error, "sbapi_get_number_of_serial_number_features");
    if (!num_serial_numbers) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Could not find serial number features\n",
                driverName.c_str(), functionName.c_str());
        return false;
    }

    long* serial_number_ids = new long[num_serial_numbers];
    num_serial_numbers = sbapi_get_serial_number_features(device_id, &error,
            serial_number_ids, sizeof(serial_number_ids));
    _check_error(device_id, error, "sbapi_get_serial_number_features");

    if (num_serial_numbers) {
        // Assume only 1 SN per device
        long serial_number_id = serial_number_ids[0];
        delete[] serial_number_ids;
        int serial_number_length = sbapi_get_serial_number_maximum_length(device_id,
                serial_number_id, &error);
        _check_error(device_id, error, "sbapi_get_serial_number_maximum_length");
        if (serial_number_length) {
            char* serial_number_buf = new char[serial_number_length];
            int serial_number_bytes = sbapi_get_serial_number(device_id, serial_number_id, &error,
                    serial_number_buf, sizeof(serial_number_buf));
            _check_error(device_id, error, "sbapi_get_serial_number");
            if (serial_number_bytes && (error == 0)) {
                std::string serial = serial_number_buf;
                delete[] serial_number_buf;
                if (serial.find(_serial_num) != std::string::npos) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                            "%s::%s: Matched serial number %s\n",
                            driverName.c_str(), functionName.c_str(), _serial_num.c_str());
                    _device_id = device_id;
                    setStringParam(P_serialNum, _serial_num);
                    callParamCallbacks();
                    return true;
                }
            }
        }
    }

    return false;
}


bool drvSeaBreezeAPI::_get_device_features() {
/*-------------------------------------------------------------------- 
Get spectrometer fearures and push to records.
----------------------------------------------------------------------*/
    std::string functionName = "_get_device_features";
    char buf[32];
    int error, spectrum_length = 0, num_features = 0;
  
    num_features = sbapi_get_number_of_spectrometer_features(_device_id, &error); 
    _check_error(_device_id, error, "sbapi_get_number_of_spectrometer_features");
    if (!num_features) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Device 0x%02lX has no features\n",
                driverName.c_str(), functionName.c_str(), _device_id);
        return false;
    }

    long* features = new long[num_features];
    num_features = sbapi_get_spectrometer_features(_device_id, &error,
            features, sizeof(features));
    _check_error(_device_id, error, "sbapi_get_spectrometer_features");
    if (!num_features) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Device 0x%02lX: could not get features\n",
                driverName.c_str(), functionName.c_str(), _device_id);
        delete[] features;
        return false;
    }

    // Assume only 1 optical bench (spectrometer) per device
    _feature_id = features[0];
    delete[] features;

    sbapi_get_device_type(_device_id, &error, buf, sizeof(buf));
    _check_error(_device_id, error, "sbapi_get_device_type");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrometer model: %s\n",
            driverName.c_str(), functionName.c_str(), buf);
    setStringParam(P_model, buf);

    _min_integration_time = sbapi_spectrometer_get_minimum_integration_time_micros(_device_id,
            _feature_id, &error);
    _check_error(_device_id, error, "sbapi_spectrometer_get_minimum_integration_time_micros");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrometer min. integration time: %ld us\n",
            driverName.c_str(), functionName.c_str(), _min_integration_time);
    setDoubleParam(P_minIntegrationTime, _min_integration_time/1000.0);

    spectrum_length = sbapi_spectrometer_get_formatted_spectrum_length(_device_id,
            _feature_id, &error);
    _check_error(_device_id, error, "sbapi_spectrometer_get_formatted_spectrum_length");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrum length: %d pixels\n",
            driverName.c_str(), functionName.c_str(), spectrum_length);
    setIntegerParam(P_spectrumLength, spectrum_length);
    if ((spectrum_length <= 0) || (spectrum_length != _spectrum_length)) {
        return false;
    }
        
    callParamCallbacks();
    return true;
}


void drvSeaBreezeAPI::_get_spectrum() {
/*-------------------------------------------------------------------- 
Get wavelengths and spectrum arrays, optionally subtract background, push to records.
----------------------------------------------------------------------*/
    std::string functionName = "_get_spectrum";
    int error;
    int subtract_background = 0;
    asynStatus status = asynSuccess;
    int spectrum_length = 0;

    spectrum_length = sbapi_spectrometer_get_formatted_spectrum_length(_device_id,
            _feature_id, &error);
    _check_error(_device_id, error, "sbapi_spectrometer_get_formatted_spectrum_length");
    if ((spectrum_length <= 0) || (spectrum_length != _spectrum_length)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Error getting spectrum length\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

    spectrum_length = sbapi_spectrometer_get_wavelengths(_device_id,
            _feature_id, &error, _wavelengths, _spectrum_length);
    _check_error(_device_id, error, "sbapi_spectrometer_get_wavelengths");
    // If we get a transfer error, the device has been disconnected
    if (error == ERROR_TRANSFER_ERROR) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Device 0x%02lX disconnected, error=%d\n",
                driverName.c_str(), functionName.c_str(), _device_id, error);
        _connected = false;
        sbapi_close_device(_device_id, &error);
        return;
    } else if ((spectrum_length <= 0) || (spectrum_length != _spectrum_length)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Error getting wavelengths\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

    spectrum_length = sbapi_spectrometer_get_formatted_spectrum(_device_id,
            _feature_id, &error, _spectrum, _spectrum_length);
    _check_error(_device_id, error, "sbapi_spectrometer_get_formatted_spectrum");
    if ((spectrum_length <= 0) || (spectrum_length != _spectrum_length)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Error getting spectrum\n",
                driverName.c_str(), functionName.c_str());
        return;
    }

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


int drvSeaBreezeAPI::_check_error(long index, int error, const std::string& func) {
/*-------------------------------------------------------------------- 
Given an error code, get error string.
----------------------------------------------------------------------*/
    std::string functionName = "_check_error";
    const char* err_buf;

    err_buf = sbapi_get_error_string(error);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: %s: error code=%d (%s) for device 0x%02lX\n",
            driverName.c_str(), functionName.c_str(), func.c_str(), error, err_buf, index);
    return error;
}


asynStatus drvSeaBreezeAPI::writeInt32(asynUser *pasynUser, epicsInt32 value) {
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
    } else if (function == P_reconn) {
        if (value && !_connected) {
            _connect();
            epicsEventSignal(_eventId);
        }
    } else if (function == P_shutter) {
        if (_connected) {
            int num_shutter_features = sbapi_get_number_of_shutter_features(_device_id, &error);
            _check_error(_device_id, error, "sbapi_get_number_of_shutter_features");
            if (num_shutter_features) {
                long* features;
                features = new long[num_shutter_features];
                num_shutter_features = sbapi_get_shutter_features(_device_id,
                        &error, features, sizeof(features));
                if (num_shutter_features) {
                    // Assume only 1 shutter per device
                    long feature = features[0];
                    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                            "%s::%s: Setting shutter (%d) for device 0x%02lX\n",
                            driverName.c_str(), functionName.c_str(), value, _device_id);
                    sbapi_shutter_set_shutter_open(_device_id, feature, &error, (unsigned char)value);
                    _check_error(_device_id, error, "sbapi_shutter_set_shutter_open");
                }
            } else {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: Device 0x%02lX has no shutter features\n",
                        driverName.c_str(), functionName.c_str(), _device_id);
            }
        }
    } else if (function == P_getBkg) {
        if (_connected) {
            sbapi_spectrometer_get_formatted_spectrum(_device_id,
            _feature_id, &error, _background_spectrum, _spectrum_length);
            _check_error(_device_id, error, "sbapi_spectrometer_get_formatted_spectrum");
        }
    } else if (function == P_clearBkg) {
        for (int i=0; i<_spectrum_length; i++) {
            _background_spectrum[i] = 0.0;
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


asynStatus drvSeaBreezeAPI::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
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
                sbapi_spectrometer_set_integration_time_micros(_device_id,
                        _feature_id, &error, value*1000);
                _check_error(_device_id, error, "sbapi_spectrometer_set_integration_time_micros");
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


void drvSeaBreezeAPI::exitHandler(void *arg) {
/*---------------------------------------------------------
    Exit handler, delete the drvSeaBreezeAPI object.
-----------------------------------------------------------*/
    const std::string functionName = "exitHandler";

    drvSeaBreezeAPI *pdrvSeaBreezeAPI = (drvSeaBreezeAPI *)arg;
    if (pdrvSeaBreezeAPI) delete pdrvSeaBreezeAPI;
}

// Configuration routine.  Called directly, or from the iocsh function below
extern "C" {
int drvSeaBreezeAPIConfigure(const char* port, const char* serialNum, int numPixels) {
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvSeaBreezeAPI class.
 *  port    The name of the asyn port driver to be created.
 *----------------------------------------------------------------------------*/
    new drvSeaBreezeAPI(port, serialNum, numPixels);
    return asynSuccess;
}

static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"serialNum", iocshArgString};
static const iocshArg initArg2 = {"numPixels", iocshArgInt};
static const iocshArg* const initArgs[] = {&initArg0, &initArg1, &initArg2};
static const iocshFuncDef initFuncDef = {"drvSeaBreezeAPIConfigure", 3, initArgs};

static void initCallFunc(const iocshArgBuf *args) {
    drvSeaBreezeAPIConfigure(args[0].sval, args[1].sval, args[2].ival);
}

void drvSeaBreezeAPIRegister(void){
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvSeaBreezeAPIRegister);
}

