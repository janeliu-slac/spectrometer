
/*==============================================================
#  Abs: C++ source for Ocean Optics USB spectrometer driver, using the OceanDirect API.
#
#  Name: drvOceanDirect.cpp
#
#  Desc: This file should include drvOceanDirect.h
#
#  Facility: FACET
#
#  Auth: 18-Nov-2024, M. Dunning (mdunning)
===============================================================*/

#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include <memory>
#include <limits>

#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsTime.h>
#ifdef EVR_SUPPORT
#include <evrTime.h>
#endif
#include <iocsh.h>
#include <alarm.h>
#include <errlog.h>

#include "drvOceanDirect.h"
#include "api/OceanDirectAPI.h"
#include "api/OceanDirectAPIConstants.h"

namespace {
    const std::string driverName = "drvOceanDirect";
    enum {SHUTTER_CLOSED, SHUTTER_OPEN, SHUTTER_NA};
    const int MAX_ERROR_COUNT = 5;
    const double DEFAULT_POLL_DELAY = 0.1;
}

static void pollerThreadC(void * pPvt) {
    drvOceanDirect *pdrvOceanDirect = (drvOceanDirect *)pPvt;
    pdrvOceanDirect->pollerThread();
}


/* Constructor for the drvOceanDirect class */
drvOceanDirect::drvOceanDirect(const char *port, const char* serialNum, int numPixels):
        asynPortDriver(port, 1,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask | asynDrvUserMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    ASYN_CANBLOCK, /* asynFlags. This driver blocks and it is not multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/
                    _running(true),
                    _exited(false),
                    _error_count(0),
                    _event_id(epicsEventCreate(epicsEventEmpty)),
                    _spectrum_length(numPixels),
                    _update_time(DEFAULT_POLL_DELAY),
                    _min_update_time(0.001),
                    _serial_num(serialNum),
                    _device_id(0),
                    _min_integration_time(0),
                    _connected(0),
                    _has_shutter(0),
                    _has_temperature(0),
                    _has_nonlinear_cal(0),
                    _has_autonull(0)
{
    const std::string functionName = "drvOceanDirect";
    
    // Asyn parameter table
    createParam(acquireString,             asynParamInt32,         &P_acquire);
    createParam(updateTimeString,          asynParamFloat64,       &P_updateTime);
    createParam(updateTimeActString,       asynParamFloat64,       &P_updateTimeAct);
    createParam(updateRateActString,       asynParamFloat64,       &P_updateRateAct);
    createParam(apiVersionString,          asynParamFloat64,       &P_apiVersion);
    createParam(serialNumString,           asynParamOctet,         &P_serialNum);
    createParam(modelString,               asynParamOctet,         &P_model);
    createParam(wavelengthsString,         asynParamFloat64Array,  &P_wavelengths);
    createParam(spectrumString,            asynParamFloat64Array,  &P_spectrum);
    createParam(spectrumLengthString,      asynParamInt32,         &P_spectrumLength);
    createParam(minIntegrationTimeString,  asynParamFloat64,       &P_minIntegrationTime);
    createParam(integrationTimeString,     asynParamFloat64,       &P_integrationTime);
    createParam(shutterString,             asynParamInt32,         &P_shutter);
    createParam(numAveString,              asynParamInt32,         &P_numAve);
    createParam(smoothingWidthString,      asynParamInt32,         &P_smoothingWidth);
    createParam(subtractBkgString,         asynParamInt32,         &P_subtractBkg);
    createParam(getBkgString,              asynParamInt32,         &P_getBkg);
    createParam(clearBkgString,            asynParamInt32,         &P_clearBkg);
    createParam(connStatusString,          asynParamInt32,         &P_conn);
    createParam(reconnectString,           asynParamInt32,         &P_reconn);
    createParam(trigModeString,            asynParamInt32,         &P_trigMode);
    createParam(eventCodeString,           asynParamInt32,         &P_eventCode);

    // Start the main thread
    epicsThreadId tid = epicsThreadCreate("drvOceanDirectMain",
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

drvOceanDirect::~drvOceanDirect() {
/*-----------------------------------------------------------
    Destructor.
--------------------------------------------------------------- */
    const std::string functionName = "~drvOceanDirect";
    int error;
  
    lock();
    _running = false;
    epicsEventSignal(_event_id);
    unlock();
    
    while(!_exited) {
        epicsThreadSleep(0.2);
    }

    // Close device connection & clean up
    lock();
    if (_connected && _device_id) {
        odapi_close_device(_device_id, &error);
    }
    odapi_shutdown();
    unlock();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Exiting...\n", driverName.c_str(), functionName.c_str());
}

void drvOceanDirect::pollerThread() {
/*-------------------------------------------------------------------- 
    This function runs in a separate thread.  
----------------------------------------------------------------------*/
    std::string functionName = "pollerThread";
    int num_params = 0, acquire = 0;
    long poll_count = 0;
    epicsTimeStamp ts0, ts1, ts2;
    double tdiff = 0.0, tdiff_cycle, poll_delay = DEFAULT_POLL_DELAY;
    asynStatus status = asynSuccess;
    
    status = getNumParams(&num_params);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Failed to get number of parameters\n",
                driverName.c_str(), functionName.c_str());
    }

    lock();
    // Set some initial conditions
    setIntegerParam(P_acquire, acquire);
    setDoubleParam(P_updateTime, poll_delay);
    setDoubleParam(P_updateTimeAct, 0.0);
    setDoubleParam(P_updateRateAct, 0.0);
    setIntegerParam(P_subtractBkg, 0);
    setIntegerParam(P_eventCode, 0);
    callParamCallbacks();

    // Initialize the API
    odapi_initialize();
    epicsThreadSleep(0.2);

    // Find and connect to spectrometer
    _connect();

    // Get initial values
    if (_connected) {
        _get_int_time();
        _get_shutter();
        _get_trig_mode();
        _get_num_ave();
        _get_smoothing_width();
    }
    unlock();

    while(_running) {
        lock();
        epicsTimeGetCurrent(&ts0);

        // Update the poll delay time based on the actual time required to read data
        if (tdiff >= _update_time) {
            poll_delay = 0.0;
        } else {
            poll_delay = _update_time - tdiff;
        }

        getIntegerParam(P_acquire, &acquire);
        // Release the lock while we wait for a command to start or wait for updateTime
        unlock();
        if (acquire) {
            epicsEventWaitWithTimeout(_event_id, poll_delay);
        } else {
            epicsEventWait(_event_id);
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
        tdiff_cycle = epicsTimeDiffInSeconds(&ts2, &ts0);
        double rate = (tdiff_cycle < 0.00001) ? 0.0 : (1.0/tdiff_cycle);
        setDoubleParam(P_updateTimeAct, tdiff_cycle);
        setDoubleParam(P_updateRateAct, rate);
        callParamCallbacks();
        
        unlock();
        poll_count++;
    } // End of while loop
    
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: Main thread exiting...\n",
            driverName.c_str(), functionName.c_str());
    _exited = true;
}


bool drvOceanDirect::_connect() {
/*--------------------------------------------------------------------
Find and connect to spectrometer.
----------------------------------------------------------------------*/
    std::string functionName = "connect";
    int error = 0;
    int num_probed_devices = 0, num_device_ids = 0, status = asynSuccess, num_params = 0;
    bool found = false;

    num_probed_devices = odapi_probe_devices();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: odapi_probe_devices() found %d devices\n",
            driverName.c_str(), functionName.c_str(), num_probed_devices);

    num_device_ids = odapi_get_number_of_device_ids();
    if (num_device_ids <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: odapi_get_number_of_device_ids found %d device IDs\n",
                driverName.c_str(), functionName.c_str(), num_device_ids);
        return false;
    }

    std::vector<long> ids(num_device_ids); 
    int retrieved_id_count = odapi_get_device_ids(ids.data(), num_device_ids);
    if (retrieved_id_count <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: odapi_get_device_ids found %d device IDs\n",
                driverName.c_str(), functionName.c_str(), retrieved_id_count);
        return false;
    }

    for (const long& dev : ids) {
        odapi_open_device(dev, &error);
        if (error == 0) {
            char device_type_buf[64];
            int ret_bytes = odapi_get_device_type(dev, &error, device_type_buf, sizeof(device_type_buf));
            if ((error == 0) && ret_bytes) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: Opened device 0x%02lX (%s)\n",
                        driverName.c_str(), functionName.c_str(), dev, device_type_buf);
            } else {
                _check_error(dev, error, "odapi_get_device_type");
            }
            found = _find_device_by_serial_number(dev);
            if (found) break;
        } else {
            _check_error(dev, error, "odapi_open_device");
        }
    }

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


bool drvOceanDirect::_find_device_by_serial_number(long device_id) {
/*-------------------------------------------------------------------- 
Find device given a serial number.
----------------------------------------------------------------------*/
    std::string functionName = "_find_device_by_serial_number";
    int error = 0;

    int serial_number_length = odapi_get_serial_number_maximum_length(device_id, &error);
    if ((error == 0) && serial_number_length) {
        std::unique_ptr<char> serial_number(new char[serial_number_length]);
        int serial_number_bytes = odapi_get_serial_number(device_id, &error,
                serial_number.get(), serial_number_length);
        if (serial_number_bytes && (error == 0)) {
            std::string serial = serial_number.get();
            if (serial.find(_serial_num) != std::string::npos) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: Matched serial number %s\n",
                        driverName.c_str(), functionName.c_str(), _serial_num.c_str());
                _device_id = device_id;
                setStringParam(P_serialNum, _serial_num);
                callParamCallbacks();
                return true;
            }
        } else {
            _check_error(device_id, error, "odapi_get_serial_number");
        }
    } else {
        _check_error(device_id, error, "odapi_get_serial_number_maximum_length");
    }

    return false;
}


bool drvOceanDirect::_get_device_features() {
/*-------------------------------------------------------------------- 
Get spectrometer features and push to records.
----------------------------------------------------------------------*/
    std::string functionName = "_get_device_features";
    int error = 0, spec_len = 0;
  
    char device_type_buf[64];
    int ret_bytes = odapi_get_device_type(_device_id, &error, device_type_buf, sizeof(device_type_buf));
    if ((error == 0) && ret_bytes) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Model: %s\n",
                driverName.c_str(), functionName.c_str(), device_type_buf);
        setStringParam(P_model, device_type_buf);
    } else {
        _check_error(_device_id, error, "odapi_get_device_type");
    }

    double api_ver = odapi_get_api_version();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: API version: %f\n",
            driverName.c_str(), functionName.c_str(), api_ver);
    setDoubleParam(P_apiVersion, api_ver);

    _min_integration_time = odapi_get_minimum_integration_time_micros(_device_id, &error);
    _check_error(_device_id, error, "odapi_get_minimum_integration_time_micros");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Min. integration time: %ld us\n",
            driverName.c_str(), functionName.c_str(), _min_integration_time);
    setDoubleParam(P_minIntegrationTime, _min_integration_time/1000.0);

    _has_shutter = odapi_is_feature_enabled(_device_id, &error, FEATURE_ID_SHUTTER);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Shutter: %d\n",
            driverName.c_str(), functionName.c_str(), _has_shutter);

    _has_temperature = odapi_is_feature_enabled(_device_id, &error, FEATURE_ID_TEMPERATURE);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Temp: %d\n",
            driverName.c_str(), functionName.c_str(), _has_temperature);

    _has_nonlinear_cal = odapi_is_feature_enabled(_device_id, &error, FEATURE_ID_NONLINEARITYCAL);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Nonlinear Cal: %d\n",
            driverName.c_str(), functionName.c_str(), _has_nonlinear_cal);

    _has_autonull = odapi_is_feature_enabled(_device_id, &error, FEATURE_ID_AUTO_NULLING);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Auto-nulling: %d\n",
            driverName.c_str(), functionName.c_str(), _has_autonull);

    spec_len = odapi_get_formatted_spectrum_length(_device_id, &error);
    _check_error(_device_id, error, "odapi_get_formatted_spectrum_length");
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: Spectrum length: %d pixels\n",
            driverName.c_str(), functionName.c_str(), spec_len);
    setIntegerParam(P_spectrumLength, spec_len);
    if ((spec_len <= 0) || (spec_len != _spectrum_length)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Error getting spectrum length, desired length=%d, reported length=%d\n",
                driverName.c_str(), functionName.c_str(), _spectrum_length, spec_len);
        return false;
    }

    // Resize vectors to actual spectrum length 
    _wavelengths.resize(spec_len);
    _spectrum.resize(spec_len);
    _background_spectrum.resize(spec_len);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: _wavelengths.size=%zu, _spectrum.size=%zu, _background_spectrum.size=%zu\n",
            driverName.c_str(), functionName.c_str(), _wavelengths.size(), _spectrum.size(),
                    _background_spectrum.size());
        
    callParamCallbacks();
    return true;
}


void drvOceanDirect::_get_spectrum() {
/*-------------------------------------------------------------------- 
Get wavelengths and spectrum arrays, optionally subtract background, push to records.
----------------------------------------------------------------------*/
    std::string functionName = "_get_spectrum";
    int error;
    int subtract_background = 0;
    asynStatus status = asynSuccess;
    int spec_len = 0;

    spec_len = odapi_get_formatted_spectrum(_device_id, &error, _spectrum.data(), _spectrum_length);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_get_formatted_spectrum");
    }

    // If we get a transfer error, the device has likely been disconnected; 
    // if we get multiple errors, set connection state to disconnected and close device
    if (error == ERROR_TRANSFER_ERROR) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Device 0x%02lX: transfer error (error code %d)\n",
                driverName.c_str(), functionName.c_str(), _device_id, error);
        _error_count++;
        if (_error_count > MAX_ERROR_COUNT) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: Disconnecting device 0x%02lX after multiple transfer errors (error code %d)\n",
                    driverName.c_str(), functionName.c_str(), _device_id, error);
            _connected = false;
            odapi_close_device(_device_id, &error);
        }
        return;
    // ** odapi_get_formatted_spectrum does not correctly return the spectrum length, so we can't validate the value **
    // ** This may be fixed in a future API version **
    //} else if ((spec_len <= 0) || (spec_len != _spectrum_length)) {
    //    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    //            "%s::%s: Error getting spectrum, desired length=%d, actual length=%d\n",
    //            driverName.c_str(), functionName.c_str(), _spectrum_length, spec_len);
    //    return;
    }

#ifdef EVR_SUPPORT
    epicsTimeStamp evr_timestamp;
    int event_code = 0;
    getIntegerParam(P_eventCode, &event_code);
    if (evrTimeGet(&evr_timestamp, event_code) == 0) {
        if (setTimeStamp(&evr_timestamp) != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: Error in setTimeStamp()\n",
                    driverName.c_str(), functionName.c_str());
        }
    }
#endif

    // If we've had an error, clear error count and print a diagnostic message
    if (_error_count) {
        if (_error_count > MAX_ERROR_COUNT) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: Device 0x%02lX OK after %d errors\n",
                    driverName.c_str(), functionName.c_str(), _device_id, _error_count);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: Device 0x%02lX OK after %d errors\n",
                    driverName.c_str(), functionName.c_str(), _device_id, _error_count);
        }
        _error_count = 0;
    }

    spec_len = odapi_get_wavelengths(_device_id, &error, _wavelengths.data(), _spectrum_length);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_get_wavelengths");
    }
    if ((spec_len <= 0) || (spec_len != _spectrum_length)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: Error getting wavelengths\n",
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

    status = doCallbacksFloat64Array((epicsFloat64*)_wavelengths.data(), _spectrum_length, P_wavelengths, 0);
    status = doCallbacksFloat64Array((epicsFloat64*)_spectrum.data(), _spectrum_length, P_spectrum, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Error in spectrum callback\n",
                driverName.c_str(), functionName.c_str());
    }
}


void drvOceanDirect::_get_int_time() {
/*-------------------------------------------------------------------- 
    Get integration time, push value to parameter library.
----------------------------------------------------------------------*/
    std::string functionName = "_get_int_time";
    int error = 0;
    unsigned long inttime = 0.0;

    inttime = odapi_get_integration_time_micros(_device_id, &error);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_get_integration_time_micros");
    }
    setDoubleParam(P_integrationTime, inttime/1000.0);
    callParamCallbacks();
}


void drvOceanDirect::_set_int_time(epicsFloat64 value) {
/*-------------------------------------------------------------------- 
    Set integration time. Validate, verify setting.
----------------------------------------------------------------------*/
    std::string functionName = "_set_int_time";
    int error = 0;

    if (value < _min_integration_time/1000.0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s:%s: warning, integration time too small, changed from %f to %f\n",
            driverName.c_str(), functionName.c_str(), value, _min_integration_time/1000.0);
        value = _min_integration_time/1000.0;
        setDoubleParam(P_integrationTime, value);
    }
    if (_connected) {
        odapi_set_integration_time_micros(_device_id, &error, static_cast<unsigned long>(value*1000));
        _check_error(_device_id, error, "odapi_set_integration_time_micros");
        _get_int_time();
    }
}


void drvOceanDirect::_get_shutter() {
/*-------------------------------------------------------------------- 
    Get shutter open/closed state, push value to parameter library.
----------------------------------------------------------------------*/
    std::string functionName = "_get_shutter";
    int error = 0;
    int state = SHUTTER_NA;

    if (_has_shutter) {
        bool is_open = odapi_adv_get_shutter_state(_device_id, &error);
        if (error == 0) {
            state = is_open ? SHUTTER_OPEN : SHUTTER_CLOSED;
        } else {
            _check_error(_device_id, error, "odapi_adv_get_shutter_state");
        }
    }

    setIntegerParam(P_shutter, state);
    callParamCallbacks();
}


void drvOceanDirect::_get_trig_mode() {
/*-------------------------------------------------------------------- 
    Get trigger mode, push value to parameter library.
----------------------------------------------------------------------*/
    std::string functionName = "_get_trig_mode";
    int error = 0;
    int mode = 0;

    mode = odapi_adv_get_trigger_mode(_device_id, &error);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_adv_get_trigger_mode");
    }
    setIntegerParam(P_trigMode, mode);
    callParamCallbacks();
}


void drvOceanDirect::_get_num_ave() {
/*-------------------------------------------------------------------- 
    Get number of scans to average, push value to parameter library.
----------------------------------------------------------------------*/
    std::string functionName = "_get_num_ave";
    int error = 0;
    unsigned int num = 0;

    num = odapi_get_scans_to_average(_device_id, &error);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_get_scans_to_average");
    }
    setIntegerParam(P_numAve, num);

    // Min integration can change when averaging
    // ** The API function odapi_get_minimum_averaging_integration_time_micros doesn't seem to return the correct value **
    // ** This may be fixed in a future API version **
    //if (num > 1) {
    //    _min_integration_time = odapi_get_minimum_averaging_integration_time_micros(_device_id, &error);
    //    _check_error(_device_id, error, "odapi_get_minimum_averaging_integration_time_micros");
    //    setDoubleParam(P_minIntegrationTime, _min_integration_time/1000.0);
    //} else if (num == 1) {
    //    _min_integration_time = odapi_get_minimum_integration_time_micros(_device_id, &error);
    //    _check_error(_device_id, error, "odapi_get_minimum_integration_time_micros");
    //    setDoubleParam(P_minIntegrationTime, _min_integration_time/1000.0);
    //}

    callParamCallbacks();
}


void drvOceanDirect::_get_smoothing_width() {
/*-------------------------------------------------------------------- 
    Get boxcar smoothing width, push value to parameter library.
----------------------------------------------------------------------*/
    std::string functionName = "_get_smoothing_width";
    int error = 0;
    unsigned short int width = 0;

    width = odapi_get_boxcar_width(_device_id, &error);
    if (error != 0) {
        _check_error(_device_id, error, "odapi_get_boxcar_width");
    }
    setIntegerParam(P_smoothingWidth, width);
    callParamCallbacks();
}


int drvOceanDirect::_check_error(long index, int error, const std::string& func) {
/*-------------------------------------------------------------------- 
Given an error code, get error string.
----------------------------------------------------------------------*/
    std::string functionName = "_check_error";
    int err_length = 0;

    err_length = odapi_get_error_string_length(error);
    std::unique_ptr<char> err_buf(new char[err_length]);
    if (!err_buf) {
        return -1;
    }

    odapi_get_error_string(error, err_buf.get(), sizeof(err_buf));
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: %s: error code=%d (%s) for device 0x%02lX\n",
            driverName.c_str(), functionName.c_str(), func.c_str(), error, err_buf.get(), index);
    return err_length;
}


asynStatus drvOceanDirect::writeInt32(asynUser *pasynUser, epicsInt32 value) {
/*-------------------------------------------------------------------- 
    asynPortDriver override.
----------------------------------------------------------------------*/
    std::string functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    int error = 0;

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setIntegerParam(function, value);
    
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_acquire) {
        /* If acquire was set then wake up the acquisition task */
        if (value) {
            epicsEventSignal(_event_id);
        } else {
            setDoubleParam(P_updateTimeAct, 0.0);
            setDoubleParam(P_updateRateAct, 0.0);
        }
    } else if (function == P_reconn) {
        if (value && !_connected) {
            _connect();
            epicsEventSignal(_event_id);
        }
    } else if (function == P_shutter) {
        if (_has_shutter) {
            if (_connected) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: Setting shutter (%d) for device 0x%02lX\n",
                        driverName.c_str(), functionName.c_str(), value, _device_id);
                odapi_adv_set_shutter_open(_device_id, &error, static_cast<unsigned char>(value));
                _check_error(_device_id, error, "odapi_shutter_set_shutter_open");
                _get_shutter();
            }
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: Device 0x%02lX has no shutter\n",
                    driverName.c_str(), functionName.c_str(), _device_id);
        }
    } else if (function == P_getBkg) {
        if (_connected) {
            odapi_get_formatted_spectrum(_device_id, &error, 
                    _background_spectrum.data(), _spectrum_length);
            _check_error(_device_id, error, "odapi_get_formatted_spectrum");
        }
    } else if (function == P_clearBkg) {
        if (_connected) {
            std::fill(_background_spectrum.begin(), _background_spectrum.end(), 0.0);
        }
    } else if (function == P_trigMode) {
        if (_connected) {
            odapi_set_trigger_mode(_device_id, &error, value);
            _check_error(_device_id, error, "odapi_set_trigger_mode");
            _get_trig_mode();
        }
    } else if (function == P_numAve) {
        if (_connected) {
            unsigned int num = static_cast<unsigned int>(value);
            if (num < 1) num = 1;
            if (num > std::numeric_limits<unsigned int>::max()) {
                num = std::numeric_limits<unsigned int>::max();
            }
            odapi_set_scans_to_average(_device_id, &error, num);
            _check_error(_device_id, error, "odapi_set_scans_to_average");
            _get_num_ave();
        }
    } else if (function == P_smoothingWidth) {
        if (_connected) {
            unsigned short int width = static_cast<unsigned short int>(value);
            if (width < 0) width = 0;
            if (width > std::numeric_limits<unsigned short int>::max()) { 
                width = std::numeric_limits<unsigned short int>::max();
            }
            odapi_set_boxcar_width(_device_id, &error, width);
            _check_error(_device_id, error, "odapi_set_boxcar_width");
            _get_smoothing_width();
        }
    } else {
        /* All other parameters just get set in parameter list, no need to act on them here */
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();
    
    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                  "%s:%s: ERROR: status=%d, function=%d, name=%s, value=%d", 
                  driverName.c_str(), functionName.c_str(), status, function, paramName, value);
    } else {        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%d\n", 
              driverName.c_str(), functionName.c_str(), function, paramName, value);
    }
    return status;
}


asynStatus drvOceanDirect::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
/*-------------------------------------------------------------------- 
    asynPortDriver override.
----------------------------------------------------------------------*/
    std::string functionName = "writeFloat64";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    epicsInt32 acquire;
    const char *paramName;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setDoubleParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    if (function == P_updateTime) {
        _update_time = value;
        /* Make sure the update time is valid. If not change it and put back in parameter library */
        if (_update_time < _min_update_time) {
            asynPrint(pasynUser, ASYN_TRACE_WARNING,
                    "%s:%s: warning, update time too small, changed from %f to %f\n", 
                    driverName.c_str(), functionName.c_str(), value, _min_update_time);
            _update_time = _min_update_time;
        }
        setDoubleParam(P_updateTime, _update_time);
        /* If the update time has changed and we are acquiring then wake up the acquisition task */
        getIntegerParam(P_acquire, &acquire);
        if (acquire) epicsEventSignal(_event_id);
    } else if (function == P_integrationTime) {
        _set_int_time(value);
    } else {
        /* All other parameters just get set in parameter list, no need to act on them here */
    }
    
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();
    
    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                  "%s:%s: status=%d, function=%d, name=%s, value=%f", 
                  driverName.c_str(), functionName.c_str(), status, function, paramName, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, name=%s, value=%f\n", 
              driverName.c_str(), functionName.c_str(), function, paramName, value);
    }
    return status;
}


void drvOceanDirect::report(FILE *fp, int details) {
/*-------------------------------------------------------------------- 
    asynPortDriver override.
----------------------------------------------------------------------*/
    double api_ver = 0.0;
    std::string model;
    
    getDoubleParam(P_apiVersion, &api_ver);
    fprintf(fp, "OceanDirect driver (drvOceanDirect), API version %f\n", api_ver);
    if (details < 1) {
        fprintf(fp, "\n");
        return;
    }
    
    getStringParam(P_model, model);
    fprintf(fp, "    Model: %s\n", model.c_str());
    fprintf(fp, "    Spectrum length: %d pixels\n", _spectrum_length);
    fprintf(fp, "    Min. integration time: %ld us\n", _min_integration_time);
    fprintf(fp, "    Has shutter: %d\n", _has_shutter);
    fprintf(fp, "    Has temp. readback: %d\n", _has_temperature);
    fprintf(fp, "    Has nonlinear cal.: %d\n", _has_nonlinear_cal);
    fprintf(fp, "    Has auto-nulling: %d\n", _has_autonull);
    fprintf(fp, "\n");

    asynPortDriver::report(fp, details);
}


void drvOceanDirect::exitHandler(void *arg) {
/*---------------------------------------------------------
    Exit handler, delete the drvOceanDirect object.
-----------------------------------------------------------*/
    drvOceanDirect *pdrvOceanDirect = (drvOceanDirect*)arg;
    if (pdrvOceanDirect) delete pdrvOceanDirect;
}


// Configuration routine.  Called directly, or from the iocsh function below
extern "C" {
int drvOceanDirectConfigure(const char* port, const char* serialNum, int numPixels) {
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvOceanDirect class.
 *  port    The name of the asyn port driver to be created.
 *----------------------------------------------------------------------------*/
    new drvOceanDirect(port, serialNum, numPixels);
    return asynSuccess;
}

static const iocshArg initArg0 = {"port", iocshArgString};
static const iocshArg initArg1 = {"serialNum", iocshArgString};
static const iocshArg initArg2 = {"numPixels", iocshArgInt};
static const iocshArg* const initArgs[] = {&initArg0, &initArg1, &initArg2};
static const iocshFuncDef initFuncDef = {"drvOceanDirectConfigure", 3, initArgs};

static void initCallFunc(const iocshArgBuf *args) {
    drvOceanDirectConfigure(args[0].sval, args[1].sval, args[2].ival);
}

void drvOceanDirectRegister(void){
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvOceanDirectRegister);
}

