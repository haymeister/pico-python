# This is the instrument-specific file for the PS3000a series of instruments.
#
# pico-python is Copyright (c) 2013-2014 By:
# Colin O'Flynn <coflynn@newae.com>
# Mark Harfouche <mark.harfouche@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This is the low level driver file for a specific Picoscope.

By this, I mean if parameters want to get passed as strings, they should be
handled by PSBase
All functions here should take things as close to integers as possible, the
only exception here is for array parameters. Array parameters should be passed
in a pythonic way through numpy since the PSBase class should not be aware of
the specifics behind how the clib is called.

The functions should not have any default values as these should be handled
by PSBase.
"""

from __future__ import division
from __future__ import absolute_import
from __future__ import print_function
from __future__ import unicode_literals
import string

import inspect

# to load the proper dll
import platform

# Do not import or use ill definied data types
# such as short int or long
# use the values specified in the h file
# float is always defined as 32 bits
# double is defined as 64 bits
from ctypes import byref, POINTER, create_string_buffer, c_float, \
  c_int16, c_int32, c_uint32, c_void_p, c_int8, c_double
from ctypes import c_int32 as c_enum

from picoscope.picobase import _PicoscopeBase


class PS4000a(_PicoscopeBase):
    """The following are low-level functions for the PS4000a"""

    LIBNAME = "ps4000a"

    NUM_CHANNELS = 8
    CHANNELS = dict(zip(string.uppercase[0:NUM_CHANNELS], xrange(NUM_CHANNELS)))

    CHANNEL_RANGE = [
                   {"rangeV": 10E-3, "apivalue": 0, "rangeStr": "10 mV"},
                   {"rangeV": 20E-3, "apivalue": 1, "rangeStr": "20 mV"},
                   {"rangeV": 50E-3, "apivalue": 2, "rangeStr": "50 mV"},
                   {"rangeV": 100E-3, "apivalue": 3, "rangeStr": "100 mV"},
                   {"rangeV": 200E-3, "apivalue": 4, "rangeStr": "200 mV"},
                   {"rangeV": 500E-3, "apivalue": 5, "rangeStr": "500 mV"},
                   {"rangeV": 1.0, "apivalue": 6, "rangeStr": "1 V"},
                   {"rangeV": 2.0, "apivalue": 7, "rangeStr": "2 V"},
                   {"rangeV": 5.0, "apivalue": 8, "rangeStr": "5 V"},
                   {"rangeV": 10.0, "apivalue": 9, "rangeStr": "10 V"},
                   {"rangeV": 20.0, "apivalue": 10, "rangeStr": "20 V"},
                   {"rangeV": 50.0, "apivalue": 11, "rangeStr": "50 V"},
                   {"rangeV": 100.0, "apivalue": 12, "rangeStr": "100 V"},
                   {"rangeV": 200.0, "apivalue": 13, "rangeStr": "200 V"}
                   ]

    ERROR_CODES = [[0x00, "PICO_OK", "The PicoScope XXXX is functioning correctly."],
      [0x01, "PICO_MAX_UNITS_OPENED", "An attempt has been made to open more than PS3000_MAX_UNITS."],
      [0x02, "PICO_MEMORY_FAIL", "Not enough memory could be allocated on the host machine."],
      [0x03, "PICO_NOT_FOUND", "No PicoScope XXXX could be found."],
      [0x04, "PICO_FW_FAIL", "Unable to download firmware."],
      [0x05, "PICO_OPEN_OPERATION_IN_PROGRESS", "?"],
      [0x06, "PICO_OPERATION_FAILED", "?"],
      [0x07, "PICO_NOT_RESPONDING", "The PicoScope XXXX is not responding to commands from the PC."],
      [0x08, "PICO_CONFIG_FAIL", "The configuration info has become corrupt or is missing"],
      [0x09, "PICO_KERNEL_DRIVER_TOO_OLD", "?"],
      [0x0A, "PICO_EEPROM_CORRUPT", "?"],
      [0x0B, "PICO_OS_NOT_SUPPORTED", "The OS is not supported"],
      [0x0C, "PICO_INVALID_HANDLE", "?"],
      [0x0D, "PICO_INVALID_PARAMETER", "?"],
      [0x0E, "PICO_INVALID_TIMEBASE", "?"],
      [0x0F, "PICO_INVALID_VOLTAGE", "?"]
      ]

    MY_ERROR_CODES = {
      "PICO_OK": 0x00,
      "PICO_USB_3_0_DEVICE_NON_USB3_0_PORT": 0x11E
      }

    CHANNEL_COUPLINGS = {"AC": 0, "DC": 1}

    has_sig_gen = True
    WAVE_TYPES = {"Sine": 0, "Square": 1, "Triangle": 2,
                  "RampUp": 3, "RampDown": 4,
                  "Sinc": 5, "Gaussian": 6, "HalfSine": 7, "DCVoltage": 8,
                  "WhiteNoise": 9}

    SIGGEN_TRIGGER_TYPES = {"Rising": 0, "Falling": 1,
                            "GateHigh": 2, "GateLow": 3}
    SIGGEN_TRIGGER_SOURCES = {"None": 0, "ScopeTrig": 1, "AuxIn": 2,
                              "ExtIn": 3, "SoftTrig": 4, "TriggerRaw": 5}

    AWGPhaseAccumulatorSize = 32
    AWGBufferAddressWidth   = 14
    AWGMaxSamples           = 2 ** AWGBufferAddressWidth

    AWGDACInterval          = 12.5E-9  # in seconds
    AWGDACFrequency         = 1 / AWGDACInterval

    # Note this is NOT what is written in the Programming guide as of version
    # 10_5_0_28
    # This issue was acknowledged in this thread
    # http://www.picotech.com/support/topic13217.html
    AWGMaxVal               = 0x0FFF
    AWGMinVal               = 0x0000

    AWG_INDEX_MODES = {"Single": 0, "Dual": 1, "Quad": 2}

    TIME_UNITS = {"FS": 0, "PS": 1, "NS": 2, "US": 3, "MS": 4, "S": 5}

    MAX_VALUE = 32767
    MIN_VALUE = -32767

    MAX_TIMEBASES = 2**32-1              # variable depending on model

    UNIT_INFO_TYPES = {"DriverVersion"        : 0x0,
                     "USBVersion"             : 0x1,
                     "HardwareVersion"        : 0x2,
                     "VarianInfo"             : 0x3,
                     "BatchAndSerial"         : 0x4,
                     "CalDate"                : 0x5,
                     # "ErrorCode"              : 0x6,
                     "KernelVersion"          : 0x6}

    channelBuffersPtr = [c_void_p()]*NUM_CHANNELS
    channelBuffersLen = [0]*NUM_CHANNELS

    def __init__(self, serialNumber=None, connect=True):
        """Load DLL etc"""
        if platform.system() == 'Linux':
            from ctypes import cdll
            self.lib = cdll.LoadLibrary("lib" + self.LIBNAME + ".so")
        else:
            from ctypes import windll
            self.lib = windll.LoadLibrary(self.LIBNAME + ".dll")

        super(PS4000a, self).__init__(serialNumber, connect)

    def _lowLevelOpenUnit(self, sn):
        c_handle = c_int16()
        c_serial = c_int8()

        if sn:
            m = self.lib.ps4000aOpenUnit(byref(c_handle), byref(sn))
        else:
            m = self.lib.ps4000aOpenUnit(byref(c_handle), None)
        if c_handle.value > 0:
            self.handle = c_handle.value
        if (m == self.MY_ERROR_CODES["PICO_USB_3_0_DEVICE_NON_USB3_0_PORT"]):
            m = self.lib.ps4000aChangePowerSource(c_handle, c_uint32(m))
        if m < 0:
            raise IOError("Failed to Find PS4000a Unit.")

        self.suggested_time_units = self.TIME_UNITS["NS"]

    def _lowLevelCloseUnit(self):
        m = self.lib.ps4000aCloseUnit(c_int16(self.handle))
        self.checkResult(m)

    def _lowLevelSetChannel(self, chNum, enabled, coupling, VRange,
                            VOffset, BWLimited):
        if abs(VOffset) > 0.:
            maxOffset = c_float(); minOffset = c_float()
            m = self.lib.ps4000aGetAnalogueOffset(c_int16(self.handle),
                                                  c_enum(VRange),
                                                  c_enum(coupling),
                                                  byref(maxOffset),
                                                  byref(minOffset))
            self.checkResult(m)
            if VOffset > maxOffset.value or VOffset < minOffset.value:
                raise ValueError('PS4000a setChannel: invalid offset %f V'%VOffset)
        m = self.lib.ps4000aSetChannel(c_int16(self.handle),
                                       c_enum(chNum),
                                       c_int16(enabled),
                                       c_enum(coupling),
                                       c_enum(VRange),
                                       c_float(VOffset))
        self.checkResult(m)

    def _lowLevelSetSimpleTrigger(self, enabled, trigsrc, threshold_adc,
                                  direction, timeout_ms, auto):
        m = self.lib.ps4000aSetSimpleTrigger(
            c_int16(self.handle), c_int16(enabled),
            c_enum(trigsrc), c_int16(threshold_adc),
            c_enum(direction), c_uint32(timeout_ms), c_int16(auto))
        self.checkResult(m)

    def _lowLevelStop(self):
        m = self.lib.ps4000aStop(c_int16(self.handle))
        self.checkResult(m)

    def _lowLevelRunBlock(self, numPreTrigSamples, numPostTrigSamples,
                        timebase, oversample, segmentIndex):
        #TODO: Fix 'delay' which is where trigger occurs in block
        #TODO: Add callbacks
        timeIndisposedMs = c_int32()
        m = self.lib.ps4000aRunBlock(
            c_int16(self.handle), c_int32(numPreTrigSamples), c_int32(numPostTrigSamples),c_uint32(timebase),byref(timeIndisposedMs),c_uint32(segmentIndex),None,None)
        if not m==0:
            raise IOError('Error calling %s: parameter out of range'%(inspect.stack()[1][3]))

        return timeIndisposedMs.value

    def _lowLevelGetUnitInfo(self, info):
        s = create_string_buffer(256)
        requiredSize = c_int16(0)

        m = self.lib.ps4000aGetUnitInfo(c_int16(self.handle), byref(s),
                                       c_int16(len(s)), byref(requiredSize),
                                       c_enum(info))
        self.checkResult(m)
        if requiredSize.value > len(s):
            s = create_string_buffer(requiredSize.value + 1)
            m = self.lib.ps4000aGetUnitInfo(c_int16(self.handle), byref(s),
                                           c_int16(len(s)),
                                           byref(requiredSize), c_enum(info))
            self.checkResult(m)

        # should this be ascii instead?
        # I think they are equivalent...
        return s.value.decode('utf-8')

    def _lowLevelFlashLed(self,times):
        m = self.lib.ps4000aFlashLed(c_int16(self.handle),c_int16(times))
        self.checkHandleResult(m)

    def checkHandleResult(self,ec):
        if not ec==0:
            raise IOError('Error calling %s: invalid handle given'%(inspect.stack()[1][3]))
    def checkResult(self,ec):
        if not ec==0:
            print('ec:',ec)
            raise IOError('Error calling %s: '%(inspect.stack()[1][3]))

    def _lowLevelEnumerateUnits(self):
        count = c_int16(0)
        m = self.lib.ps4000aEnumerateUnits(byref(count), None, None)
        self.checkResult(m)
        # a serial number is rouhgly 8 characters
        # an extra character for the comma
        # and an extra one for the space after the comma?
        # the extra two also work for the null termination
        serialLth = c_int16(count.value * (8 + 2))
        serials = create_string_buffer(serialLth.value + 1)

        m = self.lib.ps4000aEnumerateUnits(byref(count), serials, byref(serialLth))
        self.checkResult(m)

        serialList = str(serials.value.decode('utf-8')).split(',')

        serialList = [x.strip() for x in serialList]

        return serialList


    def getTimeBaseNum(self, sampleTimeS):
        time_interval = c_int32()
        max_samples = c_int32()
        tb = int(sampleTimeS/12.5e-9)-1
        if tb>0 and tb<self.MAX_TIMEBASES:
            rv = self.lib.ps4000aGetTimebase(c_int16(self.handle), c_uint32(tb),
                                        c_int32(512), byref(time_interval),
                                        byref(max_samples), c_uint32(0))
            if rv ==0:
                return tb
            else:
                self.checkResult(rv)

    def getTimestepFromTimebase(self, timebase):
        time_interval = c_int32()
        m = self.lib.ps4000aGetTimebase(c_int16(self.handle), c_uint32(timebase),
                                      c_int32(512), byref(time_interval),
                                      c_void_p(), c_uint32(0))
        if not m==0:
            raise IOError('Error calling %s: invalid parameters given'%(inspect.stack()[1][3]))
        return (time_interval.value / 1.0E9)

    def _lowLevelGetTimebase(self, tb, noSamples, oversample, segmentIndex):
        """ return (timeIntervalSeconds, maxSamples). """
        maxSamples = c_int32()
        interval = c_int32()
        time_units = c_int16()

        m = self.lib.ps4000aGetTimebase(c_int16(self.handle), c_uint32(tb),
                                      c_int32(noSamples), byref(interval),
                                      byref(maxSamples), c_uint32(0))
        if not m==0:
            raise IOError('Error calling %s: invalid parameters given'%(inspect.stack()[1][3]))

        return (interval.value/1e9, maxSamples.value)

    def _lowLevelIsReady(self):
        ready = c_int16()
        self.lib.ps4000aIsReady(c_int16(self.handle),byref(ready))        
        if not ready.value == 0:
            return True
        else:
            return False

    def _lowLevelGetValues(self, numSamples, startIndex, downSampleRatio,
                          downSampleMode, segmentIndex):

        #TODO: Check overflow in channelBuffersLen against numSamples, but need to
        #      not raise error if channelBuffersPtr is void

        overflow = c_int16()
        numSamples = c_uint32(numSamples)
        m = self.lib.ps4000aGetValues(
            c_int16(self.handle),
            c_uint32(startIndex),
            byref(numSamples),
            c_uint32(downSampleRatio),                  #downsample factor
            c_enum(downSampleMode),
            c_uint32(segmentIndex),
            byref(overflow))
        if m == 0:
            return (numSamples.value, overflow.value)
        else:
            self.checkResult(m)

    def _lowLevelSetDataBuffer(self, channel, data, downSampleMode, segmentIndex):
        dataPtr = data.ctypes.data_as(POINTER(c_int16))
        numSamples = len(data)

        self.channelBuffersPtr[channel] = dataPtr
        self.channelBuffersLen[channel] = numSamples
        m = self.lib.ps4000aSetDataBuffer(c_int16(self.handle),c_enum(channel),self.channelBuffersPtr[channel],c_int32(numSamples),c_uint32(segmentIndex),c_enum(downSampleMode))
        self.checkResult(m)

    def _lowLevelClearDataBuffer(self, channel, segmentIndex):
        m = self.lib.ps4000aSetDataBuffer(c_int16(self.handle), c_enum(channel),
                                         c_void_p(), c_int32(0), c_uint32(segmentIndex),
                                          c_enum(0))
        self.checkResult(m)


    def _lowLevelSetSigGenBuiltInSimple(self, offsetVoltage, pkToPk, waveType,
                                        frequency, shots, triggerType,
                                        triggerSource):
        m = self.lib.ps4000aSetSigGenBuiltIn(
            c_int16(self.handle),
            c_int32(int(offsetVoltage * 1000000)),
            c_uint32(int(pkToPk        * 1000000)),
            c_enum(waveType),
            c_double(frequency), c_double(frequency),
            c_double(0), c_double(0), c_enum(0), c_enum(0),
            c_uint32(shots), c_uint32(0),
            c_enum(triggerType), c_enum(triggerSource),
            c_int16(0))
        self.checkResult(m)

    def setWhiteNoise(self,pkToPk):
        offsetVoltage = 0.
        m = self.lib.ps4000aSetSigGenBuiltIn(
            c_int16(self.handle),
            c_int32(int(0 * 1000000)),
            c_uint32(int(pkToPk * 1000000)),
            c_enum(0),                    #for white noise
            c_double(1000.), c_double(1000.),
            c_double(0), c_double(.1), c_enum(0), c_enum(1),
            c_uint32(0), c_uint32(0),
            c_enum(0), c_enum(0),         #trigger type and source
            c_int16(0))
        self.checkResult(m)


    def _lowLevelSetAWGSimpleDeltaPhase(self, waveform, deltaPhase,
                                        offsetVoltage, pkToPk, indexMode,
                                        shots, triggerType, triggerSource):
        """ waveform should be an array of shorts """

        waveformPtr = waveform.ctypes.data_as(POINTER(c_int16))

        m = self.lib.ps4000aSetSigGenArbitrary(
            c_int16(self.handle),
            c_int32(int(offsetVoltage * 1E6)),  # offset voltage in microvolts
            c_uint32(int(pkToPk * 1E6)),         # pkToPk in microvolts
            c_uint32(int(deltaPhase)),           # startDeltaPhase
            c_uint32(int(deltaPhase)),           # stopDeltaPhase
            c_uint32(0),                         # deltaPhaseIncrement
            c_uint32(0),                         # dwellCount
            waveformPtr,                         # arbitraryWaveform
            c_int32(len(waveform)),              # arbitraryWaveformSize
            c_enum(0),                           # sweepType for deltaPhase
            c_enum(0),            # operation (adding random noise and whatnot)
            c_enum(indexMode),                   # single, dual, quad
            c_uint32(shots),
            c_uint32(0),                         # sweeps
            c_uint32(triggerType),
            c_uint32(triggerSource),
            c_int16(0))                          # extInThreshold
        self.checkResult(m)
