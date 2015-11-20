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

import inspect

# to load the proper dll
import platform

# Do not import or use ill definied data types
# such as short int or long
# use the values specified in the h file
# float is always defined as 32 bits
# double is defined as 64 bits
from ctypes import byref, POINTER, create_string_buffer, c_float, \
    c_int16, c_int32, c_uint32, c_void_p
from ctypes import c_int32 as c_enum

from picoscope.picobase import _PicoscopeBase


class PS3000(_PicoscopeBase):
    """The following are low-level functions for the PS3000"""

    LIBNAME = "ps3000"

    NUM_CHANNELS = 4
    CHANNELS     =  {"A": 0, "B": 1, "C": 2, "D": 3}

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
                     {"rangeV": 200.0, "apivalue": 13, "rangeStr": "200 V"},
                     {"rangeV": 400.0, "apivalue": 14, "rangeStr": "400 V"},
                     ]

    ERROR_CODES = [[0x00, "PICO_OK", "The PicoScope XXXX is functioning correctly."],
        [0x01, "PICO_MAX_UNITS_OPENED", "An attempt has been made to open more than PS3000_MAX_UNITS."],
        [0x02, "PICO_MEMORY_FAIL", "Not enough memory could be allocated on the host machine."],
        [0x03, "PICO_NOT_FOUND", "No PicoScope XXXX could be found."],
        [0x04, "PICO_FW_FAIL", "Unable to download firmware."],
        [0x05, "PICO_NOT_RESPONDING", "The PicoScope XXXX is not responding to commands from the PC."],
        [0x06, "PICO_CONFIG_FAIL", "The configuration info has become corrupt or is missing"],
        [0x07, "PICO_OS_NOT_SUPPORTED", "The OS is not supported"]]


    CHANNEL_COUPLINGS = {"DC": 1, "AC": 0}

    has_sig_gen = False

    TIME_UNITS = {"FS": 0, "PS": 1, "NS": 2, "US": 3, "MS": 4, "S": 5}

    MAX_VALUE = 32767
    MIN_VALUE = -32767

    MAX_TIMEBASES = 19                    # variable depending on model

    UNIT_INFO_TYPES = {"DriverVersion"          : 0x0,
                       "USBVersion"             : 0x1,
                       "HardwareVersion"        : 0x2,
                       "VarianInfo"             : 0x3,
                       "BatchAndSerial"         : 0x4,
                       "CalDate"                : 0x5,
                       "ErrorCode"              : 0x6,
                       "KernelVersion"          : 0x7}

    channelBuffersPtr = [c_void_p(), c_void_p(), c_void_p(), c_void_p()]
    channelBuffersLen = [0, 0, 0, 0]

    def __init__(self, serialNumber=None, connect=True):
        """Load DLL etc"""
        if platform.system() == 'Linux':
            from ctypes import cdll
            self.lib = cdll.LoadLibrary("lib" + self.LIBNAME + ".so")
        else:
            from ctypes import windll
            self.lib = windll.LoadLibrary(self.LIBNAME + ".dll")

        super(PS3000, self).__init__(serialNumber, connect)

    def _lowLevelOpenUnit(self, sn):
        c_handle = c_int16()
        if sn is not None:
            raise ValueError("PS3000 Doesn't Support Open by Serial Number")

        m = self.lib.ps3000_open_unit()
        if m < 0:
            raise IOError("Failed to Find PS3000 Unit. Should you be using PS3000a driver?")       
        self.handle = m
        self.suggested_time_units = self.TIME_UNITS["NS"]

    def _lowLevelCloseUnit(self):
        m = self.lib.ps3000_close_unit(c_int16(self.handle))
        self.checkHandleResult(m)

    def _lowLevelSetChannel(self, chNum, enabled, coupling, VRange, VOffset,
                            BWLimited):
        m = self.lib.ps3000_set_channel(c_int16(self.handle), c_enum(chNum),
                                        c_int16(enabled), c_enum(coupling),
                                        c_enum(VRange))
        if m == 0:
            raise IOError('Error calling %s: parameter out of range' % (inspect.stack()[1][3]))

    def _lowLevelStop(self):
        m = self.lib.ps3000_stop(c_int16(self.handle))
        self.checkHandleResult(m)

    def _lowLevelRunBlock(self, numPreTrigSamples, numPostTrigSamples,
                          timebase, oversample, segmentIndex):
        # NOTE: Oversample is NOT used!
        # TODO: Fix 'delay' which is where trigger occurs in block
        if numPreTrigSamples > 0:
            raise ValueError("numPreTrigSamples isn't supported on PS3000")
        timeIndisposedMs = c_int32()
        m = self.lib.ps3000_run_block(
            c_int16(self.handle), c_uint32(numPostTrigSamples),
            c_uint32(timebase), c_int16(1), byref(timeIndisposedMs))
        if m == 0:
            raise IOError('Error calling %s: parameter out of range' % (inspect.stack()[1][3]))
        return timeIndisposedMs.value

    def _lowLevelGetUnitInfo(self, info):
        s = create_string_buffer(256)
        m = self.lib.ps3000_get_unit_info(c_int16(self.handle), byref(s),
                                          c_int16(len(s)), c_enum(info))
        if m == 0:
            raise IOError('Error calling %s: parameter out of range' % (inspect.stack()[1][3]))
        else:
            print('%d bytes returned' % (m))
        # should this be ascii instead?
        # I think they are equivalent...
        return s.value.decode('utf-8')

    def _lowLevelFlashLed(self, times):
        m = self.lib.ps3000_flash_led(c_int16(self.handle))
        self.checkHandleResult(m)

    def checkHandleResult(self, ec):
        if ec == 0:
            raise IOError('Error calling %s: invalid handle given'%(inspect.stack()[1][3]))

    def _lowLevelEnumerateUnits(self):
        print('Enumerate Units not implemented by PS3000')

    def getTimeBaseNum(self, sampleTimeS):
        """ps3000 doesn't seem to have published formula like other scopes"""
        time_interval = c_int32()
        timebases = [None]*self.MAX_TIMEBASES
        #Convert to nS
        sampleTimenS = sampleTimeS * 1E9
        tb = 0
        while tb < self.MAX_TIMEBASES:
            rv = self.lib.ps3000_get_timebase(c_int16(self.handle), c_int16(tb),
                                              c_uint32(512), byref(time_interval),
                                              c_void_p(), c_int16(1),  c_void_p())            
            if rv != 0:
                timebases[tb] = time_interval.value            
            tb += 1
        #Figure out closest option
        besterror = 1E99
        bestindx = 0
        for indx, val in enumerate(timebases):
            if val is not None:
                error = sampleTimenS - val
                if abs(error) < besterror:
                    besterror = abs(error)
                    bestindx = indx
        return bestindx

    def getTimestepFromTimebase(self, timebase):
        time_interval = c_int32()
        m = self.lib.ps3000_get_timebase(c_int16(self.handle), c_int16(timebase),
                                         c_uint32(512), byref(time_interval),
                                         c_void_p(), c_int16(1),  c_void_p())        
        if m == 0:
            raise IOError('Error calling %s: invalid parameters given' % (inspect.stack()[1][3]))
        return (time_interval.value / 1.0E9)

    def _lowLevelGetTimebase(self, tb, noSamples, oversample, segmentIndex):
        """ return (timeIntervalSeconds, maxSamples). """
        maxSamples = c_int32()
        interval = c_int32()
        time_units = c_int16()

        m = self.lib.ps3000_get_timebase(c_int16(self.handle), c_int16(tb),
                                         c_int32(noSamples), byref(interval),
                                         byref(time_units), c_int16(oversample),
                                         byref(maxSamples))
        if m == 0:
            raise IOError('Error calling %s: invalid parameters given' % (inspect.stack()[1][3]))
        return (interval.value/1e9, maxSamples.value)

    def _lowLevelIsReady(self):
        ready = c_int16()
        ready = self.lib.ps3000_ready(c_int16(self.handle))
        if ready > 0:
            return True
        elif ready == 0:
            return False
        else:
            raise IOError("ps3000_ready returned %d" % ready.value)

    def _lowLevelGetValues(self, numSamples, startIndex, downSampleRatio,
                           downSampleMode, segmentIndex):

        # TODO: Check overflow in channelBuffersLen against numSamples, but need to
        #      not raise error if channelBuffersPtr is void

        overflow = c_int16()
        rv = self.lib.ps3000_get_values(
            c_int16(self.handle),
            self.channelBuffersPtr[0],
            self.channelBuffersPtr[1],
            self.channelBuffersPtr[2],
            self.channelBuffersPtr[3],
            byref(overflow), c_int32(numSamples))

        return (rv, overflow.value)

    def _lowLevelSetDataBuffer(self, channel, data, downSampleMode, segmentIndex):
        dataPtr = data.ctypes.data_as(POINTER(c_int16))
        numSamples = len(data)

        self.channelBuffersPtr[channel] = dataPtr
        self.channelBuffersLen[channel] = numSamples

    def _lowLevelClearDataBuffer(self, channel, segmentIndex):
        self.channelBuffersPtr[channel] = c_void_p()
        self.channelBuffersLen[channel] = 0
