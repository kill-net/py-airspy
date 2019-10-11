# -*- coding: utf-8 -*-
from ctypes import *
import logging
import numpy as np
import SoapySDR
import os

logging.basicConfig()
logger = logging.getLogger('AirSpy Core')
logger.setLevel(logging.DEBUG)
libairspy = CDLL('libairspy.so.0')
# Data structures
_libusb_device_handle = c_void_p
_pthread_t = c_ulong

AIRSPY_SUCCESS = 0
AIRSPY_TRUE = 1
AIRSPY_ERROR_INVALID_PARAM = -2
AIRSPY_ERROR_NOT_FOUND = -5
AIRSPY_ERROR_BUSY = -6
AIRSPY_ERROR_NO_MEM = -11
AIRSPY_ERROR_LIBUSB = -1000
AIRSPY_ERROR_THREAD = -1001
AIRSPY_ERROR_STREAMING_THREAD_ERR = -1002
AIRSPY_ERROR_STREAMING_STOPPED = -1003
AIRSPY_ERROR_OTHER = -9999

AIRSPY_SAMPLE_FLOAT32_IQ = 0  # 2 * 32bit float per sample
AIRSPY_SAMPLE_FLOAT32_REAL = 1  # 1 * 32bit float per sample
AIRSPY_SAMPLE_INT16_IQ = 2  # 2 * 16bit int per sample
AIRSPY_SAMPLE_INT16_REAL = 3  # 1 * 16bit int per sample
AIRSPY_SAMPLE_UINT16_REAL = 4  # 1 * 16bit unsigned int per sample
AIRSPY_SAMPLE_RAW = 5  # Raw packed samples from the device
AIRSPY_SAMPLE_END = 6  # Number of supported sample types


 
AIRSPY_SAMPLE_FLOAT32_IQ = 0   # 2 * 32bit float per sample
AIRSPY_SAMPLE_FLOAT32_REAL = 1 # 1 * 32bit float per sampl
AIRSPY_SAMPLE_INT16_IQ = 2     # 2 * 16bit int per sample
AIRSPY_SAMPLE_INT16_REAL = 3   # 1 * 16bit int per sample
AIRSPY_SAMPLE_UINT16_REAL = 4  # 1 * 16bit unsigned int per sample
AIRSPY_SAMPLE_RAW = 5          # Raw packed samples from the device
AIRSPY_SAMPLE_END = 6           # Number of supported sample types


class airspy_device(Structure):
    pass


class airspy_transfer(Structure):
    _fields_ = [("airspy_device", POINTER(airspy_device)),
                ("ctx", c_void_p),
                ("samples", c_void_p),
                ("sample_count", c_int),
                ("dropped_samples", POINTER(c_uint64)),
                ("airspy_sample_type", c_int)]


_callback = CFUNCTYPE(c_int, POINTER(airspy_transfer))

# airspy_init() deprecated #
# extern ADDAPI int ADDCALL airspy_init(void);
libairspy.airspy_init.restype = c_int
libairspy.airspy_init.argtypes = []
# airspy_exit() deprecated #
# extern ADDAPI int ADDCALL airspy_exit(void);
libairspy.airspy_exit.restype = c_int
libairspy.airspy_exit.argtypes = []

# extern ADDAPI int ADDCALL airspy_list_devices(uint64_t *serials, int count);
libairspy.airspy_list_devices.restype = c_int
libairspy.airspy_list_devices.argtypes = [POINTER(c_uint64), c_int]

# extern ADDAPI int ADDCALL airspy_open_sn(struct airspy_device** device, uint64_t serial_number);
libairspy.airspy_open_sn.restype = c_int
libairspy.airspy_open_sn.argtypes = [POINTER(POINTER(airspy_device)), c_uint64]

# extern ADDAPI int ADDCALL airspy_open(struct airspy_device** device);
libairspy.airspy_open.restype = c_int
libairspy.airspy_open.argtypes = [POINTER(POINTER(airspy_device))]

# extern ADDAPI int ADDCALL airspy_close(struct airspy_device* device);
libairspy.airspy_close.restype = c_int
libairspy.airspy_close.argtypes = [POINTER(airspy_device)]

# extern ADDAPI int ADDCALL airspy_get_samplerates(struct airspy_device* device, uint32_t* buffer, const uint32_t len);
libairspy.airspy_get_samplerates.restype = c_int
libairspy.airspy_get_samplerates.argtypes = [POINTER(airspy_device), c_uint32]

# Parameter samplerate can be either the index of a samplerate or directly its value in Hz within the list returned by airspy_get_samplerates() #
# extern ADDAPI int ADDCALL airspy_set_samplerate(struct airspy_device* device, uint32_t samplerate);
libairspy.airspy_set_samplerate.restype = c_int
libairspy.airspy_set_samplerate.argtypes = [POINTER(airspy_device), c_uint32]

# extern ADDAPI int ADDCALL airspy_set_conversion_filter_float32(struct airspy_device* device, const float *kernel, const uint32_t len);
libairspy.airspy_set_conversion_filter_float32.restype = c_int
libairspy.airspy_set_conversion_filter_float32.argtypes = [POINTER(airspy_device), POINTER(c_float), c_uint32]

# extern ADDAPI int ADDCALL airspy_set_conversion_filter_int16(struct airspy_device* device, const int16_t *kernel, const uint32_t len);
libairspy.airspy_set_conversion_filter_int16.restype = c_int
libairspy.airspy_set_conversion_filter_int16.argtypes = [POINTER(airspy_device), POINTER(c_int16), c_uint32]

# extern ADDAPI int ADDCALL airspy_start_rx(struct airspy_device* device, airspy_sample_block_cb_fn callback, void* rx_ctx);
libairspy.airspy_start_rx.restype = c_int
libairspy.airspy_start_rx.argtypes = [POINTER(airspy_device), _callback, c_void_p]

# extern ADDAPI int ADDCALL airspy_stop_rx(struct airspy_device* device);
libairspy.airspy_stop_rx.restype = c_int
libairspy.airspy_stop_rx.argtypes = [POINTER(airspy_device)]

# return AIRSPY_TRUE if success #
# extern ADDAPI int ADDCALL airspy_is_streaming(struct airspy_device* device);
libairspy.airspy_is_streaming.restype = c_int
libairspy.airspy_is_streaming.argtypes = [POINTER(airspy_device)]

# extern ADDAPI int ADDCALL airspy_set_sample_type(struct airspy_device* device, enum airspy_sample_type sample_type);
libairspy.airspy_set_sample_type.restype = c_int
libairspy.airspy_set_sample_type.argtypes = [POINTER(airspy_device), c_int]

# Parameter freq_hz shall be between 24000000(24MHz) and 1750000000(1.75GHz) #
# extern ADDAPI int ADDCALL airspy_set_freq(struct airspy_device* device, const uint32_t freq_hz);
libairspy.airspy_set_freq.restype = c_int
libairspy.airspy_set_freq.argtypes = [POINTER(airspy_device), c_uint32]

# Parameter value shall be between 0 and 15 #
# extern ADDAPI int ADDCALL airspy_set_lna_gain(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_lna_gain.restype = c_int
libairspy.airspy_set_lna_gain.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value shall be between 0 and 15 #
# extern ADDAPI int ADDCALL airspy_set_mixer_gain(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_mixer_gain.restype = c_int
libairspy.airspy_set_mixer_gain.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value shall be between 0 and 15 #
# extern ADDAPI int ADDCALL airspy_set_vga_gain(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_vga_gain.restype = c_int
libairspy.airspy_set_vga_gain.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value:
#	0=Disable LNA Automatic Gain Control
#	1=Enable LNA Automatic Gain Control
#
# extern ADDAPI int ADDCALL airspy_set_lna_agc(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_lna_agc.restype = c_int
libairspy.airspy_set_lna_agc.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value:
#	0=Disable MIXER Automatic Gain Control
#	1=Enable MIXER Automatic Gain Control
#
# extern ADDAPI int ADDCALL airspy_set_mixer_agc(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_mixer_agc.restype = c_int
libairspy.airspy_set_mixer_agc.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value: 0..21 #
# extern ADDAPI int ADDCALL airspy_set_linearity_gain(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_linearity_gain.restype = c_int
libairspy.airspy_set_linearity_gain.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value: 0..21 #
# extern ADDAPI int ADDCALL airspy_set_sensitivity_gain(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_sensitivity_gain.restype = c_int
libairspy.airspy_set_sensitivity_gain.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value shall be 0=Disable BiasT or 1=Enable BiasT #
# extern ADDAPI int ADDCALL airspy_set_rf_bias(struct airspy_device* dev, uint8_t value);
libairspy.airspy_set_rf_bias.restype = c_int
libairspy.airspy_set_rf_bias.argtypes = [POINTER(airspy_device), c_uint8]

# Parameter value shall be 0=Disable Packing or 1=Enable Packing #
# extern ADDAPI int ADDCALL airspy_set_packing(struct airspy_device* device, uint8_t value);
libairspy.airspy_set_packing.restype = c_int
libairspy.airspy_set_packing.argtypes = [POINTER(airspy_device), c_uint8]

# extern ADDAPI int ADDCALL airspy_board_partid_serialno_read(struct airspy_device* device, airspy_read_partid_serialno_t* read_partid_serialno);
# Parameter sector_num shall be between 2 & 13 (sector 0 & 1 are reserved) #
# extern ADDAPI int ADDCALL airspy_spiflash_erase_sector(struct airspy_device* device, const uint16_t sector_num);
libairspy.airspy_set_packing.restype = c_int
libairspy.airspy_set_packing.argtypes = [POINTER(airspy_device), c_uint16]

# extern ADDAPI int ADDCALL airspy_si5351c_write(struct airspy_device* device, uint8_t register_number, uint8_t value);
libairspy.airspy_si5351c_write.restype = c_int
libairspy.airspy_si5351c_write.argtypes = [POINTER(airspy_device), c_uint8, POINTER(c_uint8)]
# extern ADDAPI int ADDCALL airspy_si5351c_read(struct airspy_device* device, uint8_t register_number, uint8_t* value);
libairspy.airspy_si5351c_read.restype = c_int
libairspy.airspy_si5351c_read.argtypes = [POINTER(airspy_device), c_uint8, POINTER(c_uint8)]

# extern ADDAPI int ADDCALL airspy_r820t_write(struct airspy_device* device, uint8_t register_number, uint8_t value);
libairspy.airspy_r820t_write.restype = c_int
libairspy.airspy_r820t_write.argtypes = [POINTER(airspy_device), c_uint8, POINTER(c_uint8)]
# extern ADDAPI int ADDCALL airspy_r820t_read(struct airspy_device* device, uint8_t register_number, uint8_t* value);
libairspy.airspy_r820t_read.restype = c_int
libairspy.airspy_r820t_read.argtypes = [POINTER(airspy_device), c_uint8, POINTER(c_uint8)]

# Parameter value shall be 0=clear GPIO or 1=set GPIO #
# extern ADDAPI int ADDCALL airspy_gpio_write(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t value);
libairspy.airspy_gpio_write.restype = c_int
libairspy.airspy_gpio_write.argtypes = [POINTER(airspy_device), c_uint8, c_uint8, POINTER(c_uint8)]
# Parameter value corresponds to GPIO state 0 or 1 #
# extern ADDAPI int ADDCALL airspy_gpio_read(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t* value);
libairspy.airspy_gpio_read.restype = c_int
libairspy.airspy_gpio_read.argtypes = [POINTER(airspy_device), c_uint8, c_uint8, POINTER(c_uint8)]

# Parameter value shall be 0=GPIO Input direction or 1=GPIO Output direction #
# extern ADDAPI int ADDCALL airspy_gpiodir_write(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t value);
libairspy.airspy_gpiodir_write.restype = c_int
libairspy.airspy_gpiodir_write.argtypes = [POINTER(airspy_device), c_uint8, c_uint8, POINTER(c_uint8)]
# extern ADDAPI int ADDCALL airspy_gpiodir_read(struct airspy_device* device, airspy_gpio_port_t port, airspy_gpio_pin_t pin, uint8_t* value);
libairspy.airspy_gpiodir_read.restype = c_int
libairspy.airspy_gpiodir_read.argtypes = [POINTER(airspy_device), c_uint8, c_uint8, POINTER(c_uint8)]

# extern ADDAPI int ADDCALL airspy_spiflash_write(struct airspy_device* device, const uint32_t address, const uint16_t length, unsigned char* const data);
libairspy.airspy_spiflash_write.restype = c_int
libairspy.airspy_spiflash_write.argtypes = [POINTER(airspy_device), c_uint32, c_uint16, POINTER(c_byte)]
# extern ADDAPI int ADDCALL airspy_spiflash_read(struct airspy_device* device, const uint32_t address, const uint16_t length, unsigned char* data);
libairspy.airspy_spiflash_read.restype = c_int
libairspy.airspy_spiflash_read.argtypes = [POINTER(airspy_device), c_uint32, c_uint16, POINTER(c_byte)]

# extern ADDAPI const char* ADDCALL airspy_error_name(enum airspy_error errcode);
libairspy.airspy_error_name.restype = POINTER(c_char)
libairspy.airspy_error_name.argtypes = [c_int8]

# extern ADDAPI const char* ADDCALL airspy_board_id_name(enum airspy_board_id board_id);
libairspy.airspy_board_id_name.restype = POINTER(c_byte)
libairspy.airspy_board_id_name.argtypes = [c_int8]

# extern ADDAPI int ADDCALL airspy_spiflash_erase(struct airspy_device* device);
libairspy.airspy_spiflash_erase.restype = c_int
libairspy.airspy_spiflash_erase.argtypes = [POINTER(airspy_device)]

# extern ADDAPI int ADDCALL airspy_board_id_read(struct airspy_device* device, uint8_t* value);
libairspy.airspy_board_id_read.restype = c_int
libairspy.airspy_board_id_read.argtypes = [POINTER(airspy_device), POINTER(c_uint8)]

# Parameter length shall be at least 128bytes #
# extern ADDAPI int ADDCALL airspy_version_string_read(struct airspy_device* device, char* version, uint8_t length);
libairspy.airspy_version_string_read.restype = c_int
libairspy.airspy_version_string_read.argtypes = [POINTER(airspy_device), POINTER(c_byte), c_uint8]


class airspy(object):

    def __init__(self):
        self.device = POINTER(airspy_device)()
        self.callback = None
        self.is_open = False

    def __del__(self):
        if self.is_open == True:
            self.exit()

    def setup(self):
        ret = libairspy.airspy_init()
        if ret == AIRSPY_SUCCESS:
            self.is_open = True
            logger.debug('Successfully init device')
            return AIRSPY_SUCCESS
        else:
            logger.error('airspy  init Detected!')

    def exit(self):
        ret = self.close()
        libairspy.airspy_exit()
        return ret

    def open(self):

        ret = libairspy.airspy_open(self.device)
        if ret == AIRSPY_SUCCESS:
            self.is_open = True
            logger.debug('Successfully open airspy device')
        else:
            logger.error('airspy_open fail code:[{code}]!'.format(code=ret))
        ret = libairspy.airspy_set_sample_type(self.device,AIRSPY_SAMPLE_FLOAT32_IQ)
        if ret == AIRSPY_SUCCESS:
            self.is_open = True
            logger.debug('Successfully airspy_set_sample_type')
            return AIRSPY_SUCCESS
        else:
            logger.error('airspy_set_sample_type  fail code:[{code}]!'.format(code=ret))

    def close(self):
        ret = libairspy.airspy_close(self.device)
        if ret == AIRSPY_SUCCESS:
            self.is_open = False
            logger.debug('Successfully close airspy device')
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to close!')

    def start_rx_mode(self, set_callback):
        self.callback = _callback(set_callback)
        ret = libairspy.airspy_start_rx(self.device, self.callback, None)
        if ret == AIRSPY_SUCCESS:
            logger.debug('Successfully start airspy in Recieve Mode')
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to start airspy in Recieve Mode')

    def stop_rx_mode(self):

        ret = libairspy.airspy_stop_rx(self.device)
        if ret == AIRSPY_SUCCESS:
            logger.debug('Successfully stop airspy in Recieve Mode')
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to stop airspy in Recieve Mode')
        return ret

    def set_freq(self, freq_hz):
        ret = libairspy.airspy_set_freq(self.device, freq_hz)
        if ret == AIRSPY_SUCCESS:
            logger.debug('Successfully set frequency with value [%dMhz]', freq_hz/1e6)
            return AIRSPY_SUCCESS
        else:
            logger.error('Error setting frequency with value [%d]', freq_hz)

    def is_streaming(self):
        ret = libairspy.airspy_is_streaming(self.device)
        if (ret == 1):
            return True
        else:
            return False

    def set_lna_gain(self, value):
        result = libairspy.airspy_set_lna_gain(self.device, value)
        if result == AIRSPY_SUCCESS:
            logger.debug('Successfully set LNA gain to [%d]', value)
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to set LNA gain to [%d]', value)

    def set_mixer_gain(self, value):
        result = libairspy.airspy_set_mixer_gain(self.device, value)
        if result == AIRSPY_SUCCESS:
            logger.debug('Successfully set MIX gain to [%d]', value)
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to set MIX gain to [%d]', value)

    def set_vga_gain(self, value):
        ''' Sets the vga gain, in 2db steps, maximum value of 62 '''
        result = libairspy.airspy_set_vga_gain(self.device, value)
        if result == AIRSPY_SUCCESS:
            logger.debug('Successfully set VGA gain to [%d]', value)
            return AIRSPY_SUCCESS
        else:
            logger.error('Failed to set VGA gain to [%d]', value)

    def set_sample_rate(self, freq):
        result = libairspy.airspy_set_samplerate(self.device, freq)
        if result != AIRSPY_SUCCESS:
            logger.error('Error setting Sample Rate with Frequency [%dMhz]', freq/1e6)
        else:
            logger.debug(
                'Successfully set Sample Rate with Frequency [%dMhz]', freq/1e6)
            return AIRSPY_SUCCESS

    def get_lna_gain(self):
      pass

    def get_mix_gain(self):
        pass

    def get_vga_gain(self):
        pass

    def get_sample_rate(self):
        pass




def packed_bytes_to_iq( bytes,removedc=True):
    ''' Convenience function to unpack array of bytes to Python list/array
    of complex numbers and normalize range.  size 16*32*512 262 144
    '''
    # use NumPy array
    iq = np.empty(len(bytes) // 2, 'complex64')
    iq.real, iq.imag = bytes[::2], bytes[1::2]
    iq /= 128.0
    if removedc:
        iq.real=iq.real-np.average(iq.real)
        iq.imag = iq.imag - np.average(iq.imag)
    return iq

