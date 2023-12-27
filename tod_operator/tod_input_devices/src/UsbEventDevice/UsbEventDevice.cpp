#include "UsbEventDevice.h"

// Initialize open_device
UsbEventDevice::UsbEventDevice(const std::string &device_namespace)
    : _ok{openDevice(device_namespace)}, _autocenter_off{false} {
    if (_ok) {
        initializeDevice();
    }
}

void UsbEventDevice::set_force_feedback(const double ff_value) {
    createEvent(ff_value);
}

void UsbEventDevice::reset() {
    deleteEffect();
    initializeDevice();
}

bool UsbEventDevice::openDevice(const std::string &device_namespace) {
    // iterate through event numbers until device could be opened with write permission
    std::string device_name;
    for (int i=0; i < 30; ++i) {
        device_name = device_namespace + std::to_string(i);
        _device_handle = open(device_name.c_str(), O_RDWR|O_NONBLOCK);
        if (_device_handle >= 0) {
             std::cout << "Device is opened "<< device_name  << std::endl;
            break;
        }
    }
    if (_device_handle < 0) {
        fprintf(stderr, "ERROR: could not open a device in namespace %s - [%s:%d]\n",
                device_namespace.c_str(), __FILE__, __LINE__);
    }
    return (_device_handle >= 0);
}

void UsbEventDevice::initializeDevice() {
    if (!_ok) {
        return;
    }

    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];

    struct input_event event;
    struct input_absinfo abs_info;

    // Which buttons has the device?
    memset(key_bits, 0, sizeof(key_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0) {
        fprintf(stderr, "ERROR: can not get key bits (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }

    // Which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        fprintf(stderr, "ERROR: can not get abs bits (%s) [%s:%d]\n",
                strerror(errno), __FILE__, __LINE__);
        exit(1);
    }

    // Check if selected axis is available
    if (!testBit(_axis_code, abs_bits)) {
        fprintf(stderr, "ERROR: selected axis %s not available [%s:%d] (see available ones with fftest)\n",
                axis_names[_axis_index].c_str(), __FILE__, __LINE__);
        exit(1);
    }
    // get axis value range
    if (ioctl(_device_handle, EVIOCGABS(_axis_code), &abs_info) < 0) {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }

    _axis_min = abs_info.minimum;
    _axis_max = abs_info.maximum;
    if (_axis_min >= _axis_max) {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }

    // Switch off auto centering
    if (_autocenter_off) {
        memset(&event, 0, sizeof(event));
        event.type = EV_FF;
        event.code = FF_AUTOCENTER;
        event.value = 0;
        if (write(_device_handle, &event, sizeof(event)) != sizeof(event)) {
           std::cout << "failed to disable auto centering" << std::endl;
            exit(1);
        }
    }

    // Now get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }

    // check force feedback is supported?
    if (!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);
    }else { std::cout << "force feedback supported" << std::endl; }

    // init effect and get effect id
    memset(&_effect, 0, sizeof(_effect));
    _effect.type = FF_CONSTANT;
    _effect.id = -1;     // initial value
    _effect.trigger.button = 0;
    _effect.trigger.interval = 0;
    _effect.replay.length = 0xffff; // longest value
    _effect.replay.delay = 0;   // delay from write(...)
    _effect.u.constant.level = 0;
    _effect.direction = 0xC000;
    _effect.u.constant.envelope.attack_length = 0;
    _effect.u.constant.envelope.attack_level = 0;
    _effect.u.constant.envelope.fade_length = 0;
    _effect.u.constant.envelope.fade_level = 0;

    // upload
    if (ioctl(_device_handle, EVIOCSFF, &_effect) < 0) {
         std::cout << "failed to upload m_effect" << std::endl;
        exit(1);
    }

    // start _effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = _effect.id;
    event.value = 1;
    if (write(_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}

// update the device: set force and query joystick position
void UsbEventDevice::createEvent(const double force) {
    // set force and upload effect
    double force2set = std::clamp(force, -0.8, 0.8); // -1.0, 1.0);
    _effect.u.constant.level = (int16_t) (force2set*32767.0);
    _effect.direction = 0xC000;
    _effect.u.constant.envelope.attack_level = (int16_t) (force*32767.0); // this one counts!
    _effect.u.constant.envelope.fade_level = (int16_t)(force*32767.0); // only to be safe
    if (ioctl(_device_handle, EVIOCSFF, &_effect) < 0) {
        perror("upload effect");
        // We do not exit here. Indeed, too frequent updates may be refused,
        // but that is not a fatal error
    }
}


void UsbEventDevice::deleteEffect() {
    // Delete effect
    if (_effect.id != -1) {
        if (ioctl(_device_handle, EVIOCRMFF, _effect.id) < 0) {
            fprintf(stderr, "ERROR: removing effect failed (%s) [%s:%d]\n",
                    strerror(errno), __FILE__, __LINE__);
            // exit(1);
        }
        _effect.id = -1;
    }
}
