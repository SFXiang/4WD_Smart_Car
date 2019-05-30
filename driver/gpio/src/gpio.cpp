#include "ros_gpio_control/gpio.h"
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

const std::string GPIO::IN = "in";
const std::string GPIO::OUT = "out";
const std::string GPIO::OUT_LOW = "low";
const std::string GPIO::OUT_HIGH = "high";
const std::string GPIO::EDGE_NONE = "none";
const std::string GPIO::EDGE_RISE = "rise";
const std::string GPIO::EDGE_FALL = "fall";
const std::string GPIO::EDGE_BOTH = "both";

GPIO::GPIO(int gpio)
{
    this->gpio = gpio;
    if (exported())
        std::cout << "GPIO " << gpio << " already exported" << std::endl;
    else
        export_gpio();
    // Outra opção: se direction for chamado, inicializar fd_value lá se
    // &direction == &GPIO::GPIO_IN (mas não daria pra ler o valor de pinos
    // de saída)

    direction(GPIO::IN);

    fd_value = open(("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value").c_str(), O_RDWR);
    
    if (fd_value == -1) {
        perror("open");
        throw std::runtime_error("open failed");
    }
    poll_targets.events = POLLPRI;
    poll_targets.fd = fd_value;
}

GPIO::~GPIO()
{
    if (exported())
        write_to_file("/sys/class/gpio/unexport", std::to_string(gpio));
    close(fd_value);
}

void GPIO::export_gpio()
{
    write_to_file("/sys/class/gpio/export", std::to_string(gpio));
    if (exported())
        return;
    throw std::runtime_error("GPIO " + std::to_string(gpio) + " export failed");
}

int GPIO::poll(int timeout)
{
    int poll_status = ::poll(&poll_targets, 1, timeout);
    if (poll_status == -1) {
        perror("poll");
        return 0;
    }
    return poll_status;
}

void GPIO::poll()
{
    poll(-1);
}

void GPIO::direction(const std::string& direction)
{
    write_to_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction", direction);
}

void GPIO::active_low()
{
    write_to_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/active_low", "1");
}

void GPIO::edge(const std::string& edge_type)
{
    write_to_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/edge", edge_type);
}

void GPIO::operator=(bool value)
{
    write_to_file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", std::to_string(value));
}

GPIO::operator bool() const
{
    return read();
}

void GPIO::write_to_file(const std::string& name, const std::string& value)
{
    std::fstream file(name, std::fstream::out);
    file << value;
}

int GPIO::read() const
{
    int value;
    std::fstream file("/sys/class/gpio/gpio" + std::to_string(gpio) + "/value", std::fstream::in);
    file >> value;
    return value;
}

bool GPIO::exported()
{
    struct stat st;
    if (stat(("/sys/class/gpio/gpio" + std::to_string(gpio)).c_str(), &st) == -1) {
        if (errno == ENOENT)
            return 0;
        else {
            perror("stat");
            throw std::runtime_error("stat error");
        }
    }
    return st.st_mode & (S_IFDIR | S_IFLNK);
}

// Could be optimized to run one thread and poll() on a fd set
GPIOButton::GPIOButton(int gpio, int switch_debounce)
    : button(gpio)
{
    button.direction(GPIO::IN);
    button.edge(GPIO::EDGE_BOTH);
}

GPIOButton::operator bool() const
{
    return button;
}

GPIOButton::~GPIOButton()
{
}
