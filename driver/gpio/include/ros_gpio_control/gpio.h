#ifndef ROS_GPIO_CONTROL_GPIO_H
#define ROS_GPIO_CONTROL_GPIO_H

#include <poll.h>
#include <string>

class GPIO {
public:
    static const std::string IN, OUT, OUT_LOW, OUT_HIGH;
    static const std::string EDGE_NONE, EDGE_RISE, EDGE_FALL, EDGE_BOTH;

    explicit GPIO(int gpio);

    ~GPIO();

    void export_gpio();

    int poll(int timeout);

    void poll();

    void direction(const std::string& direction);

    void active_low();

    void edge(const std::string& edge_type);

    void operator=(bool value);

    operator bool() const;
    
    int read() const;
    
    struct pollfd poll_targets;

private:
    int fd_value;
    
    int gpio;
    
    

    void write_to_file(const std::string& name, const std::string& value);

    

    bool exported();
};

class GPIOButton {
public:
    explicit GPIOButton(int gpio, int switch_debounce = 50);
    ~GPIOButton();
    operator bool() const;

private:
    GPIO button;
};

#endif // ROS_GPIO_CONTROL_GPIO_H
