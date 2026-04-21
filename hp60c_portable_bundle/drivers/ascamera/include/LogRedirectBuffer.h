#pragma once

#include <string>
#include <streambuf>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

class LogRedirectBuffer : public std::streambuf
{
public:
    explicit LogRedirectBuffer(rclcpp::Logger logger) : logger_(logger)
    {
        buffer_ = new char[BUFFER_SIZE];
        setp(buffer_, buffer_ + BUFFER_SIZE);
    }

    ~LogRedirectBuffer() override
    {
        sync();
        delete[] buffer_;
    }

protected:
    int sync() override
    {
        if (pbase() != pptr()) {
            std::string msg(pbase(), pptr() - pbase());
            msg.erase(std::remove(msg.begin(), msg.end(), '\n'), msg.end());
            msg.erase(std::remove(msg.begin(), msg.end(), '\r'), msg.end());
            RCLCPP_INFO_STREAM(logger_, msg);
            setp(buffer_, buffer_ + BUFFER_SIZE);
        }
        return 0;
    }

    int overflow(int c) override
    {
        sync();
        if (c != EOF) {
            *pptr() = c;
            pbump(1);
        }
        return c;
    }

private:
    rclcpp::Logger logger_;
    static const int BUFFER_SIZE = 1024;
    char *buffer_;
};