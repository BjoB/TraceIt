#pragma once

#include <iostream>

#define TRACEIT_LOG_INFO(msg)      std::cout << "INFO: " << msg << std::endl;
#define TRACEIT_LOG_WARN(msg)      std::cout << "WARN: " << msg << std::endl;
#define TRACEIT_LOG_ERROR(msg)     std::cout << "ERROR: " << msg << std::endl;