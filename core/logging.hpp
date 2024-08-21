#include "core/rpi2cam_app.hpp"

#define LOG(level, text)                                                                                               \
	do                                                                                                                 \
	{                                                                                                                  \
		if (RPi2CamApp::GetVerbosity() >= level)                                                                     \
			std::cerr << text << std::endl;                                                                            \
	} while (0)
#define LOGNLF(level, text)                                                                                               \
	do                                                                                                                 \
	{                                                                                                                  \
		if (RPi2CamApp::GetVerbosity() >= level)                                                                     \
			std::cerr << text << "\r" <<std::flush;                                                                  \
	} while (0)
#define LOG_ERROR(text) std::cerr << text << std::endl
