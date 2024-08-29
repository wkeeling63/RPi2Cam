/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpi2cam_vid.cpp - libcamera video record app for 2 cameras.
 * WEK
 */

#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>

#include "core/rpi2cam_encoder.hpp"

using namespace std::placeholders;

// Some keypress/signal handling.
// static int signal_received;
bool aborted = false;
RPi2CamEncoder *global_app;
static void default_signal_handler(int signal_number)
{
//	fprintf(stdout, "%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
//	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
//	if (signal_received == SIGINT) 
	if (signal_number == SIGINT) 
	{
		global_app->AbortEncoder();
		aborted = true;
	}
}
// now only get signal WEK set bool above if SIGINT, remove get_key.. and check bool in loop
/* static int get_key_or_signal(Options const *options)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	int key = 0;
	if (signal_received == SIGINT)
		return 'x';

	return key;
} */

// The main even loop for the application.
static void event_loop(RPi2CamEncoder &app)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	Options const *options = app.GetOptions();
	app.OpenCamera();
	app.ConfigureVideo();
	app.StartEncoder();     
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	// Monitoring for signals.
	signal(SIGINT, default_signal_handler);
	
	for (unsigned int count = 0; ; count++)
	{
		RPi2CamEncoder::Msg msg = app.Wait();
		if (msg.type == RPi2CamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPi2CamEncoder::MsgType::Quit)
			return;
		else if (msg.type != RPi2CamEncoder::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
			
//		int key = get_key_or_signal(options);  //replace with bool of aborted

		LOGNLF(2, "Frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = options->timeout &&
					   ((now - start_time) > options->timeout.value);
//		if (timeout || key == 'x' || key == 'X') // replace with aborted bool
		if (timeout || aborted)  
		{
			LOG(2, "Frame " << count);
//			if (key == 'x')   // replace with aborted bool
			if (aborted)  
				LOG(1, "Halting: user interupted!");
			if (timeout)
				LOG(1, "Halting: reached timeout of " << options->timeout.get<std::chrono::milliseconds>()
													  << " milliseconds.");
			app.StopCamera(); // stop complains if encoder very slow to close
			app.StopEncoder();
			return;
		}

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		app.EncodeBuffer(completed_request);
	}
}

int main(int argc, char *argv[])
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	try
	{
		RPi2CamEncoder app;
		global_app = &app; // allow global access to AbortEncoder
		Options *options = app.GetOptions();  
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();
			event_loop(app);
		}
		else
			exit(-1);
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
