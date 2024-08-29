/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.hpp - common program options
 * WEK
 */

#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstdio>
#include <string>
 

#include <boost/program_options.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <libcamera/transform.h>

#include "core/logging.hpp"
#include "core/version.hpp"
#include "core/stream_enum.hpp"

static constexpr double DEF_FRAMERATE = 30.0;

struct Bitrate
{
public:
	Bitrate() : bps_(0) {}

	void set(const std::string &s)
	{
		static const std::map<std::string, uint64_t> match
		{
			{ "bps", 1 },
			{ "b", 1 },
			{ "kbps", 1000 },
			{ "k", 1000 },
			{ "K", 1000 },
			{ "mbps", 1000 * 1000 },
			{ "m", 1000 * 1000 },
			{ "M", 1000 },
		};

		try
		{
			std::size_t end_pos;
			float f = std::stof(s, &end_pos);
			bps_ = f;

			for (const auto &m : match)
			{
				auto found = s.find(m.first, end_pos);
				if (found != end_pos || found + m.first.length() != s.length())
					continue;
				bps_ = f * m.second;
				break;
			}
		}
		catch (std::exception const &e)
		{
			throw std::runtime_error("Invalid bitrate string provided");
		}
	}

	uint64_t bps() const
	{
		return bps_;
	}

	uint64_t kbps() const
	{
		return bps_ / 1000;
	} 

	uint64_t mbps() const  //mbps not used in rpi2cam-vid project
	{
		return bps_ / (1000 * 1000);
	} 

	explicit constexpr operator bool() const
	{
		return !!bps_;
	} 

private:
	uint64_t bps_;
};

struct Mode
{
	Mode() : Mode(0, 0, 0, true) {}
	Mode(unsigned int w, unsigned int h, unsigned int b, bool p) : width(w), height(h), bit_depth(b), packed(p), framerate(0) {}
	Mode(std::string const &mode_string);
	unsigned int width;
	unsigned int height;
	unsigned int bit_depth;
	bool packed;
	double framerate;
	libcamera::Size Size() const { return libcamera::Size(width, height); }
	std::string ToString() const;
	void update(const libcamera::Size &size, const std::optional<float> &fps);
}; 

void cp_uint(std::vector<unsigned int> parm);
void cp_int(std::vector<int> parm);
void cp_str(std::vector<std::string> parm);
void cp_flt(std::vector<float> parm);
bool bcm2835_encoder_available();

template <typename DEFAULT>
struct TimeVal
{
	TimeVal() : value(0) {}

	void set(const std::string &s)
	{
		static const std::map<std::string, std::chrono::nanoseconds> match
		{
			{ "min", std::chrono::minutes(1) },
			{ "sec", std::chrono::seconds(1) },
			{ "s", std::chrono::seconds(1) },
			{ "ms", std::chrono::milliseconds(1) },
			{ "us", std::chrono::microseconds(1) },
			{ "ns", std::chrono::nanoseconds(1) },
		};

		try
		{
			std::size_t end_pos;
			float f = std::stof(s, &end_pos);
			value = std::chrono::duration_cast<std::chrono::nanoseconds>(f * DEFAULT { 1 });

			for (const auto &m : match)
			{
				auto found = s.find(m.first, end_pos);
				if (found != end_pos || found + m.first.length() != s.length())
					continue;
				value = std::chrono::duration_cast<std::chrono::nanoseconds>(f * m.second);
				break;
			}
		}
		catch (std::exception const &e)
		{
			throw std::runtime_error("Invalid time string provided");
		}
	}

	template <typename C = DEFAULT>
	int64_t get() const
	{
		return std::chrono::duration_cast<C>(value).count();
	}

	explicit constexpr operator bool() const
	{
		return !!value.count();
	}

	std::chrono::nanoseconds value;
};

struct Options
{
	Options()
		: options_("\nCamera options maked with (*) take a value or two values\n"
		"if one value is set it is used for both cameras, if two values are set the first is used for camera 0 and the second for 1."
		"\n\nValid options are", 120, 80), app_(nullptr)
	{
		using namespace boost::program_options;
		// clang-format off
		options_.add_options()
			("help,h", value<bool>(&help)->default_value(false)->implicit_value(true),
			 "Print this help message")
			("version", value<bool>(&version)->default_value(false)->implicit_value(true),
			 "Displays the build version number")
			("list-cameras", value<bool>(&list_cameras)->default_value(false)->implicit_value(true),
			 "Lists the available cameras attached to the system.")
			("verbose,v", value<unsigned int>(&verbose)->default_value(1)->implicit_value(2),
			 "Set verbosity level. Level 0 is no output, 1 is default, 2 is verbose.")
			("config,c", value<std::string>(&config_file)->implicit_value("config.txt"),
			 "Read the options from a file. If no filename is specified, default to config.txt. "
			 "In case of duplicate options, the ones provided on the command line will be used. "
			 "Note that the config file must only contain the long form options.")
			("output,o", value<std::vector<std::string>>(&output_v_)->multitoken(),
			 "Set the output file name and streams to include in output. Streams can be \"Cam0\", "
			 "\"Cam1\", \"Ovrly\" and \"Audio\".  A stream can be inluded only once per file. "
			 "Multiple file may be written."
			 "To list the available cameras use the --list-cameras option. ")
			("timeout,t", value<std::string>(&timeout_)->default_value("5sec"),
			 "Time for which program runs. If no units are provided default to ms.")
			("width", value<std::vector<unsigned int>>(&width_v)->multitoken()
				->default_value(std::vector<unsigned int>{1280, 1280}, "1280")->notifier(&cp_uint),  
			"(*) Set the image width of camera/cameras")
			("height", value<std::vector<unsigned int>>(&height_v)->multitoken()
				->default_value(std::vector<unsigned int>{720, 720}, "720")->notifier(&cp_uint),
			"(*) Set the image height of camera/cameras")
			("framerate", value<std::vector<float>>(&framerate_v_)->multitoken()
				->default_value(std::vector<float>{-1.0, -1.0}, "DEF_FRAMERATE")->notifier(&cp_flt),
			 "(*) Set the fixed framerate")
			("orientation", value<std::vector<unsigned int>>(&orientation_v)->multitoken()
				->default_value(std::vector<unsigned int>{0, 0}, "000")->notifier(&cp_uint),
			"(*) Set of flags (0=no & 1=yes for image flip TVH "
			"in the order of Transpose/Vertical/Horizontal."
			"Transpose flips image on the axis running from top left to bottom right "
			"and is processed after vertical and horizontal.")
			("roi", value<std::vector<std::string>>(&roi_v)->multitoken()
				->default_value(std::vector<std::string>{"0,0,0,0 0,0,0,0"}, "0,0,0,0")->notifier(&cp_str),
			"(*) Set region of interest (digital zoom) e.g. 0.25,0.25,0.5,0.5")
			("shutter", value<std::vector<std::string>>(&shutter_v_)->multitoken()
				->default_value(std::vector<std::string>{"0 0"}, "0")->notifier(&cp_str),
			"(*) Set a fixed shutter speed. If no units are provided default to us")
			("gain", value<std::vector<float>>(&gain_v)->multitoken()
				->default_value(std::vector<float>{0, 0}, "0")->notifier(&cp_flt),
			"(*) Set a fixed (analog) gain value")
			("metering", value<std::vector<std::string>>(&metering_v)->multitoken()
				->default_value(std::vector<std::string>{"center", "center"}, "center")->notifier(&cp_str),
			"(*) Set the metering mode (center, spot, average, custom)")
			("exposure", value<std::vector<std::string>>(&exposure_v)->multitoken()
				->default_value(std::vector<std::string>{"normal", "normal"}, "normal")->notifier(&cp_str),
			"(*) Set the exposure mode (normal, sport)")
			("ev", value<std::vector<float>>(&ev_v)->multitoken()
				->default_value(std::vector<float>{0, 0}, "0")->notifier(&cp_flt),
			"(*) Set the EV exposure compensation, where 0 = no change")
			("awb", value<std::vector<std::string>>(&awb_v)->multitoken()
				->default_value(std::vector<std::string>{"auto", "auto"}, "auto")->notifier(&cp_str),
			 "(*) Set the AWB mode (auto, incandescent, tungsten, fluorescent, indoor, daylight, cloudy, custom)")
			("awbgains", value<std::vector<std::string>>(&awbgains_v)->multitoken()
				->default_value(std::vector<std::string>{"0,0", "0,0"}, "0,0")->notifier(&cp_str),
			 "(*) Set explict red and blue gains (disable the automatic AWB algorithm)")
			("brightness", value<std::vector<float>>(&brightness_v)->multitoken()
				->default_value(std::vector<float>{0, 0}, "0")->notifier(&cp_flt),
			 "(*) Adjust the brightness of the output images, in the range -1.0 to 1.0")
			("contrast", value<std::vector<float>>(&contrast_v)->multitoken()
				->default_value(std::vector<float>{1.0, 1.0}, "1.0")->notifier(&cp_flt),
			 "(*) Adjust the contrast of the output image, where 1.0 = normal contrast")
			("saturation", value<std::vector<float>>(&saturation_v)->multitoken()
				->default_value(std::vector<float>{1.0, 1.0}, "1.0")->notifier(&cp_flt),
			 "(*) Adjust the colour saturation of the output, where 1.0 = normal and 0.0 = greyscale")
			("sharpness", value<std::vector<float>>(&sharpness_v)->multitoken()
				->default_value(std::vector<float>{1.0, 1.0}, "1.0")->notifier(&cp_flt),
			 "(*) Adjust the sharpness of the output image, where 1.0 = normal sharpening")
			("denoise", value<std::vector<std::string>>(&denoise_v)->multitoken()
				->default_value(std::vector<std::string>{"auto", "auto"}, "auto")->notifier(&cp_str),
			 "(*) Sets the Denoise operating mode: auto, off, cdn_off, cdn_fast, cdn_hq")
			("mode", value<std::vector<std::string>>(&mode_string_v)->multitoken()->notifier(&cp_str),
			 "(*) Camera mode as W:H:bit-depth:packing, where packing is P (packed) or U (unpacked)")
			("buffer-count", value<std::vector<unsigned int>>(&buffer_count_v)->multitoken()
				->default_value(std::vector<unsigned int>{0, 0}, "0")->notifier(&cp_uint),
			 "(*) Number of in-flight requests (and buffers) configured.")
			("autofocus-mode", value<std::vector<std::string>>(&afMode_v)->multitoken()
				->default_value(std::vector<std::string>{"default", "default"}, "default")->notifier(&cp_str),
			 "(*) Control to set the mode of the AF (autofocus) algorithm.(manual, auto, continuous)")
			("autofocus-range", value<std::vector<std::string>>(&afRange_v)->multitoken()
				->default_value(std::vector<std::string>{"normal", "normal"}, "normal")->notifier(&cp_str),
			 "(*) Set the range of focus distances that is scanned.(normal, macro, full)")
			("autofocus-speed", value<std::vector<std::string>>(&afSpeed_v)->multitoken()
				->default_value(std::vector<std::string>{"normal", "normal"}, "normal")->notifier(&cp_str),
			 "(*) Control that determines whether the AF algorithm is to move the lens as quickly as possible or more steadily.(normal, fast)")
			("autofocus-window", value<std::vector<std::string>>(&afWindow_v)->multitoken()
				->default_value(std::vector<std::string>{"0,0,0,0", "0,0,0,0"}, "0,0,0,0")->notifier(&cp_str),
			"(*) Sets AfMetering to  AfMeteringWindows an set region used, e.g. 0.25,0.25,0.5,0.5")
			("lens-position", value<std::vector<std::string>>(&lens_position_v_)->multitoken()
				->default_value(std::vector<std::string>{""}, "")->notifier(&cp_str),
			 "(*) Set the lens to a particular focus position, expressed as a reciprocal distance (0 moves the lens to infinity), or \"default\" for the hyperfocal distance")
			("hdr", value<std::vector<std::string>>(&hdr_v)->multitoken()
				->default_value(std::vector<std::string>{"off", "off"}, "off")->notifier(&cp_str),
			 "(*) Enable High Dynamic Range, where supported. Available values are \"off\" , \"auto\", "
			 "\"sensor\" for sensor HDR (e.g. for Camera Module 3), "
			 "\"single-exp\" for PiSP based single exposure multiframe HDR")
			("flicker-period", value<std::vector<std::string>>(&flicker_period_v_)->multitoken()
				->default_value(std::vector<std::string>{"0s", "0s"}, "0s")->notifier(&cp_str),
			 "(*) Manual flicker correction period"
			 "\nSet to 10000us to cancel 50Hz flicker."
			 "\nSet to 8333us to cancel 60Hz flicker.\n")
			("tuning-file", value<std::string>(&tuning_file)->default_value("-"),
			 "Name of camera tuning file to use, omit this option for libcamera default behaviour")		 			
			("bitrate,b", value<std::string>(&bitrate_)->default_value("0bps"),
			 "Set the video bitrate for encoding. If no units are provided, default to bits/second.")
			("profile", value<std::string>(&profile),
			 "Set the encoding profile")
			("level", value<std::string>(&level),
			 "Set the encoding level")
			("intra,g", value<unsigned int>(&intra)->default_value(0),
			 "Set the intra frame period")
			("inline", value<bool>(&inline_headers)->default_value(false)->implicit_value(true),
			 "Force PPS/SPS header with every I frame (h264 only)")
#if LIBAV_PRESENT
			("libav-video-codec", value<std::string>(&libav_video_codec)->default_value("h264_v4l2m2m"),
			 "Sets the libav video codec to use. "
			 "To list available codecs, run  the \"ffmpeg -codecs\" command.")
			("libav-video-codec-opts", value<std::string>(&libav_video_codec_opts),
			 "Sets the libav video codec options to use. "
			 "These override the internal defaults (check 'encoderOptions*()' in 'encoder/libav_encoder.cpp' for the defaults). "
			 "Separate key and value with \"=\" and multiple options with \";\". "
			 "e.g.: \"preset=ultrafast;profile=high;partitions=i8x8,i4x4\". "
			 "To list available options for a given codec, run the \"ffmpeg -h encoder=libx264\" command for libx264.")
			("libav-format", value<std::string>(&libav_format),
			 "Sets the libav encoder output format to use. "
			 "Leave blank to try and deduce this from the filename.\n"
			 "To list available formats, run  the \"ffmpeg -formats\" command.")
			("audio-codec", value<std::string>(&audio_codec)->default_value("aac"),
			 "Sets the libav audio codec to use.\n"
			 "To list available codecs, run  the \"ffmpeg -codecs\" command.")
			("audio-source", value<std::string>(&audio_source)->default_value("pulse"),
			 "Audio source to record from. Valid options are \"pulse\" and \"alsa\"")
			("audio-device", value<std::string>(&audio_device),
			 "Audio device to record from.  To list the available devices,\n"
			 "for pulseaudio, use the following command:\n"
			 "\"pactl list | grep -A2 'Source #' | grep 'Name: '\"\n"
			 "or for alsa, use the following command:\n"
			 "\"arecord -L\"")
			("audio-channels", value<uint32_t>(&audio_channels)->default_value(0),
			 "Number of channels to use for recording audio. Set to 0 to use default value.")
			("audio-bitrate", value<std::string>(&audio_bitrate_)->default_value("32kbps"),
			 "Set the audio bitrate for encoding. If no units are provided, default to bits/second.")
			("audio-samplerate", value<uint32_t>(&audio_samplerate)->default_value(0),
			 "Set the audio sampling rate in Hz for encoding. Set to 0 to use the input sample rate.")
			("overlay-main", value<unsigned int>(&overlay_main)->default_value(0),
			 "Set the overlay main camera \"0 or 1\"")
			("overlay-w", value<unsigned int>(&overlay_width)->default_value(0),
			 "Set the overlay width. Defaults to 1/3 of main width.")
			("overlay-h", value<unsigned int>(&overlay_height)->default_value(0),
			 "Set the overlay height. Defaults to 1/3 of main height.")
			("overlay-location", value<std::string>(&overlay_location_)->default_value("leftbottom"),
			 "Set overlay location. X:Y, LeftBottom, RightBottom, RightTop, LeftTop or Center")
#endif
			;
		// clang-format on
	}

	virtual ~Options() {}

	bool help;
	bool version;
	bool list_cameras;
	unsigned int verbose;
	TimeVal<std::chrono::milliseconds> timeout;
	std::string config_file;
	std::vector<unsigned int> width_v;    
	std::vector<unsigned int> height_v;   
	std::vector<unsigned int> orientation_v;
	std::vector<std::string> roi_v; 
	float roi_x_a[MAX_CAMS], roi_y_a[MAX_CAMS], roi_width_a[MAX_CAMS], roi_height_a[MAX_CAMS]; 
	TimeVal<std::chrono::microseconds> shutter_a[MAX_CAMS];
	std::vector<float> gain_v;  
	std::vector<std::string> metering_v; 
	int metering_index_a[MAX_CAMS]; 
	std::vector<std::string> exposure_v; 
	int exposure_index_a[MAX_CAMS]; 
	std::vector<float> ev_v;
	std::vector<std::string> awb_v; 
	int awb_index_a[MAX_CAMS]; 
	std::vector<std::string> awbgains_v; 
	float awb_gain_r_a[MAX_CAMS]; 
	float awb_gain_b_a[MAX_CAMS]; 
	std::vector<float> brightness_v;
	std::vector<float> contrast_v;
	std::vector<float> saturation_v;
	std::vector<float> sharpness_v;
	std::optional<float> framerate_a[MAX_CAMS]; 
	std::vector<std::string> denoise_v;
	std::string tuning_file;
	unsigned int camera;  
	std::vector<std::string> mode_string_v;  
	Mode mode_a[MAX_CAMS]; 
	std::vector<unsigned int> buffer_count_v;
	std::vector<std::string> afMode_v;
	int afMode_index_a[MAX_CAMS];
	std::vector<std::string> afRange_v;
	int afRange_index_a[MAX_CAMS];
	std::vector<std::string> afSpeed_v;
	int afSpeed_index_a[MAX_CAMS];
	std::vector<std::string> afWindow_v;
	float afWindow_x_a[MAX_CAMS], afWindow_y_a[MAX_CAMS], afWindow_width_a[MAX_CAMS], afWindow_height_a[MAX_CAMS]; 
	std::optional<float> lens_position_a[MAX_CAMS];
	bool set_default_lens_position_a[MAX_CAMS];
	std::vector<std::string> hdr_v; 
	TimeVal<std::chrono::microseconds> flicker_period_a[MAX_CAMS];
	
	Bitrate bitrate;
	std::string profile;
	std::string level;
	std::optional<int> level_a[MAX_CAMS];
	unsigned int intra;
	bool inline_headers;
	std::string libav_video_codec;
	std::string libav_video_codec_opts;
	std::string libav_format;
	std::string audio_codec;
	std::string audio_device;
	std::string audio_source;
	uint32_t audio_channels;
	Bitrate audio_bitrate;
	uint32_t audio_samplerate;

	unsigned int overlay_main;
	unsigned int overlay_x, overlay_y, overlay_width, overlay_height;

//to here 
	bool stream_selected[4] {false, false, false, false};
	std::map<std::string, std::vector<Streams>> format_table;

	virtual bool Parse(int argc, char *argv[]);
	virtual void Print() const;

	void SetApp(RPi2CamApp *app) { app_ = app; }

protected:
	boost::program_options::options_description options_;

private:
	std::vector<float> framerate_v_;
	std::vector<std::string> lens_position_v_;
	std::string timeout_;
	std::vector<std::string> mt_shutter_;
	std::vector<std::string> shutter_v_;
	std::vector<std::string> flicker_period_v_;
	RPi2CamApp *app_;
//from here
	std::string bitrate_;
#if LIBAV_PRESENT
	std::string audio_bitrate_;
#endif /* LIBAV_PRESENT */
	std::vector<std::string> output_v_;
	bool stream_available_[4] {false, false, false, false};
	std::string overlay_location_;
};
