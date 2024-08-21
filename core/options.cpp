/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 * WEK
 */
#include <algorithm>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <map>
#include <string>
#include <sys/ioctl.h>

#include <libcamera/formats.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>

#include "core/options.hpp"

namespace fs = std::filesystem;

static const std::map<int, std::string> cfa_map =
{
	{ properties::draft::ColorFilterArrangementEnum::RGGB, "RGGB" },
	{ properties::draft::ColorFilterArrangementEnum::GRBG, "GRBG" },
	{ properties::draft::ColorFilterArrangementEnum::GBRG, "GBRG" },
	{ properties::draft::ColorFilterArrangementEnum::RGB, "RGB" },
	{ properties::draft::ColorFilterArrangementEnum::MONO, "MONO" },
};

static const std::map<libcamera::PixelFormat, unsigned int> bayer_formats =
{
	{ libcamera::formats::SRGGB10_CSI2P, 10 },
	{ libcamera::formats::SGRBG10_CSI2P, 10 },
	{ libcamera::formats::SBGGR10_CSI2P, 10 },
	{ libcamera::formats::R10_CSI2P,     10 },
	{ libcamera::formats::SGBRG10_CSI2P, 10 },
	{ libcamera::formats::SRGGB12_CSI2P, 12 },
	{ libcamera::formats::SGRBG12_CSI2P, 12 },
	{ libcamera::formats::SBGGR12_CSI2P, 12 },
	{ libcamera::formats::SGBRG12_CSI2P, 12 },
	{ libcamera::formats::SRGGB16,       16 },
	{ libcamera::formats::SGRBG16,       16 },
	{ libcamera::formats::SBGGR16,       16 },
	{ libcamera::formats::SGBRG16,       16 },
};


Mode::Mode(std::string const &mode_string) : Mode()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (!mode_string.empty())
	{
		char p;
		int n = sscanf(mode_string.c_str(), "%u:%u:%u:%c", &width, &height, &bit_depth, &p);
		if (n < 2)
			throw std::runtime_error("Invalid mode");
		else if (n == 2)
			bit_depth = 12, packed = true;
		else if (n == 3)
			packed = true;
		else if (toupper(p) == 'P')
			packed = true;
		else if (toupper(p) == 'U')
			packed = false;
		else
			throw std::runtime_error("Packing indicator should be P or U");
	}
}

std::string Mode::ToString() const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (bit_depth == 0)
		return "unspecified";
	else
	{
		std::stringstream ss;
		ss << width << ":" << height << ":" << bit_depth << ":" << (packed ? "P" : "U");
		if (framerate)
			ss << "(" << framerate << ")";
		return ss.str();
	}
}

void Mode::update(const libcamera::Size &size, const std::optional<float> &fps)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (!width)
		width = size.width;
	if (!height)
		height = size.height;
	if (!bit_depth)
		bit_depth = 12;
	if (fps)
		framerate = fps.value();
} 

void cp_uint(std::vector<unsigned int> parm)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (parm.size() > MAX_CAMS) throw std::runtime_error("More that 2 values for camera parameter");
}

void cp_int(std::vector<int> parm)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (parm.size() > MAX_CAMS) throw std::runtime_error("More that 2 values for camera parameter");
}

void cp_str(std::vector<std::string> parm)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (parm.size() > MAX_CAMS) throw std::runtime_error("More that 2 values for camera parameter");
}

void cp_flt(std::vector<float> parm)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (parm.size() > MAX_CAMS) throw std::runtime_error("More that 2 values for camera parameter");
}

bool bcm2835_encoder_available()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	const char hw_codec[] = "/dev/video11";
	struct v4l2_capability caps;
	memset(&caps, 0, sizeof(caps));
	int fd = open(hw_codec, O_RDWR, 0);
	if (fd)
	{
		int ret = ioctl(fd, VIDIOC_QUERYCAP, &caps);
		close(fd);

		if (!ret && !strncmp((char *)caps.card, "bcm2835-codec-encode", sizeof(caps.card)))
			return true;
	}
	return false;
}

static int xioctl(int fd, unsigned long ctl, void *arg)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

static bool set_imx708_subdev_hdr_ctrl(int en, const std::string &cam_id)
{
	for (unsigned int i = 0; i < 16; i++)
	{
		const fs::path test_dir { "/sys/class/video4linux/v4l-subdev" + std::to_string(i) + "/device" };
		const fs::path module_dir { test_dir.string() + "/driver/module" };
		const fs::path id_dir { test_dir.string() + "/of_node" };

		if (fs::exists(module_dir) && fs::is_symlink(module_dir))
		{
			fs::path ln = fs::read_symlink(module_dir);
			if (ln.string().find("imx708") != std::string::npos &&
				fs::is_symlink(id_dir) && fs::read_symlink(id_dir).string().find(cam_id) != std::string::npos)
			{
				const std::string dev_node { "/dev/v4l-subdev" + std::to_string(i) };
				int fd = open(dev_node.c_str(), O_RDONLY, 0);
				if (fd < 0)
					continue;

				v4l2_control ctrl { V4L2_CID_WIDE_DYNAMIC_RANGE, en };
				if (!xioctl(fd, VIDIOC_G_CTRL, &ctrl) && ctrl.value != en)
				{
					ctrl.value = en;
					if (!xioctl(fd, VIDIOC_S_CTRL, &ctrl))
					{
						close(fd);
						return true;
					}
				}
				close(fd);
			}
		}
	}
	return false;
}

bool Options::Parse(int argc, char *argv[])
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	using namespace boost::program_options;
	using namespace libcamera;
	variables_map vm;
	
	std::map<std::string, int> metering_table =
		{ { "center", libcamera::controls::MeteringCentreWeighted },
			{ "spot", libcamera::controls::MeteringSpot },
			{ "average", libcamera::controls::MeteringMatrix },
			{ "matrix", libcamera::controls::MeteringMatrix },
			{ "custom", libcamera::controls::MeteringCustom } };
			
	std::map<std::string, int> exposure_table =
		{ { "normal", libcamera::controls::ExposureNormal },
			{ "sport", libcamera::controls::ExposureShort },
			{ "short", libcamera::controls::ExposureShort },
			{ "long", libcamera::controls::ExposureLong },
			{ "custom", libcamera::controls::ExposureCustom } };
			
	std::map<std::string, int> awb_table =
		{ { "auto", libcamera::controls::AwbAuto },
			{ "normal", libcamera::controls::AwbAuto },
			{ "incandescent", libcamera::controls::AwbIncandescent },
			{ "tungsten", libcamera::controls::AwbTungsten },
			{ "fluorescent", libcamera::controls::AwbFluorescent },
			{ "indoor", libcamera::controls::AwbIndoor },
			{ "daylight", libcamera::controls::AwbDaylight },
			{ "cloudy", libcamera::controls::AwbCloudy },
			{ "custom", libcamera::controls::AwbCustom } };
			
	std::map<std::string, int> afMode_table =
		{ { "default", -1 },
			{ "manual", libcamera::controls::AfModeManual },
			{ "auto", libcamera::controls::AfModeAuto },
			{ "continuous", libcamera::controls::AfModeContinuous } };
			
	std::map<std::string, int> afRange_table =
		{ { "normal", libcamera::controls::AfRangeNormal },
			{ "macro", libcamera::controls::AfRangeMacro },
			{ "full", libcamera::controls::AfRangeFull } };
			
	std::map<std::string, int> afSpeed_table =
		{ { "normal", libcamera::controls::AfSpeedNormal },
		    { "fast", libcamera::controls::AfSpeedFast } };
		    
	std::map<std::string, Streams> stream_table =
		{ { "0", Camera0 },
			{ "1", Camera1 },
			{ "o", Overlay },
		    { "a", AudioOut },
		    { "cam0", Camera0 },
			{ "cam1", Camera1 },
			{ "ovrly", Overlay },
		    { "audio", AudioOut } };
		    
	enum Locations { LeftBottom, RightBottom, RightTop, LeftTop, Center };
		    
	std::map<std::string, Locations> location_table =
		{ { "leftbottom", LeftBottom },
			{ "rightbottom", RightBottom },
			{ "righttop", RightTop },
		    { "lefttop", LeftTop },
		    { "center", Center },
		    { "lb", LeftBottom },
			{ "rb", RightBottom },
			{ "rt", RightTop },
		    { "lt", LeftTop },
		    { "c", Center } };	  
		    	
	// Read options from the command line
	store(parse_command_line(argc, argv, options_), vm);
	notify(vm);
	// Read options from a file if specified
	std::ifstream ifs(config_file.c_str());
	if (ifs)
	{
		store(parse_config_file(ifs, options_), vm);
		notify(vm);
	}
  
	if (help)
	{
		std::cout << options_;
		return false;
	}

	libcamera::logSetTarget(libcamera::LoggingTargetNone);
	app_->initCameraManager();
		
	if (version)
	{
		std::cout << "rpi2cam-vid build: " << RPi2CamAppsVersion() << std::endl;
		std::cout << "libcamera build: " << libcamera::CameraManager::version() << std::endl;
		return false;
	}

	bool log_env_set = getenv("LIBCAMERA_LOG_LEVELS");
	// Unconditionally set the logging level to error for a bit. why? set this at this point 
	if (!log_env_set)
		libcamera::logSetLevel("*", "ERROR");

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = app_->GetCameras();

	if (cameras.size() == 0)
	{
		std::cout << "No camera found!!" << std::endl;
		return false;
	}
	if (cameras.size() == 1)  
	{
		stream_available_[Camera0] = true;
	} else if (cameras.size())
	{
		stream_available_[Camera0] = stream_available_[Camera1] = 
		stream_available_[Overlay] = true;
	} else
	{
		std::cout << "more than 2 cameras found!!" << std::endl;
		return false;
	}
	if (!audio_device.empty()) stream_available_[AudioOut] = true;
	
// parse output_v_ 
	if (output_v_.size() == 0 && !list_cameras)  throw std::runtime_error("No --output parameter");
	bool file_last = false;
	std::string file;
	std::vector<std::string>::iterator it;
	for(it = output_v_.begin(); it != output_v_.end(); it++)    
	{	
		std::string lc = *it;
		boost::to_lower(lc);
		if (stream_table.count(lc) == 0) 
		{
			if (file_last) throw std::runtime_error("File with no stream for: " + *--it);
			file_last = true;
			file = *it;
			if ( !format_table.insert( std::pair<std::string, Streams>( *it, {} )).second) throw std::runtime_error("File already used: " + *it);
		}
		else
		{
			file_last = false;
			if (!stream_available_[stream_table[lc]]) throw std::runtime_error("Stream File " + *it + " is not available");
			for (Streams st:format_table[file]) {if (st == stream_table[lc]) throw std::runtime_error("Stream " + *it + "already in " + file);}
			stream_selected[stream_table[lc]] = true;
			format_table[file].push_back(stream_table[lc]);
		}
	}
	if (file_last) throw std::runtime_error("File with no stream for: " + *--it);
	if (stream_selected[Overlay] || (stream_selected[Camera0] && stream_selected[Camera1])) 
	{
		camera = 2;
		multi_cam = true;
	}
	else if (stream_selected[Camera0]) camera = 0;
	else camera = 1;

	// if only one camera param given and two cameras selected dup it 	
	if (multi_cam) 
	{			    
		if (width_v.size() == 1) width_v.push_back(width_v[0]);
		if (height_v.size() == 1) height_v.push_back(height_v[0]);
		if (orientation_v.size() == 1) orientation_v.push_back(height_v[0]);
		if (roi_v.size() == 1) roi_v.push_back(roi_v[0]);
		if (shutter_v_.size() == 1) shutter_v_.push_back(shutter_v_[0]);
		if (gain_v.size() == 1) gain_v.push_back(gain_v[0]);
		if (metering_v.size() == 1) metering_v.push_back(metering_v[0]);
		if (exposure_v.size() == 1) exposure_v.push_back(exposure_v[0]);
		if (ev_v.size() == 1) ev_v.push_back(ev_v[0]);
		if (awb_v.size() == 1) awb_v.push_back(awb_v[0]);
		if (awbgains_v.size() == 1) awbgains_v.push_back(awbgains_v[0]);
		if (brightness_v.size() == 1) brightness_v.push_back(brightness_v[0]);
		if (contrast_v.size() == 1) contrast_v.push_back(contrast_v[0]);
		if (saturation_v.size() == 1) saturation_v.push_back(saturation_v[0]);
		if (sharpness_v.size() == 1) sharpness_v.push_back(sharpness_v[0]);
		if (framerate_v_.size() == 1) framerate_v_.push_back(framerate_v_[0]);
		if (denoise_v.size() == 1) denoise_v.push_back(denoise_v[0]);
		if (mode_string_v.size() == 1) mode_string_v.push_back(mode_string_v[0]);
		if (buffer_count_v.size() == 1) buffer_count_v.push_back(buffer_count_v[0]);
		if (afMode_v.size() == 1) afMode_v.push_back(afMode_v[0]);
		if (afRange_v.size() == 1) afRange_v.push_back(afRange_v[0]);
		if (afSpeed_v.size() == 1) afSpeed_v.push_back(afSpeed_v[0]);
		if (afWindow_v.size() == 1) afWindow_v.push_back(afWindow_v[0]);
		if (lens_position_v_.size() == 1) lens_position_v_.push_back(lens_position_v_[0]);
		if (hdr_v.size() == 1) hdr_v.push_back(hdr_v[0]);
		if (flicker_period_v_.size() == 1) flicker_period_v_.push_back(flicker_period_v_[0]);
	}
	   		    
	for (int i = 0; i < (multi_cam ? 2 : 1); i++)   
	{
		if (sscanf(roi_v[i].c_str(), "%f,%f,%f,%f", &roi_x_a[i], &roi_y_a[i], &roi_width_a[i], &roi_height_a[i]) != 4)
			roi_x_a[i] = roi_y_a[i] = roi_width_a[i] = roi_height_a[i] = 0; // don't set digital zoom
	
	// Convert time strings to durations 		
		shutter_a[i].set(shutter_v_[i]);
		flicker_period_a[i].set(flicker_period_v_[i]);

		if (metering_table.count(metering_v[i]) == 0)
			throw std::runtime_error("Invalid metering mode: " + metering_v[i]);
		metering_index_a[i] = metering_table[metering_v[i]];
		
		if (exposure_table.count(exposure_v[i]) == 0)
			throw std::runtime_error("Invalid exposure mode:" + exposure_v[i]);
		exposure_index_a[i] = exposure_table[exposure_v[i]];
		
		if (awb_table.count(awb_v[i]) == 0)
			throw std::runtime_error("Invalid AWB mode: " + awb_v[i]);
		awb_index_a[i] = awb_table[awb_v[i]];
		if (sscanf(awbgains_v[i].c_str(), "%f,%f", &awb_gain_r_a[i], &awb_gain_b_a[i]) != 2)
			throw std::runtime_error("Invalid AWB gains");
			
		brightness_v[i] = std::clamp(brightness_v[i], -1.0f, 1.0f);
		contrast_v[i] = std::clamp(contrast_v[i], 0.0f, 15.99f); // limits are arbitrary.
		saturation_v[i]= std::clamp(saturation_v[i], 0.0f, 15.99f); // limits are arbitrary..
		sharpness_v[i] = std::clamp(sharpness_v[i], 0.0f, 15.99f); // limits are arbitrary..
		
		if (afMode_table.count(afMode_v[i]) == 0)
			throw std::runtime_error("Invalid AfMode:" + afMode_v[i]);
		afMode_index_a[i] = afMode_table[afMode_v[i]];
		if (afRange_table.count(afRange_v[i]) == 0)
			throw std::runtime_error("Invalid AfRange mode:" + afRange_v[i]);
		afRange_index_a[i] = afRange_table[afRange_v[i]];
		if (afSpeed_table.count(afSpeed_v[i]) == 0)
			throw std::runtime_error("Invalid afSpeed mode:" + afSpeed_v[i]);
		afSpeed_index_a[i] = afSpeed_table[afSpeed_v[i]];
		if (sscanf(afWindow_v[i].c_str(), "%f,%f,%f,%f", &afWindow_x_a[i], &afWindow_y_a[i], &afWindow_width_a[i], &afWindow_height_a[i]) != 4)
		afWindow_x_a[i] = afWindow_y_a[i] = afWindow_width_a[i] = afWindow_height_a[i] = 0; // don't set auto focus windows
	// lens_position is even more awkward, because we have two "default"
	// behaviours: Either no lens movement at all (if option is not given),
	// or libcamera's default control value (typically the hyperfocal).
		float f = 0.0;
		set_default_lens_position_a[i] = false;
		if (std::istringstream(lens_position_v_[i]) >> f)
			lens_position_a[i] = f;
		else if (lens_position_v_[i] == "default")
			set_default_lens_position_a[i] = true;
		else if (!lens_position_v_[i].empty())
			throw std::runtime_error("Invalid lens position: " + lens_position_v_[i]);
			
		if (mode_string_v.size()) 
		{
			fprintf(stdout, "%s:%s:%d %s\n", __FILE__, __PRETTY_FUNCTION__, __LINE__,mode_string_v[i].c_str());
			mode_a[i] = Mode(mode_string_v[i]);
		}
		
		if (hdr_v[i] != "off" && hdr_v[i] != "single-exp" && hdr_v[i] != "sensor" && hdr_v[i] != "auto")
			throw std::runtime_error("Invalid HDR option provided: " + hdr_v[i]);
	
		const std::string cam_id = *cameras[i]->properties().get(libcamera::properties::Model);
		if (cam_id.find("imx708") != std::string::npos)
		{
			// HDR control. Set the sensor control before opening or listing any cameras.
			// Start by disabling HDR unconditionally. Reset the camera manager if we have
			// actually switched the value of the control
			bool changed = set_imx708_subdev_hdr_ctrl(0, cameras[i]->id());

			if (hdr_v[i] == "sensor" || hdr_v[i] == "auto")
			{
				// Turn on sensor HDR.  Reset the camera manager if we have switched the value of the control.
				changed |= set_imx708_subdev_hdr_ctrl(1, cameras[i]->id());
				hdr_v[i] = "sensor";
			}
			if (changed)
			{
				cameras.clear();
				app_->initCameraManager();
				cameras = app_->GetCameras();
			}
		}
			
	// This is to get round the fact that the boost option parser does not
	// allow std::optional types.
		if (framerate_v_[i] != -1.0) framerate_a[i] = framerate_v_[i];
			
		if (orientation_v[i] > 7) throw std::runtime_error("Orientation parameter is to large" + orientation_v[i]);	
		
	}
	
	// We have to pass the tuning file name through an environment variable.
	// Note that we only overwrite the variable if the option was given.
	if (tuning_file != "-")
		setenv("LIBCAMERA_RPI_TUNING_FILE", tuning_file.c_str(), 1);

	if (!verbose || list_cameras)
		libcamera::logSetTarget(libcamera::LoggingTargetNone);
	
	if (list_cameras)
	{
		RPi2CamApp::verbosity = 1;
		if (cameras.size() != 0)
		{
			unsigned int idx = 0;
			std::cout << "Available cameras" << std::endl
					  << "-----------------" << std::endl;
			for (auto const &cam : cameras)
			{
				cam->acquire();
				std::stringstream sensor_props;
				sensor_props << "Camera" << idx++ <<  " : " << *cam->properties().get(libcamera::properties::Model) << " [";
				auto area = cam->properties().get(properties::PixelArrayActiveAreas);
				if (area)
					sensor_props << (*area)[0].size().toString() << " ";
				std::unique_ptr<CameraConfiguration> config = cam->generateConfiguration({libcamera::StreamRole::Raw});
				if (!config)
					throw std::runtime_error("failed to generate capture configuration");
				const StreamFormats &formats = config->at(0).formats();
				if (!formats.pixelformats().size())
					continue;
				unsigned int bits = 0;
				for (const auto &pix : formats.pixelformats())
				{
					const auto &b = bayer_formats.find(pix);
					if (b != bayer_formats.end() && b->second > bits)
						bits = b->second;
				}
				if (bits)
					sensor_props << bits << "-bit ";
				auto cfa = cam->properties().get(properties::draft::ColorFilterArrangement);
				if (cfa && cfa_map.count(*cfa))
					sensor_props << cfa_map.at(*cfa) << " ";
				sensor_props.seekp(-1, sensor_props.cur);
				sensor_props << "] (" << cam->id() << ")";
				std::cout << sensor_props.str() << std::endl;
				ControlInfoMap control_map;
				Size max_size;
				PixelFormat max_fmt;
				std::cout << "    Modes: ";
				unsigned int i = 0;
				for (const auto &pix : formats.pixelformats())
				{
					if (i++) std::cout << "           ";
					std::string mode("'" + pix.toString() + "' : ");
					std::cout << mode;
					unsigned int num = formats.sizes(pix).size();
					for (const auto &size : formats.sizes(pix))
					{
						RPi2CamApp::SensorMode sensor_mode(size, pix, 0);
						std::cout << size.toString() << " ";
						config->at(0).size = size;
						config->at(0).pixelFormat = pix;
						config->sensorConfig = libcamera::SensorConfiguration();
						config->sensorConfig->outputSize = size;
						config->sensorConfig->bitDepth = sensor_mode.depth();
						config->validate();
						cam->configure(config.get());
						if (size > max_size)
						{
							control_map = cam->controls();
							max_fmt = pix;
							max_size = size;
						}
						auto fd_ctrl = cam->controls().find(&controls::FrameDurationLimits);
						auto crop_ctrl = cam->controls().at(&controls::ScalerCrop).max().get<Rectangle>();
						double fps = fd_ctrl == cam->controls().end() ? NAN : (1e6 / fd_ctrl->second.min().get<int64_t>());
						std::cout << std::fixed << std::setprecision(2) << "["
								  << fps << " fps - " << crop_ctrl.toString() << " crop" << "]";
						if (--num)
						{
							std::cout << std::endl;
							for (std::size_t s = 0; s < mode.length() + 11; std::cout << " ", s++);
						}
					}
					std::cout << std::endl;
				}
				if (verbose > 1)
				{
					std::stringstream ss;
					ss << "\n    Available controls for " << max_size.toString() << " " << max_fmt.toString()
					   << " mode:\n    ";
					std::cout << ss.str();
					for (std::size_t s = 0; s < ss.str().length() - 10; std::cout << "-", s++);
					std::cout << std::endl;
					std::vector<std::string> ctrls;
					for (auto const &[id, info] : control_map)
						ctrls.emplace_back(id->name() + " : " + info.toString());
					std::sort(ctrls.begin(), ctrls.end(), [](auto const &l, auto const &r) { return l < r; });
					for (auto const &c : ctrls)
						std::cout << "    " << c << std::endl;
				}
				std::cout << std::endl;
				cam->release();		
			}
		} 
		else
		{
			std::cout << "No cameras available!" << std::endl;
			return false;
		}
		return false;
	}

//	bool log_env_set = getenv("LIBCAMERA_LOG_LEVELS");  //needed?? WEK
	// Unconditionally set the logging level to error for a bit.
//	if (!log_env_set) //needed?? WEK
//		libcamera::logSetLevel("*", "ERROR");  //needed?? WEK
		
	// Convert time string to durations
	timeout.set(timeout_);
	

	// Reset log level to Info.
	if (verbose && !log_env_set)
		libcamera::logSetLevel("*", "INFO");

	// Set the verbosity
	RPi2CamApp::verbosity = verbose;
	
	bitrate.set(bitrate_);
#if LIBAV_PRESENT
	audio_bitrate.set(audio_bitrate_);
#endif /* LIBAV_PRESENT */

		
	if (libav_video_codec == "h264_v4l2m2m" && !(bcm2835_encoder_available()))
		{
			LOG(1, "Hardware encode unavailable - h264_v4l2m2m overriding to libx264");
			libav_video_codec = "libx264";
		} 
		
		// From https://en.wikipedia.org/wiki/Advanced_Video_Coding#Levels
	for (int cam = (camera == 1) ? 1 : 0; cam < ((camera == 0) ? 1 : 2); cam++)
	{
		double mbps = ((width_v[cam] + 15) >> 4) * ((height_v[cam] + 15) >> 4) * framerate_a[cam].value_or(DEF_FRAMERATE);
		if (libav_video_codec == "libx264" && mbps > 245760.0)
		{
			LOG(1, "Overriding H.264 level 4.2 for camera " << cam << "streams.");
			level_a[cam] = 42 ;
		}
	} 
	
	// parse overlay location and size  
    if (stream_selected[Overlay])
	{
		unsigned int m = overlay_main;
		if (!overlay_width) overlay_width = width_v[m]/3;	
		if (!overlay_height) overlay_height = height_v[m]/3;
		if (sscanf(overlay_location_.c_str(), "%u:%u", &overlay_x, &overlay_y) != 2)
		{
			std::string lc = overlay_location_;
			boost::to_lower(lc);
			if (location_table.count(lc) == 0)
			{
				std::cout << "Invalid overlay location:" << overlay_location_ << std::endl;
				return false;
			}
			Locations l = location_table[lc];
			switch (l) 
			{
				case LeftBottom:   //not working
					{overlay_x = 0;
						overlay_y = height_v[m] - overlay_height;
					break;}
				case RightBottom:  //not working
					{overlay_x = width_v[m] - overlay_width;
						overlay_y = height_v[m] - overlay_height;
					break;}
				case RightTop:
					{overlay_x = width_v[m] - overlay_width;
						overlay_y = 0;
					break;}
				case LeftTop:
					{overlay_x = 0;
						overlay_y = 0;
					break;}
				case Center:  
					{overlay_x = (width_v[m]/2) - (overlay_width/2);
						overlay_y = (height_v[m]/2) - (overlay_height/2);}
			}
		}	
	} 
	return true;
}

void Options::Print() const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);

	std::map< Streams, std::string> stream_desc_table =
		{ { Camera0, "Camera O" },
			{ Camera1, "Camera 1"},
			{ Overlay, "Overlay" },
		    { AudioOut, "Audio" } };
		    
	std::cerr << "Options:" << std::endl;
	std::cerr << "    verbose: " << verbose << std::endl;
	if (!config_file.empty())
		std::cerr << "    config file: " << config_file << std::endl;
	
	std::string line = "    output: ";
	for (const auto& pair : format_table) 
	{ 
		line += pair.first + " streams: ";
		for (Streams st : pair.second) 
		{
			line += stream_desc_table[st] + " ";
		}
		std::cerr << line << std::endl;
		line.clear();
		line = "            ";
		}
	
	std::cerr << "    timeout: " << timeout.get() << "ms" << std::endl;
	std::cerr << std::endl;

	for (int cam = (camera == 1) ? 1 : 0; cam < ((camera == 0) ? 1 : 2); cam++)
		{
		std::cerr << "Camera " << cam << std::endl;
		std::cerr << "    width: " << width_v[cam] << std::endl;
		std::cerr << "    height: " << height_v[cam] << std::endl;
		std::cerr << "    framerate: " << framerate_a[cam].value_or(DEF_FRAMERATE) << std::endl;
		if (mode_string_v.size())
			std::cerr << "    mode: " << mode_a[cam].ToString() << std::endl;
		std::cerr << "    orientation: ";
		for (int i = 2; i >= 0; i--) {std::cerr << ((orientation_v[cam] >> i) & 1);}
		std::cerr << std::endl; 
		if (shutter_a[cam]) std::cerr << "    shutter: " << shutter_a[cam].get() << "us" << std::endl;
		if (gain_v[cam]) std::cerr << "    gain: " << gain_v[cam] << std::endl;
		if (roi_width_a[cam] == 0 || roi_height_a[cam] == 0)
			std::cerr << "    roi: all" << std::endl;
		else
			std::cerr << "    roi: " << roi_x_a[cam] << "," << roi_y_a[cam] << "," << roi_width_a[cam] << "," << roi_height_a[cam] << std::endl;
		std::cerr << "    metering: " << metering_v[cam] << std::endl;
		std::cerr << "    exposure: " << exposure_v[cam] << std::endl;
		std::cerr << "    ev: " << ev_v[cam] << std::endl;
		std::cerr << "    awb: " << awb_v[cam] << std::endl;
		if (awb_gain_r_a[cam] && awb_gain_b_a[cam])
			std::cerr << "    awb gains: red " << awb_gain_r_a[cam] << " blue " << awb_gain_b_a[cam] << std::endl;
		std::cerr << "    brightness: " << brightness_v[cam] << std::endl;
		std::cerr << "    contrast: " << contrast_v[cam] << std::endl;
		std::cerr << "    saturation: " << saturation_v[cam] << std::endl;
		std::cerr << "    sharpness: " << sharpness_v[cam] << std::endl;
		std::cerr << "    denoise: " << denoise_v[cam] << std::endl;
		if (buffer_count_v[cam] > 0)
			std::cerr << "    buffer-count: " << buffer_count_v[cam] << std::endl;
		if (afMode_index_a[cam] != -1)
			std::cerr << "    autofocus-mode: " << afMode_index_a[cam] << std::endl;
		if (afRange_index_a[cam] != -1)
		std::cerr << "    autofocus-range: " << afRange_v[cam] << std::endl;
		if (afSpeed_index_a[cam] != -1)
			std::cerr << "    autofocus-speed: " << afSpeed_v[cam] << std::endl;
		if (afWindow_width_a[cam] == 0 || afWindow_height_a[cam]  == 0)
			std::cerr << "    autofocus-window: all" << std::endl;
		else
			std::cerr << "    autofocus-window: " << afWindow_x_a[cam]  << "," << afWindow_y_a[cam]  << "," << afWindow_width_a[cam]  << ","
				  << afWindow_height_a[cam]  << std::endl;
		if (!lens_position_v_[cam].empty())
			std::cerr << "    lens-position: " << lens_position_v_[cam] << std::endl;
		if (flicker_period_a[cam])
			std::cerr << "    flicker period: " << flicker_period_a[cam].get() << "us" << std::endl;
		std::cerr << "    hdr: " << hdr_v[cam] << std::endl;
		std::cerr << std::endl;
		}
		
	std::cerr << "    tuning-file: " << (tuning_file == "-" ? "(libcamera)" : tuning_file) << std::endl;
	std::cerr << "    bitrate: " << bitrate.kbps() << "kbps" << std::endl;
	std::cerr << "    profile: " << profile << std::endl;
	std::cerr << "    level:  " << level << std::endl;
	std::cerr << "    intra: " << intra << std::endl;
	std::cerr << "    inline: " << inline_headers << std::endl;
	std::cerr << "    libav-video-codec: " << libav_video_codec << std::endl;
	std::cerr << "    libav-video-codec-opts: " << libav_video_codec_opts << std::endl;
	std::cerr << "    libav-format: " << libav_format << std::endl;
	if (stream_selected[AudioOut])
	{
		std::cerr << "    audio-codec: " << audio_codec << std::endl;
		std::cerr << "    audio-source: " << audio_source << std::endl;
		std::cerr << "    audio-device: " << audio_device << std::endl;
		std::cerr << "    audio-channels: " << audio_channels << std::endl;
		std::cerr << "    audio-bitrate: " << audio_bitrate_ << std::endl;
		std::cerr << "    audio-samplerate: " << audio_samplerate << std::endl;
	}
	if (stream_selected[Overlay])
	{
		std::cerr << "    overlay-main: " << overlay_main << std::endl;
		std::cerr << "    overlay-w: " << overlay_width << std::endl;
		std::cerr << "    overlay-h: " << overlay_height << std::endl;
		std::cerr << "    overlay-location: " << overlay_location_ << std::endl;
	}
}

  
