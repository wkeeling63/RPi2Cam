/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * rpi2cam_app.cpp - base class for libcamera apps.
 * WEK
 */

#include "core/rpi2cam_app.hpp"
#include "core/options.hpp"

#include <cmath>
#include <fcntl.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/dma-buf.h>
#include <linux/videodev2.h>

#include <libcamera/base/shared_fd.h>
#include <libcamera/orientation.h>

unsigned int RPi2CamApp::verbosity = 1;

enum class Platform
{
	MISSING,
	UNKNOWN,
	LEGACY,
	VC4,
	PISP,
};

Platform get_platform()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	bool unknown = false;
	for (unsigned int device_num = 0; device_num < 256; device_num++)
	{
		char device_name[16];
		snprintf(device_name, sizeof(device_name), "/dev/video%u", device_num);
				int fd = open(device_name, O_RDWR, 0);

		if (fd < 0)
			continue;

		v4l2_capability caps;
		unsigned long request = VIDIOC_QUERYCAP;

		int ret = ioctl(fd, request, &caps);
		close(fd);

		if (ret)
			continue;

		// We are not concerned with UVC devices for now.
		if (!strncmp((char *)caps.driver, "uvcvideo", sizeof(caps.card)))
			continue;

		if (!strncmp((char *)caps.card, "bcm2835-isp", sizeof(caps.card)))
			return Platform::VC4;
		else if (!strncmp((char *)caps.card, "pispbe", sizeof(caps.card)))
			return Platform::PISP;
		else if (!strncmp((char *)caps.card, "bm2835 mmal", sizeof(caps.card)))
			return Platform::LEGACY;
		else
			unknown = true;
	}

	return unknown ? Platform::UNKNOWN : Platform::MISSING;
}

static void set_pipeline_configuration(Platform platform)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	// Respect any pre-existing value in the environment variable.
	char const *existing_config = getenv("LIBCAMERA_RPI_CONFIG_FILE");
	if (existing_config && existing_config[0])
		return;
		
	// Otherwise point it at whichever of these we find first (if any) for the given platform.
	static const std::vector<std::pair<Platform, std::string>> config_files = {
		{ Platform::VC4, "/usr/local/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml" },
		{ Platform::VC4, "/usr/share/libcamera/pipeline/rpi/vc4/rpi_apps.yaml" },
	};

	for (auto &config_file : config_files)
	{
		struct stat info;
		if (config_file.first == platform && stat(config_file.second.c_str(), &info) == 0)
		{
			setenv("LIBCAMERA_RPI_CONFIG_FILE", config_file.second.c_str(), 1);
			break;
		}
	}
}

RPi2CamApp::RPi2CamApp(std::unique_ptr<Options> opts) : options_(std::move(opts))
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);            
	Platform platform = get_platform();
	if (platform == Platform::LEGACY)
	{
		// If we definitely appear to be running the old camera stack, complain and give up.
		fprintf(stderr, "ERROR: the system appears to be configured for the legacy camera stack\n");
		exit(-1);
	}
	else if (platform == Platform::UNKNOWN)
	{
		fprintf(stderr, "ERROR: rpicam-apps currently only supports the Raspberry Pi platforms.\n"
						"Contributions for other platforms are welcome at https://github.com/raspberrypi/rpicam-apps.\n");
		exit(-1);
	}

	if (!options_) 
		options_ = std::make_unique<Options>();

	options_->SetApp(this);
	set_pipeline_configuration(platform);
		
	for (unsigned int i = 0; i < MAX_CAMS; i++) {
		camera_acquired_[i] = false;
		camera_started_[i] = false;
		controls_[i] = {controls::controls};
	}
}

RPi2CamApp::~RPi2CamApp()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (!options_->help)
		LOG(2, "Closing RPiCam application");
	StopCamera();
	Teardown();
	CloseCamera();
}

void RPi2CamApp::initCameraManager()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	camera_manager_.reset();
	camera_manager_ = std::make_unique<CameraManager>();
	int ret = camera_manager_->start();
	if (ret)
		throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));
}

void RPi2CamApp::OpenCamera()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	LOG(2, "Opening camera...");

	if (!camera_manager_)
		initCameraManager();

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = GetCameras();
	if (cameras.size() == 0)
		throw std::runtime_error("no cameras available");

	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)  
	{
		int cam = (options_->multi_cam) ? i : options_->camera;
		std::string const &cam_id = cameras[cam]->id();
		camera_[cam] = camera_manager_->get(cam_id);

		if (!camera_[cam])
			throw std::runtime_error("failed to find camera " + cam_id);

		if (camera_[cam]->acquire())
			throw std::runtime_error("failed to acquire camera " + cam_id);
		camera_acquired_[cam] = true;

		LOG(2, "Acquired camera " << cam_id);

		// We're going to make a list of all the available sensor modes, but we only populate
		// the framerate field if the user has requested a framerate (as this requires us actually
		// to configure the sensor, which is otherwise best avoided).

		std::unique_ptr<CameraConfiguration> config = camera_[cam]->generateConfiguration({ libcamera::StreamRole::Raw });  //WEK raw needed or should be video?
		const libcamera::StreamFormats &formats = config->at(0).formats();

		bool log_env_set = getenv("LIBCAMERA_LOG_LEVELS");
		// Suppress log messages when enumerating camera modes.
		if (!log_env_set)
		{
			libcamera::logSetLevel("RPI", "ERROR");
			libcamera::logSetLevel("Camera", "ERROR");
		}

		for (const auto &pix : formats.pixelformats())
		{
			for (const auto &size : formats.sizes(pix))
			{
				double framerate = 0;
				if (options_->framerate_a[i])
				{
					SensorMode sensorMode(size, pix, 0);
					config->at(0).size = size;
					config->at(0).pixelFormat = pix;
					config->sensorConfig = libcamera::SensorConfiguration();
					config->sensorConfig->outputSize = size;
					config->sensorConfig->bitDepth = sensorMode.depth();
					config->validate();
					camera_[cam]->configure(config.get());
					auto fd_ctrl = camera_[cam]->controls().find(&controls::FrameDurationLimits);
					framerate = 1.0e6 / fd_ctrl->second.min().get<int64_t>();
				}
				sensor_modes_.emplace_back(size, pix, framerate);
			}
		}


		if (!log_env_set)
		{
			libcamera::logSetLevel("RPI", "INFO");
			libcamera::logSetLevel("Camera", "INFO");
		}
	}
}

void RPi2CamApp::CloseCamera()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);

	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)
	{
		int cam = (options_->multi_cam) ? i : options_->camera;
		if (camera_acquired_[cam])
			camera_[cam]->release();
		camera_acquired_[cam] = false;

		camera_[cam].reset();
	}

	camera_manager_.reset();

	if (!options_->help)
		LOG(2, "Camera closed");
}

Mode RPi2CamApp::selectMode(const Mode &mode) const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	auto scoreFormat = [](double desired, double actual) -> double
	{
		double score = desired - actual;
		// Smaller desired dimensions are preferred.
		if (score < 0.0)
			score = (-score) / 8;
		// Penalise non-exact matches.
		if (actual != desired)
			score *= 2;

		return score;
	};

	constexpr float penalty_AR = 1500.0;
	constexpr float penalty_BD = 500.0;
	constexpr float penalty_FPS = 2000.0; 

	double best_score = std::numeric_limits<double>::max(), score;
	SensorMode best_mode;

	LOG(1, "Mode selection for " << mode.ToString());
	for (const auto &sensor_mode : sensor_modes_)
	{
		double reqAr = static_cast<double>(mode.width) / mode.height;
		double fmtAr = static_cast<double>(sensor_mode.size.width) / sensor_mode.size.height;

		// Similar scoring mechanism that our pipeline handler does internally.
		score = scoreFormat(mode.width, sensor_mode.size.width);
		score += scoreFormat(mode.height, sensor_mode.size.height);
		score += penalty_AR * scoreFormat(reqAr, fmtAr);
		if (mode.framerate)
			score += penalty_FPS * std::abs(mode.framerate - std::min(sensor_mode.fps, mode.framerate));
		score += penalty_BD * abs((int)(mode.bit_depth - sensor_mode.depth()));

		if (score <= best_score)
		{
			best_score = score;
			best_mode.size = sensor_mode.size;
			best_mode.format = sensor_mode.format;
			best_mode.fps = mode.framerate;
		}

		LOG(2, "    " << sensor_mode.ToString() << " - Score: " << score);
	}
	
	LOG(1, " Selected mode: " << best_mode.ToString());
	return { best_mode.size.width, best_mode.size.height, best_mode.depth(), mode.packed };
} 

void RPi2CamApp::ConfigureVideo()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	LOG(2, "Configuring video...");

	StreamRoles stream_roles = { StreamRole::VideoRecording };

	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)
	{
		int cam = (options_->multi_cam) ? i : options_->camera;
		configuration_[cam] = camera_[cam]->generateConfiguration(stream_roles);
		if (!configuration_[cam])
			throw std::runtime_error("failed to generate video configuration");

		// Now we get to override any of the default settings from the options_->
		StreamConfiguration &cfg = configuration_[cam]->at(0);
		cfg.pixelFormat = libcamera::formats::YUV420;
		cfg.bufferCount = 6; // 6 buffers is better than 4
		if (options_->buffer_count_v[cam] > 0)
			cfg.bufferCount = options_->buffer_count_v[cam];
		cfg.size.width = options_->width_v[cam];
		cfg.size.height = options_->height_v[cam]; 
		if (cfg.size.width >= 1280 || cfg.size.height >= 720)
			cfg.colorSpace = libcamera::ColorSpace::Rec709;   //HDTV colorspace
		else
			cfg.colorSpace = libcamera::ColorSpace::Smpte170m; // NTSC/PAL/SDTV colorspace
		configuration_[cam]->orientation = libcamera::Orientation::Rotate0 * (libcamera::Transform) options_->orientation_v[cam];

// mode code called here 
		if (options_->mode_string_v.size())
		{												
			options_->mode_a[cam].update(configuration_[cam]->at(0).size, options_->framerate_a[cam]);  // WEK so it always uses h and w so why have it in mode parm?
			options_->mode_a[cam] = selectMode(options_->mode_a[cam]);
			configuration_[cam]->at(0).size = options_->mode_a[cam].Size(); 
			configuration_[cam]->sensorConfig = libcamera::SensorConfiguration();
			configuration_[cam]->sensorConfig->outputSize = options_->mode_a[cam].Size();
			configuration_[cam]->sensorConfig->bitDepth = options_->mode_a[cam].bit_depth;
		}

		configureDenoise(cam, options_->denoise_v[cam] == "auto" ? "cdn_fast" : options_->denoise_v[cam]);
		setupCapture(cam);
		
		if (cam)
			streams_[LibcStreamType::Camera1] = configuration_[cam]->at(0).stream();
		else
			streams_[LibcStreamType::Camera0] = configuration_[cam]->at(0).stream();
		
	}
	LOG(2, "Video setup complete");
}

void RPi2CamApp::Teardown()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);

	if (!options_->help)
		LOG(2, "Tearing down requests, buffers and configuration");

	for (auto &iter : mapped_buffers_)
	{
		for (auto &span : iter.second)
			munmap(span.data(), span.size());
	}
	mapped_buffers_.clear();

	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)
	{
		configuration_[i].reset();
		frame_buffers_[i].clear(); 
	}
	streams_.clear();
}

void RPi2CamApp::StartCamera()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	// This makes all the Request objects that we shall need.
	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)
	{
		int cam = (options_->multi_cam) ? i : options_->camera;
		makeRequests(cam);

		if (!controls_[cam].get(controls::ScalerCrop) && options_->roi_width_a[cam] != 0 && options_->roi_height_a[cam] != 0)
		{
			Rectangle sensor_area = camera_[cam]->controls().at(&controls::ScalerCrop).max().get<Rectangle>();

			int x = options_->roi_x_a[cam] * sensor_area.width;
			int y = options_->roi_y_a[cam] * sensor_area.height;
			int w = options_->roi_width_a[cam] * sensor_area.width;
			int h = options_->roi_height_a[cam] * sensor_area.height;
			Rectangle crop(x, y, w, h);
			crop.translateBy(sensor_area.topLeft());
			LOG(2, "Using crop " << crop.toString());
			controls_[cam].set(controls::ScalerCrop, crop);
		}

		if (!controls_[cam].get(controls::AfWindows) && !controls_[cam].get(controls::AfMetering) && options_->afWindow_width_a[cam]  != 0 &&
			options_->afWindow_height_a[cam]  != 0)
		{
			Rectangle sensor_area = camera_[cam]->controls().at(&controls::ScalerCrop).max().get<Rectangle>();
			int x = options_->afWindow_x_a[cam] * sensor_area.width;
			int y = options_->afWindow_y_a[cam]  * sensor_area.height;
			int w = options_->afWindow_width_a[cam]  * sensor_area.width;
			int h = options_->afWindow_height_a[cam]  * sensor_area.height;
			Rectangle afwindows_rectangle[1];
			afwindows_rectangle[0] = Rectangle(x, y, w, h);
			afwindows_rectangle[0].translateBy(sensor_area.topLeft());
			LOG(2, "Using AfWindow " << afwindows_rectangle[0].toString());
			//activate the AfMeteringWindows
			controls_[cam].set(controls::AfMetering, controls::AfMeteringWindows);
			//set window
			controls_[cam].set(controls::AfWindows, afwindows_rectangle);
		}

		// Framerate is a bit weird. If it was set programmatically, we go with that, but
		// otherwise it applies only to preview/video modes. For stills capture we set it
		// as long as possible so that we get whatever the exposure profile wants.
		if (!controls_[cam].get(controls::FrameDurationLimits))
		{
			if (!options_->framerate_a[cam] || options_->framerate_a[cam].value() > 0)
			{
				int64_t frame_time = 1000000 / options_->framerate_a[cam].value_or(DEF_FRAMERATE); // in us
				controls_[cam].set(controls::FrameDurationLimits,
							libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
			}
		}

		if (!controls_[cam].get(controls::ExposureTime) && options_->shutter_a[cam])
			controls_[cam].set(controls::ExposureTime, options_->shutter_a[cam].get<std::chrono::microseconds>());
		if (!controls_[cam].get(controls::AnalogueGain) && options_->gain_v[cam])
			controls_[cam].set(controls::AnalogueGain, options_->gain_v[cam]);
		if (!controls_[cam].get(controls::AeMeteringMode))
			controls_[cam].set(controls::AeMeteringMode, options_->metering_index_a[cam]);
		if (!controls_[cam].get(controls::AeExposureMode))
			controls_[cam].set(controls::AeExposureMode, options_->exposure_index_a[cam]);
		if (!controls_[cam].get(controls::ExposureValue))
			controls_[cam].set(controls::ExposureValue, options_->ev_v[cam]);
		if (!controls_[cam].get(controls::AwbMode))
			controls_[cam].set(controls::AwbMode, options_->awb_index_a[cam]);
		if (!controls_[cam].get(controls::ColourGains) && options_->awb_gain_r_a[cam] && options_->awb_gain_b_a[cam])
			controls_[cam].set(controls::ColourGains,
					  libcamera::Span<const float, 2>({ options_->awb_gain_r_a[cam], options_->awb_gain_b_a[cam] }));
		if (!controls_[cam].get(controls::Brightness))
			controls_[cam].set(controls::Brightness, options_->brightness_v[cam]);
		if (!controls_[cam].get(controls::Contrast))
			controls_[cam].set(controls::Contrast, options_->contrast_v[cam]);
		if (!controls_[cam].get(controls::Saturation))
			controls_[cam].set(controls::Saturation, options_->saturation_v[cam]);
		if (!controls_[cam].get(controls::Sharpness))
			controls_[cam].set(controls::Sharpness, options_->sharpness_v[cam]);

		if (!controls_[cam].get(controls::HdrMode) &&
			(options_->hdr_v[cam] == "auto" || options_->hdr_v[cam] == "single-exp"))
			controls_[cam].set(controls::HdrMode, controls::HdrModeSingleExposure);

		// AF Controls, where supported and not already set
		if (!controls_[cam].get(controls::AfMode) && camera_[cam]->controls().count(&controls::AfMode) > 0)
		{
			int afm = options_->afMode_index_a[cam];
			if (afm == -1)
			{
				// Choose a default AF mode based on other options
				if (options_->lens_position_a[cam] || options_->set_default_lens_position_a[cam])
					afm = controls::AfModeManual;
				else
					afm = camera_[cam]->controls().at(&controls::AfMode).max().get<int>();
			}
			controls_[cam].set(controls::AfMode, afm);
		}
		if (!controls_[cam].get(controls::AfRange) && camera_[cam]->controls().count(&controls::AfRange) > 0)
			controls_[cam].set(controls::AfRange, options_->afRange_index_a[cam]);
		if (!controls_[cam].get(controls::AfSpeed) && camera_[cam]->controls().count(&controls::AfSpeed) > 0)
			controls_[cam].set(controls::AfSpeed, options_->afSpeed_index_a[cam]);

		if (controls_[cam].get(controls::AfMode).value_or(controls::AfModeManual) == controls::AfModeAuto)
		{
			// When starting a viewfinder or video stream in AF "auto" mode,
			// trigger a scan now (but don't move the lens when capturing a still).
			// If an application requires more control over AF triggering, it may
			// override this behaviour with prior settings of AfMode or AfTrigger.
			if (!controls_[cam].get(controls::AfTrigger))
				controls_[cam].set(controls::AfTrigger, controls::AfTriggerStart);
		}
		else if ((options_->lens_position_a[cam] || options_->set_default_lens_position_a[cam]) &&
			 camera_[cam]->controls().count(&controls::LensPosition) > 0 && !controls_[cam].get(controls::LensPosition))
		{
			float f;
			if (options_->lens_position_a[cam])
				f = options_->lens_position_a[cam].value();
			else
				f = camera_[cam]->controls().at(&controls::LensPosition).def().get<float>();
			LOG(2, "Setting LensPosition: " << f);
			controls_[cam].set(controls::LensPosition, f);
		}

		if (options_->flicker_period_a[cam] && !controls_[cam].get(controls::AeFlickerMode) &&
			camera_[cam]->controls().find(&controls::AeFlickerMode) != camera_[cam]->controls().end() &&
			camera_[cam]->controls().find(&controls::AeFlickerPeriod) != camera_[cam]->controls().end())
		{
			controls_[cam].set(controls::AeFlickerMode, controls::FlickerManual);
			controls_[cam].set(controls::AeFlickerPeriod, options_->flicker_period_a[cam].get<std::chrono::microseconds>());
		}

		if (camera_[cam]->start(&controls_[cam]))
			throw std::runtime_error("failed to start camera");
		controls_[cam].clear();
		camera_started_[cam] = true;
		cameras_running_++;
		last_timestamp_[cam] = 0;

		camera_[cam]->requestCompleted.connect(this, &RPi2CamApp::requestComplete);

		for (std::unique_ptr<Request> &request : requests_[cam])
		{
			if (camera_[cam]->queueRequest(request.get()) < 0)
				throw std::runtime_error("Failed to queue request");
		}
	}
	LOG(2, "Camera started!");
}

void RPi2CamApp::StopCamera()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	for (unsigned int i = 0; i < (options_->multi_cam ? 2 : 1); i++)
	{
		int cam = (options_->multi_cam) ? i : options_->camera;
		{
			// We don't want QueueRequest to run asynchronously while we stop the camera.
			std::lock_guard<std::mutex> lock(camera_stop_mutex_);
			if (camera_started_[cam])
			{
				if (camera_[cam]->stop())
					throw std::runtime_error("failed to stop camera");

				camera_started_[cam] = false;
				cameras_running_--;
			}
		}

		if (camera_[cam])
			camera_[cam]->requestCompleted.disconnect(this, &RPi2CamApp::requestComplete);

		// An application might be holding a CompletedRequest, so queueRequest will get
		// called to delete it later, but we need to know not to try and re-queue it.
		completed_requests_.clear();
		msg_queue_.Clear();
		requests_[cam].clear();
		controls_[cam].clear(); // no need for mutex here
	}
	if (!options_->help)
		LOG(2, "Camera stopped!");
}

RPi2CamApp::Msg RPi2CamApp::Wait()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	return msg_queue_.Wait();
}

void RPi2CamApp::queueRequest(CompletedRequest *completed_request)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	BufferMap buffers(std::move(completed_request->buffers));

	// This function may run asynchronously so needs protection from the
	// camera stopping at the same time.
	std::lock_guard<std::mutex> stop_lock(camera_stop_mutex_);

	// An application could be holding a CompletedRequest while it stops and re-starts
	// the camera, after which we don't want to queue another request now.
	bool request_found;
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		auto it = completed_requests_.find(completed_request);
		if (it != completed_requests_.end())
		{
			request_found = true;
			completed_requests_.erase(it);
		}
		else
			request_found = false;
	}

	Request *request = completed_request->request;
	delete completed_request;
	assert(request);

	if ((!camera_started_[0] && !camera_started_[1]) || !request_found)  
		return;

	for (auto const &p : buffers)
	{
		struct dma_buf_sync dma_sync {};
		dma_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;

		auto it = mapped_buffers_.find(p.second);
		if (it == mapped_buffers_.end())
			throw std::runtime_error("failed to identify queue request buffer");

		int ret = ::ioctl(p.second->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
		if (ret)
			throw std::runtime_error("failed to sync dma buf on queue request");

		if (request->addBuffer(p.first, p.second) < 0)
			throw std::runtime_error("failed to add buffer to request in QueueRequest");
	}
	{
		std::lock_guard<std::mutex> lock(control_mutex_);
		request->controls() = std::move(controls_[request->cookie()]);
	}

	if (camera_[request->cookie()]->queueRequest(request) < 0)
		throw std::runtime_error("failed to queue request");
}

void RPi2CamApp::PostMessage(MsgType &t, MsgPayload &p)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	msg_queue_.Post(Msg(t, std::move(p)));
}

libcamera::Stream *RPi2CamApp::GetCameraStream(int cam) const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	LibcStreamType st;
	if (cam)
		st = LibcStreamType::Camera1;
	else
		st = LibcStreamType::Camera0;
	auto it = streams_.find(st);
	if (it == streams_.end()) 
		return nullptr; 
	return it->second;
}

const libcamera::CameraManager *RPi2CamApp::GetCameraManager() const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	return camera_manager_.get();
}

StreamInfo RPi2CamApp::GetCameraStreamInfo(int cam) const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	LibcStreamType st;
	if (cam)
		st = LibcStreamType::Camera1;
	else
		st = LibcStreamType::Camera0;
	auto it = streams_.find(st);
	if (it == streams_.end()) 		
		throw std::runtime_error("failed to find camera stream info");
	Stream *stream = it->second;
	StreamConfiguration const &cfg = stream->configuration();
	StreamInfo info;
	info.width = cfg.size.width;
	info.height = cfg.size.height;
	info.stride = cfg.stride;
	info.pixel_format = cfg.pixelFormat;
	info.colour_space = cfg.colorSpace;
	info.camera = cam;
	return info;
}
StreamInfo RPi2CamApp::GetStreamInfo(Stream const *stream) const
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	
	StreamConfiguration const &cfg = stream->configuration();
	StreamInfo info;
	info.width = cfg.size.width;
	info.height = cfg.size.height;
	info.stride = cfg.stride;
	info.pixel_format = cfg.pixelFormat;
	info.colour_space = cfg.colorSpace;
	for (auto it = streams_.begin(); it != streams_.end(); ++it)
		if (it->second == stream) 
		{
			if (it->first == LibcStreamType::Camera0)
				info.camera = 0;
			else 
				info.camera = 1;
		}
	return info;
}

void RPi2CamApp::setupCapture(int cam_num)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	// First finish setting up the configuration.
	for (auto &config : *configuration_[cam_num])
		config.stride = 0;
	CameraConfiguration::Status validation = configuration_[cam_num]->validate();
	if (validation == CameraConfiguration::Invalid)
		throw std::runtime_error("failed to valid stream configurations");
	else if (validation == CameraConfiguration::Adjusted)
		LOG(1, "Stream configuration adjusted");

	if (camera_[cam_num]->configure(configuration_[cam_num].get()) < 0)
		throw std::runtime_error("failed to configure streams");
	LOG(2, "Camera streams configured");

	LOG(2, "Available controls:");
	for (auto const &[id, info] : camera_[cam_num]->controls())
		LOG(2, "    " << id->name() << " : " << info.toString());

	// Next allocate all the buffers we need, mmap them and store them on a free list.
	for (StreamConfiguration &config : *configuration_[cam_num])
	{
		Stream *stream = config.stream();
		std::vector<std::unique_ptr<FrameBuffer>> fb;


		for (unsigned int i = 0; i < config.bufferCount; i++)
		{
			std::string name("rpicam-apps" + std::to_string(i));
			libcamera::UniqueFD fd = dma_heap_.alloc(name.c_str(), config.frameSize);

			if (!fd.isValid())
				throw std::runtime_error("failed to allocate capture buffers for stream");

			std::vector<FrameBuffer::Plane> plane(1);
			plane[0].fd = libcamera::SharedFD(std::move(fd));
			plane[0].offset = 0;
			plane[0].length = config.frameSize;

			fb.push_back(std::make_unique<FrameBuffer>(plane));
			void *memory = mmap(NULL, config.frameSize, PROT_READ | PROT_WRITE, MAP_SHARED, plane[0].fd.get(), 0);
			mapped_buffers_[fb.back().get()].push_back(
						libcamera::Span<uint8_t>(static_cast<uint8_t *>(memory), config.frameSize));
		}
		frame_buffers_[cam_num][stream] = std::move(fb);
	}
	LOG(2, "Buffers allocated and mapped");
}

void RPi2CamApp::makeRequests(int cam_num)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	std::map<Stream *, std::queue<FrameBuffer *>> free_buffers;
	
  	for (auto &kv : frame_buffers_[cam_num])
	{
		free_buffers[kv.first] = {};
		for (auto &b : kv.second)
			free_buffers[kv.first].push(b.get());
	}

	while (true)
	{
		for (StreamConfiguration &config : *configuration_[cam_num])
		{
			Stream *stream = config.stream();
			if (stream == configuration_[cam_num]->at(0).stream())
			{
				if (free_buffers[stream].empty())
				{
					LOG(2, "Requests created for camera " << cam_num);
					return;
				}
				std::unique_ptr<Request> request = camera_[cam_num]->createRequest(cam_num);
				if (!request)
					throw std::runtime_error("failed to make request");
				requests_[cam_num].push_back(std::move(request));
			}
			else if (free_buffers[stream].empty())
				throw std::runtime_error("concurrent streams need matching numbers of buffers");

			FrameBuffer *buffer = free_buffers[stream].front();
			free_buffers[stream].pop();
			if (requests_[cam_num].back()->addBuffer(stream, buffer) < 0)
				throw std::runtime_error("failed to add buffer to request");
		}
	}
	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
}

void RPi2CamApp::requestComplete(Request *request)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (request->status() == Request::RequestCancelled)
	{
		// If the request is cancelled while the camera is still running, it indicates
		// a hardware timeout. Let the application handle this error.
		if (camera_started_[request->cookie()])
			msg_queue_.Post(Msg(MsgType::Timeout));

		return;
	}

	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
	for (auto const &buffer_map : request->buffers())
	{
		auto it = mapped_buffers_.find(buffer_map.second);
		if (it == mapped_buffers_.end())
			throw std::runtime_error("failed to identify request complete buffer");

		int ret = ::ioctl(buffer_map.second->planes()[0].fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
		if (ret)
			throw std::runtime_error("failed to sync dma buf on request complete");
	}

	CompletedRequest *r = new CompletedRequest(sequence_++, request);
	CompletedRequestPtr payload(r, [this](CompletedRequest *cr) { this->queueRequest(cr); });
	{
		std::lock_guard<std::mutex> lock(completed_requests_mutex_);
		completed_requests_.insert(r);
	}

	// We calculate the instantaneous framerate in case anyone wants it.
	// Use the sensor timestamp if possible as it ought to be less glitchy than
	// the buffer timestamps.

	auto ts = payload->metadata.get(controls::SensorTimestamp);
	uint64_t timestamp = ts ? *ts : payload->buffers.begin()->second->metadata().timestamp;
	if (last_timestamp_[request->cookie()] == 0 || last_timestamp_[request->cookie()] == timestamp)
		payload->framerate = 0;
	else
		payload->framerate = 1e9 / (timestamp - last_timestamp_[request->cookie()] );
	last_timestamp_[request->cookie()]  = timestamp;

	msg_queue_.Post(Msg(MsgType::RequestComplete, std::move(payload)));
}

void RPi2CamApp::configureDenoise(int cam_num, const std::string &denoise_mode)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	using namespace libcamera::controls::draft;

	static const std::map<std::string, NoiseReductionModeEnum> denoise_table = {
		{ "off", NoiseReductionModeOff },
		{ "cdn_off", NoiseReductionModeMinimal },
		{ "cdn_fast", NoiseReductionModeFast },
		{ "cdn_hq", NoiseReductionModeHighQuality }
	};
	NoiseReductionModeEnum denoise;

	auto const mode = denoise_table.find(denoise_mode);
	if (mode == denoise_table.end())
		throw std::runtime_error("Invalid denoise mode " + denoise_mode);
	denoise = mode->second;

	controls_[cam_num].set(NoiseReductionMode, denoise);
}
