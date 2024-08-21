/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * rpi2cam_encoder.cpp - libav video encoder.
 * WEK
 */

#include <poll.h>
#include <string.h>
#include <sys/mman.h>

#include <libdrm/drm_fourcc.h>

#include <chrono>
#include <iostream>

#include "core/options.hpp"
#include "rpi2cam_encoder.hpp"


namespace {

void encoderOptionsGeneral(Options const *options, AVCodecContext *codec, int cam)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	codec->framerate = { (int)(options->framerate_a[cam].value_or(DEF_FRAMERATE) * 1000), 1000 };
	codec->profile = FF_PROFILE_UNKNOWN;

	if (!options->profile.empty())
	{
		const AVCodecDescriptor *desc = avcodec_descriptor_get(codec->codec_id);
		for (const AVProfile *profile = desc->profiles; profile && profile->profile != FF_PROFILE_UNKNOWN; profile++)
		{
			if (!strncasecmp(options->profile.c_str(), profile->name, options->profile.size()))
			{
				codec->profile = profile->profile;
				break;
			}
		}
		if (codec->profile == FF_PROFILE_UNKNOWN)
			throw std::runtime_error("libav: no such profile " + options->profile);
	}
	
	codec->level = options->level.empty() ? FF_LEVEL_UNKNOWN : std::stof(options->level) * 10;
	codec->level = options->level_a[cam].value_or(codec->level);
	
	codec->gop_size = options->intra ? options->intra : (int)(options->framerate_a[cam].value_or(DEF_FRAMERATE));

	if (options->bitrate)
		codec->bit_rate = options->bitrate.bps();

	if (!options->libav_video_codec_opts.empty())
	{
		const std::string &opts = options->libav_video_codec_opts;
		for (std::string::size_type i = 0, n = 0; i != std::string::npos; i = n)
		{
			n = opts.find(';', i);
			const std::string opt = opts.substr(i, n - i);
			if (n != std::string::npos)
				n++;
			if (opt.empty())
				continue;
			std::string::size_type kn = opt.find('=');
			const std::string key = opt.substr(0, kn);
			const std::string value = (kn != std::string::npos) ? opt.substr(kn + 1) : "";
			int ret = av_opt_set(codec, key.c_str(), value.c_str(), AV_OPT_SEARCH_CHILDREN);
			if (ret < 0)
			{
				char err[AV_ERROR_MAX_STRING_SIZE];
				av_strerror(ret, err, sizeof(err));
				throw std::runtime_error("libav: codec option error " + opt + ": " + err);
			}
		}
	}
}

void encoderOptionsH264M2M(AVCodecContext *codec)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	codec->pix_fmt = AV_PIX_FMT_DRM_PRIME;
	codec->max_b_frames = 0;
}

void encoderOptionsLibx264(AVCodecContext *codec)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	codec->max_b_frames = 1;
	codec->me_range = 16;
	codec->me_cmp = 1; // No chroma ME
	codec->me_subpel_quality = 0;
	codec->thread_count = 0;
	codec->thread_type = FF_THREAD_FRAME;
	codec->slices = 1;

	av_opt_set(codec->priv_data, "preset", "superfast", 0);
	av_opt_set(codec->priv_data, "partitions", "i8x8,i4x4", 0);
	av_opt_set(codec->priv_data, "weightp", "none", 0);
	av_opt_set(codec->priv_data, "weightb", "0", 0);
	av_opt_set(codec->priv_data, "motion-est", "dia", 0);
	av_opt_set(codec->priv_data, "sc_threshold", "0", 0);
	av_opt_set(codec->priv_data, "rc-lookahead", "0", 0);
	av_opt_set(codec->priv_data, "mixed_ref", "0", 0);
}

const std::map<std::string, std::function<void(AVCodecContext *)>> optionsMap =
{
	{ "h264_v4l2m2m", encoderOptionsH264M2M },
	{ "libx264", encoderOptionsLibx264 },
};

} // namespace

AVFormatContext *RPi2CamEncoder::initFormat(Options const *options, std::string dest)
{
	const std::string tcp { "tcp://" };
	const std::string udp { "udp://" };

	// Setup an appropriate stream/container format.
	const char *format = nullptr;
	if (options->libav_format.empty())
	{
	// Check if output_file_ starts with a "tcp://" or "udp://" url.
	// C++ 20 has a convenient starts_with() function for this which we may eventually use.	
		if (dest.empty() ||
			dest.find(tcp.c_str(), 0, tcp.length()) != std::string::npos ||
			dest.find(udp.c_str(), 0, udp.length()) != std::string::npos)
		{
			if (options->libav_video_codec == "h264_v4l2m2m" || options->libav_video_codec == "libx264")
				format = "h264";
			else
				throw std::runtime_error("libav: please specify output format with the --libav-format argument");
		}
	}
	else
		format = options->libav_format.c_str();

	AVFormatContext *fmt_ctx;
	avformat_alloc_output_context2(&fmt_ctx, nullptr, format, dest.c_str());
	if (!fmt_ctx)
		throw std::runtime_error("libav: cannot allocate output context, try setting with --libav-format");

//	out_fmt_ctx_->debug = FF_FDEBUG_TS;
	if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
	{
		output_global_headers = true;
		fprintf(stdout, "%s:%s:%d %s requires global headers\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, dest.c_str());
	} else 
		fprintf(stdout, "%s:%s:%d %s does not requires global headers\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, dest.c_str());
	
	return fmt_ctx;
}

void RPi2CamEncoder::initVideoCodec(Options const *options, Streams const vctx)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	int cam = (vctx == Overlay) ? options_->overlay_main : (int)vctx;
	StreamInfo info = GetCameraStreamInfo(cam);
	
	const AVCodec *codec = avcodec_find_encoder_by_name(options->libav_video_codec.c_str());
	if (!codec)
		throw std::runtime_error("libav: cannot find video encoder " + options->libav_video_codec);
		
	codec_ctx_[vctx] = avcodec_alloc_context3(codec);
	
	if (!codec_ctx_[vctx])
		throw std::runtime_error("libav: Cannot allocate video context");
	codec_ctx_[vctx]->width = info.width;
	codec_ctx_[vctx]->height = info.height;
	// usec timebase
	codec_ctx_[vctx]->time_base = { 1, 1000 * 1000 };
	codec_ctx_[vctx]->sw_pix_fmt = AV_PIX_FMT_YUV420P;
	codec_ctx_[vctx]->pix_fmt = AV_PIX_FMT_YUV420P;

	if (info.colour_space)
	{
		using libcamera::ColorSpace; 

		static const std::map<ColorSpace::Primaries, AVColorPrimaries> pri_map = {
			{ ColorSpace::Primaries::Smpte170m, AVCOL_PRI_SMPTE170M },
			{ ColorSpace::Primaries::Rec709, AVCOL_PRI_BT709 },
			{ ColorSpace::Primaries::Rec2020, AVCOL_PRI_BT2020 },
		};

		static const std::map<ColorSpace::TransferFunction, AVColorTransferCharacteristic> tf_map = {
			{ ColorSpace::TransferFunction::Linear, AVCOL_TRC_LINEAR },
			{ ColorSpace::TransferFunction::Srgb, AVCOL_TRC_IEC61966_2_1 },
			{ ColorSpace::TransferFunction::Rec709, AVCOL_TRC_BT709 },
		};

		static const std::map<ColorSpace::YcbcrEncoding, AVColorSpace> cs_map = {
			{ ColorSpace::YcbcrEncoding::None, AVCOL_SPC_UNSPECIFIED },
			{ ColorSpace::YcbcrEncoding::Rec601, AVCOL_SPC_SMPTE170M },
			{ ColorSpace::YcbcrEncoding::Rec709, AVCOL_SPC_BT709 },
			{ ColorSpace::YcbcrEncoding::Rec2020, AVCOL_SPC_BT2020_CL },
		};

		auto it_p = pri_map.find(info.colour_space->primaries);
		if (it_p == pri_map.end())
			throw std::runtime_error("libav: no match for colour primaries in " + info.colour_space->toString());
		codec_ctx_[vctx]->color_primaries = it_p->second;

		auto it_tf = tf_map.find(info.colour_space->transferFunction);
		if (it_tf == tf_map.end())
			throw std::runtime_error("libav: no match for transfer function in " + info.colour_space->toString());
		codec_ctx_[vctx]->color_trc = it_tf->second;

		auto it_cs = cs_map.find(info.colour_space->ycbcrEncoding);
		if (it_cs == cs_map.end())
			throw std::runtime_error("libav: no match for ycbcr encoding in " + info.colour_space->toString());
		codec_ctx_[vctx]->colorspace = it_cs->second;

		codec_ctx_[vctx]->color_range =
			info.colour_space->range == ColorSpace::Range::Full ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
	}

	// Apply any codec specific options:
	auto fn = optionsMap.find(options->libav_video_codec);
	if (fn != optionsMap.end())
		fn->second(codec_ctx_[vctx]);

	// Apply general options.
	encoderOptionsGeneral(options, codec_ctx_[vctx], cam);

	if (output_global_headers) codec_ctx_[vctx]->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	int ret = avcodec_open2(codec_ctx_[vctx], codec, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open video codec: " + std::to_string(ret));
		
	if (vctx == Overlay) initOvrlyFilter(options_);
}

void RPi2CamEncoder::initAddStreams(Options const *options)
{
	AVFormatContext *fmt;
	for (const auto& pair : out_fmt_) 
	{ 
		fmt = pair.first; 
        for ( Streams st : pair.second) 
        { 
			AVStream *strm = avformat_new_stream(fmt, codec_ctx_[st]->codec);
			if (!strm)
				throw std::runtime_error("libav: cannot allocate stream for vidout output context");
			if (codec_ctx_[st]->codec_type == AVMEDIA_TYPE_VIDEO)
			{
	// The avi stream context seems to need the video stream time_base set to
	// 1/framerate to report the correct framerate in the container file.
	//
	// This seems to be a limitation/bug in ffmpeg:
	// https://github.com/FFmpeg/FFmpeg/blob/3141dbb7adf1e2bd5b9ff700312d7732c958b8df/libavformat/avienc.c#L527
				if (!strncmp(fmt->oformat->name, "avi", 3))
				{
					int cam = st > 1 ? options->overlay_main : (int)st;
					strm->time_base = { 1000, (int)(options->framerate_a[cam].value_or(DEF_FRAMERATE) * 1000) };
				}
				else
					strm->time_base = codec_ctx_[st]->time_base;
				strm->avg_frame_rate = strm->r_frame_rate = codec_ctx_[st]->framerate;
			} else if (codec_ctx_[st]->codec_type == AVMEDIA_TYPE_AUDIO)
			{
				strm->time_base = codec_ctx_[st]->time_base;
			} else throw std::runtime_error("libav: Media type is not Video or Audio");
			avcodec_parameters_from_context(strm->codecpar, codec_ctx_[st]);
        } 
    } 
}

void RPi2CamEncoder::initOvrlyFilter(Options const *options)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	ovrlyfg_ = avfilter_graph_alloc();
	AVFilterContext *ovrlfltr;
	AVFilterContext *scalefltr;

     
    int ret, pix_fmt = 0;
    char args[512];
    if (!ovrlyfg_) 
		throw std::runtime_error("libav: Could not allocate overlay"); 
		
	if (options->libav_video_codec == "h264_v4l2m2m") 
	{
		pix_fmt = AV_PIX_FMT_DRM_PRIME;
	}
	else if (options->libav_video_codec == "libx264")
	{
		pix_fmt = AV_PIX_FMT_YUV420P;
	}
	else
		throw std::runtime_error("unknow video codec set"); 
	
	
	StreamInfo info = GetCameraStreamInfo(options->overlay_main);
	snprintf(args, sizeof(args), "video_size=%dx%d:pix_fmt=%d:time_base=1/1000000:pixel_aspect=0/1", 
		info.width, info.height, pix_fmt);
    ret = avfilter_graph_create_filter(&mainbuf_, avfilter_get_by_name("buffer"), "mainbuf", args, NULL, ovrlyfg_);
	if (ret < 0) throw std::runtime_error("libav: untable to create main buffer");
	
	unsigned int ovrly_cam = options->overlay_main ? 0 : 1; 
	info = GetCameraStreamInfo(ovrly_cam);
	snprintf(args, sizeof(args), "video_size=%dx%d:pix_fmt=%d:time_base=1/1000000:pixel_aspect=0/1", 	
		info.width, info.height, pix_fmt);
    ret = avfilter_graph_create_filter(&ovrlbuf_, avfilter_get_by_name("buffer"), "ovrlbuf", args, NULL, ovrlyfg_);
	if (ret < 0) throw std::runtime_error("libav: untable to create overlay buffer"); 
	
	snprintf(args, sizeof(args), "x=%d:y=%d:repeatlast=1:eval=init", options->overlay_x, options->overlay_y);
	ret = avfilter_graph_create_filter(&ovrlfltr, avfilter_get_by_name("overlay"), "overlay", args, NULL, ovrlyfg_);
	if (ret < 0) throw std::runtime_error("libav: untable to create overlay filter"); 
	
	snprintf(args, sizeof(args), "%d:%d", options->overlay_width, options->overlay_height);
	ret = avfilter_graph_create_filter(&scalefltr, avfilter_get_by_name("scale"), "scale", args, NULL, ovrlyfg_);
	if (ret < 0) throw std::runtime_error("libav: untable to create scale filter"); 
	
	ret = avfilter_graph_create_filter(&sinkbuf_, avfilter_get_by_name("buffersink"), "sink", NULL, NULL, ovrlyfg_);
	if (ret < 0) throw std::runtime_error("libav: untable to create sink buffer"); 
	
	if (options->libav_video_codec == "h264_v4l2m2m") 
	{
		enum AVPixelFormat pf_enum[] = {AV_PIX_FMT_DRM_PRIME, AV_PIX_FMT_NONE};
		ret = av_opt_set_int_list(sinkbuf_, "pix_fmts", pf_enum, AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
	}
	else 
	{
		enum AVPixelFormat pf_enum[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE};
		ret = av_opt_set_int_list(sinkbuf_, "pix_fmts", pf_enum, AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
	}
	if (ret < 0) throw std::runtime_error("libav: untable to set format list"); 
	
	ret = avfilter_link(mainbuf_, 0, ovrlfltr, 0);
	if (ret < 0) throw std::runtime_error("libav: link main cam to overlay failed"); 
    ret = avfilter_link(ovrlbuf_, 0, scalefltr, 0);
    if (ret < 0) throw std::runtime_error("libav: link 2nd cam to scale filter failed"); 
    ret = avfilter_link(scalefltr, 0, ovrlfltr, 1);
    if (ret < 0) throw std::runtime_error("libav: link scale filter to overlay filter failed"); 
    ret = avfilter_link(ovrlfltr, 0, sinkbuf_, 0);
    if (ret < 0) throw std::runtime_error("libav: link overlay filter to sink buffer failed");
   
    ret = avfilter_graph_config(ovrlyfg_, NULL);
    if (ret < 0) throw std::runtime_error("libav: configure filter graph");
}  

void RPi2CamEncoder::initAudioCodec(Options const *options)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
#if LIBAVUTIL_VERSION_MAJOR < 58
	AVInputFormat *input_fmt = (AVInputFormat *)av_find_input_format(options->audio_source.c_str());
#else
	const AVInputFormat *input_fmt = (AVInputFormat *)av_find_input_format(options->audio_source.c_str());
#endif

	assert(in_fmt_ctx_ == nullptr);

	int ret;
	AVDictionary *format_opts = nullptr;

	if (options->audio_channels != 0)
		ret = av_dict_set_int(&format_opts, "channels", options->audio_channels, 0);

	ret = avformat_open_input(&in_fmt_ctx_, options->audio_device.c_str(), input_fmt, &format_opts);
	if (ret < 0)
	{
		av_dict_free(&format_opts);
		throw std::runtime_error("libav: cannot open " + options->audio_source + " input device " + options->audio_device);
	}

	av_dict_free(&format_opts);

	avformat_find_stream_info(in_fmt_ctx_, nullptr);

	AVStream *strmAudioIn = nullptr;
	for (unsigned int i = 0; i < in_fmt_ctx_->nb_streams; i++)
	{
		if (in_fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
		{
			strmAudioIn = in_fmt_ctx_->streams[i];
			break;
		}
	}

	if (!strmAudioIn)
		throw std::runtime_error("libav: couldn't find a audio stream.");

	const AVCodec *codec_in = avcodec_find_decoder(strmAudioIn->codecpar->codec_id);
	codec_ctx_[AudioIn] = avcodec_alloc_context3(codec_in);
	avcodec_parameters_to_context(codec_ctx_[AudioIn], strmAudioIn->codecpar);
	// usec timebase
	codec_ctx_[AudioIn]->time_base = { 1, 1000 * 1000 };
	ret = avcodec_open2(codec_ctx_[AudioIn], codec_in, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open audio in codec: " + std::to_string(ret));

	const AVCodec *codec_out = avcodec_find_encoder_by_name(options->audio_codec.c_str());
	if (!codec_out)
		throw std::runtime_error("libav: cannot find audio encoder " + options->audio_codec);

	codec_ctx_[AudioOut] = avcodec_alloc_context3(codec_out);
	if (!codec_ctx_[AudioOut])
		throw std::runtime_error("libav: cannot allocate audio in context");

#if LIBAVUTIL_VERSION_MAJOR < 57
	codec_ctx_[AudioOut]->channels = stream_[AudioIn]->codecpar->channels;
	codec_ctx_[AudioOut]->channel_layout = av_get_default_channel_layout(stream_[AudioIn]->codecpar->channels);
#else
	av_channel_layout_default(&codec_ctx_[AudioOut]->ch_layout, strmAudioIn->codecpar->ch_layout.nb_channels);
#endif

	codec_ctx_[AudioOut]->sample_rate = options->audio_samplerate ? options->audio_samplerate
																  : strmAudioIn->codecpar->sample_rate;
	codec_ctx_[AudioOut]->sample_fmt = codec_out->sample_fmts[0];
	codec_ctx_[AudioOut]->bit_rate = options->audio_bitrate.bps();
	// usec timebase
	codec_ctx_[AudioOut]->time_base = { 1, 1000 * 1000 };

	if (output_global_headers) codec_ctx_[AudioOut]->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	ret = avcodec_open2(codec_ctx_[AudioOut], codec_out, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: unable to open audio codec: " + std::to_string(ret));
}

void RPi2CamEncoder::StartEncoder()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);

/*   StreamInfo info;
// check camera selected and overlay and make sure all streams to be encoded have info	WEK update for stream_selected
	if (options_->multi_cam)
	{
		info = GetCameraStreamInfo(Camera0);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("camera 0 steam is not configured");
		info = GetCameraStreamInfo(Camera1);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("camera 1 steam is not configured");
	}
	else
	{
		info = GetCameraStreamInfo(options_->camera);
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error("selected camera stream is not configured");
	} */
	std::map< int, std::string> type_table =
		{ { 0, "Camera O" },
			{ 1, "Camera 1"},
			{ 2, "Overlay" } };
	for (int i = 0; i < 3; i++)
	{
		StreamInfo info = GetCameraStreamInfo((i==2 ? options_->overlay_main : i));
		if (!info.width || !info.height || !info.stride)
			throw std::runtime_error( type_table[i] +" steam is not configured");
	}

	avdevice_register_all();

	if (options_->verbose >= 2)   
		av_log_set_level(AV_LOG_VERBOSE);
//	av_log_set_level(AV_LOG_ERROR);

	for (const auto& fmt : options_->format_table) 
	{ 
		out_fmt_.insert(std::pair<AVFormatContext *, std::vector<Streams>>
			(initFormat(options_, fmt.first), fmt.second));
	}
	for (int i=0;i<3;i++)
	{
		if (options_->stream_selected[i]) 
			initVideoCodec(options_, (Streams)i);
	}
       
	if (options_->stream_selected[AudioOut])
	{
		initAudioCodec(options_);
		av_dump_format(in_fmt_ctx_, 0, options_->audio_device.c_str(), 0);
	}
	
	initAddStreams(options_);

	for (const auto& fmt : out_fmt_) 
	{ 
        av_dump_format(fmt.first, 0, fmt.first->url, 1);
    } 

	LOG(2, "libav: codec init completed");

	video_thread_ = std::thread(&RPi2CamEncoder::videoThread, this);

	if (options_->stream_selected[AudioOut])
		audio_thread_ = std::thread(&RPi2CamEncoder::audioThread, this);  
} 

void RPi2CamEncoder::StopEncoder() 
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	if (options_->stream_selected[AudioOut]) 
	{
		abort_audio_ = true;
		audio_thread_.join();
	}

	abort_video_ = true;
	video_thread_.join();

	for (const auto& fmt : out_fmt_) 
	{ 
        avformat_free_context(fmt.first);
    }
     
	// free all video contexts
	avcodec_free_context(&codec_ctx_[Camera0]);
	avcodec_free_context(&codec_ctx_[Camera1]);
	avcodec_free_context(&codec_ctx_[Overlay]);

	if (options_->stream_selected[AudioOut]) 
	{
		avformat_free_context(in_fmt_ctx_);
		avcodec_free_context(&codec_ctx_[AudioIn]);
		avcodec_free_context(&codec_ctx_[AudioOut]);  
	}  
 
	LOG(2, "libav: codec closed");
} 

void RPi2CamEncoder::EncodeBuffer(CompletedRequestPtr &completed_request)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	Stream *stream = GetCameraStream(completed_request->request->cookie());
    StreamInfo info =  GetCameraStreamInfo(completed_request->request->cookie());
	FrameBuffer *buffer = completed_request->buffers[stream];
	BufferReadSync r(this, buffer);
	libcamera::Span span = r.Get()[0];
	void *mem = span.data();
	if (!buffer || !mem)
		throw std::runtime_error("no buffer to encode");
	auto ts = completed_request->metadata.get(controls::SensorTimestamp);
	int64_t timestamp_ns = ts ? *ts : buffer->metadata().timestamp;
	{
		std::lock_guard<std::mutex> lock(encode_buffer_queue_mutex_);
		encode_buffer_queue_.push(completed_request); // creates a new reference
	}
	AVFrame *frame = av_frame_alloc();
	if (!frame)
		throw std::runtime_error("libav: could not allocate AVFrame");

	if (!video_start_ts_)
		video_start_ts_ = timestamp_ns / 1000;

	if (info.camera == 0) 
		frame->opaque = (void *) &cam0;
	else
		frame->opaque = (void *) &cam1;

// make this consistant with other code WEK and fix segment fault if running camera stream unused 
	if (options_->stream_selected[Overlay]) 
		frame->format = codec_ctx_[Overlay]->pix_fmt;

	else

		frame->format = codec_ctx_[info.camera]->pix_fmt;

	frame->width = info.width;
	frame->height = info.height;
	frame->linesize[0] = info.stride;
	frame->linesize[1] = frame->linesize[2] = info.stride >> 1;
	frame->pts = (timestamp_ns / 1000) - video_start_ts_;

	if (frame->format == AV_PIX_FMT_DRM_PRIME)
	{
		std::scoped_lock<std::mutex> lock(drm_queue_lock_);
		drm_frame_queue_.emplace(std::make_unique<AVDRMFrameDescriptor>());
		frame->buf[0] = av_buffer_create((uint8_t *)drm_frame_queue_.back().get(), sizeof(AVDRMFrameDescriptor),
										 &RPi2CamEncoder::releaseBuffer, this, 0);
		frame->data[0] = frame->buf[0]->data;

		AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor *)frame->data[0];
		desc->nb_objects = 1;
		desc->objects[0].fd = buffer->planes()[0].fd.get();
		desc->objects[0].size = span.size();
		desc->objects[0].format_modifier = DRM_FORMAT_MOD_INVALID;

		desc->nb_layers = 1;
		desc->layers[0].format = DRM_FORMAT_YUV420;
		desc->layers[0].nb_planes = 3;
		desc->layers[0].planes[0].object_index = 0;
		desc->layers[0].planes[0].offset = 0;
		desc->layers[0].planes[0].pitch = info.stride;
		desc->layers[0].planes[1].object_index = 0;
		desc->layers[0].planes[1].offset = info.stride * info.height;
		desc->layers[0].planes[1].pitch = info.stride >> 1;
		desc->layers[0].planes[2].object_index = 0;
		desc->layers[0].planes[2].offset = info.stride * info.height * 5 / 4;
		desc->layers[0].planes[2].pitch = info.stride >> 1;
	}
	else
	{
		frame->buf[0] = av_buffer_create((uint8_t *)mem, span.size(), &RPi2CamEncoder::releaseBuffer, this, 0);
		av_image_fill_pointers(frame->data, AV_PIX_FMT_YUV420P, frame->height, frame->buf[0]->data, frame->linesize);
		av_frame_make_writable(frame);
	}

	std::scoped_lock<std::mutex> lock(video_mutex_);
	frame_queue_.push(frame);
	video_cv_.notify_all();
}

void RPi2CamEncoder::initOutput()
{
//	fprintf(stdout, "%s:%s:%d >%s<\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, options_->output.c_str());
	int ret;
	
	if (first_video) 
	{
		for (const auto& fmt : out_fmt_) 
		{ 
			int st_ix = 0;
			for (Streams st : fmt.second) 
			{ 
				if (st < AudioOut)
				{
					avcodec_parameters_from_context(fmt.first->streams[st_ix]->codecpar, codec_ctx_[first_stream]);
					st_ix++;
				}
			} 
		}
		first_video = false;
	} 

	char err[64];
	for (const auto& fmt : out_fmt_) 
	{ 
		std::string url = fmt.first->url;
		ret = avio_open2(&fmt.first->pb, fmt.first->url, AVIO_FLAG_WRITE, nullptr, nullptr);
		if (ret < 0)
		{
			av_strerror(ret, err, sizeof(err));
			throw std::runtime_error("libav: unable to open output mux for " + url + ": " + err);
		}

		ret = avformat_write_header(fmt.first, nullptr);
		if (ret < 0)
		{
			av_strerror(ret, err, sizeof(err));
			throw std::runtime_error("libav: unable write output mux header for " + url + ": " + err);
		}
	}
}

void RPi2CamEncoder::deinitOutput()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	for (const auto& fmt : out_fmt_) 
	{
		av_write_trailer(fmt.first);
		avio_closep(&fmt.first->pb);  
	}
}

void RPi2CamEncoder::encode(AVPacket *pkt, unsigned int stream_id)
{
//	fprintf(stdout, "%s:%s:%d stream %d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, stream_id);
	int ret = 0;

	while (ret >= 0)
	{
		ret = avcodec_receive_packet(codec_ctx_[stream_id], pkt);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
		{
			av_packet_unref(pkt);
			break;
		}
		else if (ret < 0)
			throw std::runtime_error("libav: error receiving packet: " + std::to_string(ret));
		// Initialise the ouput mux on the first received video packet, as we may need
		// to copy global header data from the encoder.
		if (first_video && stream_id < AudioOut)
		{
			if (first_stream == NoStream) first_stream = stream_id;
		}
		if (stream_id < AudioOut && !output_ready_)
		{                                                                                             
			initOutput();
			output_ready_ = true;
		}

		for (const auto& fmt : out_fmt_) 
		{ 
			AVFormatContext *fmt_ptr = fmt.first;  
			int s_ix=0;
			for (unsigned int st : fmt.second) 
			{ 
				if (st == stream_id)
				{
					pkt->pos = -1;
					pkt->duration = 0;
					AVPacket *cp_pkt = av_packet_clone(pkt);
					if (!cp_pkt) throw std::runtime_error("unable to clone packet");
					cp_pkt->stream_index = s_ix;
					av_packet_rescale_ts(cp_pkt, codec_ctx_[stream_id]->time_base, 
						fmt_ptr->streams[s_ix]->time_base);  
						
					std::scoped_lock<std::mutex> lock(output_mutex_);
					// cp_pkt is now blank (av_interleaved_write_frame() takes ownership of
					// its contents and resets pkt), so that no unreferencing is necessary.
					// This would be different if one used av_write_frame().
					ret = av_interleaved_write_frame(fmt_ptr, cp_pkt);
					if (ret < 0)
					{
						char err[AV_ERROR_MAX_STRING_SIZE];
						av_strerror(ret, err, sizeof(err));
						throw std::runtime_error("libav: error writing output: " + std::string(err));
					}  
					av_packet_free(&cp_pkt);  // WEK needed or not?
				} 
				s_ix++;
			} 		
		} 
	}
}

extern "C" void RPi2CamEncoder::releaseBuffer(void *opaque, uint8_t *data)
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	RPi2CamEncoder *enc = static_cast<RPi2CamEncoder *>(opaque);
	{
		std::lock_guard<std::mutex> lock(enc->encode_buffer_queue_mutex_);
		if (enc->encode_buffer_queue_.empty())
			throw std::runtime_error("no buffer available to return");
		enc->encode_buffer_queue_.pop(); // drop shared_ptr reference
	}
	// Pop the entry from the queue to release the AVDRMFrameDescriptor allocation
	std::scoped_lock<std::mutex> lock(enc->drm_queue_lock_);
	if (!enc->drm_frame_queue_.empty())
		enc->drm_frame_queue_.pop();
}

void RPi2CamEncoder::videoThread()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	AVPacket *pkt = av_packet_alloc();
	AVFrame *frame = nullptr;
	int ret;

	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(video_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				// Must check the abort first, to allow items in the output
				// queue to have a callback.
				if (abort_video_ && frame_queue_.empty())
					goto done;

				if (!frame_queue_.empty())
				{
					frame = frame_queue_.front();
					frame_queue_.pop();
					break;
				}
				else
					video_cv_.wait_for(lock, 200ms);
			}
		}
		unsigned int camera = *(static_cast<int *>(frame->opaque)); 
		if (options_->stream_selected[Overlay])
		{
			AVFrame *fframe = av_frame_alloc();
			if (camera == options_->overlay_main)
			{
				ret = av_buffersrc_add_frame_flags(mainbuf_, frame, AV_BUFFERSRC_FLAG_KEEP_REF);
				if (ret < 0) throw std::runtime_error("libav: error sending main frame: " + std::to_string(ret));
			} 
			else
			{
				ret = av_buffersrc_add_frame_flags(ovrlbuf_, frame, AV_BUFFERSRC_FLAG_KEEP_REF);
				if (ret < 0) throw std::runtime_error("libav: error sending overlay frame: " + std::to_string(ret));
			}
			do 
			{	
				ret = av_buffersink_get_frame(sinkbuf_, fframe);
				if (!(ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)) 
					throw std::runtime_error("libav: error getting flitered frame: " + std::to_string(ret));
				if (ret >= 0)
				{
					if (avcodec_send_frame(codec_ctx_[Overlay], fframe) < 0)  
						throw std::runtime_error("libav: error sending flitered frame to codec");
					encode(pkt, Overlay);
					av_frame_unref(fframe);
				}
			} while (!(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF));
			av_frame_free(&fframe);
		} 

		if (options_->stream_selected[camera])
		{
			ret = avcodec_send_frame(codec_ctx_[camera], frame);  
			if (ret < 0)
				throw std::runtime_error("libav: error sending camera frame: " + std::to_string(ret)); 	
			encode(pkt, camera);
		}
		av_frame_free(&frame);
	}

done:
// flush any valid encoder
	if (codec_ctx_[Camera0] != nullptr) {
		avcodec_send_frame(codec_ctx_[Camera0], nullptr);
		encode(pkt, Camera0);}
	if (codec_ctx_[Camera1] != nullptr) {
		avcodec_send_frame(codec_ctx_[Camera1], nullptr);
		encode(pkt, Camera1);}
		
	if (codec_ctx_[Overlay] != nullptr) {
		avcodec_send_frame(codec_ctx_[Overlay], nullptr);
		encode(pkt, Overlay);}
		
	if (ovrlyfg_ != nullptr)
		avfilter_graph_free(&ovrlyfg_);
			
	av_packet_free(&pkt);
	deinitOutput();
}

void RPi2CamEncoder::audioThread()
{
//	fprintf(stdout, "%s:%s:%d \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
	const AVSampleFormat required_fmt = codec_ctx_[AudioOut]->sample_fmt;
	int ret;

#if LIBAVUTIL_VERSION_MAJOR < 57
	uint32_t out_channels = codec_ctx_[AudioOut]->channels;
#else
	uint32_t out_channels = codec_ctx_[AudioOut]->ch_layout.nb_channels;
#endif

	SwrContext *conv;
	AVAudioFifo *fifo;

#if LIBAVUTIL_VERSION_MAJOR < 57
	conv = swr_alloc_set_opts(nullptr, av_get_default_channel_layout(codec_ctx_[AudioOut]->channels), required_fmt,
							  stream_[AudioOut]->codecpar->sample_rate,
							  av_get_default_channel_layout(codec_ctx_[AudioIn]->channels),
							  codec_ctx_[AudioIn]->sample_fmt, codec_ctx_[AudioIn]->sample_rate, 0, nullptr);

	// 2 seconds FIFO buffer
	fifo = av_audio_fifo_alloc(required_fmt, codec_ctx_[AudioOut]->channels, codec_ctx_[AudioOut]->sample_rate * 2);
#else
	ret = swr_alloc_set_opts2(&conv, &codec_ctx_[AudioOut]->ch_layout, required_fmt,
//							  stream_[AudioOut]->codecpar->sample_rate, &codec_ctx_[AudioIn]->ch_layout,
							  codec_ctx_[AudioOut]->sample_rate, &codec_ctx_[AudioIn]->ch_layout,
							  codec_ctx_[AudioIn]->sample_fmt, codec_ctx_[AudioIn]->sample_rate, 0, nullptr);
	if (ret < 0)
		throw std::runtime_error("libav: cannot create swr context");

	// 2 seconds FIFO buffer
	fifo = av_audio_fifo_alloc(required_fmt, codec_ctx_[AudioOut]->ch_layout.nb_channels,
							   codec_ctx_[AudioOut]->sample_rate * 2);
#endif

	swr_init(conv);

	AVPacket *in_pkt = av_packet_alloc();
	AVPacket *out_pkt = av_packet_alloc();
	AVFrame *in_frame = av_frame_alloc();
	uint8_t **samples = nullptr;
	int sample_linesize = 0;

	int max_output_samples = av_rescale_rnd(codec_ctx_[AudioOut]->frame_size, codec_ctx_[AudioOut]->sample_rate,
											codec_ctx_[AudioIn]->sample_rate, AV_ROUND_UP);
	ret = av_samples_alloc_array_and_samples(&samples, &sample_linesize, out_channels, max_output_samples, required_fmt,
											 0);

	if (ret < 0)
		throw std::runtime_error("libav: failed to alloc sample array");

	while (!abort_audio_)
	{
		// Audio In
		ret = av_read_frame(in_fmt_ctx_, in_pkt);
		if (ret < 0)
			throw std::runtime_error("libav: cannot read audio in frame");

		ret = avcodec_send_packet(codec_ctx_[AudioIn], in_pkt);
		if (ret < 0)
			throw std::runtime_error("libav: cannot send pkt for decoding audio in");

		ret = avcodec_receive_frame(codec_ctx_[AudioIn], in_frame);
		if (ret && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
			throw std::runtime_error("libav: error getting decoded audio in frame");

		// Audio Resample/Conversion
		int num_output_samples =
			av_rescale_rnd(swr_get_delay(conv, codec_ctx_[AudioIn]->sample_rate) + in_frame->nb_samples,
						   codec_ctx_[AudioOut]->sample_rate, codec_ctx_[AudioIn]->sample_rate, AV_ROUND_UP);
		
		if (num_output_samples > max_output_samples)
		{
			av_freep(&samples[0]);
			max_output_samples = num_output_samples;
			ret = av_samples_alloc_array_and_samples(&samples, &sample_linesize, out_channels, max_output_samples,
													 required_fmt, 0);
			if (ret < 0)
				throw std::runtime_error("libav: failed to alloc sample array");
		}

		ret = swr_convert(conv, samples, num_output_samples, (const uint8_t **)in_frame->extended_data,
						  in_frame->nb_samples);
		if (ret < 0)
			throw std::runtime_error("libav: swr_convert failed");

		if (!cameras_running_ && !abort_audio_) av_audio_fifo_reset(fifo);

		if (av_audio_fifo_space(fifo) < num_output_samples)
		{
			LOG(1, "libav: Draining audio fifo, configure a larger size");
			av_audio_fifo_drain(fifo, num_output_samples);
		}

		av_audio_fifo_write(fifo, (void **)samples, num_output_samples);

		av_frame_unref(in_frame);
		av_packet_unref(in_pkt);

		// Not yet ready to generate encoded audio!
		if (!output_ready_)
			continue;

		// Audio Out
		while (av_audio_fifo_size(fifo) >= codec_ctx_[AudioOut]->frame_size)
		{
			AVFrame *out_frame = av_frame_alloc();
			out_frame->nb_samples = codec_ctx_[AudioOut]->frame_size;

#if LIBAVUTIL_VERSION_MAJOR < 57
			out_frame->channels = codec_ctx_[AudioOut]->channels;
			out_frame->channel_layout = av_get_default_channel_layout(codec_ctx_[AudioOut]->channels);
#else
			av_channel_layout_copy(&out_frame->ch_layout, &codec_ctx_[AudioOut]->ch_layout);
#endif

			out_frame->format = required_fmt;
			out_frame->sample_rate = codec_ctx_[AudioOut]->sample_rate;

			av_frame_get_buffer(out_frame, 0);
			av_audio_fifo_read(fifo, (void **)out_frame->data, codec_ctx_[AudioOut]->frame_size);

			AVRational num = { 1, out_frame->sample_rate };
			int64_t ts = av_rescale_q(audio_samples_, num, codec_ctx_[AudioOut]->time_base);

			out_frame->pts = ts;

			audio_samples_ += codec_ctx_[AudioOut]->frame_size;
		
			ret = avcodec_send_frame(codec_ctx_[AudioOut], out_frame);
			if (ret < 0)
				throw std::runtime_error("libav: error encoding frame: " + std::to_string(ret));

			encode(out_pkt, AudioOut);
			av_frame_free(&out_frame);
		}
	}

	// Flush the encoder
	avcodec_send_frame(codec_ctx_[AudioOut], nullptr);
	encode(out_pkt, AudioOut);

	swr_free(&conv);
	av_freep(&samples[0]);
	av_audio_fifo_free(fifo);

	av_packet_free(&in_pkt);
	av_packet_free(&out_pkt);
	av_frame_free(&in_frame);
}
