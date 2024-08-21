/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpi2cam_encoder.cpp - libcamera video encoding class.
 * WEK
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "core/rpi2cam_app.hpp"
#include "core/stream_info.hpp"
#include "core/options.hpp"
#include "core/stream_enum.hpp"

#include "core/rpi2cam_encoder.hpp"

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavcodec/codec_desc.h"
#include "libavdevice/avdevice.h"
#include "libavformat/avformat.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavutil/imgutils.h"
#include "libavutil/timestamp.h"
#include "libavutil/version.h"
#include "libswresample/swresample.h"
#include "libavfilter/buffersrc.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/avfilter.h"
}

typedef std::function<void(void *, size_t, int64_t, bool)> EncodeOutputReadyCallback;
typedef std::function<void(void *)> InputDoneCallback;

class RPi2CamEncoder : public RPi2CamApp

{
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;
	
	RPi2CamEncoder() : RPi2CamApp(std::make_unique<Options>()), options_(GetOptions()), output_ready_(false), abort_video_(false),
	abort_audio_(false), video_start_ts_(0), audio_samples_(0), in_fmt_ctx_(nullptr) {};

	void StartEncoder();
	void EncodeBuffer(CompletedRequestPtr &completed_request);
	void StopEncoder(); 
	
protected:

private:
	AVFormatContext *initFormat(Options const *options, std::string dest);
	void initVideoCodec(Options const *options, Streams const vctx);
	void initAddStreams(Options const *options);
	void initOvrlyFilter(Options const *options);
	void initAudioCodec(Options const *options);

	void initOutput();
	void deinitOutput();
	void encode(AVPacket *pkt, unsigned int stream_id);

	void videoThread();
	void audioThread();

	static void releaseBuffer(void *opaque, uint8_t *data);

	std::queue<CompletedRequestPtr> encode_buffer_queue_;
	std::mutex encode_buffer_queue_mutex_;
	EncodeOutputReadyCallback encode_output_ready_callback_;
	InputDoneCallback input_done_callback_;
	
	Options const *options_;
	
	std::atomic<bool> output_ready_;
	bool abort_video_;
	bool abort_audio_;
	uint64_t video_start_ts_;
	uint64_t audio_samples_;

	std::queue<AVFrame *> frame_queue_;
	std::mutex video_mutex_;
	std::mutex output_mutex_;
	std::condition_variable video_cv_;
	std::thread video_thread_;
	std::thread audio_thread_;
	
	AVCodecContext *codec_ctx_[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

	AVFormatContext *in_fmt_ctx_;
	bool output_global_headers = false;
	bool first_video = true;
	unsigned int first_stream = NoStream;
	std::map<AVFormatContext *, std::vector<Streams>> out_fmt_;
	
	AVFilterGraph *ovrlyfg_ = nullptr; 
	AVFilterContext *mainbuf_;   
	AVFilterContext *ovrlbuf_;   
	AVFilterContext *sinkbuf_;    

	std::mutex drm_queue_lock_;
	std::queue<std::unique_ptr<AVDRMFrameDescriptor>> drm_frame_queue_;
	const int cam0 = 0;
	const int cam1 = 1;

};
