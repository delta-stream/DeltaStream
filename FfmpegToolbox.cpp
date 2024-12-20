

#include "FfmpegToolbox.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
// #pragma GCC optimize("O0")

using namespace std;

struct MotionVectorData {
    int frame_number;
    int source;
    int w, h;
    int src_x, src_y;
    int dst_x, dst_y;
    uint64_t flags;
    int motion_x, motion_y, motion_scale;
};


void FfmpegToolbox::save_video_buffer_to_file(const std::string &filename) {
    std::ofstream outfile(filename, std::ios::out | std::ios::binary);
    if (!outfile) {
        std::cerr << "Failed to open file for writing: " << filename <<
                std::endl;
        return;
    }

    outfile.write(reinterpret_cast<const char *>(video_buffer.data()),
                  video_buffer.size()
    );
    if (!outfile) {
        std::cerr << "Failed to write video buffer to file: " << filename <<
                std::endl;
    }

    outfile.close();
}

void FfmpegToolbox::create_video_from_images(
    const cv::Mat &color_frame,
    const cv::Mat &prev_color_frame,
    int fps, int width, int height
) {

    video_buffer.clear();

    AVFormatContext *fmt_ctx = nullptr;
    AVStream *stream = nullptr;
    AVCodecContext *codec_ctx = nullptr;
    const AVCodec *codec = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *pkt = av_packet_alloc();
    if (!pkt) {
        std::cerr << "Failed to allocate AVPacket\n";
        return;
    }

    avformat_alloc_output_context2(&fmt_ctx, nullptr, "mp4", nullptr);
    codec = avcodec_find_encoder(AV_CODEC_ID_MPEG4);
    if (!codec) {
        std::cerr << "Codec not found\n";
        return;
    }

    stream = avformat_new_stream(fmt_ctx, nullptr);
    if (!stream) {
        std::cerr << "Could not allocate stream\n";
        return;
    }

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "Could not allocate codec context\n";
        return;
    }

    codec_ctx->codec_id = codec->id;
    codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_ctx->width = width;
    codec_ctx->height = height;
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx->time_base = {1, fps};
    codec_ctx->framerate = {fps, 1};
    codec_ctx->gop_size = 10;

    if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "Could not open codec\n";
        return;
    }

    if (avcodec_parameters_from_context(stream->codecpar, codec_ctx) < 0) {
        std::cerr << "Could not copy codec parameters\n";
        return;
    }

    const auto buffer = static_cast<uint8_t *>(av_malloc(8192));
    AVIOContext *avio_ctx = avio_alloc_context(buffer, 8192, 1, nullptr,
                                               nullptr,
                                               [](void *opaque,
                                                  const uint8_t *buf,
                                                  int buf_size) {
                                                   video_buffer.insert(
                                                       video_buffer.end(), buf,
                                                       buf + buf_size
                                                   );
                                                   return buf_size;
                                               }, nullptr
    );
    fmt_ctx->pb = avio_ctx;

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "movflags", "empty_moov+default_base_moof+frag_keyframe",
                0
    );

    int ret = avformat_write_header(fmt_ctx, &opts);
    av_dict_free(&opts); // 옵션 딕셔너리 해제

    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Error occurred when writing header: " << errbuf <<
                std::endl;
        return;
    }


    SwsContext *sws_ctx = sws_getContext(width, height, AV_PIX_FMT_BGR24,
                                         width, height, AV_PIX_FMT_YUV420P,
                                         SWS_BICUBIC, nullptr, nullptr, nullptr
    );

    frame = av_frame_alloc();
    frame->format = AV_PIX_FMT_YUV420P;
    frame->width = width;
    frame->height = height;
    av_image_alloc(frame->data, frame->linesize, width, height,
                   AV_PIX_FMT_YUV420P, 32
    );

    for (const auto &img: {prev_color_frame, color_frame}) {
        const uint8_t *src_slices[1] = {img.data};
        int src_stride[1] = {static_cast<int>(img.step)};
        sws_scale(sws_ctx, src_slices, src_stride, 0, height, frame->data,
                  frame->linesize
        );

        ret = avcodec_send_frame(codec_ctx, frame);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "Error sending frame to codec: " << errbuf <<
                    std::endl;
            break;
        }

        while (ret >= 0) {
            ret = avcodec_receive_packet(codec_ctx, pkt);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            else if (ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, sizeof(errbuf));
                std::cerr << "Error encoding frame: " << errbuf << std::endl;
                return;
            }

            pkt->stream_index = stream->index;
            av_packet_rescale_ts(pkt, codec_ctx->time_base, stream->time_base);

            if (av_interleaved_write_frame(fmt_ctx, pkt) < 0) {
                std::cerr << "Error writing frame\n";
                return;
            }
            av_packet_unref(pkt);
        }
    }

    av_write_trailer(fmt_ctx);
    avcodec_free_context(&codec_ctx);
    av_frame_free(&frame);
    sws_freeContext(sws_ctx);
    avio_context_free(&avio_ctx);
    avformat_free_context(fmt_ctx);
    av_packet_free(&pkt);
}


int FfmpegToolbox::read_packet(void *opaque, uint8_t *buf, int buf_size) {
    const auto bd = static_cast<BufferData *>(opaque);
    const int remaining = bd->size - bd->offset;
    const int copy_size = FFMIN(buf_size, remaining);
    if (copy_size <= 0)
        return AVERROR_EOF;
    memcpy(buf, bd->data + bd->offset, copy_size);
    bd->offset += copy_size;
    return copy_size;
}

vector<AVMotionVector> FfmpegToolbox::extract_motion_vectors() {

    int ret = 0;
    AVFormatContext *fmt_ctx = nullptr;
    AVIOContext *avio_ctx = nullptr;
    AVCodecContext *codec_ctx = nullptr;
    const AVCodec *codec = nullptr;
    AVStream *video_stream = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *pkt = nullptr;
    int video_stream_idx = -1;
    int video_frame_count = 0;
    AVDictionary *opts = nullptr;

    std::vector<AVMotionVector> motion_vectors;
    std::vector<MotionVectorData> motion_vectors2;

    BufferData bd = {
        video_buffer.data(), static_cast<int>(video_buffer.size()), 0
    };

    uint8_t *avio_ctx_buffer = nullptr;
    size_t avio_ctx_buffer_size = 4096;

    fmt_ctx = avformat_alloc_context();
    if (!fmt_ctx) {
        std::cerr << "Could not allocate AVFormatContext" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    avio_ctx_buffer = (uint8_t *) av_malloc(avio_ctx_buffer_size);
    if (!avio_ctx_buffer) {
        std::cerr << "Could not allocate AVIOContext buffer" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    avio_ctx = avio_alloc_context(avio_ctx_buffer, avio_ctx_buffer_size, 0, &bd,
                                  &FfmpegToolbox::read_packet, nullptr, nullptr
    );
    if (!avio_ctx) {
        std::cerr << "Could not allocate AVIOContext" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    fmt_ctx->pb = avio_ctx;

    fmt_ctx->flags |= AVFMT_FLAG_CUSTOM_IO;
    fmt_ctx->probesize = 32 * 1024;

    ret = avformat_open_input(&fmt_ctx, nullptr, nullptr, nullptr);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "Could not open input: " << errbuf << std::endl;
        goto end;
    }

    ret = avformat_find_stream_info(fmt_ctx, nullptr);
    if (ret < 0) {
        std::cerr << "Could not find stream information" << std::endl;
        goto end;
    }

    ret = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    if (ret < 0) {
        std::cerr << "Could not find video stream in input" << std::endl;
        goto end;
    }
    video_stream_idx = ret;
    video_stream = fmt_ctx->streams[video_stream_idx];

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "Could not allocate codec context" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    ret = avcodec_parameters_to_context(codec_ctx, video_stream->codecpar);
    if (ret < 0) {
        std::cerr << "Failed to copy codec parameters to codec context" <<
                std::endl;
        goto end;
    }

    av_dict_set(&opts, "flags2", "+export_mvs", 0);
    // Force macroblock decisions to use rate-distortion optimization
    av_dict_set(&opts, "mbd", "rd", 0);

    // Disable quarter-pixel motion estimation
    av_dict_set(&opts, "flags", "-qpel", 0);

    ret = avcodec_open2(codec_ctx, codec, &opts);
    av_dict_free(&opts); // opts 해제
    if (ret < 0) {
        std::cerr << "Failed to open codec" << std::endl;
        goto end;
    }


    frame = av_frame_alloc();
    if (!frame) {
        std::cerr << "Could not allocate frame" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    pkt = av_packet_alloc();
    if (!pkt) {
        std::cerr << "Could not allocate packet" << std::endl;
        ret = AVERROR(ENOMEM);
        goto end;
    }

    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == video_stream_idx) {
            ret = avcodec_send_packet(codec_ctx, pkt);
            if (ret < 0) {
                std::cerr << "Error sending packet to decoder" << std::endl;
                break;
            }

            while (ret >= 0) {
                ret = avcodec_receive_frame(codec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                else if (ret < 0) {
                    std::cerr << "Error receiving frame from decoder" <<
                            std::endl;
                    goto end;
                }

                video_frame_count++;

                // 모션 벡터 추출
                AVFrameSideData *sd = av_frame_get_side_data(
                    frame, AV_FRAME_DATA_MOTION_VECTORS
                );
                if (sd) {
                    auto mvs = reinterpret_cast<const AVMotionVector *>(sd->
                        data);
                    int mv_count = sd->size / sizeof(AVMotionVector);
                    for (int i = 0; i < mv_count; i++) {
                        const AVMotionVector *mv = &mvs[i];

                        MotionVectorData mv_data;
                        mv_data.frame_number = video_frame_count;
                        mv_data.source = mv->source;
                        mv_data.w = mv->w;
                        mv_data.h = mv->h;
                        mv_data.src_x = mv->src_x;
                        mv_data.src_y = mv->src_y;
                        mv_data.dst_x = mv->dst_x;
                        mv_data.dst_y = mv->dst_y;
                        mv_data.flags = mv->flags;
                        mv_data.motion_x = mv->motion_x;
                        mv_data.motion_y = mv->motion_y;
                        mv_data.motion_scale = mv->motion_scale;
                        motion_vectors2.push_back(mv_data);

                        motion_vectors.push_back(*mv);
                    }
                }

                av_frame_unref(frame);
            }
        }
        av_packet_unref(pkt);
    }

    avcodec_send_packet(codec_ctx, nullptr);
    while (avcodec_receive_frame(codec_ctx, frame) >= 0) {
        video_frame_count++;


        AVFrameSideData *sd = av_frame_get_side_data(
            frame, AV_FRAME_DATA_MOTION_VECTORS
        );
        if (sd) {
            auto mvs = reinterpret_cast<const AVMotionVector *>(sd->data);
            int mv_count = sd->size / sizeof(AVMotionVector);
            for (int i = 0; i < mv_count; i++) {
                const AVMotionVector *mv = &mvs[i];

                MotionVectorData mv_data;
                mv_data.frame_number = video_frame_count;
                mv_data.source = mv->source;
                mv_data.w = mv->w;
                mv_data.h = mv->h;
                mv_data.src_x = mv->src_x;
                mv_data.src_y = mv->src_y;
                mv_data.dst_x = mv->dst_x;
                mv_data.dst_y = mv->dst_y;
                mv_data.flags = mv->flags;
                mv_data.motion_x = mv->motion_x;
                mv_data.motion_y = mv->motion_y;
                mv_data.motion_scale = mv->motion_scale;
                motion_vectors2.push_back(mv_data);

                motion_vectors.push_back(*mv);
            }
        }
        av_frame_unref(frame);
    }


end:
    if (pkt)
        av_packet_free(&pkt);
    if (frame)
        av_frame_free(&frame);
    if (codec_ctx)
        avcodec_free_context(&codec_ctx);
    if (fmt_ctx)
        avformat_close_input(&fmt_ctx);
    if (avio_ctx) {
        av_freep(&avio_ctx->buffer);
        avio_context_free(&avio_ctx);
    }

    return motion_vectors;
}
