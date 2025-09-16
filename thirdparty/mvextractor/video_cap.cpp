#include <mvextractor/video_cap.hpp>

VideoCap::VideoCap()
{
    this->opts = NULL;
    this->codec = NULL;
    this->fmt_ctx = NULL;
    this->codec_params = NULL;
    this->video_dec_ctx = NULL;
    this->video_stream = NULL;
    this->video_stream_idx = -1;
    this->frame = NULL;
    this->img_convert_ctx = NULL;
    this->frame_number = 0;
    this->frame_timestamp = 0.0;
    this->is_rtsp = false;

    memset(&(this->rgb_frame), 0, sizeof(this->rgb_frame));
    this->picture = {NULL, 0, 0, 0, 0};
// TODO: do I need rgb_frame alloc here?
    this->packet = av_packet_alloc();
    if (!this->packet)
    {
        std::cout << "failed to allocate memory for AVPacket";
        return;
    }
// TODO: is this necessay?
//    av_init_packet(this->packet);
}

void VideoCap::release()
{
    std::cout << "Releasing the video capture!" << std::endl;
    if (this->img_convert_ctx != NULL)
    {
        sws_freeContext(this->img_convert_ctx);
        this->img_convert_ctx = NULL;
    }
    if (this->frame != NULL) 
    {
        av_frame_free(&(this->frame));
        this->frame = NULL;
    }

    av_frame_unref(&(this->rgb_frame));
    memset(&(this->rgb_frame), 0, sizeof(this->rgb_frame));
    this->picture = {NULL, 0, 0, 0, 0};

        if (this->video_dec_ctx != NULL) {
        avcodec_free_context(&(this->video_dec_ctx));
        this->video_dec_ctx = NULL;
    }

    if (this->fmt_ctx != NULL) {
        avformat_close_input(&(this->fmt_ctx));
        this->fmt_ctx = NULL;
    }

    if (this->opts != NULL) {
        av_dict_free(&(this->opts));
        this->opts = NULL;
    }

    if (this->packet->data) {
        av_packet_unref(this->packet);
        this->packet->data = NULL;
    }
    av_packet_free(&(this->packet));

}

bool VideoCap::open(const char *_url)
{

    this->url = _url;
    this->fmt_ctx = avformat_alloc_context();
    if (!this->fmt_ctx)
    {
        std::cout << "ERROR could not allocate memory for Format Context" << std::endl;
        return -1;
    }

    /* 
    * In MV-extractor opts are used here for rtsp with tcp so might just
    * leave it as NULL.
    */
    if(avformat_open_input(&this->fmt_ctx, this->url, NULL, &(this->opts))!= 0)
    {
        std::cout << "ERROR could not open the file" << std::endl;
        return -1;
    }

    std::cout << "Successfully opened format context. Format " << this->fmt_ctx->iformat->long_name << ", duration " << this->fmt_ctx->duration << std::endl;

    if(avformat_find_stream_info(this->fmt_ctx, NULL) < 0)
    {
        std::cout << "ERROR could not get the stream info";
        return -1;
    }

    for(u_int i = 0; i < this->fmt_ctx->nb_streams;i++){
        AVCodecParameters *localCodecParams = this->fmt_ctx->streams[i]->codecpar;
        const AVCodec *localCodec = avcodec_find_decoder(localCodecParams->codec_id);

        if(localCodec == NULL)
        {
            std::cout << "ERROR unsupported codec!" << std::endl;
            continue;
        }
        if(localCodecParams->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            std::cout << "Found video stream with resolution " << localCodecParams->width << " by " << localCodecParams->height  << std::endl;
            if(video_stream_idx != -1)
            {
                std::cout << "ERROR currenty only a single video stream is supported" << std::endl;
                return -1;
            }else
            {
                video_stream_idx = i;
                this->codec = localCodec;
                this->codec_params = localCodecParams;
            }
        }else
        {
            // we are only interested in video streams 
            continue;
        }
    }

    if(video_stream_idx == -1)
    {
        std::cout << "File " << this->url << " does not contain a video stream!" << std::endl;
        return -1;
    }
    
    this->video_dec_ctx = avcodec_alloc_context3(this->codec);
    if(!this->video_dec_ctx)
    {
        std::cout << "failed to allocated memory for AVCodecContext" << std::endl;
        return -1;
    }

    if (avcodec_parameters_to_context(this->video_dec_ctx, this->codec_params) < 0)
    {
        std::cout << "failed to copy codec params to codec context" << std::endl;
        return -1;
    }

    // ffmpeg recommends no more than 16 threads
//    this->video_dec_ctx->thread_count = std::min(std::thread::hardware_concurrency(), 16u);

    // backup encoder's width/height 
// TODO: decide if necessary
    int enc_width = this->video_dec_ctx->width;
    int enc_height = this->video_dec_ctx->height;

    // Init the video decoder with the codec and set additional option to extract motion vectors

    av_dict_set(&(this->opts), "flags2", "+export_mvs", 0);

    if (avcodec_open2(this->video_dec_ctx, this->codec, &(this->opts)) < 0)
    {
        std::cout << "failed to open codec through avcodec_open2" << std::endl;
        return -1;
    }

    this->video_stream = this->fmt_ctx->streams[this->video_stream_idx];

    // checking width/height (since decoder can sometimes alter it, eg. vp6f)
// TODO: check if necessary
    if (enc_width && (this->video_dec_ctx->width != enc_width))
        this->video_dec_ctx->width = enc_width;
    if (enc_height && (this->video_dec_ctx->height != enc_height))
        this->video_dec_ctx->height = enc_height;

    this->picture.width = this->video_dec_ctx->width;
    this->picture.height = this->video_dec_ctx->height;
    this->picture.data = NULL;

    this->frame = av_frame_alloc();
    if (!this->frame)
    {
        std::cout << "failed to allocate memory for AVFrame" << std::endl;
        return -1;
    }

    return true;
}

int VideoCap::decode_packet(AVPacket *packet, AVCodecContext *fmt_ctx, AVFrame *frame)
{
    int response = avcodec_send_packet(this->video_dec_ctx, this->packet);

    if (response < 0)
    {
        std::cout << "Error while sending a packet to the decoder" << std::endl;
        return response;
    }

    while (true)
    {
        response = avcodec_receive_frame(this->video_dec_ctx, this->frame);
        if (response == AVERROR(EAGAIN))
        {
            std::cout << "error while receiving frame with response: " << response << std::endl;
            continue;
        } else if(response == AVERROR_EOF) 
        {
            std::cout << "error while receiving frame with response: " << response << std::endl;
            break;
        } else if (response < 0)
        {
            std::cout << "Error while receiving a frame from the decoder" << std::endl;
            return response;
        }

        if (response >= 0)
        {
            std::cout << "Received frame " << this->video_dec_ctx->frame_num << std::endl;
            char frame_filename[1024];
            snprintf(frame_filename, sizeof(frame_filename), "%s-%ld.pgm", "frame", this->video_dec_ctx->frame_num);
            if (this->frame->format != AV_PIX_FMT_YUV420P)
            {
                std::cout << "Warning: the generated file may not be a grayscale image, but could e.g. be just the R component if the video format is RGB" << std::endl;
            }
            save_gray_frame(this->frame->data[0], this->frame->linesize[0], this->frame->width, this->frame->height, frame_filename);
        }
    }
    return response;
}

void VideoCap::save_gray_frame(unsigned char *buf, int wrap, int xsize, int ysize, char *filename)
{
    std::cout <<"converting frame to grayscale" << std::endl;
    FILE *f;
    int i;
    f = fopen(filename,"w");
    // writing the minimal required header for a pgm file format
    // portable graymap format -> https://en.wikipedia.org/wiki/Netpbm_format#PGM_example
    fprintf(f, "P5\n%d %d\n%d\n", xsize, ysize, 255);

    // writing line by line
    for (i = 0; i < ysize; i++)
        fwrite(buf + i * wrap, 1, xsize, f);
    fclose(f);
}

bool VideoCap::grab()
{
    std::cout << "starting to grab frames" << std::endl;
    int count_errs = 0;
    const int max_number_of_attempts = 10;

    // make sure file is opened
    if (!this->fmt_ctx || !this->video_stream)
    {
        std::cout << "Files is not opend properly"<< std::endl;
        return false;
    }
    // check if there is a frame left in the stream
    if (this->fmt_ctx->streams[this->video_stream_idx]->nb_frames > 0 && this->frame_number > this->fmt_ctx->streams[this->video_stream_idx]->nb_frames)
    {
        std::cout << "No more frames to grab from video" << std::endl;
        return false;
    }
    int response = 0;
    av_packet_unref(this->packet);

    while(true)
    {
        int readFramesRes = av_read_frame(this->fmt_ctx, this->packet);
        if(readFramesRes < 0)
        {
            if(readFramesRes == AVERROR(EOF) || readFramesRes == AVERROR_EOF)
            {
                std::cout << "Reached the end of the file. Stopping now!" << std::endl;
//                av_packet_unref(this->packet);
                return false;
            }
            continue;
        }

        // TODO: is this error counting necessary for our usecase?
        if(this->packet->stream_index != this->video_stream_idx)
        {
            av_packet_unref(this->packet);
            count_errs++;
            if(count_errs > max_number_of_attempts)
            {
                std::cout << "Tried " << max_number_of_attempts << " times to read frame. Stopping now!" << std::endl;
                return false;
            }
            continue;
        }else
        {
            std::cout << "Found video stream!" << std::endl; 
            if(avcodec_send_packet(this->video_dec_ctx, this->packet))
            {
                std::cout << "Error while sending a packet to the decoder" << std::endl;
                return false;
            }
            response = avcodec_receive_frame(this->video_dec_ctx, this->frame);
            if(response == AVERROR(EOF) || readFramesRes == AVERROR_EOF)
            {
                std::cout << "Reached the end of the file. Stopping now!" << std::endl;
                return false; 
            } else if (response == AVERROR(EAGAIN))
            {
                std::cout << "Frame didn'thave enough data, trying next one" << std::endl;
                continue;
            }else if (response < 0)
            {
                std::cout << "Failed to decode packet with response: " << response << std::endl;
                return false;
            } else if (response >= 0)
            {
                std::cout << "Found a frame and decoded successfully!" << std::endl;
                break;
            }
        }
        av_packet_unref(this->packet);
    }
    auto now = std::chrono::system_clock::now();
    this->frame_timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    this->frame_number++;
    return true;
}

bool VideoCap::retrieve(uint8_t **frame, int *step, int *width, int *height, int *cn, char *frame_type, MVS_DTYPE **motion_vectors, MVS_DTYPE *num_mvs, double *frame_timestamp)
{
    if (!(this->video_stream) || !(this->frame->data[0]))
    {
        std::cout << "There is no video stream or the frame is empty!" << std::endl;
        return false;
    }

    if (this->img_convert_ctx == NULL ||
        this->picture.width != this->video_dec_ctx->width ||
        this->picture.height != this->video_dec_ctx->height ||
        this->picture.data == NULL) {
        std::cout << "creating new Image convert context" << std::endl;
        // Some sws_scale optimizations have some assumptions about alignment of data/step/width/height
        // Also we use coded_width/height to workaround problem with legacy ffmpeg versions (like n0.8)
        int buffer_width = this->video_dec_ctx->coded_width;
        int buffer_height = this->video_dec_ctx->coded_height;

        this->img_convert_ctx = sws_getCachedContext(
                this->img_convert_ctx,
                buffer_width, buffer_height,
                this->video_dec_ctx->pix_fmt,
                buffer_width, buffer_height,
                AV_PIX_FMT_BGR24,
                SWS_BICUBIC,
                NULL, NULL, NULL
                );

        if (this->img_convert_ctx == NULL)
        {
            std::cout << "Failed to create image converter context!" << std::endl;
            return false;
        }
        std::cout << "setting rgb frame values" << std::endl;
        av_frame_unref(&(this->rgb_frame));
        this->rgb_frame.format = AV_PIX_FMT_BGR24;
        this->rgb_frame.width = buffer_width;
        this->rgb_frame.height = buffer_height;
        if (0 != av_frame_get_buffer(&(this->rgb_frame), 32))
        {
            std::cout << "Error while allocating buffer for rgb frame!" << std::endl;
            return false;
        }
        std::cout << "Resetting picture" << std::endl;
        this->picture.width = this->video_dec_ctx->width;
        this->picture.height = this->video_dec_ctx->height;
        this->picture.data = this->rgb_frame.data[0];
        this->picture.step = this->rgb_frame.linesize[0];
        this->picture.cn = 3;
    }

    // change color space of frame
    std::cout << "Changing color space of frame" << std::endl;
    sws_scale(
        this->img_convert_ctx,
        this->frame->data,
        this->frame->linesize,
        0, this->video_dec_ctx->coded_height,
        this->rgb_frame.data,
        this->rgb_frame.linesize
        );

    std::cout << "Filling picture return values of " << &this->picture << std::endl;
    *frame = this->picture.data;
    *width = this->picture.width;
    *height = this->picture.height;
    *step = this->picture.step;
    *cn = this->picture.cn;

    // get motion vectors
    AVFrameSideData *sd = av_frame_get_side_data(this->frame, AV_FRAME_DATA_MOTION_VECTORS);
    std::cout << "sd " << sd << std::endl;
    if (sd)
    {
        std::cout << "Received side data of frame" << std::endl;
        std::cout << "Getting movement vectors of side data" <<std::endl;
        AVMotionVector *mvs = (AVMotionVector *)sd->data;
        std::cout << "calculating number of movement vectors" << std::endl;
        *num_mvs = sd->size / sizeof(*mvs);
        std::cout << "Side data contains " << *num_mvs << " Movement vectors" << std::endl;
        if (*num_mvs > 0)
        {
            
            // allocate memory for motion vectors as 1D array
            if (!(*motion_vectors = (MVS_DTYPE *) malloc(*num_mvs * 10 * sizeof(MVS_DTYPE))))
            {
                std::cout << "Failed to allocate space for motion_vectors!" << std::endl;
                return false;
            }

            std::cout << "storing movement vectors in allocated memory" << std::endl;
            // store the motion vectors in the allocated memory (C contiguous)
            for (MVS_DTYPE i = 0; i < *num_mvs; ++i)
            {
                *(*motion_vectors + i*10     ) = static_cast<MVS_DTYPE>(mvs[i].source);
                *(*motion_vectors + i*10 +  1) = static_cast<MVS_DTYPE>(mvs[i].w);
                *(*motion_vectors + i*10 +  2) = static_cast<MVS_DTYPE>(mvs[i].h);
                *(*motion_vectors + i*10 +  3) = static_cast<MVS_DTYPE>(mvs[i].src_x);
                *(*motion_vectors + i*10 +  4) = static_cast<MVS_DTYPE>(mvs[i].src_y);
                *(*motion_vectors + i*10 +  5) = static_cast<MVS_DTYPE>(mvs[i].dst_x);
                *(*motion_vectors + i*10 +  6) = static_cast<MVS_DTYPE>(mvs[i].dst_y);
                *(*motion_vectors + i*10 +  7) = static_cast<MVS_DTYPE>(mvs[i].motion_x);
                *(*motion_vectors + i*10 +  8) = static_cast<MVS_DTYPE>(mvs[i].motion_y);
                *(*motion_vectors + i*10 +  9) = static_cast<MVS_DTYPE>(mvs[i].motion_scale);
                //*(*motion_vectors + i*11 + 10) = static_cast<MVS_DTYPE>(mvs[i].flags);
            }
        }
    }
    std::cout << "getting frame type and adding it to return value" << std::endl;
    // get frame type (I, P, B, etc.) and create a null terminated c-string
    frame_type[0] = av_get_picture_type_char(this->frame->pict_type);
    frame_type[1] = '\0';

    std::cout << "setting the timestamp" << std::endl;
    // return the timestamp which was computed previously in grab()
    *frame_timestamp = this->frame_timestamp;

    return true;
}

bool VideoCap::read(uint8_t **frame, int *step, int *width, int *height, int *cn, char *frame_type, MVS_DTYPE **motion_vectors, MVS_DTYPE *num_mvs, double *frame_timestamp)
{
    bool ret = this->grab();
    if (ret)
	ret = this->retrieve(frame, step, width, height, cn, frame_type, motion_vectors, num_mvs, frame_timestamp);
    return ret;
}
