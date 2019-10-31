/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file vpu_encoder_b0.c
 *
 * copyright here may be changed later
 *
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>

#include "vpu_encoder_b0.h"

unsigned int vpu_dbg_level_encoder = 0;
#ifdef DUMP_DATA
#define DATA_NUM 10
#endif
/*
 * v4l2 ioctl() operation
 *
 */
static struct vpu_v4l2_fmt  formats_compressed_enc[] = {
	{
		.name       = "H264 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_AVC,
	},
	{
		.name       = "VC1 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_VC1,
	},
	{
		.name       = "VC1 RCV Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_VC1,
	},
	{
		.name       = "MPEG2 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_MPEG2,
	},

	{
		.name       = "AVS Encoded Stream",
		.fourcc     = VPU_PIX_FMT_AVS,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_AVS,
	},
	{
		.name       = "MPEG4 ASP Encoded Stream",
		.fourcc     = VPU_PIX_FMT_ASP,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_ASP,
	},
	{
		.name       = "JPEG stills",
		.fourcc     = V4L2_PIX_FMT_JPEG,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_JPEG,
	},
	{
		.name       = "RV8 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_RV8,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_RV8,
	},
	{
		.name       = "RV9 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_RV9,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_RV9,
	},
	{
		.name       = "VP6 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_VP6,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_VP6,
	},
	{
		.name       = "VP6 SPK Encoded Stream",
		.fourcc     = VPU_PIX_FMT_SPK,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_SPK,
	},
	{
		.name       = "VP8 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_VP8,
	},
	{
		.name       = "H264/MVC Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264_MVC,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_AVC_MVC,
	},
	{
		.name       = "H265 HEVC Encoded Stream",
		.fourcc     = VPU_PIX_FMT_HEVC,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_HEVC,
	},
	{
		.name       = "VP9 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_VP9,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_VP9,
	},
	{
		.name       = "Logo",
		.fourcc     = VPU_PIX_FMT_LOGO,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_UNDEFINED,
	},
};

static struct vpu_v4l2_fmt  formats_yuv_enc[] = {
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12,
		.num_planes	= 2,
		.venc_std   = VPU_PF_YUV420_SEMIPLANAR,
	},
};

static void MU_sendMesgToFW(void __iomem *base, MSG_Type type, uint32_t value)
{
	MU_SendMessage(base, 1, value);
	MU_SendMessage(base, 0, type);
}

static int v4l2_ioctl_querycap(struct file *file,
		void *fh,
		struct v4l2_capability *cap
		)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	strncpy(cap->driver, "vpu encoder", sizeof(cap->driver) - 1);
	strlcpy(cap->card, "vpu encoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int v4l2_ioctl_enum_fmt_vid_cap_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	if (f->index >= ARRAY_SIZE(formats_yuv_enc))
		return -EINVAL;

	fmt = &formats_yuv_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}
static int v4l2_ioctl_enum_fmt_vid_out_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (f->index >= ARRAY_SIZE(formats_compressed_enc))
		return -EINVAL;

	fmt = &formats_compressed_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int v4l2_ioctl_g_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx *ctx =           v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	unsigned int i;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_mp->pixelformat = V4L2_PIX_FMT_NV12;
		pix_mp->width = ctx->q_data[V4L2_SRC].width;
		pix_mp->height = ctx->q_data[V4L2_SRC].height;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;

		for (i = 0; i < pix_mp->num_planes; i++)
			pix_mp->plane_fmt[i].sizeimage = ctx->q_data[V4L2_SRC].sizeimage[i];
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pix_mp->width = 0;
		pix_mp->height = 0;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->plane_fmt[0].bytesperline = 0;
		pix_mp->plane_fmt[0].sizeimage = ctx->q_data[V4L2_DST].sizeimage[0];
		pix_mp->pixelformat = V4L2_PIX_FMT_H264;
		pix_mp->num_planes = 1;
	} else
		return -EINVAL;
	return 0;
}

static void get_param_from_v4l2(pMEDIAIP_ENC_PARAM pEncParam,
		struct v4l2_pix_format_mplane *pix_mp,
		struct vpu_ctx *ctx
		)
{
	//get the param and update gpParameters
	pEncParam->eCodecMode           = MEDIAIP_ENC_FMT_H264;

	pEncParam->tEncMemDesc.uMemPhysAddr = ctx->encoder_mem.phy_addr;
	pEncParam->tEncMemDesc.uMemVirtAddr = ctx->encoder_mem.phy_addr;
	pEncParam->tEncMemDesc.uMemSize     = ctx->encoder_mem.size;

	pEncParam->uFrameRate           = 30;
	pEncParam->uSrcStride           = pix_mp->width;
	pEncParam->uSrcWidth            = pix_mp->width;
	pEncParam->uSrcHeight           = pix_mp->height;
	pEncParam->uSrcOffset_x         = 0;
	pEncParam->uSrcOffset_y         = 0;
	pEncParam->uSrcCropWidth        = pix_mp->width;
	pEncParam->uSrcCropHeight       = pix_mp->height;
	pEncParam->uOutWidth            = pix_mp->width;
	pEncParam->uOutHeight           = pix_mp->height;
	pEncParam->uLowLatencyMode      = 0;

	pEncParam->uIFrameInterval      = 10;

	vpu_dbg(LVL_INFO, "eCodecMode(%d) eProfile(%d) uSrcStride(%d) uSrcWidth(%d) uSrcHeight(%d) uSrcOffset_x(%d) uSrcOffset_y(%d) uSrcCropWidth(%d) uSrcCropHeight(%d) uOutWidth(%d) uOutHeight(%d) uGopBLength(%d) uLowLatencyMode(%d) uInitSliceQP(%d) uIFrameInterval(%d) eBitRateMode(%d) uTargetBitrate(%d) uMaxBitRate(%d) uMinBitRate(%d) uFrameRate(%d)\n",
			pEncParam->eCodecMode, pEncParam->eProfile, pEncParam->uSrcStride, pEncParam->uSrcWidth,
			pEncParam->uSrcHeight, pEncParam->uSrcOffset_x, pEncParam->uSrcOffset_y, pEncParam->uSrcCropWidth, pEncParam->uSrcCropHeight,
			pEncParam->uOutWidth, pEncParam->uOutHeight, pEncParam->uGopBLength, pEncParam->uLowLatencyMode, pEncParam->uInitSliceQP, pEncParam->uIFrameInterval, pEncParam->eBitRateMode, pEncParam->uTargetBitrate, pEncParam->uMaxBitRate, pEncParam->uMinBitRate, pEncParam->uFrameRate);
}

static void *phy_to_virt(u_int32 src, unsigned long long offset)
{
	void *result;

	result = (void *)(src + offset);
	return result;
}

static int v4l2_ioctl_s_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx                  *ctx = v4l2_fh_to_ctx(fh);
	int                             ret = 0;
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data               *q_data;
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	pMEDIAIP_ENC_PARAM  pEncParam;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam;

	pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[ctx->str_index],
			ctx->dev->shared_mem.base_offset);
	pEncParam = (pMEDIAIP_ENC_PARAM)phy_to_virt(pEncCtrlInterface->pEncParam,
			ctx->dev->shared_mem.base_offset);
	pEncExpertModeParam = (pMEDIAIP_ENC_EXPERT_MODE_PARAM)phy_to_virt(pEncCtrlInterface->pEncExpertModeParam,
			ctx->dev->shared_mem.base_offset);
	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];

		get_param_from_v4l2(pEncParam, pix_mp, ctx);
		q_data->fourcc = pix_mp->pixelformat;
		q_data->width = pix_mp->width;
		q_data->height = pix_mp->height;
		q_data->rect.left = 0;
		q_data->rect.top = 0;
		q_data->rect.width = pix_mp->width;
		q_data->rect.height = pix_mp->height;
		q_data->sizeimage[0] = pix_mp->width * pix_mp->height;
		q_data->sizeimage[1] = pix_mp->width * pix_mp->height / 2;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		q_data->fourcc = pix_mp->pixelformat;
		q_data->sizeimage[0] = pix_mp->plane_fmt[0].sizeimage;
	} else
		ret = -EINVAL;

	return ret;
}

static int v4l2_ioctl_expbuf(struct file *file,
		void *fh,
		struct v4l2_exportbuffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	return (vb2_expbuf(&q_data->vb2_q,
				buf
				));
}

static int v4l2_ioctl_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub
		)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static int v4l2_ioctl_reqbufs(struct file *file,
		void *fh,
		struct v4l2_requestbuffers *reqbuf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_reqbufs(&q_data->vb2_q, reqbuf);

	vpu_dbg(LVL_INFO, "%s() c_port_req_buf(%d)\n",
			__func__, ret);

	return ret;
}

static int v4l2_ioctl_querybuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	unsigned int i;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_querybuf(&q_data->vb2_q, buf);
	if (!ret) {
		if (buf->memory == V4L2_MEMORY_MMAP) {
			if (V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
				for (i = 0; i < buf->length; i++)
					buf->m.planes[i].m.mem_offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
			} else
				buf->m.offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
		}
	}

	return ret;
}

static int v4l2_ioctl_qbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_qbuf(&q_data->vb2_q, buf);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		wake_up_interruptible(&ctx->buffer_wq_output);
	else
		wake_up_interruptible(&ctx->buffer_wq_input);

	return ret;
}

static int v4l2_ioctl_dqbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_dqbuf(&q_data->vb2_q, buf, file->f_flags & O_NONBLOCK);

	return ret;
}

static bool format_is_support(struct vpu_v4l2_fmt *format_table,
		unsigned int table_size,
		struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (format_table[i].fourcc == f->fmt.pix_mp.pixelformat)
			return true;
	}
	return false;
}

static int v4l2_ioctl_try_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	unsigned int table_size;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		table_size = ARRAY_SIZE(formats_compressed_enc);
		if (!format_is_support(formats_compressed_enc, table_size, f))
			return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		table_size = ARRAY_SIZE(formats_yuv_enc);
		if (!format_is_support(formats_yuv_enc, table_size, f))
			return -EINVAL;
	} else
		return -EINVAL;

	return 0;
}

static int v4l2_ioctl_g_crop(struct file *file,
		void *fh,
		struct v4l2_crop *cr
		)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	cr->c.left = 0;
	cr->c.top = 0;
	cr->c.width = 0;
	cr->c.height = 0;

	return 0;
}

static int v4l2_ioctl_encoder_cmd(struct file *file,
		void *fh,
		struct v4l2_encoder_cmd *cmd
		)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_START:
		break;
	case V4L2_ENC_CMD_STOP:
		break;
	case V4L2_ENC_CMD_PAUSE:
		break;
	case V4L2_ENC_CMD_RESUME:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int v4l2_ioctl_streamon(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;
	ret = vb2_streamon(&q_data->vb2_q,
			i);
	return ret;
}

static int v4l2_ioctl_streamoff(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;
	ret = vb2_streamoff(&q_data->vb2_q,
			i);
	return ret;
}

static const struct v4l2_ioctl_ops v4l2_encoder_ioctl_ops = {
	.vidioc_querycap                = v4l2_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_ioctl_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_ioctl_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_expbuf                  = v4l2_ioctl_expbuf,
	.vidioc_g_crop                  = v4l2_ioctl_g_crop,
	.vidioc_encoder_cmd             = v4l2_ioctl_encoder_cmd,
	.vidioc_subscribe_event         = v4l2_ioctl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
	.vidioc_reqbufs                 = v4l2_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_ioctl_querybuf,
	.vidioc_qbuf                    = v4l2_ioctl_qbuf,
	.vidioc_dqbuf                   = v4l2_ioctl_dqbuf,
	.vidioc_streamon                = v4l2_ioctl_streamon,
	.vidioc_streamoff               = v4l2_ioctl_streamoff,
};

static int v4l2_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	pMEDIAIP_ENC_PARAM  pEncParam;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam;

	pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[ctx->str_index],
			ctx->dev->shared_mem.base_offset);
	pEncParam = (pMEDIAIP_ENC_PARAM)phy_to_virt(pEncCtrlInterface->pEncParam,
			ctx->dev->shared_mem.base_offset);
	pEncExpertModeParam = (pMEDIAIP_ENC_EXPERT_MODE_PARAM)phy_to_virt(pEncCtrlInterface->pEncExpertModeParam,
			ctx->dev->shared_mem.base_offset);

	vpu_dbg(LVL_INFO, "s_ctrl: id = %d, val = %d\n", ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE: {
		if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
			pEncParam->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP;

			// Not used for CQ mode - set zero
			pEncParam->uTargetBitrate       = 0;
			pEncParam->uMaxBitRate          = 0;
			pEncParam->uMinBitRate          = 0;
		} else if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) {
			pEncParam->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CBR;
			if (!pEncParam->uTargetBitrate) {
				// Setup some default values if not set, these should really be
				// resolution specific
				pEncParam->uTargetBitrate = 200;
				pEncParam->uMaxBitRate    = 4000;
				pEncParam->uMinBitRate    = 50;
			}
		} else
			// Only CQ and CBR supported at present, force CQ mode
			pEncParam->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP;
	}
		break;
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		if (V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE == ctrl->val)
		pEncParam->eProfile = MEDIAIP_ENC_PROF_H264_BP;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		pEncParam->uTargetBitrate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		pEncParam->uGopBLength = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		pEncParam->uInitSliceQP = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		pEncParam->uInitSliceQP = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		pEncParam->uInitSliceQP = ctrl->val;
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops   vpu_enc_ctrl_ops = {
	.s_ctrl             = v4l2_enc_s_ctrl,
};

static void vpu_encoder_ctrls(struct vpu_ctx *ctx)
{
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			V4L2_MPEG_VIDEO_BITRATE_MODE_CBR, 0x0,
			V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH, 0x0,
			V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE
			);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_BITRATE, 0, 32767000, 1, 100);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 60, 1, 16);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, 0, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, 0, 51, 1, 25);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &vpu_enc_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP, 0, 51, 1, 25);
}

static int ctrls_setup_encoder(struct vpu_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 2);
	vpu_encoder_ctrls(ctx);
	if (ctx->ctrl_handler.error) {
		vpu_dbg(LVL_ERR,
			"control initialization error (%d)",
			ctx->ctrl_handler.error);
		return -EINVAL;
	} else
		ctx->ctrl_inited = true;

	return v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
}

static void ctrls_delete_encoder(struct vpu_ctx *This)
{
	int i;

	if (This->ctrl_inited) {
		v4l2_ctrl_handler_free(&This->ctrl_handler);
		This->ctrl_inited = false;
	}
	for (i = 0; i < 2; i++)
		This->ctrls[i] = NULL;
}

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata)
{
	rpc_send_cmd_buf_encoder(&ctx->dev->shared_mem, idx, cmdid, cmdnum, local_cmddata);
	MU_SendMessage(ctx->dev->mu_base_virtaddr, 0, COMMAND);
}

/**the function is used for to convert yuv420p to yuv420sp
 * yyyy yyyy
 * uu vv
 * ->
 * yyyy yyyy
 * uv uv
 **/
static void convert_feed_stream(struct queue_data *This, struct vb2_buffer *vb)
{
	u_int8 *y_start;
	u_int8 *u_start;
	u_int8 *v_start;
	u_int8 *uv_temp;
	u_int32 i, j;
	u_int32 height = This->height;
	u_int32 width = This->width;
	u_int32 y_size, uv_size;
	y_size = height * width;
	uv_size = height * width/2;
	uv_temp = kmalloc(sizeof(u_int8)*uv_size, GFP_KERNEL);

	y_start = (u_int8 *)vb2_plane_vaddr(vb, 0);
	u_start = y_start + y_size;
	v_start = y_start + y_size*5/4;
	for (i = 0, j = 0; j < uv_size; j += 2, i++) {
		uv_temp[j] = u_start[i];
		uv_temp[j+1] = v_start[i];
	}
	memcpy(u_start, uv_temp, sizeof(u_int8)*uv_size);

	kfree(uv_temp);
}

static void v4l2_transfer_buffer_to_firmware(struct queue_data *This, struct vb2_buffer *vb)
{
	struct vpu_ctx *ctx = container_of(This, struct vpu_ctx, q_data[V4L2_SRC]);
#ifdef DUMP_DATA
	char *read_data;
	u_int32 read_idx;
#endif
	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam;
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	u_int32 uStrIdx = 0;

	vpu_dbg(LVL_INFO, "ENC_RPC_HOST_IFACE(%ld)MEDIA_ENC_API_CONTROL_INTERFACE(%ld) EncYUVBufferDesc(%ld) expertParam(%ld) encparam(%ld) MEDIAIP_ENC_FMT(%ld)\n",
			sizeof(ENC_RPC_HOST_IFACE), sizeof(MEDIA_ENC_API_CONTROL_INTERFACE),
			sizeof(BUFFER_DESCRIPTOR_TYPE), sizeof(MEDIAIP_ENC_EXPERT_MODE_PARAM),
			sizeof(MEDIAIP_ENC_PARAM), sizeof(MEDIAIP_ENC_FMT)
			);
	if (ctx->start_flag == true) {
		pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[uStrIdx],
				ctx->dev->shared_mem.base_offset);
		pEncStrBuffDesc = (pBUFFER_DESCRIPTOR_TYPE)phy_to_virt(pEncCtrlInterface->pEncStreamBufferDesc,
				ctx->dev->shared_mem.base_offset);
		pEncStrBuffDesc->start = ctx->encoder_stream.phy_addr;
		pEncStrBuffDesc->wptr = pEncStrBuffDesc->start;
		pEncStrBuffDesc->rptr = pEncStrBuffDesc->start;
		pEncStrBuffDesc->end = ctx->encoder_stream.phy_addr + ctx->encoder_stream.size;

		vpu_dbg(LVL_INFO, "pEncStrBuffDesc->start=%x, pEncStrBuffDesc->wptr=0x%x, pEncStrBuffDesc->rptr=%x, pEncStrBuffDesc->end=%x\n", pEncStrBuffDesc->start, pEncStrBuffDesc->wptr, pEncStrBuffDesc->rptr, pEncStrBuffDesc->end);

		pEncExpertModeParam = (pMEDIAIP_ENC_EXPERT_MODE_PARAM)phy_to_virt(pEncCtrlInterface->pEncExpertModeParam,
				ctx->dev->shared_mem.base_offset);
		pEncExpertModeParam->Calib.mem_chunk_phys_addr = ctx->encoder_mem.phy_addr;
		pEncExpertModeParam->Calib.mem_chunk_virt_addr = ctx->encoder_mem.phy_addr;
		pEncExpertModeParam->Calib.mem_chunk_size = ctx->encoder_mem.size;
		pEncExpertModeParam->Calib.cb_base = ctx->encoder_stream.phy_addr;
		pEncExpertModeParam->Calib.cb_size = ctx->encoder_stream.size;

#ifdef DUMP_DATA
		read_data = (char *)vb2_plane_vaddr(vb, 0);
		vpu_dbg(LVL_INFO, "transfer data from virt 0x%p: ", read_data);
		for (read_idx = 0; read_idx < DATA_NUM; read_idx++)
			vpu_dbg(LVL_INFO, " 0x%x", read_data[read_idx]);
		vpu_dbg(LVL_INFO, "\n");
 #endif
		v4l2_vpu_send_cmd(ctx, 0, GTB_ENC_CMD_CONFIGURE_CODEC, 0, NULL);
		vpu_dbg(LVL_INFO, "send command GTB_ENC_CMD_CONFIGURE_CODEC\n");

		ctx->start_flag = false;
	}
}

static bool update_yuv_addr(struct vpu_ctx *ctx, u_int32 uStrIdx)
{
	bool bGotAFrame = FALSE;

	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	pMEDIAIP_ENC_YUV_BUFFER_DESC pParamYuvBuffDesc;
	u_int32 *pphy_address;
#ifdef DUMP_DATA
	char *read_data;
	u_int32 read_idx;
#endif

	pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[uStrIdx],
			ctx->dev->shared_mem.base_offset);
	pParamYuvBuffDesc = (pMEDIAIP_ENC_YUV_BUFFER_DESC)phy_to_virt(pEncCtrlInterface->pEncYUVBufferDesc,
			ctx->dev->shared_mem.base_offset);

	wait_event_interruptible(ctx->buffer_wq_input,
			!list_empty(&This->drv_q)
			);

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);

#ifdef DUMP_DATA
		read_data = (char *)vb2_plane_vaddr(p_data_req->vb2_buf, 0);
		vpu_dbg(LVL_INFO, "transfer data from virt 0x%p: ", read_data);
		for (read_idx = 0; read_idx < DATA_NUM; read_idx++)
			vpu_dbg(LVL_INFO, " 0x%x", read_data[read_idx]);
		vpu_dbg(LVL_INFO, "\n");
 #endif
		convert_feed_stream(This, p_data_req->vb2_buf);
		pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
		pParamYuvBuffDesc->uLumaBase = *pphy_address;
    /* Not sure what the test should be here for a valid frame return from vb2_plane_cookie */
		if (pParamYuvBuffDesc->uLumaBase != 0)
			bGotAFrame = TRUE;

    /* keeps increasing, so just a frame input count rather than a Frame buffer ID */
		pParamYuvBuffDesc->uFrameID = p_data_req->id;
		list_del(&p_data_req->list);
	}
	up(&This->drv_q_lock);
	return bGotAFrame;

}

static void report_stream_done(struct vpu_ctx *ctx,  MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_DST];
	u_int32 wptr;
	u_int32 rptr;
	u_int32 start;
	u_int32 end;

	void *data_mapped;
	u_int32 length;
	u_int32 data_length = 0;
	void *rptr_virt;

	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;

	/* Windsor stream buffer descriptor
	 * pEncStrBuffDesc = &RecCmdData.tEncStreamBufferDesc;
	 *
	 * VPU driver stream buffer descriptor with full address
	 * pVpuEncStrBuffDesc
	 * *
	 * Note the wprt is updated prior to calling this function
	 */
	pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[ctx->str_index],
			ctx->dev->shared_mem.base_offset);
	pEncStrBuffDesc = (pBUFFER_DESCRIPTOR_TYPE)phy_to_virt(pEncCtrlInterface->pEncStreamBufferDesc,
			ctx->dev->shared_mem.base_offset);


	wptr = pEncStrBuffDesc->wptr | 0x80000000;
	rptr = pEncStrBuffDesc->rptr | 0x80000000;
	start = pEncStrBuffDesc->start | 0x80000000;
	end = pEncStrBuffDesc->end | 0x80000000;
	rptr_virt = ctx->encoder_stream.virt_addr + rptr - start;

	vpu_dbg(LVL_INFO, "report_stream_done eptr=%x, rptr=%x, start=%x, end=%x\n", wptr, rptr, start, end);
	wait_event_interruptible(ctx->buffer_wq_output,
			!list_empty(&This->drv_q)
			);

	if (!list_empty(&This->drv_q)) {
		down(&This->drv_q_lock);

		vpu_dbg(LVL_INFO, "report_stream_done down\n");

		p_data_req = list_first_entry(&This->drv_q, typeof(*p_data_req), list);

		vpu_dbg(LVL_INFO, "%s :p_data_req(%p)\n", __func__, p_data_req);
		vpu_dbg(LVL_INFO, "%s buf_id %d\n", __func__, p_data_req->vb2_buf->index);

		// Calculate length - the amount of space remaining in output stream buffer
		length = p_data_req->vb2_buf->planes[0].length;
		data_mapped = (void *)vb2_plane_vaddr(p_data_req->vb2_buf, 0);
		if (wptr == rptr && rptr != start)
			data_length = end - start;
		else if (rptr < wptr)
			data_length = wptr - rptr;
		else if (rptr > wptr)
			data_length = (end - rptr) + (wptr - start);

	//update the bytesused for the output buffer
	if (data_length >= length)
		p_data_req->vb2_buf->planes[0].bytesused = length;
	else
		p_data_req->vb2_buf->planes[0].bytesused = data_length;

	vpu_dbg(LVL_INFO, "%s data_length %d, length %d\n", __func__, data_length, length);
	/* Following calculations determine how much data we can transfer into p_vb2_buf
	 * and then only copy that ammount, so rptr is the actual consumed ammount at the end*/
	if ((wptr == rptr) || (rptr > wptr)) {
		if (end - rptr >= length) {
			memcpy(data_mapped, rptr_virt, length);
			rptr += length;
			if (rptr == end)
				rptr = start;
		} else {
			memcpy(data_mapped, rptr_virt, end-rptr);
			if ((length-(end-rptr)) >= (wptr-start)) {
				memcpy(data_mapped + (end-rptr), ctx->encoder_stream.virt_addr, wptr-start);
				rptr = wptr;
			} else {
				memcpy(data_mapped + (end-rptr), ctx->encoder_stream.virt_addr, length-(end-rptr));
				rptr = start+length-(end-rptr);
			}
		}
	} else {
		if (wptr - rptr >= length) {
			memcpy(data_mapped, rptr_virt, length);
			rptr += length;
		} else {
			memcpy(data_mapped, rptr_virt, wptr - rptr);
			rptr = wptr;
		}
	}

	/* Update VPU stream buffer descriptor and Windsor FW stream buffer descriptors respectively*/
	pEncStrBuffDesc->rptr = rptr;

	list_del(&p_data_req->list);
	up(&This->drv_q_lock);
	//memcpy to vb2 buffer from encpicinfo
	vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);
	}
	vpu_dbg(LVL_INFO, "report_buffer_done return\n");
}

static void enc_mem_alloc(struct vpu_ctx *ctx, MEDIAIP_ENC_MEM_REQ_DATA *req_data)
{
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;
	pMEDIAIP_ENC_MEM_POOL pEncMemPool;
	pENC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	u_int32 i;

	pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)phy_to_virt(pSharedInterface->pEncCtrlInterface[ctx->str_index],
			ctx->dev->shared_mem.base_offset);
	pEncMemPool = (pMEDIAIP_ENC_MEM_POOL)phy_to_virt(pEncCtrlInterface->pEncMemPool,
			ctx->dev->shared_mem.base_offset);

	for (i = 0; i < req_data->uEncFrmNum; i++) {
		ctx->encFrame[i].size = ((req_data->uEncFrmSize + (~req_data->uAlignmentMask))&req_data->uAlignmentMask);
		ctx->encFrame[i].virt_addr = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
				ctx->encFrame[i].size,
				(dma_addr_t *)&ctx->encFrame[i].phy_addr,
				GFP_KERNEL | GFP_DMA32
				);
		if (!ctx->encFrame[i].virt_addr)
		vpu_dbg(LVL_ERR, "%s() encFrame alloc size(%x) fail!\n", __func__, ctx->encFrame[i].size);
		else
			vpu_dbg(LVL_INFO, "%s() encFrame size(%d) encFrame virt(%p) encFrame phy(%p)\n", __func__, ctx->encFrame[i].size, ctx->encFrame[i].virt_addr, (void *)ctx->encFrame[i].phy_addr);

		pEncMemPool->tEncFrameBuffers[i].uMemPhysAddr = ctx->encFrame[i].phy_addr;
#ifdef CM4
		pEncMemPool->tEncFrameBuffers[i].uMemVirtAddr = ctx->encFrame[i].phy_addr;
#else
		pEncMemPool->tEncFrameBuffers[i].uMemVirtAddr = ctx->encFrame[i].phy_addr - ctx->dev->m0_p_fw_space_phy;
#endif
		pEncMemPool->tEncFrameBuffers[i].uMemSize = ctx->encFrame[i].size;
	}

	for (i = 0; i < req_data->uRefFrmNum; i++) {
		ctx->refFrame[i].size = ((req_data->uRefFrmSize + (~req_data->uAlignmentMask))&req_data->uAlignmentMask);
		ctx->refFrame[i].virt_addr = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
				ctx->refFrame[i].size,
				(dma_addr_t *)&ctx->refFrame[i].phy_addr,
				GFP_KERNEL | GFP_DMA32
				);

		if (!ctx->refFrame[i].virt_addr)
		vpu_dbg(LVL_ERR, "%s() refFrame alloc size(%x) fail!\n", __func__, ctx->refFrame[i].size);
		else
			vpu_dbg(LVL_INFO, "%s() refFrame size(%d) refFrame virt(%p) refFrame phy(%p)\n", __func__, ctx->refFrame[i].size, ctx->refFrame[i].virt_addr, (void *)ctx->refFrame[i].phy_addr);

		pEncMemPool->tRefFrameBuffers[i].uMemPhysAddr = ctx->refFrame[i].phy_addr;
#ifdef CM4
		pEncMemPool->tRefFrameBuffers[i].uMemVirtAddr = ctx->refFrame[i].phy_addr;
#else
		pEncMemPool->tRefFrameBuffers[i].uMemVirtAddr = ctx->refFrame[i].phy_addr - ctx->dev->m0_p_fw_space_phy;
#endif
		pEncMemPool->tRefFrameBuffers[i].uMemSize = ctx->refFrame[i].size;
	}

	ctx->actFrame.size = ((req_data->uActBufSize + (~req_data->uAlignmentMask))&req_data->uAlignmentMask);
	ctx->actFrame.virt_addr = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			ctx->actFrame.size,
			(dma_addr_t *)&ctx->actFrame.phy_addr,
			GFP_KERNEL | GFP_DMA32
			);

	if (!ctx->actFrame.virt_addr)
		vpu_dbg(LVL_ERR, "%s() actFrame alloc size(%x) fail!\n", __func__, ctx->actFrame.size);
	else
		vpu_dbg(LVL_INFO, "%s() actFrame size(%d) actFrame virt(%p) actFrame phy(%p)\n", __func__, ctx->actFrame.size, ctx->actFrame.virt_addr, (void *)ctx->actFrame.phy_addr);

	pEncMemPool->tActFrameBufferArea.uMemPhysAddr = ctx->actFrame.phy_addr;
#ifdef CM4
	pEncMemPool->tActFrameBufferArea.uMemVirtAddr = ctx->actFrame.phy_addr;
#else
	pEncMemPool->tActFrameBufferArea.uMemVirtAddr = ctx->actFrame.phy_addr - ctx->dev->m0_p_fw_space_phy;
#endif
	pEncMemPool->tActFrameBufferArea.uMemSize = ctx->actFrame.size;

}

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
	vpu_dbg(LVL_INFO, "vpu_encoder_event_handler is called\n");
	if (uStrIdx < VID_API_NUM_STREAMS) {
		switch (uEvent) {
		case VID_API_ENC_EVENT_START_DONE: {
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_START_DONE : Encoder configuration complete\n");
		update_yuv_addr(ctx, 0);
		v4l2_vpu_send_cmd(ctx, 0, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		} break;
		case VID_API_ENC_EVENT_MEM_REQUEST: {
			MEDIAIP_ENC_MEM_REQ_DATA *req_data = (MEDIAIP_ENC_MEM_REQ_DATA *)event_data;
			vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_MEM_REQUEST: need to request memory\n");
			vpu_dbg(LVL_INFO, "uEncFrmSize = %d, uEncFrmNum=%d, uRefFrmSize=%d, uRefFrmNum=%d, uActBufSize=%d, uAlignmentMask=0x%x\n", req_data->uEncFrmSize, req_data->uEncFrmNum, req_data->uRefFrmSize, req_data->uRefFrmNum, req_data->uActBufSize, req_data->uAlignmentMask);
			enc_mem_alloc(ctx, req_data);
			//update_yuv_addr(ctx,0);
			v4l2_vpu_send_cmd(ctx, 0, GTB_ENC_CMD_STREAM_START, 0, NULL);

		} break;
		case VID_API_ENC_EVENT_PARA_UPD_DONE: {
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_PARA_UPD_DONE : Parameter update complete\n");
		} break;
		case VID_API_ENC_EVENT_FRAME_DONE: {
		MEDIAIP_ENC_PIC_INFO *pEncPicInfo = (MEDIAIP_ENC_PIC_INFO *)event_data;

		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_FRAME_DONE pEncPicInfo->uPicEncodDone=%d: Encode picture done\n", pEncPicInfo->uPicEncodDone);
		if (pEncPicInfo->uPicEncodDone) {
#ifdef TB_REC_DBG
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_FRAME_DONE : Encode picture done\n");
		vpu_dbg(LVL_INFO, "       - Frame ID      : 0x%x\n", pEncPicInfo->uFrameID);

		vpu_dbg(LVL_INFO, "       - Picture Type  : %s\n", pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_B_FRAME ? "B picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_P_FRAME ? "P picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_I_FRAME ? "I picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_IDR_FRAME ? "IDR picture" : "BI picture");
		vpu_dbg(LVL_INFO, "       - Skipped frame : 0x%x\n", pEncPicInfo->uSkippedFrame);
		vpu_dbg(LVL_INFO, "       - Frame size    : 0x%x\n", pEncPicInfo->uFrameSize);
		vpu_dbg(LVL_INFO, "       - Frame CRC     : 0x%x\n", pEncPicInfo->uFrameCrc);
#endif

		/* Sync the write pointer to the local view of it */

		report_stream_done(ctx, pEncPicInfo);
		}
		} break;
		case VID_API_ENC_EVENT_FRAME_RELEASE: {
		struct queue_data *This = &ctx->q_data[V4L2_SRC];
		u_int32 *uFrameID = (u_int32 *)event_data;
		struct vb2_data_req *p_data_req;

		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_FRAME_RELEASE : Frame release - uFrameID = 0x%x\n", *uFrameID);
		p_data_req = &This->vb2_reqs[*uFrameID];
		vb2_buffer_done(p_data_req->vb2_buf,
				VB2_BUF_STATE_DONE
				);

		} break;
		case VID_API_ENC_EVENT_STOP_DONE:
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_STOP_DONE : Stop done\n");
		break;
		case VID_API_ENC_EVENT_FRAME_INPUT_DONE: {
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_FRAME_INPUT_DONE : Input done\n");
		update_yuv_addr(ctx, 0);
		v4l2_vpu_send_cmd(ctx, 0, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		} break;
		case VID_API_ENC_EVENT_TERMINATE_DONE:
		vpu_dbg(LVL_INFO, "VID_API_ENC_EVENT_TERMINATE_DONE : Codec terminated\n");
		break;
		default:
		vpu_dbg(LVL_INFO, "........unknown event : 0x%x\n", uEvent);
		break;
		}
	}
}

//This code is added for MU
static irqreturn_t fsl_vpu_mu_isr(int irq, void *This)
{
	struct vpu_dev *dev = This;
	u32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);
	if (msg == 0xaa) {
#ifdef CM4
		MU_sendMesgToFW(dev->mu_base_virtaddr, RPC_BUF_OFFSET, dev->m0_rpc_phy); //CM4 use absolute address
#else
		MU_sendMesgToFW(dev->mu_base_virtaddr, PRINT_BUF_OFFSET, dev->m0_rpc_phy - dev->m0_p_fw_space_phy + M0_PRINT_OFFSET);
		MU_sendMesgToFW(dev->mu_base_virtaddr, RPC_BUF_OFFSET, dev->m0_rpc_phy - dev->m0_p_fw_space_phy); //CM0 use relative address
		MU_sendMesgToFW(dev->mu_base_virtaddr, BOOT_ADDRESS, dev->m0_p_fw_space_phy);
#endif
		MU_sendMesgToFW(dev->mu_base_virtaddr, INIT_DONE, 2);
	} else
		schedule_work(&dev->msg_work);
	return IRQ_HANDLED;
}

/* Initialization of the MU code. */
static int vpu_mu_init(struct vpu_dev *dev)
{
	struct device_node *np;
	unsigned int	vpu_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the M0 core
	 */
#ifdef CM4
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu0-vpu-m4");
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
#else
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu1-vpu-m0");
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
#endif
	dev->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dev->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,vpu_ap_mu_id", 0, &vpu_mu_id);
	if (ret) {
		vpu_dbg(LVL_ERR, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dev->vpu_mu_id = vpu_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(&dev->plat_dev->dev, irq, fsl_vpu_mu_isr,
				IRQF_EARLY_RESUME, "vpu_mu_isr", (void *)dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "request_irq failed %d, error = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!dev->vpu_mu_init) {
		MU_Init(dev->mu_base_virtaddr);
		MU_EnableRxFullInt(dev->mu_base_virtaddr, 0);
		dev->vpu_mu_init = 1;
	}

	return ret;
}

static int vpu_next_free_instance(struct vpu_dev *dev)
{
	int idx = ffz(dev->instance_mask);

	if (idx < 0 || idx > VPU_MAX_NUM_STREAMS)
		return -EBUSY;

	return idx;
}

extern u_int32 rpc_MediaIPFW_Video_message_check_encoder(struct shared_addr *This);
static void vpu_msg_run_work(struct work_struct *work)
{
	struct vpu_dev *dev = container_of(work, struct vpu_dev, msg_work);
	struct vpu_ctx *ctx;
	struct event_msg msg;
	struct shared_addr *This = &dev->shared_mem;

	while (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_AVAILABLE) {
		rpc_receive_msg_buf_encoder(This, &msg);
		ctx = dev->ctx[msg.idx];
		vpu_api_event_handler(ctx, msg.idx, msg.msgid, msg.msgdata);
	}
	if (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_BUFFER_ERROR)
		vpu_dbg(LVL_ERR, "MSG num is too big to handle");

}

static int vpu_queue_setup(struct vb2_queue *vq,
		unsigned int *buf_count,
		unsigned int *plane_count,
		unsigned int psize[],
		struct device *allocators[])
{
	struct queue_data  *This = (struct queue_data *)vq->drv_priv;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	if ((vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ||
		(vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		) {
			*plane_count = 1;
			psize[0] = This->sizeimage[0];//check alignment
	} else {
		*plane_count = 1;
		psize[0] = This->sizeimage[0] + This->sizeimage[1];
	}
	return 0;
}

static int vpu_buf_prepare(struct vb2_buffer *vb)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
	return 0;
}


static int vpu_start_streaming(struct vb2_queue *q,
		unsigned int count
		)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
	return 0;
}


static void vpu_stop_streaming(struct vb2_queue *q)
{
	struct queue_data *This = (struct queue_data *)q->drv_priv;
	struct vb2_data_req *p_data_req;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
			vpu_dbg(LVL_INFO, "%s(%d) - list_del(%p)\n",
					__func__,
					p_data_req->id,
					p_data_req
					);
			list_del(&p_data_req->list);
		}
	}
	if (!list_empty(&q->queued_list))
		list_for_each_entry(vb, &q->queued_list, queued_entry)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	INIT_LIST_HEAD(&This->drv_q);
	up(&This->drv_q_lock);
}

static void vpu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *This = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *data_req;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	down(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "c_port_buf_queue down\n");
	data_req = &This->vb2_reqs[vb->index];
	data_req->vb2_buf = vb;
	data_req->id = vb->index;
	list_add_tail(&data_req->list, &This->drv_q);

	up(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "c_port_buf_queue up vq->type=%d\n", vq->type);

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		v4l2_transfer_buffer_to_firmware(This, vb);
}

static void vpu_prepare(struct vb2_queue *q)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
}

static void vpu_finish(struct vb2_queue *q)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
}

static struct vb2_ops v4l2_qops = {
	.queue_setup        = vpu_queue_setup,
	.wait_prepare       = vpu_prepare,
	.wait_finish        = vpu_finish,
	.buf_prepare        = vpu_buf_prepare,
	.start_streaming    = vpu_start_streaming,
	.stop_streaming     = vpu_stop_streaming,
	.buf_queue          = vpu_buf_queue,
};

static void init_vb2_queue(struct queue_data *This, unsigned int type, struct vpu_ctx *ctx)
{
	struct vb2_queue  *vb2_q = &This->vb2_q;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	// initialze driver queue
	INIT_LIST_HEAD(&This->drv_q);
	// initialize vb2 queue
	vb2_q->type = type;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vb2_q->gfp_flags = GFP_DMA32;
	vb2_q->ops = &v4l2_qops;
	vb2_q->drv_priv = This;
	vb2_q->mem_ops = (struct vb2_mem_ops *)&vb2_dma_contig_memops;
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_dbg(LVL_ERR, "%s vb2_queue_init() failed (%d)!\n",
				__func__,
				ret
				);
	else
		This->vb2_q_inited = true;
}

static void init_queue_data(struct vpu_ctx *ctx)
{
	init_vb2_queue(&ctx->q_data[V4L2_SRC], V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, ctx);
	ctx->q_data[V4L2_SRC].type = V4L2_SRC;
	sema_init(&ctx->q_data[V4L2_SRC].drv_q_lock, 1);
	init_vb2_queue(&ctx->q_data[V4L2_DST], V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, ctx);
	ctx->q_data[V4L2_DST].type = V4L2_DST;
	sema_init(&ctx->q_data[V4L2_DST].drv_q_lock, 1);
}

static void release_queue_data(struct vpu_ctx *ctx)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	if (This->vb2_q_inited)
		vb2_queue_release(&This->vb2_q);
	This = &ctx->q_data[V4L2_DST];
	if (This->vb2_q_inited)
		vb2_queue_release(&This->vb2_q);
}

#ifdef CM4
static int power_CM4_up(struct vpu_dev *dev)
{
	sc_ipc_t ipcHndl;
	sc_rsrc_t core_rsrc, mu_rsrc = -1;

	ipcHndl = dev->mu_ipcHandle;
	core_rsrc = SC_R_M4_0_PID0;
	mu_rsrc = SC_R_M4_0_MU_1A;

	if (sc_pm_set_resource_power_mode(ipcHndl, core_rsrc, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: failed to power up core_rsrc\n");
		return -EIO;
	}

	if (mu_rsrc != -1) {
		if (sc_pm_set_resource_power_mode(ipcHndl, mu_rsrc, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
			vpu_dbg(LVL_ERR, "error: failed to power up mu_rsrc\n");
			return -EIO;
		}
	}

	return 0;
}

static int boot_CM4_up(struct vpu_dev *dev, void *boot_addr)
{
	sc_ipc_t ipcHndl;
	sc_rsrc_t core_rsrc;
	sc_faddr_t aux_core_ram;
	void *core_ram_vir;
	u32 size;

	ipcHndl = dev->mu_ipcHandle;
	core_rsrc = SC_R_M4_0_PID0;
	aux_core_ram = 0x34FE0000;
	size = SZ_128K;

	core_ram_vir = ioremap(aux_core_ram,
			size
			);
	if (!core_ram_vir)
		vpu_dbg(LVL_ERR, "error: failed to remap space for core ram\n");

	memcpy((void *)core_ram_vir, (void *)boot_addr, size);

	if (sc_pm_cpu_start(ipcHndl, core_rsrc, true, aux_core_ram) != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: failed to start core_rsrc\n");
		return -EIO;
	}

	return 0;
}
#endif

static int vpu_firmware_download(struct vpu_dev *This)
{
	unsigned char *image;
	unsigned int FW_Size = 0;
	void *csr_offset, *csr_cpuwait;
	int ret = 0;

	ret = request_firmware((const struct firmware **)&This->m0_pfw,
			M0FW_FILENAME,
			This->generic_dev
			);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		if (This->m0_pfw) {
			release_firmware(This->m0_pfw);
			This->m0_pfw = NULL;
		}
		return ret;
	} else {
		vpu_dbg(LVL_INFO, "%s() request fw %s got size(%d)\n",
			__func__, M0FW_FILENAME, (int)This->m0_pfw->size);
		image = (uint8_t *)This->m0_pfw->data;
		FW_Size = This->m0_pfw->size;
	}
	memcpy(This->m0_p_fw_space_vir,
			image,
			FW_Size
			);
#ifdef CM4
	boot_CM4_up(This, This->m0_p_fw_space_vir);
#else
	csr_offset = ioremap(0x2d050000, 4);
	writel(This->m0_p_fw_space_phy, csr_offset);
	csr_cpuwait = ioremap(0x2d050004, 4);
	writel(0x0, csr_cpuwait);
#endif
	return ret;
}

static int v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int idx;
	int ret;
	u_int32 i;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	idx = vpu_next_free_instance(dev);
	if (idx < 0) {
		ret = idx;
		return ret;
	}
	set_bit(idx, &dev->instance_mask);
	init_completion(&ctx->completion);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->str_index = idx;
	ctx->dev = dev;
	ctx->ctrl_inited = false;
	ctrls_setup_encoder(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	dev->ctx[idx] = ctx;
	ctx->b_firstseq = true;
	ctx->start_flag = true;
	init_queue_data(ctx);
	init_waitqueue_head(&ctx->buffer_wq_output);
	init_waitqueue_head(&ctx->buffer_wq_input);
	mutex_lock(&dev->dev_mutex);
	if (!dev->fw_is_ready) {
		ret = vpu_firmware_download(dev);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: vpu_firmware_download fail\n");
			mutex_unlock(&dev->dev_mutex);
			return ret;
		}
		dev->fw_is_ready = true;
	}
	mutex_unlock(&dev->dev_mutex);
	ctx->encoder_stream.size = STREAM_SIZE;
	ctx->encoder_stream.virt_addr = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			ctx->encoder_stream.size,
			(dma_addr_t *)&ctx->encoder_stream.phy_addr,
			GFP_KERNEL | GFP_DMA32
			);

	if (!ctx->encoder_stream.virt_addr)
		vpu_dbg(LVL_ERR, "%s() encoder stream buffer alloc size(%x) fail!\n", __func__, ctx->encoder_stream.size);
	else
		vpu_dbg(LVL_INFO, "%s() encoder_stream_size(%d) encoder_stream_virt(%p) encoder_stream_phy(%p)\n", __func__, ctx->encoder_stream.size, ctx->encoder_stream.virt_addr, (void *)ctx->encoder_stream.phy_addr);

	ctx->encoder_mem.size = 0;
	ctx->encoder_mem.virt_addr = NULL;
	ctx->encoder_mem.phy_addr = 0;

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		ctx->encFrame[i].virt_addr = NULL;
		ctx->encFrame[i].phy_addr = 0;
		ctx->encFrame[i].size = 0;
	}

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		ctx->refFrame[i].virt_addr = NULL;
		ctx->refFrame[i].phy_addr = 0;
		ctx->refFrame[i].size = 0;
	}
	ctx->actFrame.virt_addr = NULL;
	ctx->actFrame.phy_addr = 0;
	ctx->actFrame.size = 0;

	return 0;
}

static int v4l2_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	u_int32 i;

	release_queue_data(ctx);
	ctrls_delete_encoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	clear_bit(ctx->str_index, &dev->instance_mask);

	dma_free_coherent(&ctx->dev->plat_dev->dev,
			ctx->encoder_stream.size,
			ctx->encoder_stream.virt_addr,
			ctx->encoder_stream.phy_addr
			);
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++)
		if (ctx->encFrame[i].virt_addr != NULL)
			dma_free_coherent(&ctx->dev->plat_dev->dev,
					ctx->encFrame[i].size,
					ctx->encFrame[i].virt_addr,
					ctx->encFrame[i].phy_addr
					);
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++)
		if (ctx->refFrame[i].virt_addr != NULL)
			dma_free_coherent(&ctx->dev->plat_dev->dev,
					ctx->refFrame[i].size,
					ctx->refFrame[i].virt_addr,
					ctx->refFrame[i].phy_addr
					);
	if (ctx->actFrame.virt_addr != NULL)
		dma_free_coherent(&ctx->dev->plat_dev->dev,
				ctx->actFrame.size,
				ctx->actFrame.virt_addr,
				ctx->actFrame.phy_addr
				);
	kfree(ctx);
	return 0;
}

static unsigned int v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (ctx) {
		poll_wait(filp, &ctx->fh.wait, wait);

		if (v4l2_event_pending(&ctx->fh)) {
			vpu_dbg(LVL_INFO, "%s() v4l2_event_pending\n", __func__);
			rc |= POLLPRI;
		}

		src_q = &ctx->q_data[V4L2_SRC].vb2_q;
		dst_q = &ctx->q_data[V4L2_DST].vb2_q;

		if ((!src_q->streaming || list_empty(&src_q->queued_list))
				&& (!dst_q->streaming || list_empty(&dst_q->queued_list))) {
			return rc;
		}

		poll_wait(filp, &src_q->done_wq, wait);
		if (!list_empty(&src_q->done_list))
			rc |= POLLOUT | POLLWRNORM;
		poll_wait(filp, &dst_q->done_wq, wait);
		if (!list_empty(&dst_q->done_list))
			rc |= POLLIN | POLLWRNORM;
	} else
		rc = POLLERR;

	return rc;
}

static int v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
{
	long ret = -EPERM;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct queue_data *q_data;
	enum QUEUE_TYPE type;

	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (ctx) {
		type = offset >> MMAP_BUF_TYPE_SHIFT;
		q_data = &ctx->q_data[type];

		offset &= ~MMAP_BUF_TYPE_MASK;
		offset = offset >> PAGE_SHIFT;
		vma->vm_pgoff = offset;
		ret = vb2_mmap(&q_data->vb2_q,
						vma
						);
	}

	return ret;
}

static const struct v4l2_file_operations v4l2_encoder_fops = {
	.owner = THIS_MODULE,
	.open  = v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = v4l2_release,
	.poll = v4l2_poll,
	.mmap = v4l2_mmap,
};

static struct video_device v4l2_videodevice_encoder = {
	.name   = "vpu encoder",
	.fops   = &v4l2_encoder_fops,
	.ioctl_ops = &v4l2_encoder_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
};

static int set_vpu_pwr(sc_ipc_t ipcHndl,
		sc_pm_power_mode_t pm
		)
{
	int rv = -1;
	sc_err_t sciErr;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	if (!ipcHndl) {
		vpu_dbg(LVL_ERR, "--- set_vpu_pwr no IPC handle\n");
		goto set_vpu_pwrexit;
	}

	// Power on or off PID0, ENC
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID0, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID0,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID1, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID1,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID2, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID2,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID3, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID3,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID4, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID4,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID5, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID5,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID6, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID6,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_PID7, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_PID7,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#ifdef TEST_BUILD
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_ENC,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#else
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC_0, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "--- sc_pm_set_resource_power_mode(SC_R_VPU_ENC_0,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#endif

	rv = 0;

set_vpu_pwrexit:
	return rv;
}

static void vpu_set_power(struct vpu_dev *dev, bool on)
{
	int ret;

	if (on) {
		ret = set_vpu_pwr(dev->mu_ipcHandle, SC_PM_PW_MODE_ON);
		if (ret)
			vpu_dbg(LVL_ERR, "failed to power on\n");
		pm_runtime_get_sync(dev->generic_dev);
	} else {
		pm_runtime_put_sync_suspend(dev->generic_dev);
		ret = set_vpu_pwr(dev->mu_ipcHandle, SC_PM_PW_MODE_OFF);
		if (ret)
			vpu_dbg(LVL_ERR, "failed to power off\n");
	}
}

static void vpu_setup(struct vpu_dev *This)
{
	uint32_t read_data = 0;

	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x1, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + 0x70190);
	writel(0xffffffff, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

	writel(0xE, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);

	writel(0x1f, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_SET);

	writel(0x102, This->regs_base + XMEM_CONTROL);

	read_data = readl(This->regs_base+0x70108);
	vpu_dbg(LVL_IRQ, "%s read_data=%x\n", __func__, read_data);
}

static void vpu_reset(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_CLR);
}

static int vpu_enable_hw(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	vpu_set_power(This, true);
	This->vpu_clk = clk_get(&This->plat_dev->dev, "vpu_encoder_clk");
	if (IS_ERR(This->vpu_clk)) {
		vpu_dbg(LVL_ERR, "vpu_clk get error\n");
		return -ENOENT;
	}
	clk_set_rate(This->vpu_clk, 600000000);
	clk_prepare_enable(This->vpu_clk);
	vpu_setup(This);
	return 0;
}

static void vpu_disable_hw(struct vpu_dev *This)
{
	vpu_reset(This);
	if (This->regs_base) {
		devm_iounmap(&This->plat_dev->dev,
				This->regs_base);
	}
	clk_disable_unprepare(This->vpu_clk);
	if (This->vpu_clk) {
		clk_put(This->vpu_clk);
		This->vpu_clk = NULL;
	}
	vpu_set_power(This, false);
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *reserved_node;
	struct resource reserved_res;
	unsigned int mu_id;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->plat_dev = pdev;
	dev->generic_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs_base = ioremap(ENC_REG_BASE, 0x1000000);
	if (!dev->regs_base) {
		vpu_dbg(LVL_ERR, "%s could not map regs_base\n", __func__);
		return PTR_ERR(dev->regs_base);
	}

	if (np) {
		reserved_node = of_parse_phandle(np, "boot-region", 0);
		if (!reserved_node) {
			vpu_dbg(LVL_ERR, "error: boot-region of_parse_phandle error\n");
			return -ENODEV;
		}

		if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
			vpu_dbg(LVL_ERR, "error: boot-region of_address_to_resource error\n");
			return -EINVAL;
		}
		dev->m0_p_fw_space_phy = reserved_res.start;
		reserved_node = of_parse_phandle(np, "rpc-region", 0);
		if (!reserved_node) {
			vpu_dbg(LVL_ERR, "error: rpc-region of_parse_phandle error\n");
			return -ENODEV;
		}

		if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
			vpu_dbg(LVL_ERR, "error: rpc-region of_address_to_resource error\n");
			return -EINVAL;
		}
		dev->m0_rpc_phy = reserved_res.start;
	} else
		vpu_dbg(LVL_ERR, "error: %s of_node is NULL\n", __func__);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s unable to register v4l2 dev\n", __func__);
		return ret;
	}

	platform_set_drvdata(pdev, dev);

	dev->pvpu_encoder_dev = video_device_alloc();
	if (dev->pvpu_encoder_dev) {
		strncpy(dev->pvpu_encoder_dev->name, v4l2_videodevice_encoder.name, sizeof(v4l2_videodevice_encoder.name));
		dev->pvpu_encoder_dev->fops = v4l2_videodevice_encoder.fops;
		dev->pvpu_encoder_dev->ioctl_ops = v4l2_videodevice_encoder.ioctl_ops;
		dev->pvpu_encoder_dev->release = video_device_release;
		dev->pvpu_encoder_dev->vfl_dir = v4l2_videodevice_encoder.vfl_dir;
		dev->pvpu_encoder_dev->v4l2_dev = &dev->v4l2_dev;

		video_set_drvdata(dev->pvpu_encoder_dev, dev);

		if (video_register_device(dev->pvpu_encoder_dev,
					VFL_TYPE_GRABBER,
					-1)) {
			vpu_dbg(LVL_ERR, "%s unable to register video encoder device\n",
					__func__
					);
			video_device_release(dev->pvpu_encoder_dev);
			dev->pvpu_encoder_dev = NULL;
		} else {
			vpu_dbg(LVL_INFO, "%s  register video encoder device\n",
					__func__
				   );
		}
	}

	if (!dev->mu_ipcHandle) {
		ret = sc_ipc_getMuID(&mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "--- sc_ipc_getMuID() cannot obtain mu id SCI error! (%d)\n", ret);
			return ret;
		}

		ret = sc_ipc_open(&dev->mu_ipcHandle, mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "--- sc_ipc_getMuID() cannot open MU channel to SCU error! (%d)\n", ret);
			return ret;
		}
	}

	vpu_enable_hw(dev);

	mutex_init(&dev->dev_mutex);
	dev->fw_is_ready = false;
	dev->workqueue = alloc_workqueue("vpu", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		vpu_dbg(LVL_ERR, "%s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	INIT_WORK(&dev->msg_work, vpu_msg_run_work);
#ifdef CM4
	ret = power_CM4_up(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: failed to power on CM4\n");
		return ret;
	}
#endif
	ret = vpu_mu_init(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s vpu mu init failed\n", __func__);
		return ret;
	}
	//firmware space for M0
	dev->m0_p_fw_space_vir = ioremap(dev->m0_p_fw_space_phy,
			M0_BOOT_SIZE
			);
	if (!dev->m0_p_fw_space_vir)
		vpu_dbg(LVL_ERR, "failed to remap space for M0 firmware\n");

	memset_io(dev->m0_p_fw_space_vir, 0, M0_BOOT_SIZE);

	dev->m0_rpc_virt = ioremap(dev->m0_rpc_phy,
			SHARED_SIZE
			);
	if (!dev->m0_rpc_virt)
		vpu_dbg(LVL_ERR, "failed to remap space for shared memory\n");

	memset_io(dev->m0_rpc_virt, 0, SHARED_SIZE);

#ifdef CM4
	rpc_init_shared_memory_encoder(&dev->shared_mem, dev->m0_rpc_phy, dev->m0_rpc_virt, SHARED_SIZE);
#else
	rpc_init_shared_memory_encoder(&dev->shared_mem, dev->m0_rpc_phy - dev->m0_p_fw_space_phy, dev->m0_rpc_virt, SHARED_SIZE);
#endif
	rpc_set_system_cfg_value_encoder(dev->shared_mem.pSharedInterface, VPU_REG_BASE);
	return 0;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev *dev = platform_get_drvdata(pdev);

	destroy_workqueue(dev->workqueue);
	if (dev->m0_p_fw_space_vir)
		iounmap(dev->m0_p_fw_space_vir);
	if (dev->m0_pfw) {
		release_firmware(dev->m0_pfw);
		dev->m0_pfw = NULL;
	}
	dev->m0_p_fw_space_vir = NULL;
	dev->m0_p_fw_space_phy = 0;
	if (dev->shared_mem.shared_mem_vir)
		iounmap(dev->shared_mem.shared_mem_vir);
	dev->shared_mem.shared_mem_vir = NULL;
	dev->shared_mem.shared_mem_phy = 0;

	vpu_disable_hw(dev);

	if (video_get_drvdata(dev->pvpu_encoder_dev))
		video_unregister_device(dev->pvpu_encoder_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
	return 0;
}

static int vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int vpu_runtime_resume(struct device *dev)
{
	return 0;
}

static int vpu_suspend(struct device *dev)
{
	return 0;
}

static int vpu_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "nxp,imx8qm-vpu-encoder", },
	{ .compatible = "nxp,imx8qxp-vpu-encoder", },
	{}
}
MODULE_DEVICE_TABLE(of, vpu_of_match);

static struct platform_driver vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.driver = {
		.name = "vpu-b0-encoder",
		.of_match_table = vpu_of_match,
		.pm = &vpu_pm_ops,
	},
};
module_platform_driver(vpu_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_param(vpu_dbg_level_encoder, int, 0644);
MODULE_PARM_DESC(vpu_dbg_level_encoder, "Debug level (0-2)");

