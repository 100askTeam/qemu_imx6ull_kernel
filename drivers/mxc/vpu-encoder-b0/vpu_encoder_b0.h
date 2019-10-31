/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file vpu_encoder_b0.h
 *
 * @brief VPU ENCODER B0 definition
 *
 */
#ifndef __VPU_ENCODER_B0_H__
#define __VPU_ENCODER_B0_H__

#include <linux/irqreturn.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#include <linux/mx8_mu.h>
#include <media/v4l2-event.h>
#include "vpu_encoder_rpc.h"

extern unsigned int vpu_dbg_level_encoder;

#define v4l2_fh_to_ctx(__fh) \
	container_of(__fh, struct vpu_ctx, fh)
#define v4l2_ctrl_to_ctx(__ctrl) \
	container_of((__ctrl)->handler, struct vpu_ctx, ctrl_handler)

#define MIN_SPACE 2048

#define VPU_MAX_FORMATS 4
#define VPU_MAX_BUFFER 32
#define M0FW_FILENAME "encoder_main.bin"
#define MMAP_BUF_TYPE_SHIFT 28
#define MMAP_BUF_TYPE_MASK 0xF0000000
#define M0_BOOT_SIZE 0x1000000
#define M0_PRINT_OFFSET 0x500000
#define SHARED_SIZE 0x00400000
#define MEM_SIZE  0x2800000
#define YUV_SIZE  0x4000000
#define STREAM_SIZE 0x600000
#ifdef CM4
#define VPU_REG_BASE 0x2c000000
#else
#define VPU_REG_BASE 0x40000000
#endif
#define ENC_REG_BASE 0x2c000000

#define V4L2_MAX_CTRLS 12
struct vpu_v4l2_control {
	uint32_t id;
	enum v4l2_ctrl_type type;
	uint32_t minimum;
	uint32_t maximum;
	uint32_t step;
	uint32_t default_value;
	uint32_t menu_skip_mask;
	bool is_volatile;
};

typedef enum{
	INIT_DONE = 1,
	RPC_BUF_OFFSET,
	PRINT_BUF_OFFSET,
	BOOT_ADDRESS,
	COMMAND,
	EVENT
} MSG_Type;

enum QUEUE_TYPE {
	V4L2_SRC = 0,
	V4L2_DST = 1,
};

enum vpu_video_standard {
	VPU_VIDEO_UNDEFINED = 0,
	VPU_VIDEO_AVC = 1,
	VPU_VIDEO_VC1 = 2,
	VPU_VIDEO_MPEG2 = 3,
	VPU_VIDEO_AVS = 4,
	VPU_VIDEO_ASP = 5,
	VPU_VIDEO_JPEG = 6,
	VPU_VIDEO_RV8 = 7,
	VPU_VIDEO_RV9 = 8,
	VPU_VIDEO_VP6 = 9,
	VPU_VIDEO_SPK = 10,
	VPU_VIDEO_VP8 = 11,
	VPU_VIDEO_AVC_MVC = 12,
	VPU_VIDEO_HEVC = 13,
	VPU_VIDEO_VP9 = 14,
};

#define VPU_PIX_FMT_AVS         v4l2_fourcc('A', 'V', 'S', '0')
#define VPU_PIX_FMT_ASP         v4l2_fourcc('A', 'S', 'P', '0')
#define VPU_PIX_FMT_RV8         v4l2_fourcc('R', 'V', '8', '0')
#define VPU_PIX_FMT_RV9         v4l2_fourcc('R', 'V', '9', '0')
#define VPU_PIX_FMT_VP6         v4l2_fourcc('V', 'P', '6', '0')
#define VPU_PIX_FMT_SPK         v4l2_fourcc('S', 'P', 'K', '0')
#define VPU_PIX_FMT_HEVC        v4l2_fourcc('H', 'E', 'V', 'C')
#define VPU_PIX_FMT_VP9         v4l2_fourcc('V', 'P', '9', '0')
#define VPU_PIX_FMT_LOGO        v4l2_fourcc('L', 'O', 'G', 'O')

#define VPU_PIX_FMT_TILED_8     v4l2_fourcc('Z', 'T', '0', '8')
#define VPU_PIX_FMT_TILED_10    v4l2_fourcc('Z', 'T', '1', '0')

enum vpu_pixel_format {
	VPU_HAS_COLOCATED = 0x00000001,
	VPU_HAS_SPLIT_FLD = 0x00000002,
	VPU_PF_MASK       = ~(VPU_HAS_COLOCATED | VPU_HAS_SPLIT_FLD),

	VPU_IS_TILED      = 0x000000100,
	VPU_HAS_10BPP     = 0x00000200,

	VPU_IS_PLANAR     = 0x00001000,
	VPU_IS_SEMIPLANAR = 0x00002000,
	VPU_IS_PACKED     = 0x00004000,

	// Merged definitions using above flags:
	VPU_PF_UNDEFINED  = 0,
	VPU_PF_YUV420_SEMIPLANAR = 0x00010000 | VPU_IS_SEMIPLANAR,
	VPU_PF_YUV420_PLANAR = 0x00020000 | VPU_IS_PLANAR,
	VPU_PF_UYVY = 0x00040000 | VPU_IS_PACKED,
	VPU_PF_TILED_8BPP = 0x00080000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR,
	VPU_PF_TILED_10BPP = 0x00100000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR | VPU_HAS_10BPP,
};

struct vpu_v4l2_fmt {
	char *name;
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned int venc_std;
};

struct vb2_data_req {
	struct list_head  list;
	struct vb2_buffer *vb2_buf;
	int id;
};

struct queue_data {
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int sizeimage[2];
	unsigned int fourcc;
	unsigned int vdec_std;
	struct v4l2_rect rect;
	int buf_type; // v4l2_buf_type
	bool vb2_q_inited;
	struct vb2_queue vb2_q;    // vb2 queue
	struct list_head drv_q;    // driver queue
	struct semaphore drv_q_lock;
	struct vb2_data_req vb2_reqs[VPU_MAX_BUFFER];
	enum QUEUE_TYPE type;
};
struct vpu_ctx;
struct vpu_dev {
	struct device *generic_dev;
	struct v4l2_device v4l2_dev;
	struct video_device *pvpu_encoder_dev;
	struct platform_device *plat_dev;
	struct firmware *m0_pfw;
	void *m0_p_fw_space_vir;
	u_int32 m0_p_fw_space_phy;
	void *m0_rpc_virt;
	u_int32 m0_rpc_phy;
	struct mutex dev_mutex;
	bool fw_is_ready;
	struct completion msg_complete;
	struct workqueue_struct *workqueue;
	struct work_struct msg_work;
	unsigned long instance_mask;
	sc_ipc_t mu_ipcHandle;
	struct clk *vpu_clk;
	void __iomem *mu_base_virtaddr;
	unsigned int vpu_mu_id;
	int vpu_mu_init;

	struct clk *clk_m0;
	void __iomem *regs_base;

	struct shared_addr shared_mem;
	struct vpu_ctx *ctx[VPU_MAX_NUM_STREAMS];
};

struct buffer_addr {
	void *virt_addr;
	dma_addr_t phy_addr;
	u_int32 size;
};

struct vpu_ctx {
	struct vpu_dev *dev;
	struct v4l2_fh fh;

	struct v4l2_ctrl *ctrls[V4L2_MAX_CTRLS];
	struct v4l2_ctrl_handler ctrl_handler;
	bool ctrl_inited;

	int str_index;
	struct queue_data q_data[2];
//	struct work_struct msg_work;
	struct completion completion;
	bool b_firstseq;
	bool start_flag;
	wait_queue_head_t buffer_wq_output;
	wait_queue_head_t buffer_wq_input;
	struct buffer_addr encoder_buffer;
	struct buffer_addr encoder_stream;
	struct buffer_addr encoder_mem;
	struct buffer_addr encFrame[MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES];
	struct buffer_addr refFrame[MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES];
	struct buffer_addr actFrame;

};

#define LVL_INFO 3
#define LVL_ERR  2
#define LVL_IRQ  1
#define LVL_ALL  0

#define vpu_dbg(level, fmt, arg...) \
	do { \
		if (vpu_dbg_level_encoder >= (level)) \
			printk("[DEBUG]\t " fmt, ## arg); \
	} while (0)

#endif
