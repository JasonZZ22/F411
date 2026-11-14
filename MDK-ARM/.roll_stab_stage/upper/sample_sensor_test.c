
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sys/wait.h>
#include <inttypes.h>

#include <fcntl.h>		/* low-level i/o */
#include "cvi_buffer.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_comm_isp.h"
#include "cvi_comm_sns.h"
#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_isp.h"
#include "cvi_sns_ctrl.h"
#include "sample_comm.h"
#include "ae_test.h"

static SAMPLE_VI_CONFIG_S g_stViConfig;
static SAMPLE_INI_CFG_S g_stIniCfg;

static void sys_handle_signal(int nSignal, siginfo_t *si, void *arg)
{
	UNUSED(nSignal);
	UNUSED(si);
	UNUSED(arg);

	if (g_stViConfig.s32WorkingViNum != 0) {
		SAMPLE_COMM_VI_DestroyIsp(&g_stViConfig);
		SAMPLE_COMM_VI_DestroyVi(&g_stViConfig);
	}
	SAMPLE_COMM_SYS_Exit();
	exit(1);
}

static int sys_vi_init(void)
{
	MMF_VERSION_S stVersion;
	SAMPLE_INI_CFG_S	   stIniCfg;
	SAMPLE_VI_CONFIG_S stViConfig;

	PIC_SIZE_E enPicSize;
	SIZE_S stSize;
	CVI_S32 s32Ret = CVI_SUCCESS;
	LOG_LEVEL_CONF_S log_conf;
	CVI_U32 Vb_cnt;
	VB_CONFIG_S stVbConf;
	CVI_U32 u32BlkSize, u32BlkRotSize;

	CVI_SYS_GetVersion(&stVersion);
	SAMPLE_PRT("MMF Version:%s\n", stVersion.version);

	log_conf.enModId = CVI_ID_LOG;
	log_conf.s32Level = CVI_DBG_INFO;
	CVI_LOG_SetLevelConf(&log_conf);

	// Get config from ini if found.
	s32Ret = SAMPLE_COMM_VI_ParseIni(&stIniCfg);
	if (s32Ret != CVI_SUCCESS) {
		SAMPLE_PRT("Parse fail\n");
	} else {
		SAMPLE_PRT("Parse complete\n");
	}

	//Set sensor number
	CVI_VI_SetDevNum(stIniCfg.devNum);
	/************************************************
	 * step1:  Config VI
	 ************************************************/
	s32Ret = SAMPLE_COMM_VI_IniToViCfg(&stIniCfg, &stViConfig);
	if (s32Ret != CVI_SUCCESS)
		return s32Ret;

	memcpy(&g_stViConfig, &stViConfig, sizeof(SAMPLE_VI_CONFIG_S));
	memcpy(&g_stIniCfg, &stIniCfg, sizeof(SAMPLE_INI_CFG_S));

	/************************************************
	 * step2:  Get input size
	 ************************************************/
	memset(&stVbConf, 0, sizeof(VB_CONFIG_S));
	stVbConf.u32MaxPoolCnt = 0;

	for (CVI_S32 i = 0; i < stViConfig.s32WorkingViNum; i++) {
		Vb_cnt = 3;
		bool createNewPool = true;

		s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stIniCfg.enSnsType[i], &enPicSize);
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_VI_GetSizeBySensor failed with %#x\n", s32Ret);
			return s32Ret;
		}

		s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "SAMPLE_COMM_SYS_GetPicSize failed with %#x\n", s32Ret);
			return s32Ret;
		}

		u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height,
					stViConfig.astViInfo[i].stChnInfo.enPixFormat,
					DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
		u32BlkRotSize = COMMON_GetPicBufferSize(stSize.u32Height, stSize.u32Width,
					stViConfig.astViInfo[i].stChnInfo.enPixFormat,
					DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
		u32BlkSize = u32BlkSize > u32BlkRotSize ? u32BlkSize : u32BlkRotSize;

		for (CVI_U32 j = 0; j < stVbConf.u32MaxPoolCnt; j++) {
			if (stVbConf.astCommPool[j].u32BlkSize == u32BlkSize) {
				stVbConf.astCommPool[j].u32BlkCnt += Vb_cnt;
				createNewPool = false;
				break;
			}
		}
		if (createNewPool) {
			stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkSize = u32BlkSize;
			stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkCnt = Vb_cnt;
			stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].enRemapMode = VB_REMAP_MODE_CACHED;
			SAMPLE_PRT("set VBpool [%d] %d:%d, BlkCnt= %d, Size = %d\n",
						stVbConf.u32MaxPoolCnt, stSize.u32Width, stSize.u32Height,
						stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkCnt,
						stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkSize);
			stVbConf.u32MaxPoolCnt++;
		} else {
			SAMPLE_PRT("set VBpool [%d] %d:%d, BlkCnt= %d, Size = %d\n",
						stVbConf.u32MaxPoolCnt, stSize.u32Width, stSize.u32Height,
						stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkCnt,
						stVbConf.astCommPool[stVbConf.u32MaxPoolCnt].u32BlkSize);
		}
	}

	if (stVbConf.u32MaxPoolCnt == 1) {
		stVbConf.astCommPool[0].u32BlkCnt += 2;
	}

	/************************************************
	 * step3:  Init modules
	 ************************************************/
	struct sigaction sa;

	memset(&sa, 0, sizeof(struct sigaction));
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = sys_handle_signal;
	sa.sa_flags = SA_SIGINFO|SA_RESETHAND; // Reset signal handler to system default after signal triggered
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	SAMPLE_PRT(">>> STEP sys_init, maxPool=%u\n", stVbConf.u32MaxPoolCnt);
	s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
	SAMPLE_PRT(">>> STEP sys_init ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "system init failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	SAMPLE_PRT(">>> STEP plat_vi_init\n");
	s32Ret = SAMPLE_PLAT_VI_INIT(&stViConfig);
	SAMPLE_PRT(">>> STEP plat_vi_init ret=0x%x\n", s32Ret);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "vi init failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	return CVI_SUCCESS;
}

extern void callback_FPS(int fps);
static int g_stream_fd = -1;
static volatile sig_atomic_t g_stop = 0;
static void trk_on_sig(int sig) { (void)sig; g_stop = 1; }

/* 把一帧 NV12 写到独立 fd，供 SSH 管道实时推流 */
static CVI_S32 _vi_stream_once(CVI_U8 chn)
{
	VIDEO_FRAME_INFO_S stVideoFrame;
	CVI_S32 s32Ret;

	s32Ret = CVI_VI_GetChnFrame(0, chn, &stVideoFrame, 3000);
	if (s32Ret != 0)
		return s32Ret;

	size_t total = stVideoFrame.stVFrame.u32Length[0] +
		       stVideoFrame.stVFrame.u32Length[1] +
		       stVideoFrame.stVFrame.u32Length[2];
	CVI_VOID *vir = CVI_SYS_Mmap(stVideoFrame.stVFrame.u64PhyAddr[0], total);

	CVI_SYS_IonInvalidateCache(stVideoFrame.stVFrame.u64PhyAddr[0], vir, total);

	CVI_U32 off = 0;
	for (int i = 0; i < 3; i++) {
		if (stVideoFrame.stVFrame.u32Length[i] != 0) {
			size_t len = stVideoFrame.stVFrame.u32Length[i];
			size_t done = 0;
			while (done < len) {
				ssize_t n = write(g_stream_fd, (CVI_U8 *)vir + off + done, len - done);
				if (n <= 0) break;
				done += (size_t)n;
			}
			off += len;
		}
	}
	CVI_SYS_Munmap(vir, total);
	CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);
	return CVI_SUCCESS;
}

static VENC_PACK_S g_venc_packs[8];

static CVI_S32 venc_init(int w, int h, int fps, int kbps)
{
	VENC_CHN_ATTR_S a;
	CVI_S32 r;

	memset(&a, 0, sizeof(a));
	a.stVencAttr.enType          = PT_H264;
	a.stVencAttr.u32MaxPicWidth  = w;
	a.stVencAttr.u32MaxPicHeight = h;
	a.stVencAttr.u32PicWidth     = w;
	a.stVencAttr.u32PicHeight    = h;
	a.stVencAttr.u32BufSize      = w * h / 2;
	a.stVencAttr.u32Profile      = 0;          /* baseline */
	a.stVencAttr.bByFrame        = CVI_TRUE;

	a.stRcAttr.enRcMode                    = VENC_RC_MODE_H264CBR;
	a.stRcAttr.stH264Cbr.u32Gop            = fps;
	a.stRcAttr.stH264Cbr.u32StatTime       = 1;
	a.stRcAttr.stH264Cbr.u32SrcFrameRate   = fps;
	a.stRcAttr.stH264Cbr.fr32DstFrameRate  = fps;
	a.stRcAttr.stH264Cbr.u32BitRate        = kbps;

	a.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;

	r = CVI_VENC_CreateChn(0, &a);
	if (r != CVI_SUCCESS) { SAMPLE_PRT("VENC CreateChn failed 0x%x\n", r); return r; }

	VENC_RECV_PIC_PARAM_S rp = { .s32RecvPicNum = -1 };
	r = CVI_VENC_StartRecvFrame(0, &rp);
	if (r != CVI_SUCCESS) SAMPLE_PRT("VENC StartRecvFrame failed 0x%x\n", r);
	return r;
}

/* 抓一帧 -> 硬件 H.264 编码 -> 码流写到 fd */
static CVI_S32 _venc_stream_once(CVI_U8 chn)
{
	VIDEO_FRAME_INFO_S f;
	VENC_STREAM_S st;
	CVI_S32 r;

	static int dbg = 0;
	CVI_S32 g = CVI_VI_GetChnFrame(0, chn, &f, 3000);
	if (dbg < 5) SAMPLE_PRT("DBG getframe=0x%x fmt=%d %ux%u\n", g,
			f.stVFrame.enPixelFormat, f.stVFrame.u32Width, f.stVFrame.u32Height);
	if (g != CVI_SUCCESS) { dbg++; return CVI_FAILURE; }

	r = CVI_VENC_SendFrame(0, &f, 1000);
	CVI_VI_ReleaseChnFrame(0, chn, &f);
	if (dbg < 5) SAMPLE_PRT("DBG sendframe=0x%x\n", r);
	if (r != CVI_SUCCESS) { dbg++; return r; }

	memset(&st, 0, sizeof(st));
	st.pstPack = g_venc_packs;
	CVI_S32 gs = CVI_VENC_GetStream(0, &st, 1000);
	if (dbg < 5) SAMPLE_PRT("DBG getstream=0x%x packs=%u len0=%u\n", gs,
			st.u32PackCount, st.u32PackCount ? st.pstPack[0].u32Len : 0);
	dbg++;
	if (gs != CVI_SUCCESS) return CVI_FAILURE;

	for (CVI_U32 i = 0; i < st.u32PackCount; i++) {
		size_t len = st.pstPack[i].u32Len;
		size_t done = 0;
		CVI_U8 *p = st.pstPack[i].pu8Addr;
		while (done < len) {
			ssize_t n = write(g_stream_fd, p + done, len - done);
			if (n <= 0) break;
			done += (size_t)n;
		}
	}
	CVI_VENC_ReleaseStream(0, &st);
	return CVI_SUCCESS;
}


/* ==================== 绿色 LED 跟踪 ==================== */
#include <termios.h>
#include <fcntl.h>

#define TRK_W 640
#define TRK_H 400

typedef struct {
	int found;
	int x0, y0, x1, y1;   /* 包围盒 */
	int area;
	int cx, cy;           /* 质心 */
} trk_blob_t;

static const unsigned char *g_y;
static int g_ystride;

static trk_blob_t trk_find(int rx, int ry, int rw, int rh, int thresh, int step)
{
	trk_blob_t b;
	long sx = 0, sy = 0;
	int n = 0;
	int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;

	memset(&b, 0, sizeof(b));
	if (rx < 0) rx = 0;
	if (ry < 0) ry = 0;
	if (rx + rw > TRK_W) rw = TRK_W - rx;
	if (ry + rh > TRK_H) rh = TRK_H - ry;
	if (rw <= 0 || rh <= 0) return b;

	for (int y = ry; y < ry + rh; y += step) {
		const unsigned char *row = g_y + (size_t)y * g_ystride;
		for (int x = rx; x < rx + rw; x += step) {
			if (row[x] >= thresh) {
				n++; sx += x; sy += y;
				if (x < x0) x0 = x;
				if (x > x1) x1 = x;
				if (y < y0) y0 = y;
				if (y > y1) y1 = y;
			}
		}
	}
	if (n < 4) return b;

	b.found = 1;
	b.area  = n;
	b.x0 = x0; b.y0 = y0; b.x1 = x1; b.y1 = y1;
	b.cx = (int)(sx / n);
	b.cy = (int)(sy / n);
	return b;
}

static int uart_open_port(const char *dev, int baud)
{
	struct termios tio;
	int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) return -1;
	memset(&tio, 0, sizeof(tio));
	cfmakeraw(&tio);
	cfsetispeed(&tio, baud);
	cfsetospeed(&tio, baud);
	tio.c_cflag |= (CLOCAL | CREAD);
	tio.c_cflag &= ~CRTSCTS;
	tcsetattr(fd, TCSANOW, &tio);
	return fd;
}

/* STM32 目标帧：AA 01 + dx/dy/roll（有符号大端，归一化误差×1000）+ 8 位求和校验。 */
static void uart_send_blob(int fd, const trk_blob_t *b)
{
	uint8_t buf[9];
	uint8_t checksum = 0;
	uint16_t dx_bits, dy_bits;
	int dx, dy;
	if (fd < 0 || b == NULL || !b->found) return;

	/* 图像左侧为正 yaw，图像下方为正 pitch；中心偏差归一化到 ±1000。 */
	dx = (TRK_W / 2 - b->cx) * 1000 / (TRK_W / 2);
	dy = (b->cy - TRK_H / 2) * 1000 / (TRK_H / 2);
	if (dx > 1000) dx = 1000;
	if (dx < -1000) dx = -1000;
	if (dy > 1000) dy = 1000;
	if (dy < -1000) dy = -1000;
	dx_bits = (uint16_t)dx;
	dy_bits = (uint16_t)dy;

	buf[0] = 0xAA;
	buf[1] = 0x01;
	buf[2] = (uint8_t)(dx_bits >> 8);
	buf[3] = (uint8_t)dx_bits;
	buf[4] = (uint8_t)(dy_bits >> 8);
	buf[5] = (uint8_t)dy_bits;
	buf[6] = 0; /* 当前上位机没有目标 roll 误差。 */
	buf[7] = 0;
	for (int i = 0; i < 8; ++i) checksum = (uint8_t)(checksum + buf[i]);
	buf[8] = checksum;
	write(fd, buf, sizeof(buf));
}

static void set_manual_exposure(int exp_us, int gain_q10)
{
	ISP_EXPOSURE_ATTR_S attr;
	if (CVI_ISP_GetExposureAttr(0, &attr) != CVI_SUCCESS) {
		SAMPLE_PRT(">>> GetExposureAttr 失败\n");
		return;
	}
	attr.enOpType = OP_TYPE_MANUAL;
	attr.stManual.u32ExpTime  = exp_us;
	attr.stManual.u32AGain    = gain_q10;
	attr.stManual.u32DGain    = 1024;
	attr.stManual.u32ISPDGain = 1024;
	if (CVI_ISP_SetExposureAttr(0, &attr) != CVI_SUCCESS)
		SAMPLE_PRT(">>> SetExposureAttr 失败\n");
	else
		SAMPLE_PRT(">>> 固定曝光 %dus gain=%d\n", exp_us, gain_q10);
}

/* 在 Y 平面上画矩形边框（用于预览调试） */
static void draw_box(unsigned char *y, int stride, int x0, int y0, int x1, int y1)
{
	if (x0 < 1) x0 = 1;
	if (y0 < 1) y0 = 1;
	if (x1 > TRK_W - 2) x1 = TRK_W - 2;
	if (y1 > TRK_H - 2) y1 = TRK_H - 2;
	if (x1 <= x0 || y1 <= y0) return;

	for (int x = x0; x <= x1; x++) {
		y[(size_t)y0 * stride + x] = 255;
		y[(size_t)y1 * stride + x] = 255;
	}
	for (int yy = y0; yy <= y1; yy++) {
		y[(size_t)yy * stride + x0] = 255;
		y[(size_t)yy * stride + x1] = 255;
	}
	/* 十字准星画在质心 */
	for (int d = -6; d <= 6; d++) {
		int cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
		if (cx + d >= 0 && cx + d < TRK_W) y[(size_t)cy * stride + cx + d] = 200;
		if (cy + d >= 0 && cy + d < TRK_H) y[(size_t)(cy + d) * stride + cx] = 200;
	}
}

/* trackview：检测 + 画框 + TCP 预览（每 N 帧发一次，凑 30fps） */
static void track_view_mode(int fps, int thresh, int exp_us, int gain)
{
	int uart, ls, cs;
	trk_blob_t b, last_hit;
	int has_hit = 0, roi_x = 0, roi_y = 0, roi_w = TRK_W, roi_h = TRK_H;
	int tracking = 0;
	unsigned long nframe = 0, nfound = 0;
	int every;

	memset(&last_hit, 0, sizeof(last_hit));
	setvbuf(stdout, NULL, _IONBF, 0);
	signal(SIGTERM, trk_on_sig);
	signal(SIGINT, trk_on_sig);

	callback_FPS(fps);
	usleep(300 * 1000);
	set_manual_exposure(exp_us, gain);
	usleep(300 * 1000);

	uart = uart_open_port("/dev/ttyS1", B115200);

	ls = socket(AF_INET, SOCK_STREAM, 0);
	int one = 1;
	setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in ad = {0};
	ad.sin_family = AF_INET;
	ad.sin_addr.s_addr = INADDR_ANY;
	ad.sin_port = htons(5098);
	dup2(STDERR_FILENO, STDOUT_FILENO);
	if (bind(ls, (struct sockaddr *)&ad, sizeof(ad)) < 0) { perror("bind"); return; }
	listen(ls, 1);
	SAMPLE_PRT(">>> UART1 fd=%d，TCP 5098 等待客户端...\n", uart);
retry_accept:
	SAMPLE_PRT(">>> TCP 5098 等待客户端...\n");
	cs = accept(ls, NULL, NULL);
	if (cs < 0) { perror("accept"); return; }
	SAMPLE_PRT(">>> 客户端已连接，开始跟踪\n");

	every = fps / 30; if (every < 1) every = 1;

	while (!g_stop) {
		VIDEO_FRAME_INFO_S f;
		if (g_stop) break;
		if (CVI_VI_GetChnFrame(0, 0, &f, 1000) != CVI_SUCCESS) continue;

		size_t len = f.stVFrame.u32Length[0];
		unsigned char *vir = CVI_SYS_Mmap(f.stVFrame.u64PhyAddr[0], len);
		CVI_SYS_IonInvalidateCache(f.stVFrame.u64PhyAddr[0], vir, len);

		g_y = vir;
		g_ystride = f.stVFrame.u32Stride[0];

		b = trk_find(roi_x, roi_y, roi_w, roi_h, thresh, tracking ? 1 : 2);
		nframe++;

		if (b.found) {
			nfound++;
			uart_send_blob(uart, &b);
			roi_x = b.x0 - 40; roi_y = b.y0 - 40;
			roi_w = (b.x1 - b.x0) + 80;
			roi_h = (b.y1 - b.y0) + 80;
			tracking = 1;
			last_hit = b; has_hit = 1;
		} else {
			roi_x = 0; roi_y = 0; roi_w = TRK_W; roi_h = TRK_H;
			tracking = 0;
		}

		if ((nframe % every) == 0) {
			size_t total = f.stVFrame.u32Length[0] +
				       f.stVFrame.u32Length[1] +
				       f.stVFrame.u32Length[2];
			/* 画框需要 Y 平面 + UV 平面连续，整体重映射一次 */
			unsigned char *all = CVI_SYS_Mmap(f.stVFrame.u64PhyAddr[0], total);
			CVI_SYS_IonInvalidateCache(f.stVFrame.u64PhyAddr[0], all, total);
			if (has_hit)
				draw_box(all, f.stVFrame.u32Stride[0],
					 last_hit.x0, last_hit.y0, last_hit.x1, last_hit.y1);
			size_t off = 0;
			for (int i = 0; i < 3; i++) {
				size_t l = f.stVFrame.u32Length[i];
				if (!l) continue;
				size_t done = 0;
				while (done < l) {
					ssize_t n = write(cs, all + off + done, l - done);
					if (n <= 0) goto out;
					done += (size_t)n;
				}
				off += l;
			}
			CVI_SYS_Munmap(all, total);
		}

		CVI_SYS_Munmap(vir, len);
		CVI_VI_ReleaseChnFrame(0, 0, &f);

		if ((nframe % 120) == 0)
			SAMPLE_PRT(">>> frame=%lu hit=%lu(%.0f%%) last bbox=(%d,%d)-(%d,%d) c=(%d,%d) area=%d\n",
				   nframe, nfound, nframe ? 100.0 * nfound / nframe : 0.0,
				   last_hit.x0, last_hit.y0, last_hit.x1, last_hit.y1,
				   last_hit.cx, last_hit.cy, last_hit.area);
	}
out:
	if (cs >= 0) { close(cs); cs = -1; }
	if (g_stop) {
		SAMPLE_PRT(">>> 收到停止信号，正常退出\n");
		if (uart >= 0) close(uart);
		close(ls);
		return;
	}
	SAMPLE_PRT(">>> 客户端断开，等待重连\n");
	goto retry_accept;
}

/* track 模式：ledtrack <fps> <threshold> [exp_us] [gain] */
static void track_mode(int fps, int thresh, int exp_us, int gain)
{
	int uart = uart_open_port("/dev/ttyS1", B115200);
	trk_blob_t b;
	int roi_x = 0, roi_y = 0, roi_w = TRK_W, roi_h = TRK_H;
	int tracking = 0;
	unsigned long nframe = 0, nfound = 0;
	trk_blob_t last_hit;
	int has_hit = 0;

	memset(&last_hit, 0, sizeof(last_hit));

	setvbuf(stdout, NULL, _IONBF, 0);   /* 日志不缓冲，否则看不到 */
	signal(SIGTERM, trk_on_sig);
	signal(SIGINT, trk_on_sig);
	callback_FPS(fps);
	usleep(300 * 1000);
	set_manual_exposure(exp_us, gain);
	usleep(300 * 1000);

	if (uart >= 0) {
		unsigned char hello[7] = {0xFF, 'R', 'P', 'D', 'A', 'R', 'T'};
		write(uart, hello, sizeof(hello));
		SAMPLE_PRT(">>> UART1 已打开 fd=%d，握手已发送\n", uart);
	} else {
		SAMPLE_PRT(">>> 警告：UART1 打不开，只做检测\n");
	}

	while (!g_stop) {
		VIDEO_FRAME_INFO_S f;
		CVI_S32 r = CVI_VI_GetChnFrame(0, 0, &f, 1000);

		if (g_stop) break;
		if (r != CVI_SUCCESS) continue;

		size_t len = f.stVFrame.u32Length[0];
		unsigned char *vir = CVI_SYS_Mmap(f.stVFrame.u64PhyAddr[0], len);
		CVI_SYS_IonInvalidateCache(f.stVFrame.u64PhyAddr[0], vir, len);

		g_y = vir;
		g_ystride = f.stVFrame.u32Stride[0];

		/* 跟踪时全分辨率细扫，搜索时降采样粗扫 */
		b = trk_find(roi_x, roi_y, roi_w, roi_h, thresh, tracking ? 1 : 2);

		CVI_SYS_Munmap(vir, len);
		CVI_VI_ReleaseChnFrame(0, 0, &f);

		nframe++;

		if (b.found) {
			nfound++;
			uart_send_blob(uart, &b);
			/* 动态 ROI：按 OpenMV 的做法，包围盒外扩 40 像素 */
			roi_x = b.x0 - 40; roi_y = b.y0 - 40;
			roi_w = (b.x1 - b.x0) + 80;
			roi_h = (b.y1 - b.y0) + 80;
			tracking = 1;
			last_hit = b;
			has_hit = 1;
		} else {
			roi_x = 0; roi_y = 0;
			roi_w = TRK_W; roi_h = TRK_H;
			tracking = 0;
		}

		if ((nframe % 120) == 0)
			SAMPLE_PRT(">>> frame=%lu hit=%lu(%.0f%%) now=%s last bbox=(%d,%d)-(%d,%d) c=(%d,%d) area=%d\n",
				   nframe, nfound, nframe ? 100.0 * nfound / nframe : 0.0,
				   b.found ? "HIT" : "MISS",
				   last_hit.x0, last_hit.y0, last_hit.x1, last_hit.y1,
				   last_hit.cx, last_hit.cy, last_hit.area);
	}
}
/* ====================================================== */

static void sys_vi_deinit(void)
{
	SAMPLE_COMM_VI_DestroyIsp(&g_stViConfig);

	SAMPLE_COMM_VI_DestroyVi(&g_stViConfig);

	SAMPLE_COMM_SYS_Exit();
}

static CVI_S32 _vi_get_chn_frame(CVI_U8 chn)
{
	VIDEO_FRAME_INFO_S stVideoFrame;
	VI_CROP_INFO_S crop_info = {0};

	if (CVI_VI_GetChnFrame(0, chn, &stVideoFrame, 3000) == 0) {
		FILE *output;
		size_t image_size = stVideoFrame.stVFrame.u32Length[0] + stVideoFrame.stVFrame.u32Length[1]
				  + stVideoFrame.stVFrame.u32Length[2];
		CVI_VOID *vir_addr;
		CVI_U32 plane_offset, u32LumaSize, u32ChromaSize;
		CVI_CHAR img_name[128] = {0, };

		CVI_TRACE_LOG(CVI_DBG_WARN, "width: %d, height: %d, total_buf_length: %zu\n",
			   stVideoFrame.stVFrame.u32Width,
			   stVideoFrame.stVFrame.u32Height, image_size);

		snprintf(img_name, sizeof(img_name), "sample_%d.yuv", chn);

		output = fopen(img_name, "wb");
		if (output == NULL) {
			memset(img_name, 0x0, sizeof(img_name));
			snprintf(img_name, sizeof(img_name), "/mnt/data/sample_%d.yuv", chn);
			output = fopen(img_name, "wb");
			if (output == NULL) {
				CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);
				CVI_TRACE_LOG(CVI_DBG_ERR, "fopen fail\n");
				return CVI_FAILURE;
			}
		}

		u32LumaSize =  stVideoFrame.stVFrame.u32Stride[0] * stVideoFrame.stVFrame.u32Height;
		u32ChromaSize =  stVideoFrame.stVFrame.u32Stride[1] * stVideoFrame.stVFrame.u32Height / 2;
		CVI_VI_GetChnCrop(0, chn, &crop_info);
		if (crop_info.bEnable) {
			u32LumaSize = ALIGN((crop_info.stCropRect.u32Width * 8 + 7) >> 3, DEFAULT_ALIGN) *
				ALIGN(crop_info.stCropRect.u32Height, 2);
			u32ChromaSize = (ALIGN(((crop_info.stCropRect.u32Width >> 1) * 8 + 7) >> 3, DEFAULT_ALIGN) *
				ALIGN(crop_info.stCropRect.u32Height, 2)) >> 1;
		}
		vir_addr = CVI_SYS_Mmap(stVideoFrame.stVFrame.u64PhyAddr[0], image_size);
		CVI_SYS_IonInvalidateCache(stVideoFrame.stVFrame.u64PhyAddr[0], vir_addr, image_size);
		plane_offset = 0;
		for (int i = 0; i < 3; i++) {
			if (stVideoFrame.stVFrame.u32Length[i] != 0) {
				stVideoFrame.stVFrame.pu8VirAddr[i] = vir_addr + plane_offset;
				plane_offset += stVideoFrame.stVFrame.u32Length[i];
				CVI_TRACE_LOG(CVI_DBG_WARN,
					   "plane(%d): paddr(%#llx) vaddr(%p) stride(%d) length(%d)\n",
					   i, stVideoFrame.stVFrame.u64PhyAddr[i],
					   stVideoFrame.stVFrame.pu8VirAddr[i],
					   stVideoFrame.stVFrame.u32Stride[i],
					   stVideoFrame.stVFrame.u32Length[i]);
				fwrite((void *)stVideoFrame.stVFrame.pu8VirAddr[i]
					, (i == 0) ? u32LumaSize : u32ChromaSize, 1, output);
			}
		}
		CVI_SYS_Munmap(vir_addr, image_size);

		if (CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame) != 0)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VI_ReleaseChnFrame NG\n");

		fclose(output);
		return CVI_SUCCESS;
	}
	CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VI_GetChnFrame NG\n");
	return CVI_FAILURE;
}

static long diff_in_us(struct timespec t1, struct timespec t2)
{
	struct timespec diff;

	if (t2.tv_nsec-t1.tv_nsec < 0) {
		diff.tv_sec  = t2.tv_sec - t1.tv_sec - 1;
		diff.tv_nsec = t2.tv_nsec - t1.tv_nsec + 1000000000;
	} else {
		diff.tv_sec  = t2.tv_sec - t1.tv_sec;
		diff.tv_nsec = t2.tv_nsec - t1.tv_nsec;
	}
	return (diff.tv_sec * 1000000.0 + diff.tv_nsec / 1000.0);
}

static void ViConfigReInit(SAMPLE_VI_CONFIG_S *p_stViConfig, SAMPLE_INI_CFG_S *p_stIniCfg)
{
	int s32WorkSnsId = 0;

	for (s32WorkSnsId = 0; s32WorkSnsId < p_stIniCfg->devNum; s32WorkSnsId++) {
		p_stViConfig->s32WorkingViNum					= 1 + s32WorkSnsId;
		p_stViConfig->as32WorkingViId[s32WorkSnsId]			= s32WorkSnsId;
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.enSnsType	=
			p_stIniCfg->enSnsType[s32WorkSnsId];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.MipiDev		=
			p_stIniCfg->MipiDev[s32WorkSnsId];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.s32BusId	=
			p_stIniCfg->s32BusId[s32WorkSnsId];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as16LaneId[0]   =
			p_stIniCfg->as16LaneId[s32WorkSnsId][0];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as16LaneId[1]   =
			p_stIniCfg->as16LaneId[s32WorkSnsId][1];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as16LaneId[2]   =
			p_stIniCfg->as16LaneId[s32WorkSnsId][2];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as16LaneId[3]   =
			p_stIniCfg->as16LaneId[s32WorkSnsId][3];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as16LaneId[4]   =
			p_stIniCfg->as16LaneId[s32WorkSnsId][4];

		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as8PNSwap[0] =
			p_stIniCfg->as8PNSwap[s32WorkSnsId][0];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as8PNSwap[1] =
			p_stIniCfg->as8PNSwap[s32WorkSnsId][1];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as8PNSwap[2] =
			p_stIniCfg->as8PNSwap[s32WorkSnsId][2];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as8PNSwap[3] =
			p_stIniCfg->as8PNSwap[s32WorkSnsId][3];
		p_stViConfig->astViInfo[s32WorkSnsId].stSnsInfo.as8PNSwap[4] =
			p_stIniCfg->as8PNSwap[s32WorkSnsId][4];
		p_stViConfig->astViInfo[s32WorkSnsId].stDevInfo.enWDRMode =
			p_stIniCfg->enWDRMode[s32WorkSnsId];
	}
}

static CVI_S32 sensor_dump_yuv(void)
{
	CVI_S32 loop = 0;
	CVI_U32 ok = 0, ng = 0;
	CVI_U8  chn = 0;
	int tmp;
	struct timespec start, end;

	CVI_TRACE_LOG(CVI_DBG_WARN, "Get frm from which chn(0~1): ");
	scanf("%d", &tmp);
	chn = tmp;
	CVI_TRACE_LOG(CVI_DBG_WARN, "how many loops to do(11111 is infinite: ");
	scanf("%d", &loop);
	while (loop > 0) {
		clock_gettime(CLOCK_MONOTONIC, &start);
		if (_vi_get_chn_frame(chn) == CVI_SUCCESS) {
			++ok;
			clock_gettime(CLOCK_MONOTONIC, &end);
			CVI_TRACE_LOG(CVI_DBG_WARN, "ms consumed: %f\n",
						(CVI_FLOAT)diff_in_us(start, end)/1000);
		} else
			++ng;
		//sleep(1);
		if (loop != 11111)
			loop--;
	}
	CVI_TRACE_LOG(CVI_DBG_WARN, "VI GetChnFrame OK(%d) NG(%d)\n", ok, ng);

	CVI_TRACE_LOG(CVI_DBG_WARN, "Dump VI yuv TEST-PASS\n");

	return CVI_SUCCESS;
}

static CVI_S32 sensor_flip_mirror(void)
{
	int flip;
	int mirror;
	int chnID;
	int pipeID;

	CVI_TRACE_LOG(CVI_DBG_WARN, "chn(0~1): ");
	scanf("%d", &chnID);
	CVI_TRACE_LOG(CVI_DBG_WARN, "Flip enable/disable(1/0): ");
	scanf("%d", &flip);
	CVI_TRACE_LOG(CVI_DBG_WARN, "Mirror enable/disable(1/0): ");
	scanf("%d", &mirror);
	pipeID = chnID;
	CVI_VI_SetChnFlipMirror(pipeID, chnID, flip, mirror);

	return CVI_SUCCESS;
}

static CVI_S32 sensor_dump_raw(void)
{
	VIDEO_FRAME_INFO_S stVideoFrame[2];
	VI_DUMP_ATTR_S attr;
	struct timeval tv1;
	int frm_num = 1, j = 0;
	CVI_U32 dev = 0, loop = 0;
	struct timespec start, end;
	CVI_S32 s32Ret = CVI_SUCCESS;

	memset(stVideoFrame, 0, sizeof(stVideoFrame));

	stVideoFrame[0].stVFrame.enPixelFormat = PIXEL_FORMAT_RGB_BAYER_12BPP;
	stVideoFrame[1].stVFrame.enPixelFormat = PIXEL_FORMAT_RGB_BAYER_12BPP;

	CVI_TRACE_LOG(CVI_DBG_WARN, "To get raw dump from dev(0~1): ");
	scanf("%d", &dev);

	attr.bEnable = 1;
	attr.u32Depth = 0;
	attr.enDumpType = VI_DUMP_TYPE_RAW;

	CVI_VI_SetPipeDumpAttr(dev, &attr);

	attr.bEnable = 0;
	attr.enDumpType = VI_DUMP_TYPE_IR;

	CVI_VI_GetPipeDumpAttr(dev, &attr);

	CVI_TRACE_LOG(CVI_DBG_WARN, "Enable(%d), DumpType(%d):\n", attr.bEnable, attr.enDumpType);
	CVI_TRACE_LOG(CVI_DBG_WARN, "how many loops to do (1~60)");
	scanf("%d", &loop);

	if (loop > 60)
		return s32Ret;

	while (loop > 0) {
		clock_gettime(CLOCK_MONOTONIC, &start);
		frm_num = 1;

		CVI_VI_GetPipeFrame(dev, stVideoFrame, 1000);

		if (stVideoFrame[1].stVFrame.u64PhyAddr[0] != 0)
			frm_num = 2;

		gettimeofday(&tv1, NULL);

		for (j = 0; j < frm_num; j++) {
			size_t image_size = stVideoFrame[j].stVFrame.u32Length[0];
			unsigned char *ptr = calloc(1, image_size);
			FILE *output;
			char img_name[128] = {0,}, order_id[8] = {0,};

			if (attr.enDumpType == VI_DUMP_TYPE_RAW) {
				stVideoFrame[j].stVFrame.pu8VirAddr[0]
					= CVI_SYS_Mmap(stVideoFrame[j].stVFrame.u64PhyAddr[0]
					  , stVideoFrame[j].stVFrame.u32Length[0]);
				CVI_TRACE_LOG(CVI_DBG_WARN, "paddr(%#llx) vaddr(%p)\n",
							stVideoFrame[j].stVFrame.u64PhyAddr[0],
							stVideoFrame[j].stVFrame.pu8VirAddr[0]);

				memcpy(ptr, (const void *)stVideoFrame[j].stVFrame.pu8VirAddr[0],
					stVideoFrame[j].stVFrame.u32Length[0]);
				CVI_SYS_Munmap((void *)stVideoFrame[j].stVFrame.pu8VirAddr[0],
						stVideoFrame[j].stVFrame.u32Length[0]);

				switch (stVideoFrame[j].stVFrame.enBayerFormat) {
				default:
				case BAYER_FORMAT_BG:
					snprintf(order_id, sizeof(order_id), "BG");
					break;
				case BAYER_FORMAT_GB:
					snprintf(order_id, sizeof(order_id), "GB");
					break;
				case BAYER_FORMAT_GR:
					snprintf(order_id, sizeof(order_id), "GR");
					break;
				case BAYER_FORMAT_RG:
					snprintf(order_id, sizeof(order_id), "RG");
					break;
				}

				snprintf(img_name, sizeof(img_name),
						"./vi_%d_%s_%s_w_%d_h_%d_x_%d_y_%d_tv_%" PRIdMAX "_%" PRIdMAX ".raw",
						dev, (j == 0) ? "LE" : "SE", order_id,
						stVideoFrame[j].stVFrame.u32Width,
						stVideoFrame[j].stVFrame.u32Height,
						stVideoFrame[j].stVFrame.s16OffsetLeft,
						stVideoFrame[j].stVFrame.s16OffsetTop,
						(intmax_t)tv1.tv_sec,  (intmax_t)tv1.tv_usec);

				CVI_TRACE_LOG(CVI_DBG_WARN, "dump image %s\n", img_name);

				output = fopen(img_name, "wb");

				fwrite(ptr, image_size, 1, output);
				fclose(output);
				free(ptr);
			}
		}

		CVI_VI_ReleasePipeFrame(dev, stVideoFrame);

		clock_gettime(CLOCK_MONOTONIC, &end);
		CVI_TRACE_LOG(CVI_DBG_WARN, "ms consumed: %f\n",
					(CVI_FLOAT)diff_in_us(start, end) / 1000);

		loop--;
	}

	CVI_TRACE_LOG(CVI_DBG_WARN, "Dump VI raw TEST-PASS\n");

	return s32Ret;
}

static CVI_S32 sensor_linear_wdr_switch(void)
{
	int tmp;
	CVI_U8 wdrMode = 0;
	CVI_S32 s32Ret = CVI_SUCCESS;

	SAMPLE_COMM_VI_DestroyIsp(&g_stViConfig);
	// Stop VI.
	SAMPLE_COMM_VI_DestroyVi(&g_stViConfig);
	// Close ISP device.
	s32Ret = SAMPLE_COMM_VI_CLOSE();
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "vi close failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}
	// select which mode want to switch.
	printf("Please select sensor input mode (0:linear/1:wdr) :");
	scanf("%d", &tmp);
	wdrMode = tmp;
	if (wdrMode == 0) {
		// Reset main sensor initial config to linear setting.
		g_stIniCfg.enSnsType[0] = SONY_IMX327_2L_MIPI_2M_30FPS_12BIT;
		g_stIniCfg.enWDRMode[0] = WDR_MODE_NONE;
		// Reset slave sensor initial config to linear setting.
		g_stIniCfg.enSnsType[1] = SONY_IMX327_SLAVE_MIPI_2M_30FPS_12BIT;
		g_stIniCfg.enWDRMode[1] = WDR_MODE_NONE;
	} else {
		// Reset main sensor initial config to wdr setting.
		g_stIniCfg.enSnsType[0] = SONY_IMX327_2L_MIPI_2M_30FPS_12BIT_WDR2TO1;
		g_stIniCfg.enWDRMode[0] = WDR_MODE_2To1_LINE;
		// Reset slave sensor initial config to wdr setting.
		g_stIniCfg.enSnsType[1] = SONY_IMX327_SLAVE_MIPI_2M_30FPS_12BIT_WDR2TO1;
		g_stIniCfg.enWDRMode[1] = WDR_MODE_2To1_LINE;
	}
	// Reconfig VI setting for different mode Re-initial correctly.
	ViConfigReInit(&g_stViConfig, &g_stIniCfg);
	// open Isp device.
	s32Ret = SAMPLE_COMM_VI_OPEN();
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "vi open failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}
	// Initial VI & ISP.
	s32Ret = SAMPLE_PLAT_VI_INIT(&g_stViConfig);
	if (s32Ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "vi init failed. s32Ret: 0x%x !\n", s32Ret);
		return s32Ret;
	}

	return s32Ret;
}

int sensor_dump(void)
{
	CVI_U64 addr = 0;
	CVI_U32 size = 0;
	FILE *output;
	char img_name[128] = {0, };

	CVI_TRACE_LOG(CVI_DBG_WARN, "dump addr:\n");
	scanf("%llx", &addr);
	CVI_TRACE_LOG(CVI_DBG_WARN, "dump size(0\1):\n");
	scanf("%x", &size);

	snprintf(img_name, sizeof(img_name), "register_%llx.bin", addr);

	output = fopen(img_name, "wb");
	if (output == NULL) {
		memset(img_name, 0x0, sizeof(img_name));
		snprintf(img_name, sizeof(img_name), "register_%llx.bin", addr);
		output = fopen(img_name, "wb");
		if (output == NULL) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "fopen fail\n");
			return CVI_FAILURE;
		}
	}

	void *vir_addr = CVI_SYS_Mmap(addr, size);

	fwrite(vir_addr, size, 1, output);
	fflush(output);

	CVI_SYS_Munmap(vir_addr, size);

	fclose(output);

	return CVI_SUCCESS;
}

int sensor_proc(void)
{
	CVI_S32 s32Ret = CVI_SUCCESS;
	CVI_S32 op;

	SAMPLE_PRT("---debug_info------------------------------------------------\n");
	SAMPLE_PRT("1: /proc/cvitek/vi_dbg\n");
	SAMPLE_PRT("2: /proc/cvitek/vi\n");
	SAMPLE_PRT("3: /proc/mipi-rx\n");
	scanf("%d", &op);

	switch (op) {
	case 1:
		system("cat /proc/cvitek/vi_dbg");
		break;
	case 2:
		system("cat /proc/cvitek/vi");
		break;
	case 3:
		system("cat /proc/mipi-rx");
		break;
	default:
		break;
	}

	return s32Ret;
}

//#define ENABLE_ISP_TOOL_DAEMON 1
//#define JSONRPC_PORT	(5566)
//extern void isp_daemon2_init(unsigned int port);
//extern void isp_daemon2_uninit(void);

int main(int argc, char **argv)
{
	CVI_S32 s32Ret = CVI_SUCCESS;
	CVI_S32 op;


	s32Ret = sys_vi_init();
	if (s32Ret != CVI_SUCCESS)
		return s32Ret;

#ifdef ENABLE_ISP_TOOL_DAEMON
	isp_daemon2_init(JSONRPC_PORT);
#endif

	usleep(500 * 1000);

	/* auto 模式：初始化完成后自动抓帧并退出，供脚本调用 */
	if (argc > 1 && strcmp(argv[1], "trackview") == 0) {
		int fps    = argc > 2 ? atoi(argv[2]) : 120;
		int thresh = argc > 3 ? atoi(argv[3]) : 200;
		int exp_us = argc > 4 ? atoi(argv[4]) : 800;
		int gain   = argc > 5 ? atoi(argv[5]) : 1024;
		track_view_mode(fps, thresh, exp_us, gain);
		return 0;
	}

	if (argc > 1 && strcmp(argv[1], "track") == 0) {
		int fps    = argc > 2 ? atoi(argv[2]) : 120;
		int thresh = argc > 3 ? atoi(argv[3]) : 200;
		int exp_us = argc > 4 ? atoi(argv[4]) : 800;
		int gain   = argc > 5 ? atoi(argv[5]) : 1024;
		usleep(2000 * 1000);
		dup2(STDERR_FILENO, STDOUT_FILENO);
		track_mode(fps, thresh, exp_us, gain);
		return 0;
	}

	if (argc > 1 && strcmp(argv[1], "tcp") == 0) {
		int port = argc > 2 ? atoi(argv[2]) : 5098;
		int want = argc > 3 ? atoi(argv[3]) : 30;
		usleep(2000 * 1000);
		callback_FPS(want);
		usleep(500 * 1000);

		int ls = socket(AF_INET, SOCK_STREAM, 0);
		int one = 1;
		setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
		struct sockaddr_in ad = {0};
		ad.sin_family = AF_INET;
		ad.sin_addr.s_addr = INADDR_ANY;
		ad.sin_port = htons(port);
		dup2(STDERR_FILENO, STDOUT_FILENO);
		if (bind(ls, (struct sockaddr *)&ad, sizeof(ad)) < 0) { perror("bind"); return 1; }
		listen(ls, 1);
		SAMPLE_PRT(">>> tcp listening on %d, target %dfps\n", port, want);
		int cs = accept(ls, NULL, NULL);
		if (cs < 0) { perror("accept"); return 1; }
		SAMPLE_PRT(">>> client connected\n");
		g_stream_fd = cs;
		while (1)
			_vi_stream_once(0);
	}

	if (argc > 1 && strcmp(argv[1], "venc") == 0) {
		int want = argc > 2 ? atoi(argv[2]) : 120;
		int kbps = argc > 3 ? atoi(argv[3]) : 3000;
		usleep(2000 * 1000);
		callback_FPS(want);
		usleep(500 * 1000);
		g_stream_fd = dup(STDOUT_FILENO);
		dup2(STDERR_FILENO, STDOUT_FILENO);
		if (venc_init(640, 400, want, kbps) != CVI_SUCCESS)
			return 1;
		SAMPLE_PRT(">>> venc mode: %dfps %dkbps\n", want, kbps);
		while (1)
			_venc_stream_once(0);
	}

	if (argc > 2 && strcmp(argv[1], "fps") == 0) {
		int want = atoi(argv[2]);
		usleep(2000 * 1000);
		callback_FPS(want);
		usleep(1000 * 1000);
		g_stream_fd = dup(STDOUT_FILENO);
		dup2(STDERR_FILENO, STDOUT_FILENO);
		SAMPLE_PRT(">>> fps mode: requested %d\n", want);
		while (1)
			_vi_stream_once(0);
	}

	if (argc > 1 && strcmp(argv[1], "stream") == 0) {
		/* 持续把帧写到 stdout；stderr 留给日志 */
		usleep(3000 * 1000);
		g_stream_fd = dup(STDOUT_FILENO);
		dup2(STDERR_FILENO, STDOUT_FILENO);   /* 日志转到 stderr，不污染视频流 */
		SAMPLE_PRT(">>> stream mode start fd=%d\n", g_stream_fd);
		while (1)
			_vi_stream_once(0);
	}

	if (argc > 1) {
		SAMPLE_PRT(">>> auto mode: waiting for stream...\n");
		usleep(12000 * 1000);
		SAMPLE_PRT(">>> direct capture chn0 -> sample_0.yuv\n");
		s32Ret = _vi_get_chn_frame(0);
		SAMPLE_PRT(">>> capture ret=0x%x\n", s32Ret);
		sync();
		return 0;
	}

	system("stty erase ^H");

	do {
		SAMPLE_PRT("---Basic------------------------------------------------\n");
		SAMPLE_PRT("1: dump vi raw frame\n");
		SAMPLE_PRT("2: dump vi yuv frame\n");
		SAMPLE_PRT("3: set chn flip/mirror\n");
		SAMPLE_PRT("4: linear wdr switch\n");
		SAMPLE_PRT("5: AE debug\n");
		SAMPLE_PRT("6: sensor dump\n");
		SAMPLE_PRT("7: sensor proc\n");
		SAMPLE_PRT("255: exit\n");
		scanf("%d", &op);

		switch (op) {
		case 1:
			s32Ret = sensor_dump_raw();
			break;
		case 2:
			s32Ret = sensor_dump_yuv();
			break;
		case 3:
			s32Ret = sensor_flip_mirror();
			break;
		case 4:
			s32Ret = sensor_linear_wdr_switch();
			break;
		case 5:
			s32Ret = sensor_ae_test();
			break;
		case 6:
			s32Ret = sensor_dump();
			break;
		case 7:
			s32Ret = sensor_proc();
			break;
		default:
			break;
		}
		if (s32Ret != CVI_SUCCESS) {
			CVI_TRACE_LOG(CVI_DBG_ERR, "op(%d) failed with %#x!\n", op, s32Ret);
			break;
		}
	} while (op != 255);

#ifdef ENABLE_ISP_TOOL_DAEMON
	isp_daemon2_uninit();
#endif

	sys_vi_deinit();

	return s32Ret;
}

