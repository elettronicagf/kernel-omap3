/*
 * linux/drivers/video/ti81xx/vpss/dctrl.c
 *
 * VPSS display controller driver for TI 81XX
 *
 * Copyright (C) 2009 TI
 * Author: Yihe Hu <yihehu@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA	02111-1307, USA.
 */
#define VPSS_SUBMODULE_NAME   "DCTRL"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <mach/board-ti816x.h>

#include "core.h"
#include "system.h"
#include "dc.h"
#include "sysfs.h"
#include "display_interface.h"

static struct vps_dispctrl *disp_ctrl;
static void *dc_handle;
static struct vps_payload_info  *dc_payload_info;
static struct ti81xx_external_encoder
		external_encs[VPS_DC_MAX_VENC][MAX_EXT_VENCS_PER_DISPLAY];

/*store the current VENC setting*/
static struct vps_dcvencinfo venc_info = {
	{
		{VPS_DC_VENC_HDMI,
		    {FVID2_STD_1080P_60, 1920, 1080, FVID2_SF_PROGRESSIVE,\
			148500, 60, 88, 148, 44, 4, 36, 5},
		3, 0, 0},

		{VPS_DC_VENC_DVO2,
		    {FVID2_STD_1080P_60, 1920, 1080, FVID2_SF_PROGRESSIVE,\
			148500, 60, 88, 148, 44, 4, 36, 5},
		3, 0, 0},

		{VPS_DC_VENC_SD,
		    {FVID2_STD_NTSC, 720, 480, FVID2_SF_INTERLACED,\
			216000, 60, 12, 68, 64, 5, 41, 5},
		0, 0, 0},

		{VPS_DC_VENC_HDCOMP,
		    {FVID2_STD_1080I_60, 1920, 1080, FVID2_SF_PROGRESSIVE,\
			74250, 60, 88, 148, 44, 2, 15, 5},
		2, 0, 0}

	},
	0,
	VPS_DC_MAX_VENC,
};

/*S***************************private funtions*******************/

static inline void dc_lock(struct vps_dispctrl *dctrl)
{
	mutex_lock(&dctrl->dcmutex);
}

static inline void dc_unlock(struct vps_dispctrl *dctrl)
{
	mutex_unlock(&dctrl->dcmutex);
}

static inline bool isdigitalvenc(int vid)
{
	if ((vid == VPS_DC_VENC_HDMI) ||
		(vid == VPS_DC_VENC_DVO2))
		return true;

	return false;
}

static inline bool isdigitalclk(u32 clk)
{
	if ((clk == VPS_DC_CLKSRC_VENCA) ||
	(clk == VPS_DC_CLKSRC_VENCA_DIV2) ||
	(clk == VPS_DC_CLKSRC_VENCA_DIV2_DIFF))
		return false;

	return true;

}
static inline bool isvalidclksrc(int vid, enum vps_dcvencclksrcsel clk)
{
	if ((vid == VPS_DC_VENC_HDMI) && (!isdigitalclk(clk)))
		return false;
	/*ti814x, DVO2 must be aclk*/
	if (cpu_is_ti814x()) {
		if ((vid == VPS_DC_VENC_DVO2) && (isdigitalclk(clk)))
			return false;
	}
	return true;
}

static inline bool isvalidmode(int vid, int mid)
{
	switch (vid) {
	case VPS_DC_VENC_HDMI:
if (cpu_is_ti816x())
	case VPS_DC_VENC_HDCOMP:
	case VPS_DC_VENC_DVO2:
		if ((mid == FVID2_STD_NTSC) || (mid == FVID2_STD_PAL))
			return false;
		break;
	case VPS_DC_VENC_SD:
		if (!((mid == FVID2_STD_NTSC) || (mid == FVID2_STD_PAL)))
			return false;
		break;
	}

	return true;
}

/*get the clock venc*/
static inline u32 get_plloutputvenc(int bidx)
{
	struct vps_dcvencclksrc clksrc = disp_ctrl->blenders[bidx].clksrc;

	if (bidx == SDVENC)
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_RF;

	if (cpu_is_ti814x())
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_D;

	if (isdigitalclk(clksrc.clksrc))
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_D;
	else
		return VPS_SYSTEM_VPLL_OUTPUT_VENC_A;
}

/*get the pixel clock for the standard mode*/
static inline int get_pllclock(u32 mid, u32 *freq)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vmode_info); i++) {
		if (vmode_info[i].standard == mid) {
			*freq = vmode_info[i].minfo.pixelclock;
			if ((mid == FVID2_STD_NTSC) ||
			    (mid == FVID2_STD_PAL)) {
				if (cpu_is_ti814x())
					*freq = 54000;
			}
		return 0;
		}
	}
	return -EINVAL;
}
/*get the current format based on the mode id*/
static int dc_get_format_from_mid(int mid,
					    u32 *width,
					    u32 *height,
					    u8 *scformat)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vmode_info); i++) {
		if (mid == vmode_info[i].standard) {
			*width = vmode_info[i].minfo.width;
			*height = vmode_info[i].minfo.height;
			*scformat = vmode_info[i].minfo.scanformat;
			return 0;
		}
	}

	return -EINVAL;
}

static int dc_get_timing(int mid, struct fvid2_modeinfo *minfo)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(vmode_info); i++) {
		if (mid == vmode_info[i].standard)  {
			memcpy(minfo, &vmode_info[i].minfo, sizeof(*minfo));
			if ((mid == FVID2_STD_NTSC) || (mid == FVID2_STD_PAL)) {
				if (cpu_is_ti814x())
					minfo->pixelclock = 54000;
			}

			return 0;
		}
	}
	return -EINVAL;
}
/*get the index of the desired venc id in the database*/
static int get_idx_from_vid(int vid, int *idx)
{
	int i;

	for (i = 0; i < disp_ctrl->numvencs; i++) {
		if (vid == venc_name[i].vid) {
			*idx = venc_name[i].idx;
			return 0;
		}
	}

	return -EINVAL;
}

/*get the venc id based on the name*/
static int dc_get_vencid(char *vname, int *vid)
{

	int i;

	for (i = 0; i < disp_ctrl->numvencs; i++) {
		const struct dc_vencname_info *vnid = &venc_name[i];
		if (sysfs_streq(vname, vnid->name)) {
			*vid = vnid->vid;
			return 0;
		}
	}
	return -1;
}
/*get the blender id from the indx */
static int get_bid_from_idx(int idx, int *bid)
{
	int i;

	for (i = 0; i < disp_ctrl->numvencs; i++) {
		const struct dc_vencname_info *vnid = &venc_name[i];
		if (vnid->idx == idx) {
			*bid = vnid->bid;
			return 0;
		}
	}
	return -1;

}
/*get the mode id based on the mode name*/
static int dc_get_modeid(char *mname, int *mid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vmode_info); i++) {
		const struct dc_vencmode_info *vinfo = &vmode_info[i];
		if (sysfs_streq(mname, vinfo->name)) {
			*mid = vinfo->standard;
			return 0;
		}
	}
	return -1;
}

/*get the node id based on the name*/
static int dc_get_nodeid(char *name, int *nid)
{
	int i;


	if (sysfs_streq(name, "dummy"))
		return -EINVAL;

	/*FIXME hdcomp is not valid for ti814x platform*/
	/*
	if (cpu_is_ti814x() &&
	    (sysfs_streq(name, dc_nodes[VPS_DC_HDCOMP_BLEND].name)))
		return -EINVAL;
	*/
	for (i = 0; i < ARRAY_SIZE(dc_nodes); i++) {
		const struct vps_sname_info *ninfo = &dc_nodes[i];
		if (sysfs_streq(name, ninfo->name)) {
			*nid =  ninfo->value;
			return 0;
		}
	}
	return -EINVAL;
}

/*get the venc information from M3*/
static int  dc_get_vencinfo(struct vps_dcvencinfo *vinfo)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	memcpy(disp_ctrl->vinfo, vinfo, sizeof(struct vps_dcvencinfo));
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_MODE,
			      (void *)disp_ctrl->vinfo_phy,
			      NULL);

	if (r)
		VPSSERR("failed to get venc info.\n");
	else
		memcpy(vinfo,
		       disp_ctrl->vinfo,
		       sizeof(struct vps_dcvencinfo));

	return r;

}

/*is venc running*/
static int dc_isvencrunning(int vid)
{
	struct vps_dcvencinfo vinfo;
	int r = 0;
	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = vid;
	r = dc_get_vencinfo(&vinfo);

	if (!r)
		return vinfo.modeinfo[0].isvencrunning;

	return r;
}

/*Get the current VENC output info*/
static int dc_get_output(struct vps_dcoutputinfo *oinfo)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	/*get the venc output info*/
	disp_ctrl->opinfo->vencnodenum = oinfo->vencnodenum;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_OUTPUT,
			      (void *)disp_ctrl->opinfo_phy,
			      NULL);

	if (r)
		VPSSERR("failed to get venc output info\n");
	else
		memcpy(oinfo,
		       disp_ctrl->opinfo,
		       sizeof(struct vps_dcoutputinfo));
	return r;
}

/*Set the VENC outputs*/
static int dc_set_output(struct vps_dcoutputinfo *oinfo)
{
	int r;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;


	memcpy(disp_ctrl->opinfo, oinfo, sizeof(struct vps_dcoutputinfo));
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_SET_VENC_OUTPUT,
		(void *)disp_ctrl->opinfo_phy,
		NULL);
	if (r)
		VPSSERR("failed to set venc output\n");

	return r;

}
/*set up the pll clock*/
static int dc_set_pllclock(int bidx, u32 clock)
{
	struct vps_systemvpllclk pll;
	int r = 0;

	/*FIXME: call function of HDMI driver to set HDMI for ti814x*/
	if (cpu_is_ti814x() && (bidx == HDMI))
		return r;

	pll.outputvenc = (enum vps_vplloutputclk)get_plloutputvenc(bidx);
	pll.outputclk = clock;
	r = vps_system_setpll(&pll);
	return r;
}

static int dc_set_pll_by_mid(int bidx, int mid)
{
	int r = 0;
	u32  clock;

	r =  get_pllclock(mid, &clock);
	if (r) {
		VPSSERR("nonexit mode %d\n", mid);
		return r;
	}

	r = dc_set_pllclock(bidx, clock);

	return r;
}

/*get the clock source*/
static int dc_get_clksrc(struct vps_dcvencclksrc *clksrc)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	disp_ctrl->clksrc->venc = clksrc->venc;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_VENC_CLK_SRC,
			      (void *)disp_ctrl->clksrc_phy,
			      NULL);

	if (r)
		VPSSERR("get clock source failed\n");
	else
		clksrc->clksrc = disp_ctrl->clksrc->clksrc;

	return r;
}

/*set the clock source*/
static int dc_set_clksrc(struct vps_dcvencclksrc *clksrc)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (!isvalidclksrc(clksrc->venc, clksrc->clksrc)) {
		VPSSERR("invalid clock source\n");
		return -EINVAL;
	}

	disp_ctrl->clksrc->venc = clksrc->venc;
	disp_ctrl->clksrc->clksrc = clksrc->clksrc;

	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_SET_VENC_CLK_SRC,
			      (void *)disp_ctrl->clksrc_phy,
			      NULL);
	if (r)
		VPSSERR("set clock source failed\n");

	return r;
}
/*get the format based on the venc id*/
static int dc_get_format_from_vid(int vid,
					   u32 *width,
					   u32 *height,
					   u8 *scformat)
{
	int r = 0;
	struct vps_dcvencinfo vinfo;

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = vid;

	r = dc_get_vencinfo(&vinfo);
	if (r)
		return -EINVAL;

	if (vinfo.modeinfo[0].isvencrunning == 0) {
		/*venc is not running, get from local structure*/
		int idx;
		r = get_idx_from_vid(vid, &idx);
		if (r)
			return r;
		memcpy(&vinfo.modeinfo[0].minfo, &venc_info.modeinfo[idx].minfo,
			sizeof(struct fvid2_modeinfo));

	}

	if (vinfo.modeinfo[0].minfo.standard == FVID2_STD_CUSTOM) {
		*width = vinfo.modeinfo[0].minfo.width;
		*height = vinfo.modeinfo[0].minfo.height;
		*scformat = vinfo.modeinfo[0].minfo.scanformat;
	} else {
		r = dc_get_format_from_mid(vinfo.modeinfo[0].minfo.standard,
					   width,
					   height,
					   scformat);
	}


	return 0;
}

/*get the format based on the blender id*/
static int dc_get_format_from_bid(int bid,
					   u32 *width,
					   u32 *height,
					   u8 *scformat)
{
	int i;
	int r = -EINVAL;
	for (i = 0; i < disp_ctrl->numvencs; i++) {
		if (bid == venc_name[i].bid) {
			r = dc_get_format_from_vid(venc_name[i].vid,
						   width,
						   height,
						   scformat);
			break;
		}
	}

	return r;
}

/*disable the desired vencs*/
static int dc_venc_disable(int vid)
{
	int i = 0;
	int r = 0;
	struct vps_dcvencinfo vinfo;
	int venc_ids = vid;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (vid == 0)
		return 0;

	if (vid & (~disp_ctrl->vencmask)) {
		VPSSERR("wrong venc id.\n");
		return -EINVAL;
	}


	VPSSDBG("enter venc disable\n");

	vinfo.numvencs = 0;
	/*get the id of each venc to be disabled*/
	while (venc_ids >> i) {
		if ((venc_ids >> i++) & 1)
			vinfo.modeinfo[vinfo.numvencs++].vencid =
							1 << (i - 1);
	}

	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR("faild to get venc info.\n");
		return r;
	}

	venc_ids = vid;
	for (i = 0; i < vinfo.numvencs; i++) {
		if (vinfo.modeinfo[i].isvencrunning == 0) {
			VPSSERR("venc %d already stop\n",
				vinfo.modeinfo[i].vencid);
			venc_ids &= ~vinfo.modeinfo[i].vencid;
		}
	}

	if (venc_ids && !r) {
		*disp_ctrl->dis_vencs = venc_ids;
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_DISABLE_VENC,
				      (void *)disp_ctrl->dis_vencsphy,
				      NULL);

		if (r == 0) {
			disp_ctrl->enabled_venc_ids &= ~venc_ids;
			if (disp_ctrl->tiedvenc) {
				disp_ctrl->tiedvenc &= ~venc_ids;
				venc_ids = 0;
				i = 0;
				/*calculate how vencs left in tied list*/
				while (disp_ctrl->tiedvenc >> i) {
					if ((disp_ctrl->tiedvenc >> i++) & 1)
						venc_ids++;

				}
				/*if one venc left,set tiedvenc to zero*/
				if (venc_ids == 1)
					disp_ctrl->tiedvenc = 0;
			}
		} else
			VPSSERR("failed to disable the venc.\n");

	}

	return r;
}

/*set the mode for desired vencs*/
static int dc_set_vencmode(struct vps_dcvencinfo *vinfo)
{
	int i, r = 0;
	int vencs = 0;
	struct vps_dcvencinfo vi;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;


	/*get the current setting based on the app inputs*/
	for (i = 0; i < vinfo->numvencs; i++)
		vi.modeinfo[i].vencid = vinfo->modeinfo[i].vencid;

	vi.numvencs = vinfo->numvencs;

	r = dc_get_vencinfo(&vi);

	if (r) {
		VPSSERR("failed to get venc info.\n");
		goto exit;
	}

	/*make sure current venc status is matching */
	disp_ctrl->vinfo->numvencs = 0;
	disp_ctrl->vinfo->tiedvencs = 0;
	for (i = 0; i < vinfo->numvencs; i++) {
		if (vi.modeinfo[i].isvencrunning) {
			if (vi.modeinfo[i].minfo.standard !=
			    vinfo->modeinfo[i].minfo.standard) {
				r = -EINVAL;
				VPSSERR("venc %d already running with \
						different mode\n",
						vi.modeinfo[i].vencid);
				goto exit;
			} else
				VPSSDBG("venc %d already running\n",
					vi.modeinfo[i].vencid);

		} else {
			memcpy(&disp_ctrl->vinfo->modeinfo[0],
			       &vinfo->modeinfo[i],
			       sizeof(struct vps_dcmodeinfo));
			vencs |= vinfo->modeinfo[i].vencid;
			/*remove the below codes once firmware gets fix*/
			disp_ctrl->vinfo->numvencs = 1;
			if (disp_ctrl->vinfo->numvencs) {
				mdelay(10);
				/*set the VENC Mode*/
				r = vps_fvid2_control(disp_ctrl->fvid2_handle,
						IOCTL_VPS_DCTRL_SET_VENC_MODE,
						(void *)disp_ctrl->vinfo_phy,
						NULL);
				if (r) {
					VPSSERR("failed to set venc mdoe.\n");
					goto exit;
				}
				disp_ctrl->enabled_venc_ids |= vencs;
			}


		}
	}
	/*comment out the code until firmware gets fix*/
#if 0
	if (vinfo->tiedvencs) {
		if ((vencs & vinfo->tiedvencs) != vinfo->tiedvencs) {
			r = -EINVAL;
			VPSSERR("can not tied venc\n");
			goto exit;
		} else
			disp_ctrl->vinfo->tiedvencs = vinfo->tiedvencs;
	}

	if (disp_ctrl->vinfo->numvencs) {

		/*set the VENC Mode*/
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				IOCTL_VPS_DCTRL_SET_VENC_MODE,
				(void *)disp_ctrl->vinfo_phy,
				NULL);
		if (r) {
			VPSSERR("failed to set venc mdoe.\n");
			goto exit;
		}
		disp_ctrl->enabled_venc_ids |= vencs;
	}
#endif
exit:
	return r;

}

static int dc_enum_node_input(struct vps_dispctrl *dctrl,
				struct vps_dcenumnodeinput *eninput)
{
	int r = 0;

	if ((dctrl == NULL) || (dctrl->fvid2_handle == NULL))
		return -EINVAL;

	*dctrl->dceninput = *eninput;

	r = vps_fvid2_control(dctrl->fvid2_handle,
			IOCTL_VPS_DCTRL_ENUM_NODE_INPUTS,
			(void *)dctrl->dceninput_phy,
			NULL);

	if (!r)
		*eninput = *dctrl->dceninput;

	return r;

}

static int dc_get_node_status(struct vps_dispctrl *dctrl,
			struct vps_dcnodeinput *ninput)
{
	int r = 0;

	if ((dctrl == NULL) || (dctrl->fvid2_handle == NULL))
		return -EINVAL;

	*dctrl->nodeinfo = *ninput;

	r = vps_fvid2_control(dctrl->fvid2_handle,
			IOCTL_VPS_DCTRL_GET_NODE_INPUT_STATUS,
			(void *)dctrl->ninfo_phy,
			NULL);

	if (r)
		VPSSERR("failed to get node status\n");
	else
		*ninput = *dctrl->nodeinfo;

	return r;

}

static int dc_get_comp_rtconfig(struct vps_dispctrl *dctrl,
			struct vps_dccomprtconfig *compcfg)
{
	int r = 0;

	if ((dctrl == NULL) || (dctrl->fvid2_handle == NULL))
		return -EINVAL;

	dctrl->comprtcfg->nodeid = compcfg->nodeid;

	r = vps_fvid2_control(dctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_GET_COMP_RTCONFIG,
		(void *)dctrl->comprtcfg_phy,
		NULL);

	if (r)
		VPSSERR("Get compositor RT config failed\n");
	else
		*compcfg = *dctrl->comprtcfg;

	return r;


}

static int dc_set_comp_rtconfig(struct vps_dispctrl *dctrl,
			struct vps_dccomprtconfig *compcfg)
{
	int r = 0;

	if ((dctrl == NULL) || (dctrl->fvid2_handle == NULL))
		return -EINVAL;

	*dctrl->comprtcfg = *compcfg;
	r = vps_fvid2_control(dctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_SET_COMP_RTCONFIG,
		(void *)dctrl->comprtcfg_phy,
		NULL);
	if (r)
		VPSSERR("Set compositor RT config failed\n");

	return r;

}
/*E******************************** private functions *********************/

/*S*******************************  public functions  *********************/

/*get the id(venc,blender,mode) based on the name*/
int vps_dc_get_id(char *name, int *id, enum dc_idtype type)
{

	int r = -EINVAL;
	switch (type) {
	case DC_BLEND_ID:
	case DC_NODE_ID:
		r = dc_get_nodeid(name, id);
		break;
	case DC_VENC_ID:
		r = dc_get_vencid(name, id);
		break;
	case DC_MODE_ID:
		r = dc_get_modeid(name, id);
		break;
	}

	return r;
}

/*get the tied venc information*/
int vps_dc_get_tiedvenc(u8 *tiedvenc)
{
	*tiedvenc = disp_ctrl->tiedvenc;
	return 0;
}

/*get the venc infor for the desired vencs*/
int vps_dc_get_vencinfo(struct vps_dcvencinfo *vinfo)
{
	int r;
	dc_lock(disp_ctrl);
	r = dc_get_vencinfo(vinfo);
	dc_unlock(disp_ctrl);

	return r;
}

/*get the node name based on the id*/
int vps_dc_get_node_name(int id, char *name)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(dc_nodes); i++) {
		const struct vps_sname_info *ninfo = &dc_nodes[i];
		if (id == ninfo->value) {
			strcpy(name, (char *)ninfo->name);
			return 0;
		}
	}
	return -EINVAL;

}

/*set dc config not used now*/
int vps_dc_set_config(struct vps_dcconfig *usercfg, int setflag)
{
	int r = 0;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	if (usercfg->vencinfo.numvencs > disp_ctrl->numvencs) {
		VPSSERR("num vens (%d) over max\n",
			usercfg->vencinfo.numvencs);
		return -EINVAL;
	}
	if (usercfg->vencinfo.tiedvencs & (~disp_ctrl->tiedvenc)) {
		VPSSERR("tied venc not match.\n");
		return -EINVAL;
	}
	VPSSDBG("enter set config\n");
	dc_lock(disp_ctrl);

	memcpy(disp_ctrl->dccfg, usercfg, sizeof(struct vps_dcconfig));


	if (setflag) {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_SET_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);
		if (r)
			VPSSDBG("faield to set the DC config.\n");
	} else {
		r = vps_fvid2_control(disp_ctrl->fvid2_handle,
				      IOCTL_VPS_DCTRL_CLEAR_CONFIG,
				      (void *)disp_ctrl->dccfg_phy,
				      NULL);

		if (r)
			VPSSDBG("faield to clear the DC config.\n");

	}

	dc_unlock(disp_ctrl);

	return r;
}

/*get current venc output format*/
int vps_dc_get_outpfmt(int id, u32 *width,
		       u32 *height,
		       u8 *scformat,
		       enum dc_idtype type)
{
	int r;

	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter get output format\n");

	dc_lock(disp_ctrl);
	if (type == DC_VENC_ID)
		r = dc_get_format_from_vid(id, width, height, scformat);
	 else if (type == DC_BLEND_ID)
		r = dc_get_format_from_bid(id, width, height, scformat);
	 else if (type == DC_MODE_ID)
		r = dc_get_format_from_mid(id, width, height, scformat);
	 else
		r = -EINVAL;

	dc_unlock(disp_ctrl);
	return r;
}

/* set/clear the node path/edge */
int vps_dc_set_node(u8 nodeid, u8 inputid, u8 enable)
{

	int r = 0;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL))
		return -EINVAL;

	VPSSDBG("enter set node\n");
	dc_lock(disp_ctrl);

	disp_ctrl->nodeinfo->nodeid = nodeid;
	disp_ctrl->nodeinfo->inputid = inputid;

	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_GET_NODE_INPUT_STATUS,
			      (void *)disp_ctrl->ninfo_phy,
			      NULL);

	if (r) {
		VPSSERR("failed to get node input status\n");
		goto exit;
	}
	if (disp_ctrl->nodeinfo->isenable == enable) {
		if (enable)
			VPSSDBG("node already connected\n");
		else
			VPSSDBG("node already disconnected\n");

		goto exit;
	}
	/*call ioctl to set/clear the node */
	disp_ctrl->nodeinfo->isenable = enable;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
			      IOCTL_VPS_DCTRL_NODE_INPUT,
			      (void *)disp_ctrl->ninfo_phy,
			      NULL);
	if (r)
		VPSSERR("failed to enable node.\n");

exit:
	dc_unlock(disp_ctrl);
	return r;
}

int vps_dc_set_color(struct vps_dccigrtconfig *cigconfig)
{
	int r;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL) ||
		(cigconfig == NULL))
		return -EINVAL;

	VPSSDBG("set color\n");
	dc_lock(disp_ctrl);

	memcpy(disp_ctrl->cigcfg, cigconfig, sizeof(*cigconfig));

	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_SET_CIG_RTCONFIG,
		(void *)disp_ctrl->cigcfg_phy,
		NULL);
	if (r)
		VPSSERR("failed to set CIG color\n");

	dc_unlock(disp_ctrl);
	return r;
}

int vps_dc_get_color(struct vps_dccigrtconfig *cigconfig)
{
	int r;
	if ((disp_ctrl == NULL) || (disp_ctrl->fvid2_handle == NULL) ||
		(cigconfig == NULL))
		return -EINVAL;

	VPSSDBG("get color\n");
	dc_lock(disp_ctrl);

	disp_ctrl->cigcfg->nodeid = cigconfig->nodeid;
	r = vps_fvid2_control(disp_ctrl->fvid2_handle,
		IOCTL_VPS_DCTRL_GET_CIG_RTCONFIG,
		(void *)disp_ctrl->cigcfg_phy,
		NULL);
	if (!r)
		memcpy(cigconfig, disp_ctrl->cigcfg, sizeof(*cigconfig));
	else
		VPSSERR("failed to get cig color\n");

	dc_unlock(disp_ctrl);
	return r;

}

int vps_dc_enum_node_input(struct vps_dcenumnodeinput *eninput)
{
	int r = 0;

	VPSSDBG("enum node input\n");
	dc_lock(disp_ctrl);
	r = dc_enum_node_input(disp_ctrl, eninput);
	dc_unlock(disp_ctrl);
	return r;
}

int vps_dc_get_node_status(struct vps_dcnodeinput *ninput)
{
	int r = 0;

	VPSSDBG("get node status\n");
	dc_lock(disp_ctrl);
	r = dc_get_node_status(disp_ctrl, ninput);
	dc_unlock(disp_ctrl);
	return r;

}
int vps_dc_get_timing(u32 bid, struct fvid2_modeinfo *tinfo)
{
	int i;


	for (i = 0; i < disp_ctrl->numvencs; i++) {
		if (bid == venc_name[i].bid) {
			*tinfo = venc_info.modeinfo[i].minfo;
			if ((tinfo->standard == FVID2_STD_NTSC) ||
				(tinfo->standard == FVID2_STD_PAL)) {
				if (cpu_is_ti814x())
					tinfo->pixelclock = 54000;
			}


			return 0;
		}
	}
	return -EINVAL;
}
/*E********************************* public functions *****************/

/*sysfs function for blender starting from here*/
static ssize_t blender_mode_show(struct dc_blender_info *binfo, char *buf)
{
	int i;
	u32 idx = binfo->idx;
	int l = 0;
	for (i = 0; i < ARRAY_SIZE(vmode_info); i++) {
		u32 standard = venc_info.modeinfo[idx].minfo.standard;
		if (standard == FVID2_STD_CUSTOM) {
			if (venc_info.modeinfo[idx].minfo.scanformat ==
			    FVID2_SF_INTERLACED)
				l = snprintf(buf, PAGE_SIZE, "%ux%u@%ui\n",
				venc_info.modeinfo[idx].minfo.width,
				venc_info.modeinfo[idx].minfo.height,
				venc_info.modeinfo[idx].minfo.fps);
			else
				l = snprintf(buf, PAGE_SIZE, "%ux%u@%u\n",
					venc_info.modeinfo[idx].minfo.width,
					venc_info.modeinfo[idx].minfo.height,
					venc_info.modeinfo[idx].minfo.fps);

		} else if (vmode_info[i].standard == standard) {
			l = snprintf(buf, PAGE_SIZE, "%s\n",
				vmode_info[i].name);
			break;
		}
	}
	return l;
}

static ssize_t blender_mode_store(struct dc_blender_info *binfo,
		const char *buf, size_t size)
{
	int r = 0;
	u32 idx = binfo->idx;
	u32 mid;
	int enc_status = 0;
	int dummy = 0;
	struct TI81xx_video_timings timings;
	struct ti81xx_external_encoder *extenc =
			&external_encs[binfo->idx][0];

	dc_lock(binfo->dctrl);

	/*venc should be stop before changes*/

	if (dc_isvencrunning(venc_info.modeinfo[idx].vencid)) {
		VPSSERR("stop venc before changing mode\n");
		r = -EINVAL;
		goto exit;
	}

	if (dc_get_modeid((char *)buf, &mid)) {
		VPSSERR("failed to get the mode %s.", buf);
		r = -EINVAL;
		goto exit;
	}
	/*make sure the mode is supported by the venc*/
	if (!isvalidmode(venc_info.modeinfo[idx].vencid, mid))
		goto exit;


	/*only set the PLL if it is auto mode*/
	r = dc_set_pll_by_mid(binfo->idx, mid);
	if (r)
		goto exit;

	venc_info.modeinfo[idx].minfo.standard = mid;
	dc_get_timing(mid, &venc_info.modeinfo[idx].minfo);
	if (cpu_is_ti816x() && (!def_i2cmode)) {
		if ((binfo->idx == HDCOMP) && (binfo->isdeviceon == true)) {
			if ((mid == FVID2_STD_1080P_60) ||
			    (mid == FVID2_STD_1080P_50))
				r = pcf8575_ths7360_hd_enable(
					TI816X_THS7360_SF_TRUE_HD_MODE);
			else
				r = pcf8575_ths7360_hd_enable(
					TI816X_THS7360_SF_HD_MODE);
			if (r < 0) {
				VPSSERR("failed to set THS filter\n");
				goto exit;
			}

		}
	}
	/* if an external encoder is registered to this blender,
		change the mode  */
	enc_status = extenc->status;
	if ((enc_status == TI81xx_EXT_ENCODER_DISABLED) ||
	    (enc_status == TI81xx_EXT_ENCODER_REGISTERED)) {
		/* Change mode */
		/* FIXME :
		   1. In later release two different enumerations will
		   not be used. Only the FVID enumerations for
		   resolutions.
	 *         2. Currently only support 4 resolutions, till PLL
		      locking is tested on other Std resolutions.
	 */
		timings.standard = mid;
		timings.dvi_hdmi = TI81xx_MODE_HDMI;
		if ((extenc->panel_driver) &&
		    (extenc->panel_driver->set_timing))
			extenc->panel_driver->set_timing(
						&timings,
						(void *)dummy);
	}

	r = size;
exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_timings_show(struct dc_blender_info *binfo, char *buf)
{
	int r;
	struct fvid2_modeinfo *t;

	t = &venc_info.modeinfo[binfo->idx].minfo;
	r = snprintf(buf,
			PAGE_SIZE,
			"%u,%u/%u/%u/%u,%u/%u/%u/%u,%u\n",
			t->pixelclock,
			t->width, t->hfrontporch, t->hbackporch, t->hsynclen,
			t->height, t->vfrontporch, t->vbackporch, t->vsynclen,
			t->scanformat);

	return r;

}

static ssize_t blender_timings_store(struct dc_blender_info *binfo,
		const char *buf, size_t size)
{
	int r = 0;
	struct fvid2_modeinfo t;
	u32 num;
	u32 vmode;
	if (binfo->idx == SDVENC)
		return -EINVAL;

	dc_lock(binfo->dctrl);

	/*venc should be stop before changes*/
	if (dc_isvencrunning(venc_info.modeinfo[binfo->idx].vencid)) {
		VPSSERR("stop venc before changing timing\n");
		r = -EINVAL;
		goto exit;
	}

	num = sscanf(buf, "%u,%u/%u/%u/%u,%u/%u/%u/%u,%u/%u",
			&t.pixelclock,
			&t.width, &t.hfrontporch, &t.hbackporch, &t.hsynclen,
			&t.height, &t.vfrontporch, &t.vbackporch, &t.vsynclen,
			&t.scanformat, &vmode);

	if (!((num == 11) || (num == 10))) {
		r = -EINVAL;
		VPSSERR("wrong timing input %d\n", num);
		goto exit;
	}
	/*if use did not assign mode, than we fix it to 1*/
	if (num == 10)
		vmode = 1;

	memcpy(&venc_info.modeinfo[binfo->idx].minfo, &t, sizeof(t));
	venc_info.modeinfo[binfo->idx].minfo.standard = FVID2_STD_CUSTOM;
	venc_info.modeinfo[binfo->idx].mode = vmode;
	/*calculate the refresh rate*/
	venc_info.modeinfo[binfo->idx].minfo.fps =
		(t.pixelclock * 1000)  /
		    ((t.width + t.hfrontporch + t.hbackporch + t.hsynclen) *
		    (t.height + t.vfrontporch + t.vbackporch + t.vsynclen));

	if (t.scanformat == 0)
		venc_info.modeinfo[binfo->idx].minfo.fps *= 2;
	r = dc_set_pllclock(binfo->idx, t.pixelclock);

	if (r) {
		VPSSERR("failed to set %dKHz clock\n",
			t.pixelclock);
		r = -EINVAL;
		goto exit;
	}

	r = size;
exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_enabled_show(struct dc_blender_info *binfo, char *buf)
{
	int r;
	struct vps_dcvencinfo vinfo;

	dc_lock(binfo->dctrl);

	vinfo.numvencs = 1;
	vinfo.modeinfo[0].vencid = venc_name[binfo->idx].vid;
	r = dc_get_vencinfo(&vinfo);

	if (r) {
		VPSSERR(" Failed to get venc infor\n");
		r = -EINVAL;
		goto exit;
	}

	r = snprintf(buf, PAGE_SIZE, "%d\n", vinfo.modeinfo[0].isvencrunning);

exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_enabled_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	int enabled;
	int vid;
	int r = 0;
	int enc_status = 0;
	int dummy = 0;
	struct ti81xx_external_encoder *extenc =
			&external_encs[binfo->idx][0];

	enabled = simple_strtoul(buf, NULL, 10);

	dc_lock(disp_ctrl);
	/*get vid id*/
	vid = venc_name[binfo->idx].vid;

	/* FIXME : If an external encoder is not registered per blender,
	we need not enable the VENC at all */

	if (enabled == 0) {
		r = dc_venc_disable(vid);
		if (r) {
			VPSSERR("failed to disable %s venc\n",
				binfo->name);
			r = -EINVAL;
			goto exit;
		}
		/* If external encoder driver registered and enabled
			,disable it */
		enc_status = extenc->status;
		if (enc_status == TI81xx_EXT_ENCODER_ENABLED ||
		   enc_status == TI81xx_EXT_ENCODER_RESUMED) {
			if (extenc->panel_driver &&
			    extenc->panel_driver->disable)
				extenc->panel_driver->disable(
							(void *)dummy);
			extenc->status = TI81xx_EXT_ENCODER_DISABLED;
		} else
			VPSSDBG("External already Disabled\n");
	} else {
		int idx;
		struct vps_dcvencinfo vinfo;
		get_idx_from_vid(vid, &idx);
		memcpy(&vinfo.modeinfo[0],
		   &venc_info.modeinfo[idx],
		   sizeof(struct vps_dcvencinfo));

		vinfo.numvencs = 1;
		vinfo.tiedvencs = 0;
		r = dc_set_vencmode(&vinfo);
		if (r) {
			VPSSERR("failed to enable venc %s\n",
				binfo->name);
			r = -EINVAL;
			goto exit;
		}
		/* If external encoder driver registered,
		   and not enabled yet, enable it */
		enc_status = extenc->status;
		if (enc_status == TI81xx_EXT_ENCODER_UNREGISTERED) {
			r = size;
			VPSSDBG(" ext encoder un-registered.\n");
			goto exit;
		}
		if (enc_status == TI81xx_EXT_ENCODER_SUSPENDED) {
			r = size;
			VPSSDBG(" ext encoder suspended.\n");
			goto exit;
		}
		if (extenc->panel_driver &&
		    extenc->panel_driver->enable)
			extenc->panel_driver->enable((void *)dummy);
		extenc->status = TI81xx_EXT_ENCODER_ENABLED;
	}
	r = size;
exit:
	dc_unlock(disp_ctrl);
	return r;
}

static ssize_t blender_clksrc_show(struct dc_blender_info *binfo, char *buf)
{
	int r = 0;
	struct vps_dcvencclksrc *clksrc = &binfo->clksrc;

	if (binfo->idx == SDVENC) {
		VPSSERR("no clock soure for SD VENC\n");
		return -EINVAL;
	}

	dc_lock(binfo->dctrl);
	clksrc->venc = venc_name[binfo->idx].vid;
	r = dc_get_clksrc(clksrc);
	dc_unlock(binfo->dctrl);
	if (r)
		return r;
	else
		return snprintf(buf, PAGE_SIZE, "%s\n",
			vclksrc_name[clksrc->clksrc].name);

}

static ssize_t blender_clksrc_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	int r = 0, i;
	struct vps_dcvencclksrc clksrc;
	bool found = false;

	if (binfo->idx == SDVENC) {
		VPSSERR("no clock soure for SD VENC\n");
		return -EINVAL;
	}

	dc_lock(binfo->dctrl);
	clksrc.venc = venc_name[binfo->idx].vid;

	if (dc_isvencrunning(clksrc.venc)) {
		VPSSERR("please stop venc before changing clock source\n");
		r = -EINVAL;
		goto exit;
	}
	/*found the matching clock source*/
	for (i = 0; i < ARRAY_SIZE(vclksrc_name); i++) {
		if (sysfs_streq(buf, vclksrc_name[i].name)) {
			clksrc.clksrc = vclksrc_name[i].value;
			found = true;
			break;
		}
	}
	/*set the clock source*/
	if (found == true) {
		r = dc_set_clksrc(&clksrc);
		if (!r) {
			r = size;
			/*store back*/
			binfo->clksrc.clksrc = clksrc.clksrc;
		}
	} else {
		r = -EINVAL;
		VPSSERR("invalid clock source input\n");
	}

exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_output_show(struct dc_blender_info *binfo, char *buf)
{
	struct vps_dcoutputinfo oinfo;
	int r = 0;
	int l = 0, i;

	oinfo.vencnodenum = venc_name[binfo->idx].vid;
	dc_lock(binfo->dctrl);
	r = dc_get_output(&oinfo);
	dc_unlock(binfo->dctrl);
	if (r)
		return -EINVAL;


	if (isdigitalvenc(oinfo.vencnodenum))
		l += snprintf(buf + l,
			      PAGE_SIZE - l, "%s",
			      dfmt_name[oinfo.dvofmt].name);
	else
		l += snprintf(buf + l,
			      PAGE_SIZE - l, "%s",
			      afmt_name[oinfo.afmt].name);

	if (binfo->idx != SDVENC) {
		for (i = 0 ; i < ARRAY_SIZE(datafmt_name); i++) {
			if (datafmt_name[i].value == oinfo.dataformat)
				l += snprintf(buf + l,
					      PAGE_SIZE - l, ",%s",
					      datafmt_name[i].name);
		}
	}
	if (isdigitalvenc(oinfo.vencnodenum))
		l += snprintf(buf + l, PAGE_SIZE - 1, ",%d/%d/%d/%d\n",
			oinfo.dvoactvidpolarity, oinfo.dvofidpolarity,
			oinfo.dvohspolarity, oinfo.dvovspolarity);
	else
		l += snprintf(buf + l, PAGE_SIZE - l, "\n");

	return l;

}

static ssize_t blender_output_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	struct vps_dcoutputinfo oinfo;
	int r = 0;
	char *input = (char *)buf;
	char *ptr;
	enum vps_dcdigitalfmt dfmt = VPS_DC_DVOFMT_MAX;
	enum vps_dcanalogfmt afmt = VPS_DC_A_OUTPUT_MAX;
	enum fvid2_dataformat fmt = FVID2_DF_MAX;
	u8   pol = 0;
	int   polarity[4];
	oinfo.vencnodenum = venc_name[binfo->idx].vid;

	dc_lock(binfo->dctrl);

	/*venc should be off before changed output*/
	if (dc_isvencrunning(oinfo.vencnodenum)) {
		VPSSERR("please disable VENC before changing output\n");
		r = -EINVAL;
		goto exit;
	}

	dc_get_output(&oinfo);


	/*process the input buf*/
	while ((ptr = strsep(&input, ",")) != NULL) {
		int i;
		bool found;
		found = false;
		/*check data format first*/
		for (i = 0; i < ARRAY_SIZE(datafmt_name); i++) {
			if (sysfs_streq(ptr, datafmt_name[i].name)) {
				fmt = datafmt_name[i].value;
				found = true;
			}
		}
		/*check digital format or analog format based on current venc*/
		if (!found) {
			if (isdigitalvenc(oinfo.vencnodenum)) {
				for (i = 0; i < VPS_DC_DVOFMT_MAX; i++)
					if (sysfs_streq(ptr,
					    dfmt_name[i].name)) {
						dfmt = dfmt_name[i].value;
						found = true;
						break;
					}
			} else {
				for (i = 0; i < VPS_DC_A_OUTPUT_MAX; i++)
					if (sysfs_streq(ptr,
					    afmt_name[i].name)) {
						afmt = afmt_name[i].value;
						found = true;
						break;
					}
			}
		}

		if (!found) {
			r = sscanf(ptr, "%u/%u/%u/%u",
				  &polarity[0],
				  &polarity[1],
				  &polarity[2],
				  &polarity[3]);
			if (r != 4) {
				VPSSERR("invalid output value %s\n", ptr);
				r = -EINVAL;
				goto exit;
			}
			pol = 0xFF;
		}

		if (input == NULL)
			break;
	}

	/*make sure the input is right before sending out to M3*/
	if (isdigitalvenc(oinfo.vencnodenum)) {
		if ((dfmt == VPS_DC_DVOFMT_MAX) && (fmt == FVID2_DF_MAX)
		    && (pol == 0)) {
			VPSSERR("no valid digital output settings\n");
			r = -EINVAL;
			goto exit;
		}
		if (dfmt != VPS_DC_DVOFMT_MAX)
			oinfo.dvofmt = dfmt;

		if (pol == 0xFF) {
			oinfo.dvoactvidpolarity = polarity[0];
			oinfo.dvofidpolarity = polarity[1];
			oinfo.dvohspolarity = polarity[2];
			oinfo.dvovspolarity = polarity[3];
		}

	} else {
		if ((afmt == VPS_DC_A_OUTPUT_MAX) && (fmt == FVID2_DF_MAX)) {
			VPSSERR("no valid analog output settings\n");
			r = -EINVAL;
			goto exit;

		}
		if ((binfo->idx == SDVENC) &&
		     (afmt == VPS_DC_A_OUTPUT_COMPONENT)) {
			VPSSERR("component out not supported on sdvenc\n");
			r = -EINVAL;
			goto exit;

		}
		if (afmt != VPS_DC_A_OUTPUT_MAX)
			oinfo.afmt = afmt;
	}

	if (fmt != FVID2_DF_MAX)
		oinfo.dataformat = fmt;

	r = dc_set_output(&oinfo);
	if (!r)
		r = size;
exit:
	dc_unlock(binfo->dctrl);
	return r;

}

static ssize_t blender_order_show(struct dc_blender_info *binfo, char *buf)
{
	int r;
	int l;
	struct vps_dccomprtconfig comprtcfg;

	r = get_bid_from_idx(binfo->idx, &comprtcfg.nodeid);
	if (r)
		return r;

	dc_lock(binfo->dctrl);
	r = dc_get_comp_rtconfig(binfo->dctrl, &comprtcfg);
	dc_unlock(binfo->dctrl);

	if (r)
		return r;
	if (comprtcfg.isglobalreorderenable)
		l = snprintf(buf, PAGE_SIZE, "%u,%u/%u/%u/%u\n",
			comprtcfg.isglobalreorderenable,
			comprtcfg.displayorder[0],
			comprtcfg.displayorder[1],
			comprtcfg.displayorder[2],
			comprtcfg.displayorder[3]);
	else
		l = snprintf(buf, PAGE_SIZE, "%u,%u\n",
				comprtcfg.isglobalreorderenable,
				comprtcfg.displayorder[0]);
	return l;
}

static ssize_t blender_order_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	int r;
	int num;
	struct vps_dccomprtconfig comprtcfg;

	r = get_bid_from_idx(binfo->idx, &comprtcfg.nodeid);
	if (r)
		return r;

	num = sscanf(buf, "%u,%u/%u/%u/%u",
			&comprtcfg.isglobalreorderenable,
			&comprtcfg.displayorder[0],
			&comprtcfg.displayorder[1],
			&comprtcfg.displayorder[2],
			&comprtcfg.displayorder[3]);
	/*error check*/
	if (comprtcfg.isglobalreorderenable == 1) {
		if (num != 5) {
			VPSSERR("Wrong display re-order format\n");
			return -EINVAL;
		}

	} else {
		if (num < 2) {
			VPSSERR("Wrong display re-order format\n");
			return -EINVAL;
		}
	}

	dc_lock(binfo->dctrl);
	r = dc_set_comp_rtconfig(binfo->dctrl, &comprtcfg);
	if (!r)
		r = size;
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_name_show(struct dc_blender_info *binfo, char *buf)
{

	return snprintf(buf, PAGE_SIZE,
			"%s\n", venc_name[binfo->idx].name);
}

static ssize_t blender_edid_show(struct dc_blender_info *binfo, char *buf)
{
	int r = 0;
	int enc_status = 0;
	int dummy = 0;
	struct ti81xx_external_encoder *extenc =
				&external_encs[binfo->idx][0];

	dc_lock(binfo->dctrl);

	/* FIXME : In future, need to support multiple encoders
	that are registered */
	enc_status = extenc->status;
	if ((enc_status != TI81xx_EXT_ENCODER_UNREGISTERED) &&
	    extenc->panel_driver &&
	    extenc->panel_driver->get_edid) {
		r = extenc->panel_driver->get_edid(buf, (void *)dummy);
	}
	printk(KERN_INFO "\nblender_edid_show : r = %d ", r);
	printk(KERN_INFO "\n %x %x %x %x %x %x %x %x\n",
		buf[0], buf[1], buf[2], buf[3],
		buf[126], buf[127], buf[128], buf[129]);
	if (r == -1) {
		VPSSERR(" Failed to get EDID info\n");
		r = -EINVAL;
		goto exit;
	}
	memset(buf, 0, r);
	/* FIXME : doing this becasue we do not want any screen output
	r = snprintf(buf, PAGE_SIZE, "%d\n", vinfo.modeinfo[0].isvencrunning);
	*/

exit:
	dc_unlock(binfo->dctrl);
	return r;
}

static ssize_t blender_edid_store(struct dc_blender_info *binfo,
				     const char *buf,
				     size_t size)
{
	return 0;
}

struct blender_attribute {
	struct attribute attr;
	ssize_t (*show)(struct dc_blender_info *, char *);
	ssize_t (*store)(struct dc_blender_info *, const char *, size_t);
};


#define BLENDER_ATTR(_name, _mode, _show, _store) \
	struct blender_attribute blender_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static BLENDER_ATTR(name, S_IRUGO, blender_name_show, NULL);
static BLENDER_ATTR(mode, S_IRUGO | S_IWUSR,
				blender_mode_show, blender_mode_store);
static BLENDER_ATTR(timings, S_IRUGO | S_IWUSR,
				blender_timings_show, blender_timings_store);
static BLENDER_ATTR(enabled, S_IRUGO | S_IWUSR,
				blender_enabled_show, blender_enabled_store);
static BLENDER_ATTR(output, S_IRUGO | S_IWUSR,
				blender_output_show, blender_output_store);
static BLENDER_ATTR(clksrc, S_IRUGO | S_IWUSR,
		blender_clksrc_show, blender_clksrc_store);
static BLENDER_ATTR(order, S_IRUGO | S_IWUSR,
		blender_order_show, blender_order_store);
/* currently EDID read only */
static BLENDER_ATTR(edid, S_IRUGO,
		blender_edid_show, blender_edid_store);

static struct attribute *blender_sysfs_attrs[] = {
	&blender_attr_mode.attr,
	&blender_attr_timings.attr,
	&blender_attr_enabled.attr,
	&blender_attr_output.attr,
	&blender_attr_clksrc.attr,
	&blender_attr_order.attr,
	&blender_attr_name.attr,
	&blender_attr_edid.attr,
	NULL
};

static ssize_t blender_attr_show(struct kobject *kobj,
				  struct attribute *attr,
				  char *buf)
{
	struct dc_blender_info *binfo = NULL;
	struct blender_attribute *blend_attr = NULL;

	binfo = container_of(kobj, struct dc_blender_info, kobj);

	blend_attr = container_of(attr, struct blender_attribute, attr);
	if (!blend_attr->show)
		return -ENOENT;

	return blend_attr->show(binfo, buf);
}

static ssize_t blender_attr_store(struct kobject *kobj,
				   struct attribute *attr,
				   const char *buf,
				   size_t size)
{
	struct dc_blender_info *blend;
	struct blender_attribute *blend_attr;

	blend = container_of(kobj, struct dc_blender_info, kobj);
	blend_attr = container_of(attr, struct blender_attribute, attr);

	if (!blend_attr->store)
		return -ENOENT;

	return blend_attr->store(blend, buf, size);
}

static const struct sysfs_ops blender_sysfs_ops = {
	.show = blender_attr_show,
	.store = blender_attr_store,
};

static struct kobj_type blender_ktype = {
	.sysfs_ops = &blender_sysfs_ops,
	.default_attrs = blender_sysfs_attrs,
};



/*sysfs for the display controller*/

static ssize_t dctrl_tiedvencs_show(struct vps_dispctrl *dctrl, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", disp_ctrl->tiedvenc);
}

static ssize_t dctrl_tiedvencs_store(struct vps_dispctrl *dctrl,
				     const char *buf,
				     size_t size)
{
	int r = 0;
	int vencs = 0;
	int i = 0;
	struct vps_dcvencinfo vinfo;
	dc_lock(disp_ctrl);
	vencs = simple_strtoul(buf, NULL, 10);
	if (vencs & ~disp_ctrl->vencmask) {
		r = -EINVAL;
		VPSSERR("vencs %d over limit\n", vencs);
		goto exit;
	}

	if ((vencs == 0) || (disp_ctrl->tiedvenc == vencs)) {
		r = size;
		goto exit;
	}

	vinfo.numvencs = 0;
	vinfo.tiedvencs = vencs;

	/*assemble the structure based on the venc id*/
	while (vencs >> i) {
		/*get id of each venc to be tied*/
		if ((vencs >> i++) & 1) {
			int idx;
			int vid = 1 << (i - 1);
			get_idx_from_vid(vid, &idx);
			memcpy(&vinfo.modeinfo[vinfo.numvencs++],
			       &venc_info.modeinfo[idx],
			       sizeof(struct vps_dcmodeinfo));

		}
	}
	if (vinfo.numvencs < 2) {
		VPSSERR("at least 2 vencs to tied.\n");
		r = -EINVAL;
		goto exit;
	}

	/*set the tied venc mode*/
	r = dc_set_vencmode(&vinfo);
	if (r) {
		VPSSERR("failed to set tied venc\n");
		r = -EINVAL;
		goto exit;
	}
	disp_ctrl->tiedvenc = vinfo.tiedvencs;
	r = size;
exit:
	dc_unlock(disp_ctrl);
	return r;
}

struct dctrl_attribute {
	struct attribute attr;
	ssize_t (*show)(struct vps_dispctrl *, char *);
	ssize_t (*store)(struct vps_dispctrl *, const char *, size_t);
};

#define DCTRL_ATTR(_name, _mode, _show, _store) \
	struct dctrl_attribute dctrl_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static DCTRL_ATTR(tiedvencs, S_IRUGO | S_IWUSR,
		dctrl_tiedvencs_show, dctrl_tiedvencs_store);

static struct attribute *dctrl_sysfs_attrs[] = {
	&dctrl_attr_tiedvencs.attr,
	NULL
};

static ssize_t dctrl_attr_show(struct kobject *kobj,
				struct attribute *attr,
				char *buf)
{
	struct vps_dispctrl *dctrl = NULL;
	struct dctrl_attribute *dctrl_attr = NULL;

	dctrl = container_of(kobj, struct vps_dispctrl, kobj);

	dctrl_attr = container_of(attr, struct dctrl_attribute, attr);
	if (!dctrl_attr->show)
		return -ENOENT;

	return dctrl_attr->show(dctrl, buf);
}

static ssize_t dctrl_attr_store(struct kobject *kobj,
				struct attribute *attr,
				const char *buf,
				size_t size)
{
	struct vps_dispctrl *dctrl;
	struct dctrl_attribute *dctrl_attr;

	dctrl = container_of(kobj, struct vps_dispctrl, kobj);
	dctrl_attr = container_of(attr, struct dctrl_attribute, attr);

	if (!dctrl_attr->store)
		return -ENOENT;

	return dctrl_attr->store(dctrl, buf, size);
}


static const struct sysfs_ops dctrl_sysfs_ops = {
	.show = dctrl_attr_show,
	.store = dctrl_attr_store,
};

static struct kobj_type dctrl_ktype = {
	.sysfs_ops = &dctrl_sysfs_ops,
	.default_attrs = dctrl_sysfs_attrs,
};

/*end of sysfs function for display controller*/
static int parse_def_clksrc(const char *clksrc)
{
	int r = 0, i;
	char *str, *options, *this_opt;

	if (clksrc == NULL)
		return 0;

	str = kmalloc(strlen(clksrc) + 1, GFP_KERNEL);
	strcpy(str, clksrc);
	options = str;
	VPSSDBG("clksrc %s\n", clksrc);
	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *venc, *csrc;
		int vid, idx;
		struct dc_blender_info *binfo;

		p = strchr(this_opt, ':');
		if (!p)
			break;

		*p = 0;
		venc = this_opt;
		csrc = p + 1;
		/*parse the clock source for each possible venc input*/
		for (i = 0; i < ARRAY_SIZE(vclksrc_name); i++) {
			if (sysfs_streq(csrc, vclksrc_name[i].name)) {
				if (dc_get_vencid(venc, &vid)) {
					VPSSERR("wrong venc\n");
					break;
				}
				/* no clock for SD VENC*/
				if (vid == VPS_DC_VENC_SD)
					break;

				get_idx_from_vid(vid, &idx);
				binfo = &disp_ctrl->blenders[idx];
				/*is valid clock source*/
				if (isvalidclksrc(vid,
				      vclksrc_name[i].value)) {
					binfo->clksrc.clksrc =
							vclksrc_name[i].value;

				} else
					VPSSERR("wrong clock source\n");

				break;

			}
		}
		if (i == ARRAY_SIZE(vclksrc_name))
			VPSSERR("wrong clock source\n");

		if (options == NULL)
			break;


	}
	kfree(str);
	return r;
}

static int parse_def_modes(const char *mode)

{
	char *str, *options, *this_opt;
	int r = 0;
	struct vps_dcvencinfo *vinfo = &venc_info;
	if (mode == NULL)
		return 0;

	str = kmalloc(strlen(mode) + 1, GFP_KERNEL);
	strcpy(str, mode);
	options = str;
	VPSSDBG("mode %s\n", mode);

	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		char *p, *display_str, *mode_str;
		int vid, mid;
		int idx;
		p = strchr(this_opt, ':');
		if (!p) {
			r = -EINVAL;
			break;
		}
		*p = 0;
		display_str = this_opt;
		mode_str = p + 1;

		if (dc_get_vencid(display_str, &vid)) {
			VPSSERR("venc name(%s) not existing.\n",
				display_str);
			continue;
		}
		if (dc_get_modeid(mode_str, &mid)) {
			VPSSERR("venc mode(%s) is not supported.\n",
				mode_str);
			continue;
		}

		if (!isvalidmode(vid, mid))
			continue;

		get_idx_from_vid(vid, &idx);
		vinfo->modeinfo[idx].vencid = vid;
		vinfo->modeinfo[idx].minfo.standard = mid;
		dc_get_timing(mid, &vinfo->modeinfo[idx].minfo);

	   if (options == NULL)
			break;
	}

	kfree(str);

	return r;

}
void __init vps_dc_ctrl_init(struct vps_dispctrl *dctrl)
{
	struct vps_dcedeconfig *edecfg = dctrl->dcedecfg;
	struct vps_cscconfig *hdccfg = dctrl->hdcompcsccfg;
	struct vps_cscconfig *vccfg = dctrl->vcompcsccfg;

	dctrl->dccreatecfg->edeconfig =
		(struct vps_dcedeconfig *)dctrl->dcede_phy;
	/*setup default ede values*/
	edecfg->ltienable = 1;
	edecfg->horzpeaking = 1;
	edecfg->ctienable = 1;
	edecfg->transadjustenable = 1;
	edecfg->lumapeaking = 1;
	edecfg->chromapeaking = 1;
	edecfg->minclipluma = 0;
	edecfg->maxclipluma = 1023;
	edecfg->minclipchroma = 0;
	edecfg->maxclipchroma = 1023;
	edecfg->bypass = 0;


	/*setup default CSC information for HDCOMP and VCOMP*/
	dctrl->dccreatecfg->hdcompcscconfig =
		(struct vps_cscconfig *)dctrl->hdcompcsccfg_phy;
	if (cpu_is_ti814x())
		dctrl->dccreatecfg->vcompcscconfig =
			(struct vps_cscconfig *)dctrl->vcompcsccfg_phy;
	else
		dctrl->dccreatecfg->cprocconfig =
			(struct vps_cprocconfig *)dctrl->cproccfg_phy;

	hdccfg->bypass = 0;
	hdccfg->coeff = NULL;
	hdccfg->mode = VPS_CSC_MODE_HDTV_GRAPHICS_Y2R;

	if (cpu_is_ti814x()) {
		vccfg->bypass = 0;
		vccfg->coeff = NULL;
		vccfg->mode = VPS_CSC_MODE_HDTV_GRAPHICS_Y2R;
	} else
		dctrl->cproccfg->ciecammode = VPS_CPROC_CIECAM_MODE_BT709;

}
static inline int get_payload_size(void)
{
	int size = 0;
	size  = sizeof(struct vps_dccreateconfig);
	size += sizeof(struct vps_dcedeconfig);
	size += sizeof(u32);
	size += sizeof(struct vps_dcconfig);
	size += sizeof(struct vps_dcvencinfo);
	size += sizeof(struct vps_dcnodeinput);
	size += sizeof(struct vps_dcmodeinfo);
	size += sizeof(struct vps_dcoutputinfo);
	size += sizeof(struct vps_dcvencclksrc);
	size += sizeof(struct vps_dccigrtconfig);
	size += sizeof(struct vps_dcenumnodeinput);
	size += sizeof(struct vps_dccomprtconfig);
	size += sizeof(struct vps_cscconfig);  /*hdcomp*/
	if (cpu_is_ti814x())
		size += sizeof(struct vps_cscconfig);  /*vcomp(ti814x only) */
	else
		size += sizeof(struct vps_cprocconfig); /*cproc ti816x only*/
	size += sizeof(struct vps_cscconfig);  /*sd*/
	size += sizeof(u32);  /*this is for the disable venc command*/
	/*FIXME add more here*/

	return size;
}

static inline void assign_payload_addr(struct vps_dispctrl *dctrl,
				       struct vps_payload_info *pinfo,
				       u32 *buf_offset)
{
	int offset = *buf_offset;

	/*dc create config*/
	dctrl->dccreatecfg = (struct vps_dccreateconfig *)setaddr(pinfo,
					&offset,
					&dctrl->dccreate_phy,
					sizeof(struct vps_dccreateconfig));

	/*ede config*/
	dctrl->dcedecfg = (struct vps_dcedeconfig *)setaddr(pinfo,
					&offset,
					&dctrl->dcede_phy,
					sizeof(struct vps_dcedeconfig));
	/*return status*/
	dctrl->dcrtstatus = (u32 *)setaddr(pinfo,
					&offset,
					&dctrl->dcrtst_phy,
					sizeof(u32));
	/*dc config */
	dctrl->dccfg = (struct vps_dcconfig *)setaddr(pinfo,
				      &offset,
				      &dctrl->dccfg_phy,
				      sizeof(struct vps_dcconfig));

	/* venc info*/
	dctrl->vinfo = (struct vps_dcvencinfo *)setaddr(pinfo,
					&offset,
					&dctrl->vinfo_phy,
					sizeof(struct vps_dcvencinfo));

	/*node input*/
	dctrl->nodeinfo = (struct vps_dcnodeinput *)setaddr(
					pinfo,
					&offset,
					&dctrl->ninfo_phy,
					sizeof(struct vps_dcnodeinput));
	/*venc disable*/
	dctrl->dis_vencs = (u32 *)setaddr(pinfo,
				  &offset,
				  &dctrl->dis_vencsphy,
				  sizeof(u32));

	/*venc output infor*/
	dctrl->opinfo = (struct vps_dcoutputinfo *)setaddr(
					pinfo,
					&offset,
					&dctrl->opinfo_phy,
					sizeof(struct vps_dcoutputinfo));

	/*venc clock source*/
	dctrl->clksrc = (struct vps_dcvencclksrc *)setaddr(
					pinfo,
					&offset,
					&dctrl->clksrc_phy,
					sizeof(struct vps_dcvencclksrc));

	/*CIG runtime configuration*/
	dctrl->cigcfg = (struct vps_dccigrtconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->cigcfg_phy,
					sizeof(struct vps_dccigrtconfig));
	/*DC enum node input*/
	dctrl->dceninput = (struct vps_dcenumnodeinput *)setaddr(
					pinfo,
					&offset,
					&dctrl->dceninput_phy,
					sizeof(struct vps_dcenumnodeinput));
	/*COMP RT Config*/
	dctrl->comprtcfg = (struct vps_dccomprtconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->comprtcfg_phy,
					sizeof(struct vps_dccomprtconfig));

	/*hdcomp csc config*/
	dctrl->hdcompcsccfg = (struct vps_cscconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->hdcompcsccfg_phy,
					sizeof(struct vps_cscconfig));

	/*vcomp csc config*/
	if (cpu_is_ti814x())
		dctrl->vcompcsccfg = (struct vps_cscconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->vcompcsccfg_phy,
					sizeof(struct vps_cscconfig));
	else
		dctrl->cproccfg = (struct vps_cprocconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->cproccfg_phy,
					sizeof(struct vps_cprocconfig));

	/*sd csc config*/
	dctrl->sdcsccfg = (struct vps_cscconfig *)setaddr(
					pinfo,
					&offset,
					&dctrl->sdcsccfg_phy,
					sizeof(struct vps_cscconfig));
	*buf_offset = offset;
}

int __init vps_dc_init(struct platform_device *pdev,
			  const char *mode,
			  int tied_vencs,
			  const char *clksrc)
{
	int r = 0;
	int i;
	int size = 0, offset = 0;

	VPSSDBG("dctrl init\n");

	if (cpu_is_ti816x() && (!def_i2cmode))
		ti816x_pcf8575_init();

	dc_payload_info = kzalloc(sizeof(struct vps_payload_info),
				  GFP_KERNEL);

	if (!dc_payload_info) {
		VPSSERR("allocated payload info failed.\n");
		return -ENOMEM;
	}

	/*allocate non-cacheable memory*/
	size = get_payload_size();
	dc_payload_info->vaddr = vps_sbuf_alloc(size, &dc_payload_info->paddr);
	if (dc_payload_info->vaddr == NULL) {
		VPSSERR("alloc dctrl dma buffer failed\n");
		dc_payload_info->paddr = 0u;
		r = -ENOMEM;
		goto cleanup;
	}
	dc_payload_info->size = PAGE_ALIGN(size);
	memset(dc_payload_info->vaddr, 0, dc_payload_info->size);
	/*allocate display_control memory*/
	disp_ctrl = kzalloc(sizeof(struct vps_dispctrl), GFP_KERNEL);
	if (disp_ctrl == NULL) {
		r = -ENOMEM;
		goto cleanup;
	}
	disp_ctrl->numvencs = vps_get_numvencs();
	venc_info.numvencs = disp_ctrl->numvencs;
	disp_ctrl->vencmask = (1 << VPS_DC_MAX_VENC) - 1;
	if (cpu_is_ti814x())
		disp_ctrl->vencmask -= VPS_DC_VENC_HDCOMP;

	assign_payload_addr(disp_ctrl, dc_payload_info, &offset);

	vps_dc_ctrl_init(disp_ctrl);
	/*get dc handle*/
	dc_handle = vps_fvid2_create(FVID2_VPS_DCTRL_DRV,
				     VPS_DCTRL_INST_0,
				     (void *)disp_ctrl->dccreate_phy,
				     (void *)dc_payload_info->paddr,
				     NULL);

	if (dc_handle == NULL) {
		VPSSDBG("Create FVID2 DC handle status 0x%08x.\n",
			*(u32 *)dc_payload_info->vaddr);
		r = -EINVAL;
		goto cleanup;
	}

	disp_ctrl->fvid2_handle = dc_handle;
	mutex_init(&disp_ctrl->dcmutex);

	r = kobject_init_and_add(
			&disp_ctrl->kobj,
			&dctrl_ktype,
			&pdev->dev.kobj,
			"system");
	if (r)
		VPSSERR("failed to create dctrl sysfs file.\n");

	/*create sysfs*/
	for (i = 0; i < disp_ctrl->numvencs; i++) {
		struct dc_blender_info *blend = &disp_ctrl->blenders[i];;

		blend->idx = i;
		blend->actnodes = 0;
		blend->name = (char *)venc_name[i].name;
		blend->dctrl = disp_ctrl;
		blend->isdeviceon = true;
		r = kobject_init_and_add(
			&blend->kobj, &blender_ktype,
			&pdev->dev.kobj, "display%d", i);

		if (r) {
			VPSSERR("failed to create blender \
				%d sysfs file.\n", i);
			continue;
		}
	}

	disp_ctrl->tiedvenc = tied_vencs;
	venc_info.tiedvencs = disp_ctrl->tiedvenc;


	/*parse the mode*/
	r = parse_def_modes(mode);
	if (r) {
		VPSSERR("failed to parse mode.\n");
		goto cleanup;
	}

	/*set up the default clksrc and output format*/
	for (i = 0; i < disp_ctrl->numvencs; i++) {
		struct vps_dcvencclksrc *clksrcp =
				&disp_ctrl->blenders[i].clksrc;
		struct vps_dcoutputinfo opinfo;

		clksrcp->venc = venc_name[i].vid;
		/*set the venc output*/
		opinfo.dvofidpolarity = VPS_DC_POLARITY_ACT_HIGH;
		opinfo.dvohspolarity = VPS_DC_POLARITY_ACT_HIGH;
		opinfo.dvovspolarity = VPS_DC_POLARITY_ACT_HIGH;
		opinfo.dvoactvidpolarity = VPS_DC_POLARITY_ACT_HIGH;

		switch (i) {
		case HDMI:
			opinfo.vencnodenum = VPS_DC_VENC_HDMI;
			opinfo.dvofmt = VPS_DC_DVOFMT_TRIPLECHAN_DISCSYNC;
			opinfo.dataformat = FVID2_DF_RGB24_888;
			if (cpu_is_ti816x() && (VPS_PLATFORM_CPU_REV_1_0 ==
			    cpuver))
				clksrcp->clksrc = VPS_DC_CLKSRC_VENCD_DIV2;
			else
				clksrcp->clksrc = VPS_DC_CLKSRC_VENCD;
			break;

		case DVO2:
			opinfo.vencnodenum = VPS_DC_VENC_DVO2;
			opinfo.dvofmt = VPS_DC_DVOFMT_TRIPLECHAN_DISCSYNC;
			opinfo.dataformat = FVID2_DF_RGB24_888;

			if (cpu_is_ti816x()) {
				if (VPS_PLATFORM_CPU_REV_1_0 == cpuver)
					clksrcp->clksrc =
						VPS_DC_CLKSRC_VENCD_DIV2;
				else
					clksrcp->clksrc = VPS_DC_CLKSRC_VENCD;
			} else
				clksrcp->clksrc = VPS_DC_CLKSRC_VENCA;

			break;
		case SDVENC:
			opinfo.vencnodenum = VPS_DC_VENC_SD;
			if (cpu_is_ti816x())
				opinfo.afmt = VPS_DC_A_OUTPUT_COMPOSITE;
			else
				opinfo.afmt = VPS_DC_A_OUTPUT_SVIDEO;
			opinfo.dataformat = FVID2_DF_YUV422SP_UV;
			break;
	if (cpu_is_ti816x()) {
		case HDCOMP:
			opinfo.vencnodenum = VPS_DC_VENC_HDCOMP;
			opinfo.afmt = VPS_DC_A_OUTPUT_COMPONENT;
			opinfo.dataformat = FVID2_DF_YUV422SP_UV;

			clksrcp->clksrc = VPS_DC_CLKSRC_VENCA;
			break;
	}

		}
		r = dc_set_output(&opinfo);
		if (r) {
			VPSSERR("failed to set venc  output\n");
			goto cleanup;
		}

	}
	/*parse command line clksrc*/
	r = parse_def_clksrc(clksrc);
	if (r) {
		VPSSERR("failed to parse clock source\n");
		goto cleanup;
	}

	/*set the clock source*/
	for (i = 0; i < venc_info.numvencs; i++) {
		if (disp_ctrl->blenders[i].idx != SDVENC) {
			r = dc_set_clksrc(
				&disp_ctrl->blenders[i].clksrc);

			if (r) {
				VPSSERR("failed to set clock resource");
				goto cleanup;
			}
		}
	}
	/*config the PLL*/
	for (i = 0; i < venc_info.numvencs; i++) {
		r = dc_set_pll_by_mid(i, venc_info.modeinfo[i].minfo.standard);
		if (r) {
			VPSSERR("failed to set pll");
			goto cleanup;
		}
	}

	/*set the venc mode*/
	r = dc_set_vencmode(&venc_info);
	if (r) {
		VPSSERR("Failed to set venc mode.\n");
		goto cleanup;
	}
	/*set the the THS filter, device is still registered even
	if THS setup is failed*/
	if (cpu_is_ti816x() && (!def_i2cmode)) {
		r = pcf8575_ths7375_enable(TI816X_THSFILTER_ENABLE_MODULE);
		if ((venc_info.modeinfo[HDCOMP].minfo.standard ==
		    FVID2_STD_1080P_60)  ||
		    (venc_info.modeinfo[HDCOMP].minfo.standard ==
		    FVID2_STD_1080P_50))
			r |= pcf8575_ths7360_hd_enable(
				TI816X_THS7360_SF_TRUE_HD_MODE);
		else
			r |= pcf8575_ths7360_hd_enable(
				TI816X_THS7360_SF_HD_MODE);
		if (r < 0) {
			VPSSERR("setup 7375 filter failed\n");
			disp_ctrl->blenders[HDCOMP].isdeviceon = false;
		}
		r = pcf8575_ths7360_sd_enable(TI816X_THSFILTER_ENABLE_MODULE);

		if (r < 0) {
			VPSSERR("setup 7360 filter failed.\n");
			disp_ctrl->blenders[SDVENC].isdeviceon = false;

		}
	}
	return 0;
cleanup:
	vps_dc_deinit(pdev);
	return r;
}


int __exit vps_dc_deinit(struct platform_device *pdev)
{
	int r = 0;
	int i;
	VPSSDBG("dctrl deinit\n");

	if (disp_ctrl) {
		/*disable vencs*/
		if (disp_ctrl->enabled_venc_ids != 0) {
			r = dc_venc_disable(disp_ctrl->vencmask);
			if (r) {
				VPSSERR("Failed to disable vencs.\n");
				return r;
			}
		}

		kobject_del(&disp_ctrl->kobj);
		kobject_put(&disp_ctrl->kobj);

		for (i = 0; i < disp_ctrl->numvencs; i++) {
			kobject_del(&disp_ctrl->blenders[i].kobj);
			kobject_put(&disp_ctrl->blenders[i].kobj);
		}

		kfree(disp_ctrl);
		disp_ctrl = NULL;
	}

	if (dc_payload_info) {

		/*free memory*/
		if (dc_payload_info->vaddr)
			vps_sbuf_free(dc_payload_info->paddr,
				      dc_payload_info->vaddr,
				      dc_payload_info->size);

		kfree(dc_payload_info);
		dc_payload_info = NULL;
	}

	if (dc_handle) {
		r = vps_fvid2_delete(dc_handle, NULL);
		if (r) {
			VPSSERR("failed to delete DC fvid2 handle.\n");
			return r;
		}
		dc_handle = NULL;
	}

	if (cpu_is_ti816x() && (!def_i2cmode))
		ti816x_pcf8575_exit();

	return r;
}

/*S************************  EXTERNAL device registered *****************/

int TI81xx_register_display_panel(struct TI81xx_display_driver *panel_driver,
				  struct ti81xx_venc_info *vencinfo)
{
	int i, r = 0;
	int display_num;

	if (panel_driver == NULL) {
		VPSSERR("pannel driver can not be NULL\n");
		return -1;
	}
	display_num = panel_driver->display;

	if (display_num >= disp_ctrl->numvencs) {
		r = -1;
		goto exit;
	}
	for (i = 0; i < MAX_EXT_VENCS_PER_DISPLAY; i++) {
		struct ti81xx_external_encoder *extenc =
				&external_encs[display_num][i];
		if (extenc->status == TI81xx_EXT_ENCODER_UNREGISTERED) {
			extenc->panel_driver =	panel_driver;
			extenc->status = TI81xx_EXT_ENCODER_REGISTERED;
			if (vencinfo) {
				struct vps_dcmodeinfo *dcminfo =
					&venc_info.modeinfo[display_num];
				/*copy info to external video device*/
				vencinfo->enabled =
					dc_isvencrunning(dcminfo->vencid);
				vencinfo->vtimings.standard =
						dcminfo->minfo.standard;
				vencinfo->vtimings.dvi_hdmi = TI81xx_MODE_HDMI;
				/*FIX ME, this is a temp solution here,
				will address later*/
				if (vencinfo->vtimings.standard ==
				    FVID2_STD_CUSTOM)
					vencinfo->vtimings.standard =
							FVID2_STD_1080P_60;
				vencinfo->outinfo.vencnodenum =
						venc_name[display_num].vid;
				dc_get_output(&vencinfo->outinfo);
			}
			break;
		}
	}

	return 0;
exit:
	return r;

}
EXPORT_SYMBOL(TI81xx_register_display_panel);


int TI81xx_un_register_display_panel(struct TI81xx_display_driver *panel_driver)
{
	int r = 0, i;
	int display_num;

	if (panel_driver == NULL) {
		VPSSERR("pannel driver can not be NULL\n");
		return -1;
	}
	display_num = panel_driver->display;

	if (display_num >= disp_ctrl->numvencs) {
		r = -1;
		goto exit;
	}
	for (i = 0; i < MAX_EXT_VENCS_PER_DISPLAY; i++) {
		struct ti81xx_external_encoder *extenc =
				&external_encs[display_num][i];
		if (panel_driver ==
		    extenc->panel_driver) {
			extenc->status = TI81xx_EXT_ENCODER_UNREGISTERED;
		}
	}
	/*FIXME : Enhance to fit more than one encoder per blender.*/
	return 0;
exit:
	return r;
}
EXPORT_SYMBOL(TI81xx_un_register_display_panel);
