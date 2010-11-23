
# core framebuffer
#
obj-y := msm_fb.o
ifeq ($(CONFIG_FB_MSM_LOGO),y)
obj-y += logo.o
endif

# MDP DMA/PPP engine
#
obj-y += mdp.o

obj-$(CONFIG_MSM_MDP40) += mdp_hw40.o
obj-$(CONFIG_FB_MSM_OVERLAY) += mdp4_util.o
obj-$(CONFIG_FB_MSM_OVERLAY) += mdp4_overlay.o
obj-$(CONFIG_FB_MSM_OVERLAY) += mdp4_overlay_mddi.o
obj-$(CONFIG_MSM_MDP40) += msm_rotator.o

obj-$(CONFIG_FB_MSM_LEGACY_MDP) += mdp_hw_legacy.o

obj-$(CONFIG_FB_MSM_MDP_PPP) += mdp_ppp.o
obj-$(CONFIG_MSM_MDP22) += mdp_ppp22.o
obj-$(CONFIG_MSM_MDP31) += mdp_ppp31.o

# MDDI interface
#
obj-$(CONFIG_FB_MSM_MDDI) += mddi.o

# MDDI client/panel drivers
#
obj-$(CONFIG_FB_MSM_MDDI) += mddi_client_simple.o
obj-$(CONFIG_FB_MSM_MDDI) += mddi_client_toshiba.o
obj-$(CONFIG_FB_MSM_MDDI_NOVTEC) += mddi_client_novb9f6_5582.o mddi_client_novb9f6_5580.o mddi_client_novb9f6_5560.o
obj-$(CONFIG_FB_MSM_MDDI_EPSON) += mddi_client_epson.o

# MDP LCD controller driver
obj-$(CONFIG_FB_MSM_LCDC) += mdp_lcdc.o

# Yamato GL driver
ifeq ($(CONFIG_GPU_MSM_KGSL_ADRENO205),y)
obj-$(CONFIG_GPU_MSM_KGSL) += gpu/kgsl_adreno205/
else
obj-$(CONFIG_GPU_MSM_KGSL) += gpu/kgsl/
endif

obj-$(CONFIG_MSM_HDMI) += hdmi/
obj-$(CONFIG_VIDC_720P) += vidc/
