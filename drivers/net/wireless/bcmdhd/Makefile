ifneq ($(CONFIG_BCM43241),)
BCM43XX_DRIVER=bcm43241
endif

DHDCFLAGS += -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                      \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE             \
	-DDHDTHREAD -DBDC                                                      \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG                                 \
	-DCUSTOMER_HW2 -DOOB_INTR_ONLY                                         \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWLP2P      \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                 \
	-DKEEP_ALIVE -DGET_CUSTOM_MAC_ENABLE -DPKT_FILTER_SUPPORT              \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT           \
	-DROAM_ENABLE -DDHD_USE_IDLECOUNT			              \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT                            \
	-DMIRACAST_AMPDU_SIZE=8                                               \
	-I$(src) -I$(src)/include -I$(src)/common/include

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_pno.o dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_wlfc.o dhd_ip.o linux_osl.o \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o


obj-$(CONFIG_BCMDHD) += $(BCM43XX_DRIVER).o

$(BCM43XX_DRIVER)-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
$(BCM43XX_DRIVER)-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
$(BCM43XX_DRIVER)-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-65
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=10000
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=7
DHDCFLAGS += -DVSDB -DPROP_TXSTATUS -DPROP_TXSTATUS_VSDB
DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
DHDCFLAGS += -DESCAN_RESULT_PATCH
DHDCFLAGS += -DWL_CFG80211_GON_COLLISION
DHDCFLAGS += -DWLTDLS
DHDCFLAGS += -DWL11U
DHDCFLAGS += -DMAX_DTIM_ALLOWED_INTERVAL=300
endif
ifeq (${TARGET_BUILD_VARIANT},$(filter ${TARGET_BUILD_VARIANT}, eng userdebug))
DHDCFLAGS += -DDHD_DEBUG
endif
ifeq (${TARGET_BUILD_VARIANT},$(filter ${TARGET_BUILD_VARIANT}, eng))
DHDCFLAGS += -DSDTEST
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif

################################
# INTEL platform specific features
#################################
CONFIG_INTEL_PLATFORM=y
ifneq ($(CONFIG_INTEL_PLATFORM),)
$(info bcmdhd: Compiling for Intel platform $(TARGET_PRODUCT) $(obj))
ifneq (,$(filter saltbay% mofd%,$(TARGET_PRODUCT)))
  DHDCFLAGS += -DHW_OOB
endif
  DHDCFLAGS += -DSUPPORT_AUTO_CHANNEL
  DHDCFLAGS += -DSUPPORT_MULTIPLE_REVISION -DSUPPORT_MULTIPLE_CHIPS
  DHDCFLAGS += -DCUSTOM_LISTEN_INTERVAL=3
  DHDCFLAGS += -DBOARD_INTEL
  DHDCFLAGS += -DUSE_KTHREAD_API
  DHDCFLAGS += -DCUSTOM_RXCHAIN=1
  DHDCFLAGS += -DDHD_USE_EARLYSUSPEND
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
endif

#########################
# Chip dependent feature
#########################

ifneq ($(CONFIG_BCM43241),)
$(info bcmdhd: Compiling for 43241 chip)
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=5
  DHDCFLAGS += -DUSE_SDIOFIFO_IOVAR
  DHDCFLAGS += -DCUSTOM_SDIO_F2_BLKSIZE=128
endif

EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD),m)
EXTRA_LDFLAGS += --strip-debug
endif

DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI

