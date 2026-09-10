/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Exposes one direction of the AD9081 JESD204 datapath as a libiio device, so an
 * IIOD client can stream converter samples.
 *
 * The node owns no registers. It pairs a transport core (per-converter datapath
 * controls) with the AXI DMAC that moves samples between that core and memory,
 * and presents the pair as one IIO device with one channel per converter.
 *
 * Deliberately minimal: a flat address-to-address DMA copy with no byte
 * swapping, no de-interleaving and no per-converter splitting. The DMA stream
 * layout is whatever the transport core produces -- little-endian s16 samples,
 * interleaved across the enabled converters -- which is exactly what the
 * scan_type below advertises, so a libiio client can demultiplex it itself.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

#include <iio/iio-backend.h>
#include <iio_device.h>

#include <zephyr/drivers/misc/jesd204/axi_tpl.h>

LOG_MODULE_REGISTER(axi_jesd_iio, CONFIG_JESD204_AXI_JESD_IIO_LOG_LEVEL);

#define DT_DRV_COMPAT adi_axi_jesd_iio

/**
 * @brief Sample width advertised to clients, in bits.
 *
 * Fixed by the link profile: N == NP == 16, so a converter sample occupies a
 * full 16-bit storage word with no shift and no padding. Not derived from the
 * devicetree because the transport core's sample format is configured to match
 * this and nothing else.
 */
#define AXI_JESD_IIO_SAMPLE_BITS 16

/** @brief Storage size of one converter sample, in bytes. */
#define AXI_JESD_IIO_SAMPLE_BYTES (AXI_JESD_IIO_SAMPLE_BITS / 8)

/**
 * @brief DMA staging-buffer alignment, in bytes.
 *
 * Cache maintenance operates on whole cache lines, so a staging buffer that
 * shares a line with any other object would have that object's dirty data
 * discarded by the invalidate after a capture. 64 bytes is the A53 D-cache line
 * and is also a whole number of 8-converter frames, so a chunk boundary never
 * splits a sample.
 */
#define AXI_JESD_IIO_DMA_ALIGN 64

BUILD_ASSERT(CONFIG_JESD204_AXI_JESD_IIO_BUF_SIZE % AXI_JESD_IIO_DMA_ALIGN == 0,
	     "staging buffer size must be a whole number of cache lines");

/**
 * @brief Bound on how long one staging-buffer-sized DMA transfer may take.
 *
 * The AXI DMAC in this bitstream has no interrupt wired, so a stalled transfer
 * would otherwise spin forever in the status poll below.
 */
#define AXI_JESD_IIO_XFER_TIMEOUT_MS 1000

/** @brief Devicetree configuration -- ROM, one per node. */
struct axi_jesd_iio_config {
	/** Transport core holding this direction's per-converter controls. */
	const struct device *tpl;
	/** AXI DMAC that moves this direction's samples. */
	const struct device *dmac;
	/** DMA channel within @ref dmac. */
	uint32_t dma_channel;
	/** Converters exposed, one IIO channel each; the link's M. */
	uint32_t num_channels;
	/** Cache-line-aligned DMA staging buffer for this instance. */
	uint8_t *staging;
	/** Size of @ref staging in bytes. */
	size_t staging_size;
	/** true for playback (TX/DAC), false for capture (RX/ADC). */
	bool output;
};

/** @brief Per-instance state -- RAM. */
struct axi_jesd_iio_data {
	/**
	 * Latched from enable_buffer(), because the per-transfer ops carry no
	 * cyclic flag of their own. Only meaningful for playback.
	 */
	bool cyclic;
};

/**
 * @brief Run one DMA transfer into or out of the staging buffer and wait for it.
 *
 * @param dev    IIO device.
 * @param len    Transfer size in bytes; must not exceed the staging buffer.
 * @param to_mem true for a capture transfer (peripheral to memory), false for
 *               playback (memory to peripheral).
 * @param wait   true to poll the transfer to completion before returning.
 * @retval 0 on success.
 * @retval -ETIMEDOUT if the transfer did not complete in time.
 * @retval negative errno from the DMA driver otherwise.
 */
static int axi_jesd_iio_xfer(const struct device *dev, size_t len, bool to_mem,
			     bool wait)
{
	const struct axi_jesd_iio_config *cfg = dev->config;
	struct axi_jesd_iio_data *data = dev->data;
	struct dma_block_config block = {
		.block_size = len,
	};
	struct dma_config dma_cfg = {
		.block_count = 1,
		.head_block = &block,
	};
	struct dma_status status;
	int64_t deadline;
	int ret;

	if (to_mem) {
		block.dest_address = (uintptr_t)cfg->staging;
		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.dest_data_size = AXI_JESD_IIO_SAMPLE_BYTES;
		dma_cfg.dest_burst_length = AXI_JESD_IIO_SAMPLE_BYTES;
	} else {
		block.source_address = (uintptr_t)cfg->staging;
		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = AXI_JESD_IIO_SAMPLE_BYTES;
		dma_cfg.source_burst_length = AXI_JESD_IIO_SAMPLE_BYTES;
		dma_cfg.cyclic = data->cyclic;
	}

	/*
	 * A previous playback transfer may still be running -- cyclic ones never
	 * stop on their own. Reconfiguring the channel under a live transfer is
	 * not defined, so stop it first; an error here just means it was idle.
	 */
	(void)dma_stop(cfg->dmac, cfg->dma_channel);

	ret = dma_config(cfg->dmac, cfg->dma_channel, &dma_cfg);
	if (ret) {
		LOG_ERR("%s: dma_config failed (%d)", dev->name, ret);
		return ret;
	}

	ret = dma_start(cfg->dmac, cfg->dma_channel);
	if (ret) {
		LOG_ERR("%s: dma_start failed (%d)", dev->name, ret);
		return ret;
	}

	if (!wait) {
		return 0;
	}

	/*
	 * No interrupt is wired to this DMAC, so dma_get_status() is what
	 * advances the transfer -- the poll loop is the transfer engine, not just
	 * an observer of it.
	 */
	deadline = k_uptime_get() + AXI_JESD_IIO_XFER_TIMEOUT_MS;
	do {
		ret = dma_get_status(cfg->dmac, cfg->dma_channel, &status);
		if (ret) {
			LOG_ERR("%s: dma_get_status failed (%d)", dev->name,
				ret);
			return ret;
		}
		if (k_uptime_get() > deadline) {
			LOG_ERR("%s: DMA transfer of %zu bytes timed out",
				dev->name, len);
			(void)dma_stop(cfg->dmac, cfg->dma_channel);
			return -ETIMEDOUT;
		}
	} while (status.busy);

	return 0;
}

/**
 * @brief Advertise one scan element per converter.
 *
 * @param dev        IIO device.
 * @param iio_device libiio device to populate.
 * @retval 0 on success.
 * @retval -EINVAL if a channel could not be added.
 */
static int axi_jesd_iio_add_channels(const struct device *dev,
				     struct iio_device *iio_device)
{
	const struct axi_jesd_iio_config *cfg = dev->config;
	/*
	 * Signed 16-in-16 little-endian, no shift, no scale. Fixed rather than
	 * configurable: it describes what the transport core was configured to
	 * emit, so making it settable could only ever make it wrong.
	 */
	const struct iio_data_format fmt = {
		.length = AXI_JESD_IIO_SAMPLE_BITS,
		.bits = AXI_JESD_IIO_SAMPLE_BITS,
		.shift = 0,
		.is_signed = true,
		.is_be = false,
	};

	if (iio_device == NULL) {
		return -EINVAL;
	}

	for (uint32_t i = 0; i < cfg->num_channels; i++) {
		struct iio_channel *chn;
		/* "voltage" + up to 10 digits of a uint32 + NUL. */
		char id[24];

		snprintk(id, sizeof(id), "voltage%u", i);

		chn = iio_device_add_channel(iio_device, (unsigned int)i, id,
					     NULL, NULL, cfg->output, true,
					     &fmt);
		if (iio_err(chn)) {
			LOG_ERR("%s: could not add channel %s", dev->name, id);
			return -EINVAL;
		}
	}

	LOG_DBG("%s: %u %s scan elements", dev->name, cfg->num_channels,
		cfg->output ? "output" : "input");
	return 0;
}

/**
 * @brief Point the datapath at the DMA engine, or release it again.
 *
 * Capture: the enabled channel mask is pushed into the transport core, since a
 * disabled converter contributes no samples to the DMA stream. On disable every
 * converter is re-enabled, restoring what axi_tpl_configure() left, so the
 * bring-up sample's own capture path still works after an IIOD session.
 *
 * Playback: the converters are switched from their internal DDS tone generators
 * to the DMA source. They are deliberately left there on disable -- silently
 * re-arming a tone the client never asked for would be worse than silence.
 *
 * @param dev        IIO device.
 * @param iio_device libiio device the buffer belongs to.
 * @param mask       Enabled channels.
 * @param nb_samples Samples per block. Unused: the transfer size comes from the
 *                   length the core passes to readbuf()/writebuf().
 * @param enable     true to arm, false to release.
 * @param cyclic     true if playback blocks should repeat until stopped.
 * @retval 0 on success.
 * @retval negative errno from the transport core otherwise.
 */
static int axi_jesd_iio_enable_buffer(const struct device *dev,
				      const struct iio_device *iio_device,
				      const struct iio_channels_mask *mask,
				      size_t nb_samples, bool enable,
				      bool cyclic)
{
	const struct axi_jesd_iio_config *cfg = dev->config;
	struct axi_jesd_iio_data *data = dev->data;
	int ret;

	ARG_UNUSED(nb_samples);

	if (iio_device == NULL || mask == NULL) {
		return -EINVAL;
	}

	if (!enable) {
		(void)dma_stop(cfg->dmac, cfg->dma_channel);
		data->cyclic = false;
	}

	if (cfg->output) {
		if (!enable) {
			return 0;
		}
		data->cyclic = cyclic;
		/* enable == false on the DDS argument selects the DMA source. */
		return axi_tpl_tx_dds(cfg->tpl, 0, 0, 0, false);
	}

	for (uint32_t i = 0; i < cfg->num_channels; i++) {
		struct iio_channel *chn =
			iio_device_get_channel(iio_device, (unsigned int)i);
		bool on = true;

		if (enable) {
			on = (chn != NULL) && iio_channel_is_enabled(chn, mask);
		}

		ret = axi_tpl_rx_chan_enable(cfg->tpl, i, on);
		if (ret) {
			LOG_ERR("%s: channel %u enable=%d failed (%d)",
				dev->name, i, on, ret);
			return ret;
		}
	}

	return 0;
}

/**
 * @brief Capture @p len bytes of interleaved converter samples.
 *
 * One-shot and blocking: each call arms a fresh transfer and polls it to
 * completion, so consecutive calls are not gapless -- samples arriving between
 * two transfers are lost. Adequate for inspecting the datapath, not for a
 * continuous record.
 *
 * The transfer lands in the instance's aligned staging buffer and is copied
 * from there, because the block the core hands us is plain heap memory with no
 * cache-line alignment guarantee, and invalidating an unaligned range would
 * discard dirty data belonging to neighbouring allocations. Transfers larger
 * than the staging buffer are split, which adds a gap at each chunk boundary.
 *
 * @param dev        IIO device.
 * @param iio_device libiio device the buffer belongs to. Unused.
 * @param mask       Enabled channels. Unused: the mask was already pushed into
 *                   the transport core by enable_buffer().
 * @param dst        Destination block.
 * @param len        Bytes to capture.
 * @retval len on success.
 * @retval -EINVAL if @p dst is NULL.
 * @retval -ENOTSUP on a playback device.
 * @retval negative errno from the DMA transfer otherwise.
 */
static ssize_t axi_jesd_iio_readbuf(const struct device *dev,
				    const struct iio_device *iio_device,
				    const struct iio_channels_mask *mask,
				    void *dst, size_t len)
{
	const struct axi_jesd_iio_config *cfg = dev->config;
	size_t done = 0;

	ARG_UNUSED(iio_device);
	ARG_UNUSED(mask);

	if (dst == NULL) {
		return -EINVAL;
	}
	if (cfg->output) {
		return -ENOTSUP;
	}

	while (done < len) {
		size_t chunk = MIN(len - done, cfg->staging_size);
		int ret = axi_jesd_iio_xfer(dev, chunk, true, true);

		if (ret) {
			return ret;
		}

		sys_cache_data_invd_range(cfg->staging, chunk);
		memcpy((uint8_t *)dst + done, cfg->staging, chunk);
		done += chunk;
	}

	return (ssize_t)len;
}

/**
 * @brief Hand @p len bytes of interleaved converter samples to the DAC.
 *
 * Fire-and-forget: the transfer is armed and the call returns without waiting,
 * so the samples are still on their way out when the client's write completes.
 * When the buffer was enabled cyclic, the DMA repeats this block until the
 * buffer is disabled or another writebuf() replaces it.
 *
 * The block is copied into the instance's staging buffer first, because a
 * cyclic transfer keeps reading its source long after this call returns and the
 * core is free to reuse the block by then. That caps one block at the staging
 * buffer size: a cyclic transfer cannot be split into chunks without breaking
 * the repeat, so an oversized block is refused rather than truncated.
 *
 * @param dev        IIO device.
 * @param iio_device libiio device the buffer belongs to. Unused.
 * @param mask       Enabled channels. Unused: playback drives every converter.
 * @param src        Source block.
 * @param len        Bytes to play.
 * @retval len on success.
 * @retval -EINVAL if @p src is NULL.
 * @retval -ENOTSUP on a capture device.
 * @retval -ENOMEM if @p len exceeds CONFIG_JESD204_AXI_JESD_IIO_BUF_SIZE.
 * @retval negative errno from the DMA transfer otherwise.
 */
static ssize_t axi_jesd_iio_writebuf(const struct device *dev,
				     const struct iio_device *iio_device,
				     const struct iio_channels_mask *mask,
				     const void *src, size_t len)
{
	const struct axi_jesd_iio_config *cfg = dev->config;
	int ret;

	ARG_UNUSED(iio_device);
	ARG_UNUSED(mask);

	if (src == NULL) {
		return -EINVAL;
	}
	if (!cfg->output) {
		return -ENOTSUP;
	}
	if (len > cfg->staging_size) {
		LOG_ERR("%s: block of %zu bytes exceeds the %zu byte staging buffer",
			dev->name, len, cfg->staging_size);
		return -ENOMEM;
	}

	memcpy(cfg->staging, src, len);
	sys_cache_data_flush_range(cfg->staging, len);

	ret = axi_jesd_iio_xfer(dev, len, false, false);
	if (ret) {
		return ret;
	}

	return (ssize_t)len;
}

/**
 * @brief Check that both halves of the datapath pair are present.
 *
 * @param dev IIO device.
 * @retval 0 on success.
 * @retval -ENODEV if the transport core or the DMA engine is not ready.
 */
static int axi_jesd_iio_init(const struct device *dev)
{
	const struct axi_jesd_iio_config *cfg = dev->config;

	if (!device_is_ready(cfg->tpl)) {
		LOG_ERR("%s: transport core not ready", dev->name);
		return -ENODEV;
	}
	if (!device_is_ready(cfg->dmac)) {
		LOG_ERR("%s: DMA engine not ready", dev->name);
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(iio_device, axi_jesd_iio_driver_api) = {
	.add_channels = axi_jesd_iio_add_channels,
	.enable_buffer = axi_jesd_iio_enable_buffer,
	.readbuf = axi_jesd_iio_readbuf,
	.writebuf = axi_jesd_iio_writebuf,
};

/*
 * Direction comes from the transport core's compatible, never from a property
 * of this node, so it cannot contradict the hardware it points at.
 */
#define AXI_JESD_IIO_IS_TX(inst)                                                                   \
	DT_NODE_HAS_COMPAT(DT_INST_PHANDLE(inst, adi_tpl), adi_axi_ad9081_tx_1_0)

/*
 * One IIO channel per converter, and the transport core has one datapath channel
 * per converter, so the two counts are the same number written twice. Pin them
 * together rather than trusting them to agree.
 */
#define AXI_JESD_IIO_INIT(inst)                                                                    \
	BUILD_ASSERT(DT_INST_PROP(inst, adi_converters_per_device) ==                               \
			     DT_PROP(DT_INST_PHANDLE(inst, adi_tpl), adi_num_channels),            \
		     "adi,converters-per-device must equal the transport core's "                   \
		     "adi,num-channels");                                                          \
                                                                                                   \
	static uint8_t axi_jesd_iio_staging_##inst[CONFIG_JESD204_AXI_JESD_IIO_BUF_SIZE]            \
		__aligned(AXI_JESD_IIO_DMA_ALIGN);                                                 \
                                                                                                   \
	static struct axi_jesd_iio_data axi_jesd_iio_data_##inst;                                   \
                                                                                                   \
	static const struct axi_jesd_iio_config axi_jesd_iio_config_##inst = {                      \
		.tpl = DEVICE_DT_GET(DT_INST_PHANDLE(inst, adi_tpl)),                              \
		.dmac = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(inst, 0)),                          \
		.dma_channel = DT_INST_DMAS_CELL_BY_IDX(inst, 0, channel),                         \
		.num_channels = DT_INST_PROP(inst, adi_converters_per_device),                     \
		.staging = axi_jesd_iio_staging_##inst,                                            \
		.staging_size = sizeof(axi_jesd_iio_staging_##inst),                               \
		.output = AXI_JESD_IIO_IS_TX(inst),                                                \
	};                                                                                         \
                                                                                                   \
	IIO_DEVICE_DT_INST_DEFINE(inst, axi_jesd_iio_init, NULL, &axi_jesd_iio_data_##inst,         \
				  &axi_jesd_iio_config_##inst, POST_KERNEL,                        \
				  CONFIG_JESD204_AXI_JESD_IIO_INIT_PRIORITY,                       \
				  &axi_jesd_iio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXI_JESD_IIO_INIT)
