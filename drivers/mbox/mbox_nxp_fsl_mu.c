/*
 * Wrapper of the Freescale Message Unit driver into Zephyr's MBOX model.
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/sys/util_macro.h>
#include <fsl_mu.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nxp_fsl_mu);

#define DT_DRV_COMPAT nxp_fsl_mu

#define MU_MAX_CHANNELS 2

#define MU_MAX_MBOX_PER_CHAN	1
#define MU_MBOX_SIZE		4

struct nxp_fsl_mu_data {
	mbox_callback_t cb[MU_MAX_CHANNELS];
	void *user_data[MU_MAX_CHANNELS];
};

struct nxp_fsl_mu_config {
	MU_Type *base;
	void (*config_irq)(void);
};

static int nxp_fsl_mu_send(const struct device *dev, uint32_t channel,
			    const struct mbox_msg *msg)
{
	const struct nxp_fsl_mu_config *cfg = dev->config;
	uint32_t data;

	if (channel >= MU_MAX_CHANNELS) {
		return -EINVAL;
	}

	/* Signalling mode. */
	if (msg == NULL) {
		if (kStatus_Success != MU_TriggerInterrupts(
			cfg->base, kMU_GenInt0InterruptTrigger)) {
			/* Previous interrupt has not been processed. */
			return -EBUSY;
		}
		return 0;
	}

	/* Data transfer mode. */
	if (msg->size > (MU_MBOX_SIZE * MU_MAX_MBOX_PER_CHAN)) {
		return -EMSGSIZE;
	}

	if (msg->size % MU_MBOX_SIZE != 0) {
		/* We can only send this many bytes at a time. */
		return -EMSGSIZE;
	}

	/* Each word of data takes one MU message register. */
	for (uint32_t i = 0; i < msg->size / sizeof(uint32_t); ++i) {
		/* memcpy to avoid issues when msg->data is not word-aligned. */
		memcpy(&data, msg->data, sizeof(uint32_t));
		MU_SendMsg(cfg->base, i, data);
	}
	return 0;
}

static int nxp_fsl_mu_register_callback(const struct device *dev, uint32_t channel,
					 mbox_callback_t cb, void *user_data)
{
	struct nxp_fsl_mu_data *data = dev->data;

	if (channel >= MU_MAX_CHANNELS) {
		return -EINVAL;
	}

	data->cb[channel] = cb;
	data->user_data[channel] = user_data;

	return 0;
}

static int nxp_fsl_mu_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (MU_MBOX_SIZE * MU_MAX_MBOX_PER_CHAN);
}

static uint32_t nxp_fsl_mu_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return MU_MAX_CHANNELS;
}

static int nxp_fsl_mu_set_enabled(const struct device *dev, uint32_t channel,
				   bool enable)
{
	struct nxp_fsl_mu_data *data = dev->data;
	const struct nxp_fsl_mu_config *cfg = dev->config;

	if (channel >= MU_MAX_CHANNELS) {
		return -EINVAL;
	}

	if (enable) {
		if (data->cb[channel] == NULL) {
			LOG_WRN("Enabling channel without a registered callback");
		}
		MU_EnableInterrupts(cfg->base,
			(kMU_GenInt0InterruptEnable | kMU_Rx0FullInterruptEnable));
	} else {
		MU_DisableInterrupts(cfg->base,
			(kMU_GenInt0InterruptEnable | kMU_Rx0FullInterruptEnable));
	}

	return 0;
}

static int nxp_fsl_mu_init(const struct device *dev)
{
	const struct nxp_fsl_mu_config *cfg = dev->config;

	MU_Init(cfg->base);

	/*
	 * Configure and enable interrupt group, but channel's interrupt are
	 * disabled until calling .set_enabled()
	 */
	cfg->config_irq();
	return 0;
}

static const struct mbox_driver_api nxp_fsl_mu_driver_api = {
	.send = nxp_fsl_mu_send,
	.register_callback = nxp_fsl_mu_register_callback,
	.mtu_get = nxp_fsl_mu_mtu_get,
	.max_channels_get = nxp_fsl_mu_max_channels_get,
	.set_enabled = nxp_fsl_mu_set_enabled,
};

#define MU_INIT_IRQ_FUNC(idx)					\
	void MU_##idx##_IRQHandler(void);			\
	static void nxp_fsl_mu_##idx##_init_irq(void)		\
	{							\
		IRQ_CONNECT(DT_INST_IRQN(idx),			\
			    DT_INST_IRQ(idx, priority),		\
			    MU_##idx##_IRQHandler,		\
			    NULL,				\
			    0);					\
		irq_enable(DT_INST_IRQN(idx));			\
	}

#define MU_INSTANCE_DEFINE(idx)							\
	static struct nxp_fsl_mu_data nxp_fsl_mu_##idx##_data;			\
	static struct nxp_fsl_mu_config nxp_fsl_mu_##idx##_config = {		\
		.base = (MU_Type *)DT_INST_REG_ADDR(idx),			\
		.config_irq = nxp_fsl_mu_##idx##_init_irq,			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(idx, nxp_fsl_mu_init, NULL,			\
			&nxp_fsl_mu_##idx##_data, &nxp_fsl_mu_##idx##_config,	\
			POST_KERNEL, CONFIG_MBOX_INIT_PRIORITY,			\
			&nxp_fsl_mu_driver_api)

#define MU_IRQ_HANDLER(idx)							\
	static uint32_t mu_##idx##_received_data;				\
	void MU_##idx##_IRQHandler(void)					\
	{									\
		const struct device *dev = DEVICE_DT_INST_GET(idx);		\
		const struct nxp_fsl_mu_data *data = dev->data;			\
		const struct nxp_fsl_mu_config *config = dev->config;		\
		int channel = 0;						\
		struct mbox_msg msg;						\
		struct mbox_msg *callback_msg_ptr = NULL;			\
		uint32_t flag = MU_GetStatusFlags(config->base);		\
										\
		if ((flag & kMU_Rx0FullFlag) == kMU_Rx0FullFlag) {		\
			mu_##idx##_received_data =				\
				MU_ReceiveMsgNonBlocking(config->base, 0);	\
			msg.data = (const void *)&mu_##idx##_received_data;	\
			msg.size = MU_MBOX_SIZE;				\
			callback_msg_ptr = &msg;				\
		} else if ((flag & kMU_GenInt0Flag) == kMU_GenInt0Flag) {	\
			MU_ClearStatusFlags(config->base, kMU_GenInt0Flag);	\
			callback_msg_ptr = NULL;				\
		}								\
										\
		if (data->cb[channel]) {					\
			data->cb[channel](dev, channel,				\
					data->user_data[channel],		\
					callback_msg_ptr);			\
		}								\
	}

#define MU_INST(idx)								\
	MU_INIT_IRQ_FUNC(idx);							\
	MU_INSTANCE_DEFINE(idx);						\
	MU_IRQ_HANDLER(idx);

DT_INST_FOREACH_STATUS_OKAY(MU_INST)
