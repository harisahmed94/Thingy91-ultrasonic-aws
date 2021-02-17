/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <modem/lte_lc.h>
#include <net/cloud.h>
#include <net/socket.h>
#include <nrfx/hal/nrf_gpio.h>
#include <nrfx/hal/nrf_timer.h>
#include <nrfx/hal/nrf_gpiote.h>
#include <nrfx/drivers/include/nrfx_timer.h>
#include <nrfx/drivers/include/nrfx_gpiote.h>

#include <logging/log.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 100

/* Delay multipliers */
#define U_SEC .000001d
#define M_SEC .001d
#define N_SEC .000000001d

/* Timer Tick Value in seconds */
const double timer1Tick = .0000000625d;

/* Ultrsonic Sensor */
#define trig 16
#define echoHigh 10
#define echoLow 13

bool input = 0;

/* PulseWidth measurement */
uint32_t startPulse, endPulse;
uint32_t pulseDuration = 0;
double timeDuration = 0;

/* Timer */
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(0);

char distance[100];

LOG_MODULE_REGISTER(cloud_client, CONFIG_CLOUD_CLIENT_LOG_LEVEL);

static struct cloud_backend *cloud_backend;
static struct k_delayed_work cloud_update_work;
static struct k_delayed_work connect_work;

static K_SEM_DEFINE(lte_connected, 0, 1);

/* Flag to signify if the cloud client is connected or not connected to cloud,
 * used to abort/allow cloud publications.
 */
static bool cloud_connected;

long ticksCalculator(double delayRequired)
{
	double ticks = delayRequired / timer1Tick;
	//printf("Ticks required: %4.2f \n", ceil(ticks));
	return ceil(ticks);
}

void timerInit()
{

	// configure the timer
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

	//initialise the timer
	nrfx_timer_init(&m_timer, &timer_config, NULL);
}

void timerUninit()
{

	// disabling the timer
	nrfx_timer_uninit(&m_timer);
	printk("Timer has been disabled.\n");
}

void delay(double delayRequired)
{

	// calculate how many timer ticks are required for that delay
	long ticksRequired = ticksCalculator(delayRequired);

	// configure the variables required for tick calculation
	uint32_t startDelay;
	uint32_t endDelay;
	uint32_t ticksTakenDelay = 0;

	nrfx_timer_enable(&m_timer);
	//printk("Timer should be setup and on by now. \n");

	// getting the most recent timer value
	startDelay = nrfx_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL0);
	//printk("Start time is: %d \n", startDelay);

	// check and exit the loop
	while (ticksTakenDelay <= ticksRequired)
	{
		endDelay = nrfx_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL0);
		//printk("Current endTime: %d \n", endDelay);
		ticksTakenDelay = endDelay - startDelay;
		//printk("Ticks Taken: %d \n", ticksTakenDelay);
	}

	// clearing and disabling the timer
	nrfx_timer_disable(&m_timer);
	printk("Timer delay has occured. \n");
}

void high_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if (pin == echoHigh)
	{
		if (action == NRF_GPIOTE_POLARITY_LOTOHI)
		{
			nrfx_timer_enable(&m_timer);
			startPulse = nrfx_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL0);
			printk("Timer started! \n");
		}
	}
}

void low_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if (pin == echoLow)
	{
		if (action == NRF_GPIOTE_POLARITY_HITOLO)
		{
			endPulse = nrfx_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL0);
			nrfx_timer_disable(&m_timer);
			printk("Timer stopped! \n");
		}
	}
}

void gpiote_init(void)
{
	nrfx_err_t err;

	/* Connect GPIOTE_0 IRQ to nrfx_gpiote_irq_handler */
	/* IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpiote)),
				DT_IRQ(DT_NODELABEL(gpiote), priority),
				nrfx_isr, nrfx_gpiote_irq_handler, 0); */
         IRQ_DIRECT_CONNECT(GPIOTE1_IRQn, 0, nrfx_gpiote_irq_handler, 0);

	/* Initialize GPIOTE (the interrupt priority passed as the parameter
	 * here is ignored, see nrfx_glue.h).
	 */
	err = nrfx_gpiote_init(0);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_gpiote_init error: %08x", err);
		return;
	}

	nrfx_gpiote_in_config_t high_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	high_config.pull = NRF_GPIO_PIN_PULLDOWN;

	nrfx_gpiote_in_config_t low_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	low_config.pull = NRF_GPIO_PIN_PULLDOWN;

	err = nrfx_gpiote_in_init(echoHigh, &high_config, high_pin_handler);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_gpiote_in_HIGH_init error: %08x", err);
		return;
	}
	printk("GPIOTE High event initialized \n");

	err = nrfx_gpiote_in_init(echoLow, &low_config, low_pin_handler);
	if (err != NRFX_SUCCESS)
	{
		LOG_ERR("nrfx_gpiote_in_LOW_init error: %08x", err);
		return;
	}
	printk("GPIOTE Low event initialized \n");

	nrfx_gpiote_in_event_enable(echoHigh, true);
	nrfx_gpiote_in_event_enable(echoLow, true);

	LOG_INF("nrfx_gpiote events initialized");
}

void gpiote_uninit()
{
	// disabling gpiote events
	nrfx_gpiote_in_event_disable(echoHigh);
	nrfx_gpiote_in_event_disable(echoLow);

	// disabling gpiote pins
	nrfx_gpiote_in_uninit(echoHigh);
	nrfx_gpiote_in_uninit(echoLow);

	// disabling gpiote module
	nrfx_gpiote_uninit();
	printk("GPIOTE module disabled.\n");
}

void gpio_init()
{
	nrf_gpio_cfg_output(trig);
	printk("GPIO have been configured. \n");
}

void sensor_prep()
{
	gpio_init();
	timerInit();
	gpiote_init();
}

void sensor_unprep()
{
	timerUninit();
	gpiote_uninit();
}

double ult_sensor()
{

	// Clear the trigPin by setting it LOW
	nrf_gpio_pin_write(trig, 0);

	delay(20 * U_SEC);

	// Trigger the sensor by setting the trigPin high for 10 microseconds:
	nrf_gpio_pin_write(trig, 1);
	delay(10 * U_SEC);
	nrf_gpio_pin_write(trig, 0);

	printk("Sensor has been triggered. \n");

	k_msleep(1000);

	// calculation
	pulseDuration = endPulse - startPulse;
	timeDuration = pulseDuration * timer1Tick;
	printf("Duration: %f \n", timeDuration);

	// Calculate the distance:
	double distance = timeDuration * 340.0f / 2.0f;
	printf("Distance = %4.2f m \n", distance);

	return distance;
}

static void connect_work_fn(struct k_work *work)
{
	int err;

	err = cloud_connect(cloud_backend);
	if (err)
	{
		LOG_ERR("cloud_connect, error: %d", err);
	}

	LOG_INF("Next connection retry in %d seconds",
			CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS);

	k_delayed_work_submit(&connect_work,
						  K_SECONDS(CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS));
}

static void cloud_update_work_fn(struct k_work *work)
{
	int err;

	if (!cloud_connected)
	{
		LOG_INF("Not connected to cloud, abort cloud publication");
		return;
	}

	// enabling modules for sensor to work
	sensor_prep();

	double distance = ult_sensor();
	char message[50];

	sprintf(message, "{\"state\":{\"reported\":{\"Distance\":\"%f\"}}}", distance);

	LOG_INF("Publishing message: %s", log_strdup(message));

	struct cloud_msg msg = {
		.qos = CLOUD_QOS_AT_MOST_ONCE,
		.endpoint.type = CLOUD_EP_TOPIC_MSG,
		.buf = message,
		.len = strlen(message)};

	err = cloud_send(cloud_backend, &msg);
	if (err)
	{
		LOG_ERR("cloud_send failed, error: %d", err);
	}

	// disabling modules for low power mode
	sensor_unprep();

	LOG_INF("Disconnecting from cloud");

	err = cloud_disconnect(cloud_backend);
	if (err)
	{
		LOG_ERR("disconnection failed, error: %d", err);
	}
}

void cloud_event_handler(const struct cloud_backend *const backend,
						 const struct cloud_event *const evt,
						 void *user_data)
{
	ARG_UNUSED(user_data);
	ARG_UNUSED(backend);

	switch (evt->type)
	{
	case CLOUD_EVT_CONNECTING:
		LOG_INF("CLOUD_EVT_CONNECTING");
		break;
	case CLOUD_EVT_CONNECTED:
		LOG_INF("CLOUD_EVT_CONNECTED");
		cloud_connected = true;
		break;
	case CLOUD_EVT_READY:
		LOG_INF("CLOUD_EVT_READY");

		k_delayed_work_cancel(&connect_work);

#if defined(CONFIG_CLOUD_PUBLICATION_SEQUENTIAL)
		k_delayed_work_submit(&cloud_update_work, K_NO_WAIT);
#endif
		break;
	case CLOUD_EVT_DISCONNECTED:
		LOG_INF("CLOUD_EVT_DISCONNECTED");

		cloud_connected = false;

		if (k_delayed_work_pending(&connect_work))
		{
			break;
		}

		k_delayed_work_submit(&connect_work,
							  K_SECONDS(CONFIG_CLOUD_MESSAGE_PUBLICATION_INTERVAL));
		break;
	case CLOUD_EVT_ERROR:
		LOG_INF("CLOUD_EVT_ERROR");
		break;
	case CLOUD_EVT_DATA_SENT:
		LOG_INF("CLOUD_EVT_DATA_SENT");
		break;
	case CLOUD_EVT_DATA_RECEIVED:
		LOG_INF("CLOUD_EVT_DATA_RECEIVED");
		LOG_INF("Data received from cloud: %s",
				log_strdup(evt->data.msg.buf));
		break;
	case CLOUD_EVT_PAIR_REQUEST:
		LOG_INF("CLOUD_EVT_PAIR_REQUEST");
		break;
	case CLOUD_EVT_PAIR_DONE:
		LOG_INF("CLOUD_EVT_PAIR_DONE");
		break;
	case CLOUD_EVT_FOTA_DONE:
		LOG_INF("CLOUD_EVT_FOTA_DONE");
		break;
	default:
		LOG_INF("Unknown cloud event type: %d", evt->type);
		break;
	}
}

static void work_init(void)
{
	k_delayed_work_init(&cloud_update_work, cloud_update_work_fn);
	k_delayed_work_init(&connect_work, connect_work_fn);
}

#if defined(CONFIG_BSD_LIBRARY)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type)
	{
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
			(evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
		{
			break;
		}

		printk("Network registration status: %s\n",
			   evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming\n");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d, Active time: %d\n",
			   evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE:
	{
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
					   "eDRX parameter update: eDRX: %f, PTW: %f\n",
					   evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0)
		{
			printk("%s\n", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		printk("RRC mode: %s\n",
			   evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle\n");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",
			   evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

static int configure_low_power(void)
{
	int err;

#if defined(CONFIG_LTE_PSM_ENABLE)
	/** Power Saving Mode */
	err = lte_lc_psm_req(true);
	if (err)
	{
		printk("lte_lc_psm_req, error: %d\n", err);
	}
#else
	err = lte_lc_psm_req(false);
	if (err)
	{
		printk("lte_lc_psm_req, error: %d\n", err);
	}
#endif

#if defined(CONFIG_LTE_EDRX_ENABLE)
	/** enhanced Discontinuous Reception */
	err = lte_lc_edrx_req(true);
	if (err)
	{
		printk("lte_lc_edrx_req, error: %d\n", err);
	}
#else
	err = lte_lc_edrx_req(false);
	if (err)
	{
		printk("lte_lc_edrx_req, error: %d\n", err);
	}
#endif

#if defined(CONFIG_LTE_RAI_ENABLE)
	/** Release Assistance Indication  */
	err = lte_lc_rai_req(true);
	if (err)
	{
		printk("lte_lc_rai_req, error: %d\n", err);
	}
#endif

	return err;
}

static void modem_configure(void)
{
	int err;

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already configured and LTE connected. */
	}
	else
	{
		err = lte_lc_init_and_connect_async(lte_handler);
		if (err)
		{
			printk("Modem configuration, error: %d\n", err);
			return;
		}
	}
}
#endif

void main(void)
{
	int err;

	LOG_INF("Cloud client with sensor has started");

	cloud_backend = cloud_get_binding(CONFIG_CLOUD_BACKEND);
	__ASSERT(cloud_backend != NULL, "%s backend not found",
			 CONFIG_CLOUD_BACKEND);

	err = cloud_init(cloud_backend, cloud_event_handler);
	if (err)
	{
		LOG_ERR("Cloud backend could not be initialized, error: %d",
				err);
	}

	work_init();

#if defined(CONFIG_BSD_LIBRARY)
	err = configure_low_power();
	if (err)
	{
		printk("Unable to set low power configuration, error: %d\n",
			   err);
	}

	modem_configure();
#endif

	LOG_INF("Connecting to LTE network, this may take several minutes...");

	k_sem_take(&lte_connected, K_FOREVER);

	LOG_INF("Connected to LTE network");
	LOG_INF("Connecting to cloud");

	k_delayed_work_submit(&connect_work, K_NO_WAIT);
}
