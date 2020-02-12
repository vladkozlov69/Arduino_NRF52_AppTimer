#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "app_timer.h"

#define LED_PIN 22

BLEPeripheral blePeripheral = BLEPeripheral();
BLEService service = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLEService service2 = BLEService("19b10000e8f2537e4f6cd104768a1215");
BLECharacteristic characteristic = BLECharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite, 10);
BLECharacteristic characteristic2 = BLECharacteristic("19b10001e8f2537e4f6cd104768a1215", BLERead | BLEWrite, 10);



#define APP_TIMER_PRESCALER             0                     /**< Value of the RTC2 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            1                     /**< Maximum number of simultaneously created timers. */
#define MS_BETWEEN_SAMPLING_ADC		20000		      /* e.g.: fire timeout every 10 seconds */
#define TIMER_TICKS		        APP_TIMER_TICKS(MS_BETWEEN_SAMPLING_ADC, APP_TIMER_PRESCALER)
#define APP_TIMER_OP_QUEUE_SIZE         4                     /**< Size of timer operation queues. */
static app_timer_id_t                   m_timer_id;

boolean timers_start(void)
{
  	// Start application timers.
  	uint32_t err_code = app_timer_start(m_timer_id, TIMER_TICKS, NULL);
	return NRF_SUCCESS == err_code;
}

uint16_t chV = 0;
char chV_buf[10];

void timeout_handler(void * p_context)
{
	chV++;
	//sprintf(chV_buf, "%u", chV);
	//characteristic.setValue("chV_buf");
  //  UNUSED_PARAMETER(p_context);
//   SEGGER_RTT_WriteString (0, "--> in timeout handler\n");
}

void timers_init(void)
{
  //initialize the low frequency cloc
//   uint32_t err_code = nrf_drv_clock_init(NULL);
//   APP_ERROR_CHECK(err_code);
//   nrf_drv_clock_lfclk_request();
  // Initialize timer module.
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL);

  // Create timers.
  uint32_t err_code = app_timer_create(&m_timer_id,
			      APP_TIMER_MODE_REPEATED,
			      timeout_handler);
//   APP_ERROR_CHECK(err_code);
}

void setup()
{
	pinMode(9, INPUT);
  	pinMode(LED_PIN, OUTPUT);


  	blePeripheral.setAdvertisedServiceUuid(service.uuid());
	blePeripheral.setLocalName("HRM1");
	blePeripheral.addAttribute(service);
	blePeripheral.addAttribute(service2);
	blePeripheral.addAttribute(characteristic);
	blePeripheral.addAttribute(characteristic2);

	blePeripheral.setAdvertisingInterval(600);

	characteristic.setValue("started");
	blePeripheral.begin();

	digitalWrite(LED_PIN, HIGH);

	timers_init();
  	timers_start();

	delay(1000);
}

bool pingStarted = false;
unsigned int lastPing;

void enterSleep()
{
	pingStarted = false;
	digitalWrite(LED_PIN, LOW);
	NVIC_ClearPendingIRQ(SD_EVT_IRQn);
	NVIC_ClearPendingIRQ(RADIO_NOTIFICATION_IRQn);
	sd_app_evt_wait();
}

unsigned long noCentralWakeupCount = 0;

void loop()
{
	BLECentral central = blePeripheral.central();

	if (central)
	{
		pingStarted = true;
		lastPing = millis();

		sprintf(chV_buf, "%u", chV);
		characteristic.setValue(chV_buf);

		sprintf(chV_buf, "%u", noCentralWakeupCount);
		characteristic2.setValue(chV_buf);

		if (pingStarted && (millis() - lastPing > 500))
		{
			enterSleep();
		}
	}
	else
	{
		noCentralWakeupCount++;
		enterSleep();
	}

	delay(10);
}


