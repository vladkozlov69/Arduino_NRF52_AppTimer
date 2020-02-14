#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <BLEPeripheral.h>
#include "app_timer.h"

BLEPeripheral blePeripheral = BLEPeripheral();

BLEService pressureService = BLEService("DDD0");
BLECharacteristic pressureCharacteristic = BLECharacteristic("DDD1", BLERead | BLEWrite, 16);
BLEDescriptor pressureDescriptor = BLEDescriptor("2901", "Atm. Pressure");

Adafruit_BMP280 bmp;


#define APP_TIMER_PRESCALER         0                     /**< Value of the RTC2 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS        1                     /**< Maximum number of simultaneously created timers. */
#define MS_BETWEEN_SAMPLING_ADC		20000		      /* e.g.: fire timeout every 2 seconds */
#define TIMER_TICKS		        APP_TIMER_TICKS(MS_BETWEEN_SAMPLING_ADC, APP_TIMER_PRESCALER)
#define APP_TIMER_OP_QUEUE_SIZE         4                     /**< Size of timer operation queues. */
static app_timer_id_t                   m_timer_id;

boolean timers_start(void)
{
  	// Start application timers.
  	uint32_t err_code = app_timer_start(m_timer_id, TIMER_TICKS, NULL);
	return NRF_SUCCESS == err_code;
}

volatile boolean doPoll = false;
char buf[16];

void timeout_handler(void * p_context)
{
	doPoll = true;
}

boolean timers_init(void)
{
  //initialize the low frequency clock
//   uint32_t err_code = nrf_drv_clock_init(NULL);
//   APP_ERROR_CHECK(err_code);
//   nrf_drv_clock_lfclk_request();
  // Initialize timer module.
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL);

  	// Create timers.
  	uint32_t err_code = app_timer_create(&m_timer_id,
			      APP_TIMER_MODE_REPEATED,
			      timeout_handler);
	return NRF_SUCCESS == err_code;
}

void setup()
{
	Wire.setPins(1, 3);
	Wire.begin();

	delay(100);

	bmp.begin(0x76);
	bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
				Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
				Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
				Adafruit_BMP280::FILTER_OFF);      /* Filtering. */

	blePeripheral.setLocalName("HRM1");
	blePeripheral.setAdvertisedServiceUuid(pressureService.uuid());
  	blePeripheral.addAttribute(pressureService);
  	blePeripheral.addAttribute(pressureCharacteristic);
  	blePeripheral.addAttribute(pressureDescriptor);

	blePeripheral.setAdvertisingInterval(2000);

	blePeripheral.begin();

	timers_init();
  	timers_start();

	delay(1000);
}

bool pingStarted = false;
unsigned int lastPing;

#define FPU_EXCEPTION_MASK 0x0000009F 

void enterSleep()
{
	pingStarted = false;
	// digitalWrite(LED_PIN, LOW);

    // Clear exceptions and PendingIRQ from the FPU unit 
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));      
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

	// Shut down the TWI
	NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
	// Additional tweak - see Errata 89
	// https://infocenter.nordicsemi.com/index.jsp?topic=%2Ferrata_nRF52832_Rev2%2FERR%2FnRF52832%2FRev2%2Flatest%2Fanomaly_832_89.html&cp=4_2_1_0_1_26
	*(volatile uint32_t *)0x40003FFC = 0;
	*(volatile uint32_t *)0x40003FFC;
	*(volatile uint32_t *)0x40003FFC = 1;

	NVIC_ClearPendingIRQ(SD_EVT_IRQn);
	NVIC_ClearPendingIRQ(RADIO_NOTIFICATION_IRQn);

	// Call SoftDevice Wait For event 
	sd_app_evt_wait();
}

unsigned long noCentralWakeupCount = 0;

void loop()
{
	if (doPoll)
	{
		// Bring up the TWI
		NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

		float pressure = bmp.readPressure() * 0.00750062;
		sprintf(buf, "%u", round(pressure));
		pressureCharacteristic.setValue(buf);
		doPoll = false;
	}

	BLECentral central = blePeripheral.central();

	if (central)
	{
		pingStarted = true;
		lastPing = millis();

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


