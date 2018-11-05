#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"

#include <applibs/gpio.h>
#include <applibs/log.h>
#include <applibs/wificonfig.h>

#include "led_blink_utility.h"
#include "mt3620_rdb.h"
#include "azure_iot_utilities.h"
#include "timer_utility.h"

// This sample C application for the MT3620 Reference Development Board (Azure Sphere)
// blinks an LED.
// The blink rate can be changed through a button press.
//
// It uses the API for the following Azure Sphere application libraries:
// - gpio (digital input for button)
// - log (messages shown in Visual Studio's Device Output window during debugging)

// File descriptors - initialized to invalid value
static int epollFd = -1;
static int usTimerFd = -1;


// Connectivity state
static bool connectedToIoTHub = false;

static const GPIO_Id ledsPins[3][3] = {
	{MT3620_RDB_LED1_RED, MT3620_RDB_LED1_GREEN, MT3620_RDB_LED1_BLUE}, {MT3620_RDB_LED2_RED, MT3620_RDB_LED2_GREEN, MT3620_RDB_LED2_BLUE}, {MT3620_RDB_LED3_RED, MT3620_RDB_LED3_GREEN, MT3620_RDB_LED3_BLUE} };

// LED state
static RgbLed ledTraffic = RGBLED_INIT_VALUE;
static RgbLed ledMessageEventSentReceived = RGBLED_INIT_VALUE;
static RgbLed ledNetworkStatus = RGBLED_INIT_VALUE;
static RgbLed *rgbLeds[] = { &ledTraffic, &ledMessageEventSentReceived, &ledNetworkStatus };
static const size_t rgbLedsCount = sizeof(rgbLeds) / sizeof(*rgbLeds);

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async signal safe
    terminationRequired = true;
}

// Return s - t
void TimerUtility_TimerSubtract(const struct timespec *s, const struct timespec *t, struct timespec *a)
{
	a->tv_sec = s->tv_sec - t->tv_sec;
	a->tv_nsec = s->tv_nsec - t->tv_nsec;
	if (a->tv_nsec < 0)
	{
		a->tv_sec -= 1;
		a->tv_nsec += 1e9;
	}
}

static float GetUltrasonicReading(GPIO_Id pin)
{
	struct timespec tsStart = { 0,0 };
	struct timespec tsEnd = { 0,0 };

	struct timespec tsWait = { 0, 10000 };
	int fd = GPIO_OpenAsOutput(pin, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (fd < 0) {
		Log_Debug("ERROR: Could not open Ultrasonic Output GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	nanosleep(&tsWait, NULL);
	GPIO_SetValue(fd, GPIO_Value_Low);
	close(fd);

	fd = GPIO_OpenAsInput(pin);
	if (fd < 0) {
		Log_Debug("ERROR: Could not open Ultrasonic Input GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	int result = 0;

	bool triggered = false;
	GPIO_Value_Type outVal;
		
	for (int i = 0; i < 10000; i++)
	{
		GPIO_GetValue(fd, &outVal);
		if (outVal == GPIO_Value_High)
		{
			if (!triggered)
			{
				clock_gettime(CLOCK_MONOTONIC, &tsStart);
				triggered = true;
			}
		}
		else
		{
			if (triggered)
			{
				clock_gettime(CLOCK_MONOTONIC, &tsEnd);
				Log_Debug("looped %d times\n", i);
				break;
			}
		}
	}

	close(fd);

	// never got a response return max value
	if (tsEnd.tv_sec == 0 && tsEnd.tv_nsec == 0)
	{
		return 400;
	}

	struct timespec elapsed = { 0,0 };
	TimerUtility_TimerSubtract(&tsEnd, &tsStart, &elapsed);

	return (elapsed.tv_nsec / 58000.0);
}

static bool occupiedState = false;

static void ReportStatusToIotHub(bool occupied)
{
	if (occupiedState != occupied)
	{
		occupiedState = occupied;

		if (connectedToIoTHub) {
			// Set twin property
			AzureIoT_TwinReportState("ParkingBayOccupied", occupied ? 1 : 0);
		}
		else {
			Log_Debug("WARNING: Cannot send message: not connected to the IoT Hub\n");
		}
	}
}

static void UltrasonicTimerEventHandler()
{
	if (ConsumeTimerFdEvent(usTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	float cms = GetUltrasonicReading(MT3620_GPIO0);
	
	if (cms > 8)
	{
		ReportStatusToIotHub(false);
		LedBlinkUtility_SetLed(&ledTraffic, LedBlinkUtility_Colors_Green);
	}
	else if (cms > 2)
	{
		ReportStatusToIotHub(true);
		LedBlinkUtility_SetLed(&ledTraffic, LedBlinkUtility_Colors_Yellow);
	}
	else
	{
		ReportStatusToIotHub(true);
		LedBlinkUtility_SetLed(&ledTraffic, LedBlinkUtility_Colors_Red);
	}

	Log_Debug("Appox %.1f cm\n", cms);
}

/// <summary>
///  IoT Hub connection status callback function.
/// </summary>
/// <param name="connected">'true' when the connection to the IoT Hub is established.</param>
static void IoTHubConnectionStatusChanged(bool connected)
{
	connectedToIoTHub = connected;
}

/// <summary>
///  Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	LedBlinkUtility_OpenLeds(rgbLeds, rgbLedsCount, ledsPins);

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	// Initialize the Azure IoT SDK
	if (!AzureIoT_Initialize()) {
		Log_Debug("ERROR: Cannot initialize Azure IoT Hub SDK.\n");
		return -1;
	}

	AzureIoT_SetConnectionStatusCallback(&IoTHubConnectionStatusChanged);

	Log_Debug("Opening Ultrasonic\n");
	struct timespec usCheckPeriod = { 0, 500000000 };
	usTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &usCheckPeriod, &UltrasonicTimerEventHandler, EPOLLIN);

    return 0;
}

/// <summary>
///  Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	LedBlinkUtility_SetLed(&ledTraffic, LedBlinkUtility_Colors_Off);
	LedBlinkUtility_CloseLeds(rgbLeds, rgbLedsCount);

    Log_Debug("Closing file descriptors\n");
	CloseFdAndPrintError(usTimerFd, "USTimer");
    CloseFdAndPrintError(epollFd, "Epoll");
}

/// <summary>
///  Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
	const struct timespec iothub_retry_period = { 1, 0 };
	const struct timespec timespec_1ms = { 0, 1000000 };

	struct timespec next_iothub_connect, now;
 
	Log_Debug("Parking application starting\n");
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

	// Try to connect to the IoT hub immediately
	clock_gettime(CLOCK_MONOTONIC, &next_iothub_connect);
	bool iothub_connected = false;
	
    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }

		// Set network status LED color
		LedBlinkUtility_Colors color =
			(connectedToIoTHub ? LedBlinkUtility_Colors_Green : LedBlinkUtility_Colors_Off);
		if (LedBlinkUtility_SetLed(&ledNetworkStatus, color) != 0) {
			Log_Debug("ERROR: Set color for network status LED failed\n");
			break;
		}

		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect
		// - in the case of a failure, we back off for iothub_retry_period
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (TimerUtility_TimerCompareGreater(&now, &next_iothub_connect)) {
			iothub_connected = AzureIoT_SetupClient();
			clock_gettime(CLOCK_MONOTONIC, &now);
			TimerUtility_TimerAdd(&now, &iothub_retry_period, &next_iothub_connect);
		}

		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		if (iothub_connected) {
			AzureIoT_DoPeriodicTasks();
		}

		nanosleep(&timespec_1ms, NULL);
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting\n");
    return 0;
}
