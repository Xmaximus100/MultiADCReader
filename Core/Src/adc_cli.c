/*
 * adc_cli.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Celelele
 */

#include "adc_cli.h"


extern ADC_Handler g_adc;
extern ADC_Handler * const g_adc_mgr;

/*
 * Cmd_HELP - Display help information for all available AT commands
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments
 * @param argv: Array of command argument strings
 * @return AT_OK if help was displayed, AT_UNK if no arguments provided
 * 
 * This function lists all registered AT commands with their descriptions.
 * It also informs the user that commands only work when sampling is stopped.
 */
AT_StatusTypeDef Cmd_HELP(AT_CtxT *ctx, int argc, const char *argv[]){
	if (argc>0){
		AT_Help(ctx);
		AT_Puts(ctx, "Commands only work when sampling is stopped");
		return AT_OK;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_SETUP - Configure ADC system with samples, frequency, device count, and mode
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 5)
 * @param argv: Array of command arguments [cmd, samples, freq, dev_amount, mode]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid arguments, AT_ERROR on failure
 * 
 * Sets up the ADC system with:
 * - samples_requested: Number of samples to collect (0 = 1 second interval)
 * - freq: Sampling frequency in Hz (must be < MAX_SAMPLING_FREQ)
 * - dev_amount: Number of connected ADC converters (must be < MAX_DEVICES)
 * - mode: Continuous mode (0 = single shot, 1 = continuous)
 * Automatically configures clock prescaler and starts sampling if successful.
 */
AT_StatusTypeDef Cmd_ADC_SETUP(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 5) {
		uint32_t samples_requested;
		uint32_t freq;
		uint32_t dev_amount;
		uint32_t mode;
		if (!AT_StrToUnsignedInt(argv[1], &samples_requested)) return AT_ARG;
		if (!AT_StrToUnsignedInt(argv[2], &freq)) return AT_ARG;
		if (!AT_StrToUnsignedInt(argv[3], &dev_amount)) return AT_ARG;
		if (!AT_StrToUnsignedInt(argv[4], &mode)) return AT_ARG;
		if (freq < MAX_SAMPLING_FREQ) {
			uint32_t prescaler;
			LTC2368_ConfigSampling(&g_adc_mgr->clock_handler, freq);
			LTC2368_AdjustPrescaler(&g_adc_mgr->clock_handler, g_adc_mgr->clock_handler.samp_freq, &prescaler);
			LTC2368_ConfigReading(&g_adc_mgr->clock_handler, prescaler);
		}
		else return AT_ARG;
		if (samples_requested > 0 && samples_requested < MAX_SAMPLES_REQUESTED){
			ADC_ChangeRequestedSamples(g_adc_mgr, samples_requested);
		}
		else if (samples_requested == 0) {
			uint32_t samples_to_read = (freq<MAX_SAMPLES_REQUESTED)? freq : MAX_SAMPLES_REQUESTED;
			ADC_ChangeRequestedSamples(g_adc_mgr, samples_to_read);
		}
		else return AT_ARG;
		if (dev_amount <= MAX_DEVICES && mode <= 1) {
			ADC_ManagerInit(g_adc_mgr, (uint8_t)dev_amount, (bool)mode);
		}
		else return AT_ARG;
		if (LTC2368_StartSampling(&g_adc_mgr->clock_handler) == LTC2368_OK) {
			g_adc_mgr->state = true;
			return AT_OK;
		} else return AT_ERROR;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_READ - Change the number of samples requested for collection
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, samples_requested]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid arguments, AT_UNK on wrong argument count
 * 
 * Updates the number of samples to collect. Value must be between 1 and LTC2368_MAX_MEMORY.
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_ADC_READ(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2) {
		uint32_t samples_requested;
		if (!AT_StrToUnsignedInt(argv[1], &samples_requested)) return AT_ARG;
		if (samples_requested > 0 && samples_requested < LTC2368_MAX_MEMORY){
			ADC_ChangeRequestedSamples(g_adc_mgr, samples_requested);
			return AT_OK;
		}
		else return AT_ARG;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_SAMPLING - Configure the ADC sampling frequency
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, frequency]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid frequency, AT_ERROR on failure, AT_UNK on wrong argument count
 * 
 * Sets the sampling frequency in Hz. The frequency must be less than MAX_SAMPLING_FREQ.
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_ADC_SAMPLING(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2){
		uint32_t freq;
		if (!AT_StrToUnsignedInt(argv[1], &freq)) return AT_ARG;
		if (freq < MAX_SAMPLING_FREQ) {
			LTC2368_ConfigSampling(&g_adc_mgr->clock_handler, freq);
			return AT_OK;
		}
		else return AT_ERROR;
	}
	else return AT_UNK;
}

/*
 * Cmd_CLOCK_SOURCE - Select the clock source for ADC sampling
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, source] where source is "I" (internal), "E" (external), or frequency value
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid source, AT_UNK on wrong argument count
 * 
 * Configures the clock source for ADC sampling:
 * - "I" or "i": Internal system clock (SYSTEM_FREQ)
 * - "E" or "e": External reference clock (REF_FREQ)
 * - Numeric value: Custom frequency in Hz
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_CLOCK_SOURCE(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2){
		uint32_t freq;
		if (!strcasecmp(argv[1], "I")) freq = SYSTEM_FREQ;
		else if (!strcasecmp(argv[1], "E")) freq = REF_FREQ;
		else if (AT_StrToUnsignedInt(argv[1], &freq));
		else return AT_ARG;
		LTC2368_SelectSource(&g_adc_mgr->clock_handler, freq);
		LTC2368_ConfigSampling(&g_adc_mgr->clock_handler, g_adc_mgr->clock_handler.samp_freq);
		return AT_OK;
	}
	else return AT_UNK;
}

/*
 * Cmd_CLOCK_SPEED - Configure the reading clock prescaler value
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2 or 3)
 * @param argv: Array of command arguments [cmd, prescaler] or [cmd, prescaler, display_flag]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid prescaler, AT_ERROR on display failure, AT_UNK on wrong argument count
 * 
 * Sets the prescaler value for the reading clock. If 3 arguments are provided, also displays
 * the current clock configuration. This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_CLOCK_SPEED(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2 || argc == 3){
		uint32_t psc;
		if (AT_StrToUnsignedInt(argv[1], &psc));
		else return AT_ARG;
		LTC2368_ConfigReading(&g_adc_mgr->clock_handler, psc);
		if (argc == 2) return AT_OK;
		if (ADC_DisplayConfig(g_adc_mgr)) {
			return AT_OK;
		} else return AT_ERROR;
	}
	else return AT_UNK;
}

/*
 * Cmd_CLOCK_CONFIG - Display current clock configuration
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 1)
 * @param argv: Array of command arguments [cmd]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ERROR on display failure, AT_UNK on wrong argument count
 * 
 * Displays the current clock configuration including:
 * - Sampling frequency
 * - Reading frequency
 * - Reference frequency
 * - Number of DMA nodes used
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_CLOCK_CONFIG(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 1){
		if (ADC_DisplayConfig(g_adc_mgr)) {
			return AT_OK;
		} else return AT_ERROR;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_START - Start ADC sampling
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 1)
 * @param argv: Array of command arguments [cmd]
 * @return AT_OK on success, AT_BUSY if ADC is already sampling, AT_ERROR on failure, AT_UNK on wrong argument count
 * 
 * Starts the ADC sampling process. For non-continuous mode, sets the requested samples count.
 * Sets the ADC state to active (true) on successful start.
 */
AT_StatusTypeDef Cmd_ADC_START(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 1){
		if (!g_adc_mgr->continuous)
			ADC_ChangeRequestedSamples(g_adc_mgr, g_adc_mgr->samples_requested);
		if (LTC2368_StartSampling(&g_adc_mgr->clock_handler) == LTC2368_OK) {
			g_adc_mgr->state = true;
			return AT_OK;
		} else return AT_ERROR;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_STOP - Stop ADC sampling
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 1)
 * @param argv: Array of command arguments [cmd]
 * @return AT_OK on success, AT_BUSY if ADC is not currently sampling, AT_ERROR on failure, AT_UNK on wrong argument count
 * 
 * Stops the ADC sampling process and sets the ADC state to inactive (false).
 * This command only works when sampling is active.
 */
AT_StatusTypeDef Cmd_ADC_STOP(AT_CtxT *ctx, int argc, const char *argv[]){
	if (!ADC_BusyCheck()) return AT_BUSY;
	if (argc == 1){
		if (LTC2368_StopSampling(&g_adc_mgr->clock_handler) == LTC2368_OK) {
			g_adc_mgr->state = false;
			return AT_OK;
		} else return AT_ERROR;
	} else return AT_UNK;
}

/*
 * Cmd_ADC_DISPLAY - Display collected ADC samples
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, requested_samples]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ERROR on display failure, AT_UNK on wrong argument count
 * 
 * Displays the collected ADC samples. The display format depends on the current format setting:
 * - Format 0 (Raw): Binary efficient format for high-speed data transfer
 * - Format 1 (Clear): Human-readable format with voltage values
 * If requested_samples is 0, displays the entire buffer. This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_ADC_DISPLAY(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2){
		uint32_t requested_samples;
		if (!AT_StrToUnsignedInt(argv[1], &requested_samples)) return AT_ARG;
		if (g_adc_mgr->format == 0){
			if (ADC_DisplaySamples_Raw(g_adc_mgr, KEEP_BUF, requested_samples) == false){
				return AT_ERROR;
			}
		}
		else {
			if (ADC_DisplaySamples_Clear(g_adc_mgr, KEEP_BUF, requested_samples) == false){
				return AT_ERROR;
			}
		}
		return AT_OK;
	}
	return AT_UNK;
}

/*
 * Cmd_ADC_FORMAT - Set the display format for ADC samples
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, format] where format is "R" (raw) or "C" (clear)
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid format, AT_UNK on wrong argument count
 * 
 * Sets the display format for ADC samples:
 * - "R" or "r": Raw format - binary efficient format for high-speed data transfer
 * - "C" or "c": Clear format - human-readable format with voltage values
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_ADC_FORMAT(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2){
		if (!strcasecmp(argv[1], "R")) g_adc_mgr->format = 0;
		else if (!strcasecmp(argv[1], "C")) g_adc_mgr->format = 1;
		else return AT_ARG;
		return AT_OK;
	}
	return AT_UNK;
}

/*
 * Cmd_ADC_REFRESH - Set the refresh interval for display data
 * @param ctx: AT command context containing write function and user data
 * @param argc: Number of command arguments (must be 2)
 * @param argv: Array of command arguments [cmd, refresh_interval_ms]
 * @return AT_OK on success, AT_BUSY if ADC is sampling, AT_ARG on invalid interval, AT_UNK on wrong argument count
 * 
 * Sets the refresh interval in milliseconds for periodic display updates.
 * This command only works when sampling is stopped.
 */
AT_StatusTypeDef Cmd_ADC_REFRESH(AT_CtxT *ctx, int argc, const char *argv[]){
	if (ADC_BusyCheck()) return AT_BUSY;
	if (argc == 2){
		uint32_t tmp_refresh;
		if (!AT_StrToUnsignedInt(argv[1], (uint32_t*)&tmp_refresh)) return AT_ARG;
		g_adc_mgr->refresh_interval = tmp_refresh;
		return AT_OK;
	} else return AT_UNK;
}

/*
 * ADC_CommandInit - Initialize and register all ADC-related AT commands
 * @param none
 * @return none
 * 
 * Registers all ADC-related AT commands with the AT parser system:
 * - ADC:SETUP, ADC:READ, ADC:SAMPLING, ADC:START, ADC:STOP
 * - ADC:DISPLAY, ADC:FORMAT, ADC:MODE (refresh)
 * - CLK:SOURCE, CLK:SPEED, CLK:CONFIG
 * - HELP
 * This function should be called during system initialization.
 */
void ADC_CommandInit(void){
	AT_Register("ADC:SETUP", 	Cmd_ADC_SETUP, 		"Setup: number of samples (0 for 1second interval collection), sampling frequency, number of converters, contiuous mode");
	AT_Register("ADC:READ", 	Cmd_ADC_READ, 		"Set requested number of samples or type 0 for 1second interval collection");
	AT_Register("ADC:SAMPLING", Cmd_ADC_SAMPLING, 	"Define sampling frequency from 1 to 100k in Hz");
	AT_Register("CLK:SOURCE", 	Cmd_CLOCK_SOURCE, 	"Define clock source: (I)nternal, (E)xternal or type frequency value");
	AT_Register("CLK:SPEED", 	Cmd_CLOCK_SPEED, 	"Define reading clock prescaler value");
	AT_Register("CLK:CONFIG", 	Cmd_CLOCK_CONFIG, 	"Display current clock configuration");
	AT_Register("ADC:START", 	Cmd_ADC_START, 		"Start sampling");
	AT_Register("ADC:STOP", 	Cmd_ADC_STOP, 		"Stop sampling");
	AT_Register("ADC:DISPLAY",	Cmd_ADC_DISPLAY,	"Display n amount of samples or type 0 to print out whole buffer");
	AT_Register("ADC:FORMAT",	Cmd_ADC_FORMAT,		"Set display format: (R)aw or (C)lear for raw efficient or clear readable");
	AT_Register("ADC:MODE", 	Cmd_ADC_REFRESH,	"Set refresh time of display data in ms");
	AT_Register("HELP", 		Cmd_HELP, 			"List out optional commands");
}
