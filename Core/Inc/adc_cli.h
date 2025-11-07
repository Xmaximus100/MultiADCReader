/*
 * adc_cli.h
 *
 *  Created on: Sep 4, 2025
 *      Author: Celelele
 */

#ifndef INC_ADC_CLI_H_
#define INC_ADC_CLI_H_

#include "main.h"
#include "adc_mgr.h"

AT_StatusTypeDef Cmd_ADC_READ(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_SAMPLING(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_START(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_STOP(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_DISPLAY(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_FORMAT(AT_CtxT *ctx, int argc, const char *argv[]);
AT_StatusTypeDef Cmd_ADC_REFRESH(AT_CtxT *ctx, int argc, const char *argv[]);

void ADC_CommandInit(void);

#endif /* INC_ADC_CLI_H_ */
