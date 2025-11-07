/*
 * at_parser.h
 *
 *  Created on: Aug 24, 2025
 *      Author: Celelele
 */

#pragma once
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AT_MAX_LINE 30
#define AT_MAX_RESPONSE 200
#define AT_MAX_TOKENS 10
#define AT_MAX_CMDS 40

typedef enum {
	AT_OK    	= 0x00U,
	AT_ERROR	= 0x01U,
	AT_BUSY    	= 0x02U,
	AT_ARG		= 0x03U,
	AT_UNK		= 0x04U
} AT_StatusTypeDef;

typedef AT_StatusTypeDef (*AT_WriteFunc)(const char* buf, size_t len);

typedef struct {
	AT_WriteFunc write;
	void* user;
} AT_CtxT;

typedef AT_StatusTypeDef (*AT_CmdFunc)(AT_CtxT *ctx, int argc, const char *argv[]);

void AT_Puts(AT_CtxT *ctx, const char *s);

bool AT_Register(const char *name, AT_CmdFunc cmd_func, const char *help);

void AT_Init(AT_WriteFunc func, void *user);

void AT_ReadChar(char c);

void AT_ReadBuf(char *buf, size_t len);

bool AT_StrToSignedInt(const char *s, int32_t *out);

bool AT_StrToUnsignedInt(const char *s, uint32_t *out);

void AT_Help(AT_CtxT *ctx);

#ifdef __cplusplus
}
#endif
