/*
 * at_parser.c
 *
 *  Created on: Aug 24, 2025
 *      Author: Celelele
 */

#include "at_parser.h"

typedef struct {
	const char *name;
	AT_CmdFunc func;
	const char *help;
} AT_EntryT;

static struct {
	AT_WriteFunc writer;
	void *user;
	AT_EntryT func_table[AT_MAX_CMDS];
	uint8_t ncmds;
	char cmd_line[AT_MAX_LINE];
	uint8_t cmd_ptr;
} AT_Handler;


static void Trim(char *s){
	int n = (int)strlen(s);
	while(n>0 && (s[n-1]==' ' || s[n-1]=='\t')) s[--n]=0;
	char *p = s;
	while(*p==' ' || *p=='\t') p++;
	if(p!=s) memmove(s, p, strlen(p)+1);
}

static int Tokenize(char *cmd_line, char *argv[]){
	int argc = 0;
	char *p = cmd_line;
	if (!strncasecmp(p, "AT", 2)) {
		p+=2;
		if (*p == '+') p++;
	}
	while (*p && argc < AT_MAX_TOKENS){
		/* Ignore separators like ',' or '=' */
		while (*p==' ' || *p=='\t' || *p=='=' || *p==',') p++;
		if (!*p) break;
		argv[argc++] = p;
		/* Skip the rest of separators */
		while (*p && *p!=' ' && *p!='\t' && *p!=',' && *p!='=') p++;
		if (*p) { *p=0; p++; }
	}
	return argc;
}

static AT_EntryT* FindCommand(const char *name){
	for (uint16_t i=0; i<AT_Handler.ncmds; i++){
		if (!strcasecmp(name, AT_Handler.func_table[i].name))
			return &AT_Handler.func_table[i];
	}
	return NULL;
}

static void AT_ExecuteCmd(char *cmd){
	/* Find, execute a command and display result */
	Trim(cmd);
	if (cmd[0]==0) return;
	AT_CtxT ctx = { .write=AT_Handler.writer, .user=AT_Handler.user};
	char *argv[AT_MAX_TOKENS] = {0};
	int argc = Tokenize(cmd, argv);
	if (!strcasecmp(cmd, "AT")) {
		AT_Puts(&ctx, "OK");
		return;
	}
	if (argc==0) {
		AT_Puts(&ctx, "ERROR");
		return;
	}
	AT_EntryT *e = FindCommand(cmd+3);
	if (!e) {
		AT_Puts(&ctx, "ERROR:UNKNOWN");
		return;
	}
	AT_StatusTypeDef status = e->func(&ctx, argc, (const char**)argv);
	switch(status){
		case AT_OK: AT_Puts(&ctx, "OK"); break;
		case AT_BUSY: AT_Puts(&ctx, "BUSY"); break;
		case AT_ARG: AT_Puts(&ctx, "ERROR:ARG"); break;
		default: AT_Puts(&ctx, "ERROR"); break;
	}
}

void AT_Puts(AT_CtxT *ctx, const char *s){
	if (!ctx || !ctx->write) return;
	size_t len = strlen(s);
	ctx->write(s, (uint16_t) len);
	ctx->write("\r\n", 2);
}

bool AT_Register(const char *name, AT_CmdFunc cmd_func, const char *help){
	if (!name || AT_Handler.ncmds > AT_MAX_CMDS) return false;
	for (uint16_t i=0; i<AT_Handler.ncmds; i++){
		if (!strcasecmp(name, AT_Handler.func_table[i].name)) return false;
	}
	if (!AT_Handler.ncmds) AT_Handler.ncmds = 0;
	AT_Handler.func_table[AT_Handler.ncmds++] = (AT_EntryT){ .name=name, .func=cmd_func, .help=help };
	return true;
}

void AT_Init(AT_WriteFunc func, void *user){
	memset(&AT_Handler, 0, sizeof(AT_Handler));
	AT_Handler.writer = func;
	AT_Handler.user = user;
}

void AT_ReadChar(char c){
	if (c == '\r') return;
	if (c == '\n'){
		AT_Handler.cmd_line[AT_Handler.cmd_ptr] = 0;
		AT_ExecuteCmd(AT_Handler.cmd_line);
		AT_Handler.cmd_ptr = 0;
		return;
	}
	if (c == '\b'){
		if (AT_Handler.cmd_ptr>0) AT_Handler.cmd_ptr--;
		return;
	}
	if (AT_Handler.cmd_ptr<AT_MAX_LINE-1){
		AT_Handler.cmd_line[AT_Handler.cmd_ptr++] = c;
	} else {
		AT_Handler.cmd_ptr = 0;
	}
}

void AT_ReadBuf(char *buf, size_t len){
	for (int i=0; i<AT_MAX_LINE; i++) {
		AT_ReadChar(buf[i]);
	}
}

bool AT_StrToSignedInt(const char *s, int32_t *out){
    if (!s || !out) return false;
    errno = 0;
    char *end = NULL;
    long v = strtol(s, &end, 10);      // base 10; use 0 to auto-detect 0x.., 0.., 10..
    if (end == s)           return false;                // no digits
    if (*end != '\0')       return false;                // trailing junk
    if (errno == ERANGE)    return false;                // overflow/underflow
    if (v < INT32_MIN || v > INT32_MAX) return false;    // range check
    *out = (int32_t)v;
    return true;
}

bool AT_StrToUnsignedInt(const char *s, uint32_t *out){
    if (!s || !out) return false;
    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);             // base 10
    if (end == s)           return false;
    if (*end != '\0')       return false;
    if (errno == ERANGE)    return false;
    if (v > UINT32_MAX)     return false;
    *out = (uint32_t)v;
    return true;
}

void AT_Help(AT_CtxT *ctx){
	char help_msg[AT_MAX_RESPONSE] = {0};
	sprintf(help_msg, "CMD NAME -> DESCRIPTION");
	AT_Puts(ctx, help_msg);
	if (AT_Handler.ncmds==0) {
		AT_Puts(ctx, "No Commands Assigned");
		return;
	}
	for (uint16_t i=0; i<AT_Handler.ncmds; i++){
		memset(help_msg, '\0', AT_MAX_RESPONSE);
		sprintf(help_msg, "%s -> %s", AT_Handler.func_table[i].name, AT_Handler.func_table[i].help);
		AT_Puts(ctx, help_msg);
	}
}














