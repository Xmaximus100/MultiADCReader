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


/*
 * Trim - Remove leading and trailing whitespace from a string
 * @param s: String to trim (modified in place)
 * @return none
 * 
 * Removes spaces and tabs from the beginning and end of the string.
 * Used internally by the AT parser to clean command input.
 */
static void Trim(char *s){
	int n = (int)strlen(s);
	while(n>0 && (s[n-1]==' ' || s[n-1]=='\t')) s[--n]=0;
	char *p = s;
	while(*p==' ' || *p=='\t') p++;
	if(p!=s) memmove(s, p, strlen(p)+1);
}

/*
 * Tokenize - Parse command line into tokens (arguments)
 * @param cmd_line: Command line string to tokenize (modified in place)
 * @param argv: Output array of token pointers (null-terminated)
 * @return Number of tokens found
 * 
 * Parses an AT command line into tokens, handling "AT" or "AT+" prefixes,
 * and separators like spaces, tabs, commas, and equals signs.
 * Maximum number of tokens is limited by AT_MAX_TOKENS.
 */
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

/*
 * FindCommand - Find a registered command by name
 * @param name: Command name to search for (case-insensitive)
 * @return Pointer to command entry if found, NULL otherwise
 * 
 * Searches the registered command table for a matching command name.
 * Comparison is case-insensitive. Used internally by AT_ExecuteCmd.
 */
static AT_EntryT* FindCommand(const char *name){
	for (uint16_t i=0; i<AT_Handler.ncmds; i++){
		if (!strcasecmp(name, AT_Handler.func_table[i].name))
			return &AT_Handler.func_table[i];
	}
	return NULL;
}

/*
 * AT_ExecuteCmd - Execute an AT command from a command string
 * @param cmd: Command string to execute (may be modified)
 * @return none
 * 
 * Parses and executes an AT command. Trims whitespace, tokenizes arguments,
 * finds the command handler, executes it, and sends the appropriate response
 * (OK, ERROR, BUSY, ERROR:ARG, ERROR:UNKNOWN) based on the command status.
 */
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

/*
 * AT_Puts - Send a string response via AT context write function
 * @param ctx: AT context containing write function and user data
 * @param s: String to send
 * @return none
 * 
 * Sends a string through the AT context's write function, followed by
 * carriage return and line feed (\\r\\n). Used for sending command responses.
 */
void AT_Puts(AT_CtxT *ctx, const char *s){
	if (!ctx || !ctx->write) return;
	size_t len = strlen(s);
	ctx->write(s, (uint16_t) len);
	ctx->write("\r\n", 2);
}

/*
 * AT_Register - Register a new AT command with the parser
 * @param name: Command name (e.g., "ADC:SETUP")
 * @param cmd_func: Function pointer to command handler
 * @param help: Help text describing the command
 * @return true on success, false if name is NULL, table is full, or command already exists
 * 
 * Registers a new AT command in the command table. Command names must be unique.
 * Maximum number of commands is limited by AT_MAX_CMDS.
 */
bool AT_Register(const char *name, AT_CmdFunc cmd_func, const char *help){
	if (!name || AT_Handler.ncmds > AT_MAX_CMDS) return false;
	for (uint16_t i=0; i<AT_Handler.ncmds; i++){
		if (!strcasecmp(name, AT_Handler.func_table[i].name)) return false;
	}
	if (!AT_Handler.ncmds) AT_Handler.ncmds = 0;
	AT_Handler.func_table[AT_Handler.ncmds++] = (AT_EntryT){ .name=name, .func=cmd_func, .help=help };
	return true;
}

/*
 * AT_Init - Initialize the AT command parser
 * @param func: Write function for sending responses
 * @param user: User data pointer passed to write function
 * @return none
 * 
 * Initializes the AT parser with a write function for sending responses.
 * Clears the command table and sets up the parser state. This function
 * should be called before registering any commands.
 */
void AT_Init(AT_WriteFunc func, void *user){
	memset(&AT_Handler, 0, sizeof(AT_Handler));
	AT_Handler.writer = func;
	AT_Handler.user = user;
}

/*
 * AT_ReadChar - Process a single character for AT command parsing
 * @param c: Character to process
 * @return none
 * 
 * Processes incoming characters to build command lines:
 * - Carriage return (\\r): Ignored
 * - Line feed (\\n): Executes the accumulated command
 * - Backspace (\\b): Removes last character from command buffer
 * - Other characters: Added to command buffer (up to AT_MAX_LINE-1)
 * When command buffer overflows, it is reset.
 */
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

/*
 * AT_ReadBuf - Process a buffer of characters for AT command parsing
 * @param buf: Buffer containing characters to process
 * @param len: Number of characters to process (limited to AT_MAX_LINE)
 * @return none
 * 
 * Processes a buffer of characters by calling AT_ReadChar for each character.
 * Useful for processing received data in chunks. The length is limited to
 * AT_MAX_LINE to prevent buffer overflow.
 */
void AT_ReadBuf(char *buf, size_t len){
	for (int i=0; i<AT_MAX_LINE; i++) {
		AT_ReadChar(buf[i]);
	}
}

/*
 * AT_StrToSignedInt - Convert string to signed 32-bit integer
 * @param s: String to convert
 * @param out: Pointer to output variable
 * @return true on success, false on conversion error or overflow
 * 
 * Converts a decimal string to a signed 32-bit integer. Validates that
 * the entire string is consumed and the value is within INT32_MIN to INT32_MAX range.
 * Used for parsing numeric command arguments.
 */
bool AT_StrToSignedInt(const char *s, int32_t *out){
    if (!s || !out) return false;
//    errno = 0;
    char *end = NULL;
    long v = strtol(s, &end, 10);      // base 10; use 0 to auto-detect 0x.., 0.., 10..
    if (end == s)           return false;                // no digits
    if (*end != '\0')       return false;                // trailing junk
//    if (errno == ERANGE)    return false;                // overflow/underflow
    if (v < INT32_MIN || v > INT32_MAX) return false;    // range check
    *out = (int32_t)v;
    return true;
}

/*
 * AT_StrToUnsignedInt - Convert string to unsigned 32-bit integer
 * @param s: String to convert
 * @param out: Pointer to output variable
 * @return true on success, false on conversion error or overflow
 * 
 * Converts a decimal string to an unsigned 32-bit integer. Validates that
 * the entire string is consumed and the value is within 0 to UINT32_MAX range.
 * Used for parsing numeric command arguments.
 */
bool AT_StrToUnsignedInt(const char *s, uint32_t *out){
    if (!s || !out) return false;
//    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);             // base 10
    if (end == s)           return false;
    if (*end != '\0')       return false;
//    if (errno == ERANGE)    return false;
    if (v > UINT32_MAX)     return false;
    *out = (uint32_t)v;
    return true;
}

/*
 * AT_Help - Display help information for all registered commands
 * @param ctx: AT context for sending help text
 * @return none
 * 
 * Sends a formatted list of all registered AT commands with their descriptions.
 * Each command is displayed as "CMD_NAME -> DESCRIPTION". If no commands are
 * registered, displays "No Commands Assigned".
 */
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














