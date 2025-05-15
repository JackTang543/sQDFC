#include "sUtils.h"


#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static char printf_fmt_buf[512];





#include "sBSP_UART.h"


//log输出接口
static void log_output(char* str){
    #ifdef SUTILS_OUTPUT_ENABLE
    sBSP_UART_Debug_SendBytes((uint8_t*)str,strlen(str));
    #endif
}




void sUtils_LogAssert(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output("[ASSERT]");
    log_output(printf_fmt_buf);
    log_output("\n");
}


void sUtils_LogError(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output("[ERROR]");
    log_output(printf_fmt_buf);
    log_output("\n");
}

void sUtils_LogWarn(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output("[WARN]");
    log_output(printf_fmt_buf);
    log_output("\n");
}

void sUtils_LogInfo(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output("[INFO]");
    log_output(printf_fmt_buf);
    log_output("\n");
}


void sUtils_LogPrintf(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output(printf_fmt_buf);
}

void sUtils_LogPrintfln(const char *fmt,...){
    va_list args;
    va_start(args, fmt);
    vsprintf(printf_fmt_buf, fmt, args);
    va_end(args);

    log_output(printf_fmt_buf);
    log_output("\n");
}
























