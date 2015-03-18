#include <time.h>
#include <utils/Log.h>
#include <stdarg.h>

unsigned int At_TimeMsec(void)
{
	unsigned u32Time = 0;
	struct timespec current_time;

	if (clock_gettime(CLOCK_REALTIME, &current_time) == 0)
	{
		u32Time = current_time.tv_sec * 1000;
		u32Time += current_time.tv_nsec / 1000000;
	}
	
	return u32Time;
}

int At_PrintErr(char* func_name, int line, unsigned int time, const char *fmt, ...)
{
	char buf[2*1024];
	int index = 0;
	va_list args;
	va_start(args, fmt);
	index += sprintf(buf, "<Atmel-BT INF>(%d)(%s:%d)" , time, func_name, line);
	index += sprintf(buf+index, fmt, args);
	ALOGE("%s", buf);
	return index;
}

int At_PrintWrn(char* func_name, int line, unsigned int time, const char *fmt, ...)
{
	char buf[2*1024];
	int index = 0;
	va_list args;
	va_start(args, fmt);
	index += sprintf(buf, "<Atmel-BT INF>(%d)(%s:%d)" , time, func_name, line);
	index += sprintf(buf+index, fmt, args);
	ALOGW("%s", buf);
	return index;
}

int At_PrintInf(char* func_name, int line, unsigned int time, const char *fmt, ...)
{
	char buf[2*1024];
	int index = 0;
	va_list args;
	va_start(args, fmt);
	index += sprintf(buf, "<Atmel-BT INF>(%d)(%s:%d)" , time, func_name, line);
	index += sprintf(buf+index, fmt, args);
	ALOGI("%s", buf);
	return index;
}

int At_PrintDbg(char* func_name, int line, unsigned int time, const char *fmt, ...)
{
	char buf[2*1024];
	int index = 0;
	va_list args;
	va_start(args, fmt);
	index += sprintf(buf, "<Atmel-BT INF>(%d)(%s:%d)" , time, func_name, line);
	index += sprintf(buf+index, fmt, args);
	ALOGD("%s", buf);
	return index;
}