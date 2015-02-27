#ifndef __AT_LOG_H__
#define __AT_LOG_H__

#include <utils/Log.h>

/* Logs options */
#define AT_LOGS_NOTHING          0
#define AT_LOGS_WARN             1
#define AT_LOGS_WARN_INFO        2
#define AT_LOGS_WARN_INFO_DBG    3
#define AT_LOGS_WARN_INFO_DBG_FN 4
#define AT_LOGS_ALL              5

#define AT_LOG_VERBOSITY_LEVEL AT_LOGS_ALL

#define __AT_FUNCTION__ __FUNCTION__
#define __AT_LINE__ __LINE__

unsigned int At_TimeMsec(void);
int At_PrintErr(char func_name, int line, unsigned int time, const char *fmt, ...);
int At_PrintWrn(char func_name, int line, unsigned int time, const char *fmt, ...);
int At_PrintInf(char func_name, int line, unsigned int time, const char *fmt, ...);
int At_PrintDbg(char func_name, int line, unsigned int time, const char *fmt, ...);

/* Errors will always get printed */
#define AT_ERROR(...) do {  ALOGE("<Atmel-BT ERR>(%d)(%s:%d) ",At_TimeMsec(),__AT_FUNCTION__,__AT_LINE__, __VA_ARGS__);} while(0)

/* Wraning only printed if verbosity is 1 or more */	
#if (AT_LOG_VERBOSITY_LEVEL > 0)
#define AT_WARN(...) do { ALOGW("<Atmel-BT WRN>(%d)(%s:%d) ",At_TimeMsec(),__AT_FUNCTION__,__AT_LINE__);\
	ALOGW(__VA_ARGS__);  } while(0)
#else
#define AT_WARN(...) (0)
#endif

/* Info only printed if verbosity is 2 or more */
#if (AT_LOG_VERBOSITY_LEVEL > 1)
#define AT_INFO(...) do {  ALOGI("<Atmel-BT INF>(%d)(%s:%d) ",At_TimeMsec(),__AT_FUNCTION__,__AT_LINE__);\
	ALOGI(__VA_ARGS__);  } while(0)
#else
#define AT_INFO(...) (0)
#endif

/* Debug is only printed if verbosity is 3 or more */
#if (AT_LOG_VERBOSITY_LEVEL > 2)
#define AT_DBG(...) do { ALOGD("<Atmel-BT DBG>(%d)(%s:%d) ",At_TimeMsec(),__AT_FUNCTION__,__AT_LINE__);\
	ALOGD(__VA_ARGS__);  } while(0)

#else
#define AT_DBG(...) (0)
#endif

/* Function In/Out is only printed if verbosity is 4 or more */
#if (AT_LOG_VERBOSITY_LEVEL > 3)
#define AT_FN_IN do { AT_PRINTF("(FIN) (%s:%d) \n", __AT_FUNCTION__, __AT_LINE__);  } while(0)
#define AT_FN_OUT(ret) do { AT_PRINTF("(FOUT) (%s:%d) %d.\n",__AT_FUNCTION__,__AT_LINE__,(ret));  } while(0)
#else
#define AT_FN_IN (0)
#define AT_FN_OUT(ret) (0)
#endif


#endif