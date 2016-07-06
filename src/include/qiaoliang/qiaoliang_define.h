#pragma once

#include <px4_log.h>
#include <math.h>
#include <float.h>

//#ifdef CONFIG_DEBUG
//#define	__DEBUG__							(1)
//#else/*CONFIG_DEBUG*/
//#undef	__DEBUG__	
//#endif/*CONFIG_DEBUG*/

#if __MAVLINK_LOG_FILE__
#define xia_debug(_fd, _text, ...)			mavlink_vasprintf(_fd, MAVLINK_IOC_SEND_TEXT_INFO, _text, ##__VA_ARGS__)	\
											fprintf(stderr, _text, ##__VA_ARGS__);										\
											fprintf(stderr, "\n");
#else	
#define xia_debug(_fd, _text, ...)			fprintf(stderr, _text, ##__VA_ARGS__);	\
											fprintf(stderr, "\n");

#endif/*__MAVLINK_LOG_FILE__*/

#define  __DEBUG__								(0)
#define __DAVID_DISTANCE__						(1)
#define __DAVID_DISTANCE_FIX__					(1)
#define __DAVID_MANUAL__						(1)
#define __DAVID_NAV_LOG__  						(1)
#define __DAVID_YAW_FIX__  						(1)
#define __ATT_PROTECT_FIX__  					(1)
#define __MANUAL_DZ__  							(1)
#define __ALT_CONTROL_TEST__  					(1)
#define __ARMED_FIX__  							(1)
#define __ARMED_FIX_1__  						(0)


#define __DAVID_CHAO_WARING__  					(1)
#define __DAVID_ARMED_FIX__  					(1)

#define __FMU_CONFIG__  						(1)
#define __FMU_PMW_YUNTAI__ 						(1)
#define __MC_POS_FIX__ 							(1)
#define __LAND_FIX__ 							(1)

#define __PRESSURE__ 							(1)
#define __PRESSURE_LB__ 						(1)
#define __PRESSURE_PARAM__ 						(1)
#define __PRESSURE_1__ 							(0)

#define __POS_MANAUL__ 							(0)
#define __BATT_I2C__ 							(0)

#define __RC_LOSS_TEST__ 						(0)


#if __DEBUG__
#  define __PX4FLOW_TEST_						(1)
#else
#  define __PX4FLOW_TEST_						(0)

#endif/*__DEBUG__*/

#if __PX4FLOW_TEST_
#  define PX4FLOW_WARNX(x)						xia_debug x
#else
#  define PX4FLOW_WARNX(x)						
#endif/*__PX4FLOW_TEST_*/

#define COMPARE_FLOAT(a,b)	((fabsf((float)(a) - (float)(b)) < FLT_EPSILON) ? 1 : 0)

static __inline int __COMPARE_FLOAT(double _AA, double _BB)
{
	if (fabs(_AA - _BB)< DBL_EPSILON)
	{
	 	return 0;

	} /* End if () */

	if (_AA < _BB)
	{
		return -1;

	} /* End if () */

 	return 1;
}

