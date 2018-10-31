/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "stm32l4xx_hal.h"
#include "Task_manager.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"


uint32_t cpu_load;
float32_t cpu_float;

/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */
typedef struct tskTaskControlBlock
{
	volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
	#endif

	ListItem_t			xStateListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t			*pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

	#if ( portSTACK_GROWTH > 0 )
		StackType_t		*pxEndOfStack;		/*< Points to the end of the stack on architectures where the stack grows up from low memory. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		UBaseType_t		uxCriticalNesting;	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		UBaseType_t		uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		UBaseType_t		uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
		UBaseType_t		uxMutexesHeld;
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		TaskHookFunction_t pxTaskTag;
	#endif

	#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
		void *pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#endif

	#if( configGENERATE_RUN_TIME_STATS == 1 )
		uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
	#endif

	#if ( configUSE_NEWLIB_REENTRANT == 1 )
		/* Allocate a Newlib reent structure that is specific to this task.
		Note Newlib support has been included by popular demand, but is not
		used by the FreeRTOS maintainers themselves.  FreeRTOS is not
		responsible for resulting newlib operation.  User must be familiar with
		newlib and must provide system-wide implementations of the necessary
		stubs. Be warned that (at the time of writing) the current newlib design
		implements a system-wide malloc() that must be provided with locks. */
		struct	_reent xNewLib_reent;
	#endif

	#if( configUSE_TASK_NOTIFICATIONS == 1 )
		volatile uint32_t ulNotifiedValue;
		volatile uint8_t ucNotifyState;
	#endif

	/* See the comments above the definition of
	tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
	#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
		uint8_t	ucStaticallyAllocated; 		/*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
	#endif

	#if( INCLUDE_xTaskAbortDelay == 1 )
		uint8_t ucDelayAborted;
	#endif

} tskTCB;

/* The old tskTCB name is maintained above then typedefed to the new TCB_t name
below to enable the use of older kernel aware debuggers. */
typedef tskTCB TCB_t;

uint32_t GetRunIdle(uint32_t * const pulTotalRunTime)
{
    TCB_t *pxTCB_idle = (TCB_t *)xTaskGetIdleTaskHandle();
    
#if ( configGENERATE_RUN_TIME_STATS == 1)
    {
	if( pulTotalRunTime != NULL )
	{
#ifdef portALT_GET_RUN_TIME_COUNTER_VALUE
	    portALT_GET_RUN_TIME_COUNTER_VALUE( ( *pulTotalRunTime ) );
#else
	    *pulTotalRunTime = portGET_RUN_TIME_COUNTER_VALUE();
#endif
	}
    }
#else
    {
	if( pulTotalRunTime != NULL )
	{
	    *pulTotalRunTime = 0;
	}
    }
#endif
    return 	pxTCB_idle->ulRunTimeCounter;
}

    
void Task_manager_LoadCPU()
{
    static	uint32_t idle_last_time;
    static	uint32_t LastRunTime = 0;    
    volatile uint32_t idle_time;
    uint32_t ulTotalRunTime =0;
    volatile uint32_t current_idle_time =0;
    
    uint32_t currentRunTime =0;
    
    idle_time = GetRunIdle(&ulTotalRunTime);
    current_idle_time = (idle_time - idle_last_time)*1000;
    if ((ulTotalRunTime - LastRunTime) > 10000)
    {
			currentRunTime = ulTotalRunTime - LastRunTime;
			LastRunTime = ulTotalRunTime;
			cpu_load = (1000 - current_idle_time / currentRunTime);
			cpu_float = (float32_t) cpu_load / 10;
			idle_last_time = idle_time;
			/*set load CPU*/
			//registersSetCPU_load(cpu_load);
			
    }
		
		
}

void Task_manager_Init()
{
#include "tim.h"
    HAL_TIM_Base_Start(&htim2);
}

unsigned long getRunTimeCounterValue(void)
{
    return htim2.Instance->CNT;
}
