#include "<name>_comm.h"
#include "jobs_behaviorGraph.h"
#include "stm32common/comm.h"
#include "stm32common/rtsched.h"

/**
 * @addtogroup STM32
 * @{
 * @addtogroup STM32_Pcb
 * @{
 * @addtogroup STM32_Pcb_mdaq2
 * @{
 * @addtogroup STM32_Pcb_mdaq2_Job_Behavior_Graph Job_Behavior_Graph
 * @{
 */

/** datastructure for rtsched */
static RTSNextCall job;
static <name>_comm_t comm;

/* evaluate the behavior graph inside mdaq2-baord */
static void jobsBehaviorGraph(void)
{
    <name>_process(&comm);

    job.time     = 1;/* us! */
    job.nextCall = jobsBehaviorGraph;
}

/* Initialize this job */
void jobsInitBehaviorGraph(const <name>_getInternalInputFunc sample, const <name>_setInternalOutputFunc commit)
{
    <name>_init(&comm, &commNDLComNode, sample, commit);

    /* set up this job */
    job.time = 1;/* given in us */
    job.nextCall = jobsBehaviorGraph;
    rtsRegisterCall(&job);
}

/**
 * @}
 * @}
 * @}
 * @}
 */
