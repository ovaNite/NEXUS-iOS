#ifndef NEXUS_PLANNER_H
#define NEXUS_PLANNER_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define NEXUS_CORE_REVISION 79
typedef struct {double x,y;} NexusPoint;
typedef struct {NexusPoint position;double radius;int32_t owner;} NexusBody;
typedef struct {int32_t puck;double angle,power;} NexusCandidate;
/* Proposals only. The game adapter must simulate the original physics,
 * first goal/foul rules, turn eligibility and actual input encoding.
 * Returns candidate count; -1 invalid snapshot; -2 concurrent invocation.
 * owner=-1 for opponents/ball, own owners are contiguous 0..owners-1. */
int nexus_candidates(const NexusBody*bodies,int count,int owners,const NexusPoint*outline,int vertices,
 double goalX,double goalHeight,double xmin,double xmax,double ymin,double ymax,
 int opening,int mode,NexusCandidate*out,int capacity);
#ifdef __cplusplus
}
#endif
#endif
