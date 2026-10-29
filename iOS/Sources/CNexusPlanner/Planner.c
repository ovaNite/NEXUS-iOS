#include "NexusPlanner.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdatomic.h>
typedef uint64_t U;typedef uint32_t W;
typedef struct{double x,y;}Point;
typedef struct{int puck;double angle,power,score;}Shot;
#define MAX_BALLS 32
#define PI 3.14159265358979323846
#define finite isfinite
static double absd(double x){return fabs(x);}static double min(double a,double b){return a<b?a:b;}
static int actual_arena_mode,fast_arena_mode;
#include "planner.inc"
#include "situational_routes_r72.inc"
#include "video_routes_r74.inc"
#include "post_routes_r75.inc"
static atomic_flag planner_lock=ATOMIC_FLAG_INIT;
int nexus_candidates(const NexusBody*input,int n,int owners,const NexusPoint*outline,int vertices,
 double gx,double height,double xmin,double xmax,double ymin,double ymax,
 int opening,int mode,NexusCandidate*out,int cap){
 if(!input||!outline||!out||n<2||n>MAX_BALLS||owners<1||owners>5||vertices<3||vertices>256||cap<1||cap>2048||
    !isfinite(gx)||!isfinite(height)||height<=0||!isfinite(xmin)||!isfinite(xmax)||!isfinite(ymin)||!isfinite(ymax)||xmax<=xmin||ymax<=ymin||mode<0||mode>2)return -1;
 PlanBody b[MAX_BALLS];Point polygon[256];
 for(int k=0;k<n;k++){
  if(!isfinite(input[k].position.x)||!isfinite(input[k].position.y)||!isfinite(input[k].radius)||input[k].radius<=0||input[k].owner< -1||input[k].owner>=owners)return -1;
  b[k]=(PlanBody){{input[k].position.x,input[k].position.y},input[k].radius,input[k].owner};
 }
 for(int k=0;k<vertices;k++){if(!isfinite(outline[k].x)||!isfinite(outline[k].y))return -1;polygon[k]=(Point){outline[k].x,outline[k].y};}
 if(atomic_flag_test_and_set(&planner_lock))return -2;
 actual_arena_mode=fast_arena_mode=mode;
 int count=0;U allowed=opening?kickoff_strikers72(b,n,gx):0;
 planner_set_outline(polygon,vertices);
 if(planner_build(b,n,owners,gx,height,xmin,xmax,ymin,ymax,mode)){
  video74_build(b,n,gx,height,xmin,xmax,ymin,ymax,opening,mode,allowed);
  post75_build(b,n,gx,height,xmin,xmax,ymin,ymax,opening,mode);
  for(int step=0;step<cap*16&&count<cap;step++){
   Shot q;int owner;
   int found=(step%5==1)&&video74_next(&q,&owner);
   if(!found&&step%7==3)found=post75_next(&q,&owner);
   if(!found)found=planner_next(&q,&owner);
   if(!found)break;
   if(opening&&!(allowed&((U)1<<q.puck)))continue;
   out[count++]=(NexusCandidate){q.puck,q.angle,q.power};
  }
 }
 atomic_flag_clear(&planner_lock);return count;
}
