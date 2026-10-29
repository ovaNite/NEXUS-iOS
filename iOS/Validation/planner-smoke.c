#include "NexusPlanner.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>
static NexusPoint outline[24]={
 {-122.889804,-76.141197},{-126.414804,-75.196676},{-128.995283,-72.616197},{-129.939804,-69.091197},
 {-129.939804,-27.046297},{-146.402775,-27.046297},{-146.402775,27.046297},{-129.939804,27.046297},
 {-129.939804,69.091197},{-128.995283,72.616197},{-126.414804,75.196676},{-122.889804,76.141197},
 {122.889804,76.141197},{126.414804,75.196676},{128.995283,72.616197},{129.939804,69.091197},
 {129.939804,27.046297},{146.402775,27.046297},{146.402775,-27.046297},{129.939804,-27.046297},
 {129.939804,-69.091197},{128.995283,-72.616197},{126.414804,-75.196676},{122.889804,-76.141197}
};
int main(void){
 NexusBody b[7]={{{0,0},3.9,-1},{{-110,0},8.9,0},{{-40,35},8.9,1},{{-40,-35},8.9,2},{{40,0},8.9,-1},{{115,30},8.9,-1},{{115,-30},8.9,-1}};
 NexusCandidate out[128];
 int n=nexus_candidates(b,7,3,outline,24,129.939804,54,-129.939804,129.939804,-76.141197,76.141197,0,0,out,128);assert(n>0&&n<=128);
 for(int i=0;i<n;i++)assert(out[i].puck>=1&&out[i].puck<=3&&isfinite(out[i].angle)&&out[i].power>0&&out[i].power<=1.05);
 n=nexus_candidates(b,7,3,outline,24,129.939804,54,-129.939804,129.939804,-76.141197,76.141197,1,0,out,128);assert(n>0);
 for(int i=0;i<n;i++)assert(out[i].puck==2||out[i].puck==3);
 assert(nexus_candidates(0,7,3,outline,24,130,54,-130,130,-76,76,0,0,out,128)==-1);
 b[0].position.x=NAN;assert(nexus_candidates(b,7,3,outline,24,130,54,-130,130,-76,76,0,0,out,128)==-1);
 puts("PASS: portable candidate C API, bounded output, kickoff front row and invalid snapshot rejection; Windows host only, no iOS physics or goal proof");
}
