/* Roles inferred from the current formation, never from shirt artwork.
 * Only the leading own row may take a kickoff. Normal shots keep all owners.
 * The corner probes below propose controls; native physics remains decisive. */
static U kickoff_strikers72(const PlanBody*b,int n,double goalx){
 if(!b||n<2||n>MAX_BALLS||!finite(goalx)||absd(goalx)<.01)return 0;
 double sign=goalx>0?1:-1,front=-1e9,maxr=0;U mask=0;
 for(int i=1;i<n;i++)if(b[i].owner>=0){
  if(!finite(b[i].p.x)||!finite(b[i].r)||b[i].r<=0)return 0;
  front=pmax(front,sign*b[i].p.x);maxr=pmax(maxr,b[i].r);
 }
 if(maxr<=0)return 0;
 double tolerance=pmax(maxr*.6,absd(goalx)*.015);
 for(int i=1;i<n;i++)if(b[i].owner>=0&&front-sign*b[i].p.x<=tolerance)mask|=(U)1<<i;
 return mask;
}
static struct{int n,cursor,puck[MAX_BALLS],probe[MAX_BALLS];double bearing[MAX_BALLS];} corner72;
static void corner72_reset(const PlanBody*b,int n,int opening,int mode,
                           double xmin,double xmax,double ymin,double ymax){
 memset(&corner72,0,sizeof(corner72));
 if(opening||mode||!b||n<2||n>MAX_BALLS)return;
 double edge=min(min(absd(b[0].p.x-xmin),absd(b[0].p.x-xmax)),
                 min(absd(b[0].p.y-ymin),absd(b[0].p.y-ymax)));
 if(!planner_corner_state(b[0].p,b[0].r,xmin,xmax,ymin,ymax)&&edge>b[0].r*4)return;
 /* Each owner gets the same bounded early contact sweep, including the
  * upper forward whose straight goal geometry may be inaccessible. */
 for(int i=1;i<n;i++)if(b[i].owner>=0&&b[i].owner<MAX_BALLS){
  int o=b[i].owner;corner72.puck[o]=i;
  corner72.bearing[o]=atan2(b[0].p.y-b[i].p.y,b[0].p.x-b[i].p.x);
  corner72.n=pmax(corner72.n,o+1);
 }
}
static int corner72_next(Shot*out,int*owner){
 if(!out||!owner||!corner72.n)return 0;
 static const double offsets[16]={0,-.006,.006,-.016,.016,-.04,.04,-.09,.09,-.18,.18,-.35,.35,-.65,.65,1.1};
 static const double powers[4]={1,.82,.64,.46};
 for(int k=0;k<corner72.n;k++){
  int o=(corner72.cursor++)%corner72.n,p=corner72.probe[o];
  if(!corner72.puck[o]||p>=64)continue;
  corner72.probe[o]++;
  *out=(Shot){corner72.puck[o],corner72.bearing[o]+offsets[p%16],powers[p/16],15};
  *owner=o;return 1;
 }
 return 0;
}
