/* Two outline-facet banks followed by a rounded post contact. Geometry only
 * proposes actual controls. The native first-goal latch, angle probes and
 * final replay still decide whether a route really scores. */
#define POST75_SEEDS 16
static struct {Shot q[MAX_BALLS][2][POST75_SEEDS];int count[MAX_BALLS][2],used[MAX_BALLS][2],owners,cursor;} post75;
static void post75_add(int owner,int side,Shot s){
 int n=post75.count[owner][side],p=0;
 for(int j=0;j<n;j++)if(absd(atan2(sin(s.angle-post75.q[owner][side][j].angle),cos(s.angle-post75.q[owner][side][j].angle)))<.0003&&absd(s.power-post75.q[owner][side][j].power)<.002)return;
 while(p<n&&post75.q[owner][side][p].score>=s.score)p++;
 if(p>=POST75_SEEDS)return;
 for(int j=n<POST75_SEEDS?n:POST75_SEEDS-1;j>p;j--)post75.q[owner][side][j]=post75.q[owner][side][j-1];
 post75.q[owner][side][p]=s;post75.count[owner][side]=n<POST75_SEEDS?n+1:n;
}
static void post75_build(const PlanBody*b,int n,double goalx,double height,
 double xmin,double xmax,double ymin,double ymax,int opening,int mode){
 memset(&post75,0,sizeof(post75));
 if(opening||mode||!b||n<2||n>MAX_BALLS||!finite(goalx)||absd(goalx)<.01||height<=b[0].r*3||xmax<=xmin)return;
 int facets=planner_outline_active;if(facets>32)facets=32;
 double sign=goalx>0?1:-1,r=b[0].r;
 for(int side=0;side<2;side++)for(int sample=0;sample<15;sample++){
  double ysign=side?1:-1,theta=(sample+.5)*PI/30;
  Point outward={-sign*sin(theta),-ysign*cos(theta)};
  Point contact={goalx+r*outward.x,ysign*height*.5+r*outward.y};
  for(int first=0;first<facets;first++)for(int second=0;second<facets;second++){
   Point h1,h2;
   if(!video74_double_bounce(b[0].p,contact,first,second,r,&h1,&h2))continue;
   Point incoming=punit(psub(contact,h2));double dot=pdot(incoming,outward);
   if(dot>=-.02)continue;
   Point outgoing={incoming.x-2*dot*outward.x,incoming.y-2*dot*outward.y};
   if(outgoing.x*sign<=.01)continue;
   double travel=(goalx+sign*r*1.15-contact.x)/outgoing.x;
   double crossing=contact.y+travel*outgoing.y;
   double margin=height*.5-r*1.08-absd(crossing);
   if(travel<=0||margin<0)continue;
   Point normal=punit(psub(h1,b[0].p));
   for(int i=1;i<n;i++)if(b[i].owner>=0&&b[i].owner<MAX_BALLS){
    int owner=b[i].owner;post75.owners=pmax(post75.owners,owner+1);
    Point touch={b[0].p.x-(r+b[i].r)*normal.x,b[0].p.y-(r+b[i].r)*normal.y};
    if(touch.x<xmin+b[i].r||touch.x>xmax-b[i].r||touch.y<ymin+b[i].r||touch.y>ymax-b[i].r)continue;
    Point approach=punit(psub(touch,b[i].p));double alignment=pdot(approach,normal);
    if(alignment<=.02)continue;
    double length=pdist(b[0].p,h1)+pdist(h1,h2)+pdist(h2,contact);
    double quality=alignment+margin/height-.15*length/(xmax-xmin)-.15*plan_obstacles(b,n,b[i].p,touch,b[i].r,i,0,-1);
    static const double power[3]={1,.86,.72};
    for(int p=0;p<3;p++)post75_add(owner,side,(Shot){i,atan2(approach.y,approach.x),power[p],quality-p*.002});
   }
  }
 }
}
static int post75_next(Shot*out,int*owner){
 if(!out||!owner||!post75.owners)return 0;
 int total=post75.owners*2;
 for(int j=0;j<total;j++){
  int s=(post75.cursor++)%total,o=s%post75.owners,side=s/post75.owners;
  if(post75.used[o][side]>=post75.count[o][side])continue;
  *out=post75.q[o][side][post75.used[o][side]++];*owner=o;return 1;
 }
 return 0;
}
