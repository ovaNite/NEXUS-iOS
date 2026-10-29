/* Bounded proposals motivated by the supplied corner-bank recordings.
 * Geometry orders experiments only. No simulated result, goal, award or
 * input is fabricated here; every proposal still needs native physics. */
#define VIDEO74_SEEDS 24
static struct {Shot q[MAX_BALLS][2][VIDEO74_SEEDS];int count[MAX_BALLS][2],used[MAX_BALLS][2],owners,cursor;} video74;
static void video74_add(int owner,int kind,Shot s){
 if(owner<0||owner>=MAX_BALLS||kind<0||kind>1||!finite(s.angle)||!finite(s.power)||s.power<.08||s.power>1)return;
 int n=video74.count[owner][kind];
 for(int j=0;j<n;j++){
  Shot old=video74.q[owner][kind][j];
  if(absd(atan2(sin(old.angle-s.angle),cos(old.angle-s.angle)))<.0003&&absd(old.power-s.power)<.002)return;
 }
 int p=0;while(p<n&&video74.q[owner][kind][p].score>=s.score)p++;
 if(p>=VIDEO74_SEEDS)return;
 int last=n<VIDEO74_SEEDS?n:VIDEO74_SEEDS-1;
 for(int j=last;j>p;j--)video74.q[owner][kind][j]=video74.q[owner][kind][j-1];
 video74.q[owner][kind][p]=s;video74.count[owner][kind]=n<VIDEO74_SEEDS?n+1:n;
}
static int video74_double_bounce(Point ball,Point target,int first,int second,double radius,Point*h1,Point*h2){
 Point a,t,n;double lo,hi;
 if(first==second||!planner_outline_facet(second,radius,&a,&t,&n,&lo,&hi))return 0;
 double d=pdot(psub(target,a),n);
 Point virtual_target={target.x-2*d*n.x,target.y-2*d*n.y};
 if(!planner_outline_bounce(ball,virtual_target,first,radius,h1)||
    !planner_outline_bounce(*h1,target,second,radius,h2))return 0;
 return pdist(ball,*h1)>radius*.1&&pdist(*h1,*h2)>radius*.25&&pdist(*h2,target)>radius*.1;
}
static void video74_build(const PlanBody*b,int n,double goalx,double height,
                          double xmin,double xmax,double ymin,double ymax,int opening,int mode,U allowed){
 memset(&video74,0,sizeof(video74));
 if(mode||!b||n<2||n>MAX_BALLS||!finite(goalx)||!finite(height)||height<=2*b[0].r||xmax<=xmin||ymax<=ymin)return;
 double width=xmax-xmin,sign=goalx>0?1:-1;
 int facets=planner_outline_active;if(facets>32)facets=32;
 for(int i=1;i<n;i++)if(b[i].owner>=0&&b[i].owner<MAX_BALLS&&(!opening||(allowed&((U)1<<i)))){
  int owner=b[i].owner;video74.owners=pmax(video74.owners,owner+1);
  /* Glancing contact ring, not just the centre-to-centre full-power ray.
   * Alternate powers remain coupled to the contact direction. */
  double bearing=atan2(b[0].p.y-b[i].p.y,b[0].p.x-b[i].p.x);
  static const double offsets[8]={-.055,.055,-.14,.14,-.28,.28,-.48,.48};
  static const double powers[3]={1,.82,.64};
  for(int j=0;j<8;j++){
   double a=bearing+offsets[j];Point normal={cos(a),sin(a)};
   Point touch={b[0].p.x-(b[0].r+b[i].r)*normal.x,b[0].p.y-(b[0].r+b[i].r)*normal.y};
   double angle=atan2(touch.y-b[i].p.y,touch.x-b[i].p.x);
   for(int k=0;k<3;k++)video74_add(owner,1,(Shot){i,angle,powers[k],12-j*.01-k*.002});
  }
  for(int lane=0;lane<5;lane++){
   Point target={goalx,(lane-2)*(height*.5-b[0].r*1.15)*.45};
   if(opening){
    target.x=goalx+sign*b[0].r*.90;
    target.y=(lane-2)*(height*.5-b[0].r*1.18)*.5;
   }
   for(int first=0;first<facets;first++)for(int second=0;second<facets;second++){
    Point h1,h2;
    if(!video74_double_bounce(b[0].p,target,first,second,b[0].r,&h1,&h2))continue;
    Point normal=punit(psub(h1,b[0].p));
    Point touch={b[0].p.x-(b[0].r+b[i].r)*normal.x,b[0].p.y-(b[0].r+b[i].r)*normal.y};
    if(touch.x<xmin+b[i].r||touch.x>xmax-b[i].r||touch.y<ymin+b[i].r||touch.y>ymax-b[i].r)continue;
    Point approach=punit(psub(touch,b[i].p));double align=pdot(approach,normal);
    if(align<=.02)continue;
    double length=pdist(b[0].p,h1)+pdist(h1,h2)+pdist(h2,target);
    double score=16+align-.15*length/width-.01*lane;
    double angle=atan2(approach.y,approach.x);
    /* Obstacles order the route, but never veto a contact chain. */
    score-=.15*plan_obstacles(b,n,b[i].p,touch,b[i].r,i,0,-1);
    for(int k=0;k<3;k++)video74_add(owner,0,(Shot){i,angle,powers[k],score-k*.003});
   }
  }
 }
}
static int video74_next(Shot*out,int*owner){
 if(!out||!owner||!video74.owners)return 0;
 int total=video74.owners*2;
 for(int j=0;j<total;j++){
  int slot=(video74.cursor++)%total,o=slot%video74.owners,kind=slot/video74.owners;
  if(video74.used[o][kind]>=video74.count[o][kind])continue;
  *out=video74.q[o][kind][video74.used[o][kind]++];*owner=o;return 1;
 }
 return 0;
}
