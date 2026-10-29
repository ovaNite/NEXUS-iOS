/* Geometric candidate ordering only. Every chosen shot must still pass the
 * original game's physics and the existing exact-shot verification. */
/* Try center lanes and useful mid/high powers before narrow edge shots. */
static int search_lane(int index){
 static const int lanes[5]={2,1,3,0,4};return lanes[index%5];
}
static double search_power(int index,int opening){
 static const double powers[5]={1.0,1.0,1.0,1.0,1.0};
 static const double kickoff_powers[5]={0.66,0.78,0.88,0.58,1.0};
 return opening?kickoff_powers[index%5]:powers[index%5];
}

#define PLAN_FAMILIES 10
#define PLAN_SETUP 7
#define PLAN_SAFE_BANK 8
#define PLAN_GAP 9 /* free legal goal segments, projected puck-radius blockers */
#define PLAN_PER_FAMILY 512
#define PLAN_DIRECT 0
#define PLAN_BALL_BANK 1
#define PLAN_COMBO 2
#define PLAN_PUCK_BANK 3
#define PLAN_REFINE 4
#define PLAN_SWEEP 5
#define PLAN_SAVE 6
typedef struct {Point p;double r;int owner;} PlanBody;
static Shot plans[MAX_BALLS][PLAN_FAMILIES][PLAN_PER_FAMILY];
static int plan_count[MAX_BALLS][PLAN_FAMILIES],plan_used[MAX_BALLS][PLAN_FAMILIES];
static int plan_owner_count,plan_round,plan_emergency;
/* Mandatory first-pass coverage: each owned puck gets a native goal test before
 * a stronger candidate from one puck can monopolize the search budget. */
static int plan_coverage_cursor,plan_nearest_cursor,plan_opening,plan_opening_probe;
static int plan_attack_cursor;
/* Nearest available shooter to the ball is evaluated first.
 * All other own pucks are still guaranteed early coverage. */
static int plan_owner_order[MAX_BALLS];
static U plan_present_mask,plan_rear_mask;
static U planner_signature,checked_owners;
static int plan_focus_burst;
static int search_completed,candidate_owner;
#define PLAN_OUTLINE_MAX 64
#define PLAN_CORNER_EARLY 6
static Point planner_outline[PLAN_OUTLINE_MAX];
static int planner_outline_count,planner_outline_pending,planner_outline_active;
static Shot planner_corner_early[MAX_BALLS][PLAN_CORNER_EARLY];
static int planner_corner_kind[MAX_BALLS][PLAN_CORNER_EARLY];
static int planner_corner_count[MAX_BALLS],planner_corner_cursor;
static double pdist(Point a,Point b){double x=a.x-b.x,y=a.y-b.y;return sqrt(x*x+y*y);}
static double pdot(Point a,Point b){return a.x*b.x+a.y*b.y;}
static Point psub(Point a,Point b){return (Point){a.x-b.x,a.y-b.y};}
static Point punit(Point a){double n=sqrt(pdot(a,a));return n>1e-9?(Point){a.x/n,a.y/n}:(Point){0,0};}
static double pmax(double a,double b){return a>b?a:b;}
/* Consume a fresh native polygon once. A later build without a setter cannot
 * accidentally reuse geometry from an earlier table. No field is invented. */
static int planner_set_outline(const Point*p,int n){
 planner_outline_count=planner_outline_pending=0;
 if(!p||n<3||n>PLAN_OUTLINE_MAX)return 0;
 for(int k=0;k<n;k++)if(!finite(p[k].x)||!finite(p[k].y))return 0;
 for(int k=0;k<n;k++)planner_outline[k]=p[k];
 planner_outline_count=n;planner_outline_pending=1;return 1;
}
static void plan_add(int owner,int family,Shot shot){
 if(owner<0||owner>=plan_owner_count||family<0||family>=PLAN_FAMILIES||
    !finite(shot.angle)||!finite(shot.power)||!finite(shot.score)||shot.puck<=0)return;
 if(shot.power<0.08)shot.power=0.08;if(shot.power>1.0)shot.power=1.0;
 int n=plan_count[owner][family],used=plan_used[owner][family];
 for(int i=0;i<n;i++){
  Shot old=plans[owner][family][i];
  double diff=absd(old.angle-shot.angle);diff=min(diff,absd(diff-2*PI));
  if(absd(old.power-shot.power)<0.002&&diff<0.000049)return;
 }
 int pos=used;
 while(pos<n&&plans[owner][family][pos].score>=shot.score)pos++;
 if(pos>=PLAN_PER_FAMILY)return;
 int last=n<PLAN_PER_FAMILY?n:PLAN_PER_FAMILY-1;
 for(int i=last;i>pos;i--)plans[owner][family][i]=plans[owner][family][i-1];
 plans[owner][family][pos]=shot;
 if(n<PLAN_PER_FAMILY)plan_count[owner][family]++;
}
/* Obstacles affect priority, not legality: indirect collisions are evaluated
 * by the native solver instead of being removed by this approximation. */
static double plan_obstacles(const PlanBody*b,int n,Point a,Point z,double r,int x,int y,int w){
 Point d=psub(z,a);double len2=pdot(d,d),penalty=0;
 if(len2<1e-8)return 0;
 for(int i=1;i<n;i++){
  if(i==x||i==y||i==w)continue;
  Point rel=psub(b[i].p,a);double t=pdot(rel,d)/len2;
  if(t<=0.01||t>=0.99)continue;
  Point nearest={a.x+t*d.x,a.y+t*d.y};
  double clearance=pdist(nearest,b[i].p)/(r+b[i].r+0.001);
  if(clearance<1.5)penalty+=1.4*(1.5-clearance);
 }
 return penalty;
}
/* Exclude net-pocket backs from end-wall banks. Their vertical segment
 * crosses the goal mouth; real rails above/below the posts do not. */
static void planner_field_bounds(const Point*p,int n,double height,double*xmin,double*xmax){
 double left=0,right=0;int has_left=0,has_right=0;
 for(int k=0;k<n;k++){
  Point a=p[k],b=p[(k+1)%n];
  if(absd(a.x-b.x)>0.0001||a.y*b.y<=0||
     min(absd(a.y),absd(b.y))<height*0.48)continue;
  if(a.x<0&&(!has_left||a.x<left)){left=a.x;has_left=1;}
  if(a.x>0&&(!has_right||a.x>right)){right=a.x;has_right=1;}
 }
 if(has_left&&has_right&&left<right){*xmin=left;*xmax=right;}
}
/* Two touchline reflections, in order. The unfolded ray has to intersect
 * both physical rail spans. No guessed route authorizes a release. */
static int plan_double_bounce(Point a,Point z,double first,double second,
                              double xmin,double xmax,Point*hit){
 Point virtual={z.x,2*first-2*second+z.y};double dy=virtual.y-a.y;
 if(absd(dy)<1e-9)return 0;
 double t1=(first-a.y)/dy,t2=(2*first-second-a.y)/dy;
 if(t1<=0.001||t2<=t1||t2>=0.999)return 0;
 double x1=a.x+t1*(z.x-a.x),x2=a.x+t2*(z.x-a.x);
 if(x1<=xmin||x1>=xmax||x2<=xmin||x2>=xmax)return 0;
 *hit=(Point){x1,first};return 1;
}
static int plan_bounce(Point a,Point z,double wall,double xmin,double xmax,Point*out){
 Point reflected={z.x,2*wall-z.y};double dy=reflected.y-a.y;
 if(absd(dy)<1e-8)return 0;
 double t=(wall-a.y)/dy;if(t<=0.001||t>=0.999)return 0;
 Point hit={a.x+t*(reflected.x-a.x),wall};
 if(hit.x<=xmin||hit.x>=xmax)return 0;
 *out=hit;return 1;
}
/* Reflect a desired contact across the OWN end-wall; never bounce through
 * the mouth of our goal. Geometry only creates a direction; the real table
 * physics must produce a settled, verified result before any shot is used. */
static int plan_ownwall_bounce(Point from,Point contact,double wall,
                               double ymin,double ymax,double mouth_half,
                               double radius,Point *hit){
 Point mirror={2*wall-contact.x,contact.y};double dx=mirror.x-from.x;
 if(absd(dx)<1e-8)return 0;
 double t=(wall-from.x)/dx;if(t<=0.015||t>=0.985)return 0;
 Point h={wall,from.y+t*(mirror.y-from.y)};
 if(h.y<=ymin+radius||h.y>=ymax-radius ||
    absd(h.y)<=mouth_half+radius*1.4)return 0;
 *hit=h;return 1;
}
/* Project each physical puck's inflated silhouette onto the opposing goal
 * plane from the live ball center. The complement is a list of actual clear
 * goal-mouth intervals. These are only geometric priorities: native physics
 * and exact-shot verification decide whether a candidate really scores.
 * This deliberately does not change collision hooks or add fake trajectories.
 */
typedef struct {double lo,hi;} GoalInterval;
static int planner_find_gaps(const PlanBody*b,int n,double goalx,double height,
                             GoalInterval *gaps,int cap){
 if(!b||!gaps||n<1||n>MAX_BALLS||cap<1||!finite(goalx)||!finite(height))return 0;
 double ballr=b[0].r;
 double half=height*0.5-ballr*1.015;
 double goaldist=absd(goalx-b[0].p.x);
 if(!finite(half)||!finite(goaldist)||half<=0||goaldist<=ballr*0.5)return 0;
 double forward=goalx>b[0].p.x?1.0:-1.0;
 GoalInterval occluded[MAX_BALLS];int blocks=0;
 for(int j=1;j<n;j++){
  double dx=(b[j].p.x-b[0].p.x)*forward,dy=b[j].p.y-b[0].p.y;
  double radius=b[j].r+ballr*1.035;
  if(!finite(dx)||!finite(dy)||!finite(radius)||radius<=0||
     dx<=radius||dx>=goaldist+radius)continue;
  /* Tangents from the ball to the expanded obstacle circle:
   * (dx^2-r^2)*slope^2 - 2*dx*dy*slope + dy^2-r^2 = 0.
   * No per-pixel approximation or fixed angular sampling is involved. */
  double denom=dx*dx-radius*radius;
  double discriminant=dx*dx+dy*dy-radius*radius;
  if(denom<=1e-10||discriminant<=0)continue;
  double center=dx*dy, spread=radius*sqrt(discriminant);
  double low=b[0].p.y+goaldist*(center-spread)/denom;
  double high=b[0].p.y+goaldist*(center+spread)/denom;
  if(!finite(low)||!finite(high)||high<=-half||low>=half)continue;
  if(low< -half)low=-half;
  if(high>half)high=half;
  if(high<=low)continue;
  int pos=blocks;
  while(pos>0&&occluded[pos-1].lo>low){occluded[pos]=occluded[pos-1];pos--;}
  occluded[pos]=(GoalInterval){low,high};blocks++;
 }
 double cursor=-half;int total=0;
 const double minwidth=pmax(ballr*0.025,half*0.00025);
 for(int j=0;j<blocks;j++){
  if(occluded[j].lo>cursor+minwidth&&total<cap){
   gaps[total++]=(GoalInterval){cursor,occluded[j].lo};
  }
  if(occluded[j].hi>cursor)cursor=occluded[j].hi;
  if(cursor>=half)break;
 }
 if(cursor<half-minwidth&&total<cap)gaps[total++]=(GoalInterval){cursor,half};
 return total;
}
static void planner_add_gap_shots(const PlanBody*b,int n,int i,int owner,
                                  const GoalInterval*gaps,int gap_count,
                                  double goalx,double width,double height){
 if(!gaps||gap_count<=0||!finite(width)||width<=0)return;
 static const double fractions[7]={0.5,0.25,0.75,0.10,0.90,0.035,0.965};
 for(int g=0;g<gap_count;g++){
  const double span=gaps[g].hi-gaps[g].lo;
  if(span<=0)continue;
  for(int k=0;k<7;k++){
   Point target={goalx,gaps[g].lo+span*fractions[k]};
   Point travel=punit(psub(target,b[0].p));
   if(pdot(travel,travel)<0.5)continue;
   Point touch={b[0].p.x-(b[0].r+b[i].r)*travel.x,
                b[0].p.y-(b[0].r+b[i].r)*travel.y};
   Point approach=punit(psub(touch,b[i].p));
   double alignment=pdot(approach,travel);
   if(alignment<=0.045)continue;
   double obstacles=plan_obstacles(b,n,b[i].p,touch,b[i].r,i,0,-1);
   double dist=pdist(b[i].p,touch)+pdist(b[0].p,target);
   double quality=8.0+1.5*alignment-1.3*obstacles-0.24*dist/(width+0.001)+
                  0.4*span/(height+0.001)-0.012*k;
   /* A clear corridor can miss at maximum power after secondary contacts.
    * Interleave real supported powers; every one still needs native proof. */
   static const double gap_powers[4]={1.0,0.82,0.64,0.46};
   for(int q=0;q<4;q++)plan_add(owner,PLAN_GAP,
      (Shot){i,atan2(approach.y,approach.x),gap_powers[q],quality-0.035*q});
  }
 }
}
static void plan_powers(int owner,int family,int puck,double angle,double quality,double distance,double width,int opening){
 double wanted=0.54+0.44*distance/(width+0.001);
 wanted=pmax(0.62,min(0.96,wanted));
 if(opening)wanted=pmax(0.62,min(0.90,wanted));
 /* Real controls accept <= 100 percent: generate stronger direct candidates
  * without pretending that unsupported 120-percent physics can be executed. */
 /* Ordinary Auto Goal always scans with the strongest game-supported
  * control power; never simulate a fictional >100 percent shot. On kickoff,
  * varied speeds remain necessary to avoid a goal-line foul. */
 if(!opening){
  plan_add(owner,family,(Shot){puck,angle,1.0,quality+0.02});
  if(family==PLAN_DIRECT){
   plan_add(owner,PLAN_REFINE,(Shot){puck,angle,0.82,quality-0.10});
   plan_add(owner,PLAN_REFINE,(Shot){puck,angle,0.62,quality-0.22});
   plan_add(owner,PLAN_REFINE,(Shot){puck,angle,0.42,quality-0.34});
  }
  if(family==PLAN_COMBO||family==PLAN_BALL_BANK||family==PLAN_PUCK_BANK){
   plan_add(owner,family,(Shot){puck,angle,0.82,quality-0.12});
   plan_add(owner,family,(Shot){puck,angle,0.64,quality-0.24});
  }
 }else{
  double powers[3]={wanted,0.83,0.98};
  for(int k=0;k<3;k++)plan_add(owner,family,(Shot){puck,angle,powers[k],quality-k*0.025});
 }
}
/* New routes are finite geometric proposals only. In particular, a vertical
 * goal opening is not a rail. Native physics and final control replay remain
 * the only authority for a release. Keep these small so ordinary families
 * retain their solver time. */
static int planner_ball_endwall(Point ball,Point goal,double wall,double ymin,
                                double ymax,double mouth,double radius,Point*hit){
 if(!hit||radius<=0||ymax<=ymin||!finite(ball.x)||!finite(ball.y)||
    !finite(goal.x)||!finite(goal.y)||!finite(wall))return 0;
 Point mirror={2*wall-goal.x,goal.y};double dx=mirror.x-ball.x;
 if(absd(dx)<1e-9)return 0;
 double t=(wall-ball.x)/dx;if(t<=.00001||t>=.99999)return 0;
 Point h={wall,ball.y+t*(mirror.y-ball.y)};
 /* Leave the rounded end corners to the real solver's other families. */
 if(h.y<=ymin+2*radius||h.y>=ymax-2*radius||absd(h.y)<=mouth+radius*1.4)return 0;
 *hit=h;return 1;
}
static void planner_add_endwall_goals(const PlanBody*b,int n,int i,int owner,
 double goalx,double height,double xmin,double xmax,double ymin,double ymax){
 double wall=goalx>0?xmin+b[0].r:xmax-b[0].r;
 double width=xmax-xmin,half=height*.5-b[0].r*1.05;
 for(int lane=0;lane<17;lane++){
  int step=lane==0?0:((lane&1)?-(lane+1)/2:lane/2);
  Point goal={goalx,half*.94*step/8.0},hit;
  if(!planner_ball_endwall(b[0].p,goal,wall,ymin,ymax,height*.5,b[0].r,&hit))continue;
  Point normal=punit(psub(hit,b[0].p));
  Point contact={b[0].p.x-(b[0].r+b[i].r)*normal.x,
                 b[0].p.y-(b[0].r+b[i].r)*normal.y};
  if(contact.x<xmin+b[i].r||contact.x>xmax-b[i].r||
     contact.y<ymin+b[i].r||contact.y>ymax-b[i].r)continue;
  Point approach=punit(psub(contact,b[i].p));double align=pdot(approach,normal);
  if(align<=.06)continue;
  double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1)+
    plan_obstacles(b,n,b[0].p,hit,b[0].r,0,i,-1)+
    plan_obstacles(b,n,hit,goal,b[0].r,0,i,-1);
  double quality=6.4+align-penalty-.15*(pdist(b[i].p,contact)+pdist(b[0].p,hit)+pdist(hit,goal))/width;
  plan_powers(owner,PLAN_BALL_BANK,i,atan2(approach.y,approach.x),quality,width,width,0);
 }
}
static void planner_add_rail_contacts(const PlanBody*b,int n,int i,int owner,
 double goalx,double height,double xmin,double xmax,double ymin,double ymax){
 Shot best[12];int count=0;double width=xmax-xmin,half=height*.5-b[0].r*1.05;
 for(int side=0;side<2;side++)for(int j=1;j<n;j++)if(j!=i){
  double wall=side?ymin+b[0].r:ymax-b[0].r;
  Point mirror={b[0].p.x,2*wall-b[0].p.y};
  for(int k=0;k<128;k++){
   double a=2*PI*k/128.0;Point normal={cos(a),sin(a)};
   Point contact={b[j].p.x+(b[j].r+b[0].r)*normal.x,
                  b[j].p.y+(b[j].r+b[0].r)*normal.y};
   if(contact.x<xmin+b[0].r||contact.x>xmax-b[0].r||
      contact.y<ymin+b[0].r||contact.y>ymax-b[0].r)continue;
   double dy=contact.y-mirror.y;if(absd(dy)<1e-9)continue;
   double t=(wall-mirror.y)/dy;if(t<=.001||t>=.999)continue;
   Point hit={mirror.x+t*(contact.x-mirror.x),wall};
   /* Exclude rounded corners, where a horizontal mirror is inaccurate. */
   if(hit.x<=xmin+b[0].r*2||hit.x>=xmax-b[0].r*2)continue;
   Point incoming=punit(psub(contact,mirror));double impact=pdot(incoming,normal);
   if(impact>=-.03)continue;
   Point rebound=punit((Point){incoming.x-1.6*impact*normal.x,incoming.y-1.6*impact*normal.y});
   double alignment=-1;
   for(int lane=-1;lane<=1;lane++){
    Point goal={goalx,lane*half*.75};
    double v=pdot(rebound,punit(psub(goal,contact)));if(v>alignment)alignment=v;
   }
   if(alignment<.985)continue;
   Point first=punit(psub(hit,b[0].p));
   Point touch={b[0].p.x-(b[0].r+b[i].r)*first.x,b[0].p.y-(b[0].r+b[i].r)*first.y};
   if(touch.x<xmin+b[i].r||touch.x>xmax-b[i].r||
      touch.y<ymin+b[i].r||touch.y>ymax-b[i].r)continue;
   Point approach=punit(psub(touch,b[i].p));double align=pdot(approach,first);
   if(align<=.06)continue;
   double penalty=plan_obstacles(b,n,b[i].p,touch,b[i].r,i,0,j)+
    plan_obstacles(b,n,b[0].p,hit,b[0].r,0,i,j)+
    plan_obstacles(b,n,hit,contact,b[0].r,0,i,j);
   Shot shot={i,atan2(approach.y,approach.x),1,
     6.1+alignment+align-penalty-.12*(pdist(b[i].p,touch)+pdist(b[0].p,hit)+pdist(hit,contact))/width};
   int duplicate=0;for(int q=0;q<count;q++){
    double delta=absd(atan2(sin(best[q].angle-shot.angle),cos(best[q].angle-shot.angle)));
    if(delta<.0015){
     duplicate=1;if(shot.score>best[q].score){
      int pos=q;while(pos>0&&best[pos-1].score<shot.score){best[pos]=best[pos-1];pos--;}
      best[pos]=shot;
     }break;
    }
   }
   if(duplicate)continue;
   int pos=count;while(pos>0&&best[pos-1].score<shot.score)pos--;
   if(pos>=12)continue;
   int last=count<12?count:11;for(int q=last;q>pos;q--)best[q]=best[q-1];best[pos]=shot;if(count<12)count++;
  }
 }
 for(int k=0;k<count;k++){
  plan_add(owner,PLAN_COMBO,best[k]);
  Shot softer=best[k];softer.power=.82;softer.score-=.08;plan_add(owner,PLAN_COMBO,softer);
 }
}
/* A corner ball may have no accessible direct contact: a neighboring puck
 * screens its whole legal contact arc. Propose a glancing collision where
 * the SELECTED shooter continues toward the ball after moving the blocker.
 * The coefficients are only geometric search seeds, never replacement
 * masses/restitution. Native physics must prove the complete dynamic chain. */
static int planner_corner_state(Point ball,double radius,double xmin,double xmax,double ymin,double ymax){
 return radius>0&&min(ball.x-xmin,xmax-ball.x)<radius*4&&
   min(ball.y-ymin,ymax-ball.y)<radius*4;
}
static void planner_corner_offer(int owner,Shot shot,int kind){
 if(owner<0||owner>=plan_owner_count)return;
 int count=planner_corner_count[owner];
 for(int k=0;k<count;k++){
  Shot old=planner_corner_early[owner][k];
  if(old.puck==shot.puck&&absd(atan2(sin(old.angle-shot.angle),cos(old.angle-shot.angle)))<.002){
   if(shot.score<=old.score)return;
   for(int j=k;j+1<count;j++){planner_corner_early[owner][j]=planner_corner_early[owner][j+1];planner_corner_kind[owner][j]=planner_corner_kind[owner][j+1];}
   count--;break;
  }
 }
 planner_corner_count[owner]=count;
 int same=0,worst=-1;double worst_score=1e9;
 for(int k=0;k<count;k++)if(planner_corner_kind[owner][k]==kind){same++;if(planner_corner_early[owner][k].score<worst_score){worst=k;worst_score=planner_corner_early[owner][k].score;}}
 /* Two seeds per route type prevent one narrow family taking all six. */
 if(same>=2){
  if(shot.score<=worst_score)return;
  for(int j=worst;j+1<count;j++){planner_corner_early[owner][j]=planner_corner_early[owner][j+1];planner_corner_kind[owner][j]=planner_corner_kind[owner][j+1];}count--;
 }
 int pos=0;while(pos<count&&planner_corner_early[owner][pos].score>=shot.score)pos++;
 if(pos>=PLAN_CORNER_EARLY)return;
 int last=count<PLAN_CORNER_EARLY?count:PLAN_CORNER_EARLY-1;
 for(int k=last;k>pos;k--){planner_corner_early[owner][k]=planner_corner_early[owner][k-1];planner_corner_kind[owner][k]=planner_corner_kind[owner][k-1];}
 planner_corner_early[owner][pos]=shot;planner_corner_kind[owner][pos]=kind;
 planner_corner_count[owner]=count<PLAN_CORNER_EARLY?count+1:count;
}
static int planner_glance_ball(Point start,Point blocker,double pair_radius,
 Point ball,double ball_contact_radius,Point normal,double coefficient,
 Point*first_hit,Point*ball_hit,double*angle,double*speed){
 Point hit={blocker.x+pair_radius*normal.x,blocker.y+pair_radius*normal.y};
 Point incoming=punit(psub(hit,start));double impact=pdot(incoming,normal);
 if(impact>=-.03)return 0;
 Point after={incoming.x-coefficient*impact*normal.x,incoming.y-coefficient*impact*normal.y};
 double magnitude=sqrt(pdot(after,after));if(magnitude<.06)return 0;
 Point direction={after.x/magnitude,after.y/magnitude},to_ball=psub(ball,hit);
 double along=pdot(to_ball,direction),cross=to_ball.x*direction.y-to_ball.y*direction.x;
 double inside=ball_contact_radius*ball_contact_radius-cross*cross;
 if(along<=0||inside<=0)return 0;
 double travel=along-sqrt(inside);if(travel<=.01)return 0;
 *first_hit=hit;*ball_hit=(Point){hit.x+travel*direction.x,hit.y+travel*direction.y};
 *angle=atan2(incoming.y,incoming.x);*speed=magnitude;return 1;
}
static void planner_add_corner_glances(const PlanBody*b,int n,int i,int owner,
 double goalx,double xmin,double xmax,double ymin,double ymax){
 if(!planner_corner_state(b[0].p,b[0].r,xmin,xmax,ymin,ymax))return;
 Shot best[8];int count=0;static const double coefficient[3]={.72,.86,1.0};
 for(int j=1;j<n;j++)if(j!=i&&pdist(b[j].p,b[0].p)<b[j].r+b[0].r*3.3){
  for(int k=0;k<128;k++)for(int c=0;c<3;c++){
   double a=2*PI*k/128.0;Point normal={cos(a)*(goalx<0?-1:1),sin(a)};
   Point first,touch;double angle,speed;
   if(!planner_glance_ball(b[i].p,b[j].p,b[i].r+b[j].r,b[0].p,b[i].r+b[0].r,
      normal,coefficient[c],&first,&touch,&angle,&speed))continue;
   if(first.x<xmin+b[i].r||first.x>xmax-b[i].r||first.y<ymin+b[i].r||first.y>ymax-b[i].r||
      touch.x<xmin+b[i].r||touch.x>xmax-b[i].r||touch.y<ymin+b[i].r||touch.y>ymax-b[i].r)continue;
   double penalty=plan_obstacles(b,n,b[i].p,first,b[i].r,i,j,0)+
     plan_obstacles(b,n,first,touch,b[i].r,i,j,0);
   /* Do not reject overlap with the MOVED blocker on the second leg. */
   Shot shot={i,angle,1,8.6+.45*speed-penalty-.12*pdist(b[i].p,first)/(xmax-xmin)};
   int duplicate=0;
   for(int q=0;q<count;q++)if(absd(atan2(sin(best[q].angle-angle),cos(best[q].angle-angle)))<.002){
    duplicate=1;if(shot.score>best[q].score){
     int pos=q;while(pos>0&&best[pos-1].score<shot.score){best[pos]=best[pos-1];pos--;}
     best[pos]=shot;
    }break;
   }
   if(duplicate)continue;
   int pos=0;while(pos<count&&best[pos].score>=shot.score)pos++;
   if(pos>=8)continue;int last=count<8?count:7;
   for(int q=last;q>pos;q--)best[q]=best[q-1];best[pos]=shot;if(count<8)count++;
  }
 }
 static const double powers[3]={.82,1.0,.64};
 static const double offsets[3]={0,-.003,.003};
 for(int k=0;k<count;k++){
  planner_corner_offer(owner,best[k],0);
  for(int a=0;a<3;a++)for(int p=0;p<3;p++)plan_add(owner,PLAN_COMBO,
   (Shot){i,best[k].angle+offsets[a],powers[p],best[k].score-.02*a-.015*p});
 }
}
static int planner_ray_contact(Point from,Point target,double radius,double angle,Point*hit){
 Point direction={cos(angle),sin(angle)},relative=psub(target,from);
 double along=pdot(relative,direction),cross=relative.x*direction.y-relative.y*direction.x;
 double inside=radius*radius-cross*cross;if(along<=0||inside<=0)return 0;
 double travel=along-sqrt(inside);if(travel<=.00001)return 0;
 *hit=(Point){from.x+travel*direction.x,from.y+travel*direction.y};return 1;
}
/* The intermediary may need to drive the ball INTO a corner first. Old
 * goal-aligned combos discarded that approach before native simulation. */
static void planner_add_corner_intermediaries(const PlanBody*b,int n,int i,int owner,
 double xmin,double xmax,double ymin,double ymax){
 if(!planner_corner_state(b[0].p,b[0].r,xmin,xmax,ymin,ymax))return;
 static const double inset[7]={.0002,.001,.0025,.005,.009,.015,.025};
 Shot best[4];int count=0;
 for(int j=1;j<n;j++)if(j!=i&&pdist(b[j].p,b[0].p)<b[j].r+b[0].r*3.3){
  Point rel=psub(b[j].p,b[i].p);double distance=pdist(b[j].p,b[i].p),pair=b[i].r+b[j].r;
  if(distance<=pair+.01)continue;
  double bearing=atan2(rel.y,rel.x),spread=atan2(pair,sqrt(distance*distance-pair*pair));
  for(int side=-1;side<=1;side+=2)for(int k=0;k<7;k++){
   double angle=bearing+side*(spread-inset[k]);Point contact;
   if(!planner_ray_contact(b[i].p,b[j].p,pair,angle,&contact))continue;
   if(contact.x<xmin+b[i].r||contact.x>xmax-b[i].r||contact.y<ymin+b[i].r||contact.y>ymax-b[i].r)continue;
   Point transfer=punit(psub(b[j].p,contact)),to_ball=psub(b[0].p,b[j].p);
   double along=pdot(to_ball,transfer),cross=absd(to_ball.x*transfer.y-to_ball.y*transfer.x),radius=b[j].r+b[0].r;
   if(along<=0||cross>=radius*.98)continue;
   double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,j,0);
   Shot shot={i,angle,1,9.0+.25*(1-cross/radius)-penalty-.10*distance/(xmax-xmin)};
   int pos=0;while(pos<count&&best[pos].score>=shot.score)pos++;
   if(pos>=4)continue;int last=count<4?count:3;
   for(int q=last;q>pos;q--)best[q]=best[q-1];best[pos]=shot;if(count<4)count++;
  }
 }
 static const double powers[3]={.82,1.0,.64};
 for(int k=0;k<count;k++){
  planner_corner_offer(owner,best[k],2);
  for(int p=0;p<3;p++)plan_add(owner,PLAN_COMBO,(Shot){i,best[k].angle,powers[p],best[k].score-.02*p});
 }
}
/* A circle hits the inward radius-offset of the actual native facet.
 * Neighbor half-planes shorten both ends; using unshortened segments would
 * invent contacts in the adjoining corner cap. Supports either winding. */
static int planner_outline_facet(int edge,double radius,Point*a,Point*tangent,
 Point*normal,double*lo,double*hi){
 int n=planner_outline_active;if(n<3||edge<0||edge>=n||radius<=0)return 0;
 double area=0;for(int k=0;k<n;k++){Point p=planner_outline[k],q=planner_outline[(k+1)%n];area+=p.x*q.y-p.y*q.x;}
 if(absd(area)<1e-8)return 0;
 Point p=planner_outline[edge],q=planner_outline[(edge+1)%n];
 Point d=psub(q,p);double length=sqrt(pdot(d,d));if(length<1e-8)return 0;
 *tangent=(Point){d.x/length,d.y/length};
 *normal=area>0?(Point){-tangent->y,tangent->x}:(Point){tangent->y,-tangent->x};
 *a=(Point){p.x+radius*normal->x,p.y+radius*normal->y};*lo=0;*hi=length;
 for(int side=0;side<2;side++){
  int v=side?(edge+1)%n:(edge+n-1)%n;Point u=planner_outline[v],z=planner_outline[(v+1)%n];
  Point e=punit(psub(z,u)),inward=area>0?(Point){-e.y,e.x}:(Point){e.y,-e.x};
  Point offset={u.x+radius*inward.x,u.y+radius*inward.y};
  double f=pdot(psub(*a,offset),inward),slope=pdot(*tangent,inward);
  if(absd(slope)<1e-10){if(f<-.00001)return 0;continue;}
  double cut=-f/slope;if(slope>0)*lo=pmax(*lo,cut);else *hi=min(*hi,cut);
 }
 return *hi>*lo+.0001;
}
static int planner_outline_bounce(Point ball,Point goal,int edge,double radius,Point*hit){
 Point a,tangent,normal;double lo,hi;
 if(!planner_outline_facet(edge,radius,&a,&tangent,&normal,&lo,&hi))return 0;
 double distance=pdot(psub(goal,a),normal);
 Point mirror={goal.x-2*distance*normal.x,goal.y-2*distance*normal.y},ray=psub(mirror,ball);
 double divisor=pdot(ray,normal);if(absd(divisor)<1e-9)return 0;
 double t=pdot(psub(a,ball),normal)/divisor;if(t<=.000001||t>=.999999)return 0;
 Point h={ball.x+t*ray.x,ball.y+t*ray.y};double along=pdot(psub(h,a),tangent);
 if(along<=lo+.00001||along>=hi-.00001)return 0;
 *hit=h;return 1;
}
static void planner_add_corner_facets(const PlanBody*b,int n,int i,int owner,double goalx,double height,
 double xmin,double xmax,double ymin,double ymax){
 if(!planner_corner_state(b[0].p,b[0].r,xmin,xmax,ymin,ymax))return;
 for(int edge=0;edge<planner_outline_active;edge++){
  Point a,tangent,normal;double lo,hi;
  if(!planner_outline_facet(edge,b[0].r,&a,&tangent,&normal,&lo,&hi))continue;
  /* Axis-aligned rails already have families; only real oblique facets. */
  if(absd(tangent.x)<.02||absd(tangent.y)<.02||pdot(psub(b[0].p,a),normal)>b[0].r*3)continue;
  for(int lane=0;lane<9;lane++){
   Point goal={goalx,(lane-4)/4.0*(height*.5-b[0].r*1.05)},hit;
   if(!planner_outline_bounce(b[0].p,goal,edge,b[0].r,&hit))continue;
   Point direction=punit(psub(hit,b[0].p));
   Point touch={b[0].p.x-(b[i].r+b[0].r)*direction.x,b[0].p.y-(b[i].r+b[0].r)*direction.y};
   if(touch.x<xmin+b[i].r||touch.x>xmax-b[i].r||touch.y<ymin+b[i].r||touch.y>ymax-b[i].r)continue;
   Point approach=punit(psub(touch,b[i].p));double align=pdot(approach,direction);if(align<=.04)continue;
   double penalty=plan_obstacles(b,n,b[i].p,touch,b[i].r,i,0,-1);
   Shot shot={i,atan2(approach.y,approach.x),1,6.8+align-penalty};
   planner_corner_offer(owner,shot,1);
   plan_powers(owner,PLAN_BALL_BANK,i,shot.angle,shot.score,xmax-xmin,xmax-xmin,0);
  }
 }
}
static int planner_build(const PlanBody*b,int n,int own_count,double goalx,double height,
                         double xmin,double xmax,double ymin,double ymax,int opening){
 planner_outline_active=planner_outline_pending?planner_outline_count:0;planner_outline_pending=0;
 if(n<2||n>MAX_BALLS||own_count<1||own_count>=MAX_BALLS||
    !finite(goalx)||!finite(height)||!finite(xmin)||!finite(xmax)||!finite(ymin)||!finite(ymax)||
    height<=2*b[0].r||xmax<=xmin||ymax<=ymin)return 0;
 for(int i=0;i<n;i++)if(!finite(b[i].p.x)||!finite(b[i].p.y)||!finite(b[i].r)||b[i].r<=0)return 0;
 memset(plan_count,0,sizeof(plan_count));memset(plan_used,0,sizeof(plan_used));
 memset(planner_corner_count,0,sizeof(planner_corner_count));planner_corner_cursor=0;
 plan_opening=opening;plan_opening_probe=0;plan_focus_burst=0;plan_owner_count=own_count;plan_round=0;plan_coverage_cursor=0;plan_nearest_cursor=0;plan_attack_cursor=0;plan_present_mask=0;plan_rear_mask=0;
 /* Sort real own-puck centers by distance to ball, not team-list order. */
 double owner_dist[MAX_BALLS];
 for(int o=0;o<own_count;o++){plan_owner_order[o]=o;owner_dist[o]=1e12;}
 for(int i=1;i<n;i++)if(b[i].owner>=0&&b[i].owner<own_count){
  int o=b[i].owner;double d=pdist(b[i].p,b[0].p)-b[i].r-b[0].r;
  if(d<owner_dist[o])owner_dist[o]=d;
 }
 for(int i=1;i<own_count;i++){
  int v=plan_owner_order[i],j=i;
  while(j>0&&owner_dist[plan_owner_order[j-1]]>owner_dist[v]){
   plan_owner_order[j]=plan_owner_order[j-1];j--;
  }
  plan_owner_order[j]=v;
 }
 double width=xmax-xmin,freeheight=height*0.5-b[0].r;
 GoalInterval free_gaps[MAX_BALLS+1];
 int gap_count=opening?0:planner_find_gaps(b,n,goalx,height,free_gaps,MAX_BALLS+1);

 /* Twice the mouth resolution: 401 goal targets / 81 opening targets.
  * The opening retains legal non-scoring setups
  * because it needs 3 legal speeds and cannot score straight from the break.
  * These are FINITE geometric candidates, not all real-valued angles. */
 int goal_lanes=opening?81:401;
 double lane_max=(double)((goal_lanes-1)/2);

 for(int i=1;i<n;i++){
  int owner=b[i].owner;if(owner<0||owner>=own_count)continue;
  plan_present_mask|=(U)1<<owner;
  /* A deep defender sits behind the ball relative to the opposing net.
   * Give that shooter fine approach angles and a larger native-solver budget.
   * This only changes candidate sampling, NEVER the original game's physics. */
  int rear=(b[i].p.x-b[0].p.x)*goalx < -b[0].r*absd(goalx)*1.5;
  if(rear)plan_rear_mask|=(U)1<<owner;
  /* Every owned puck tries every visible free gap before fixed mouth lanes.
   * A blocked direct shot still remains available via native bounce families. */
  if(!opening)planner_add_gap_shots(b,n,i,owner,free_gaps,gap_count,goalx,width,height);
  if(!opening&&!fast_arena_mode){
   planner_add_corner_glances(b,n,i,owner,goalx,xmin,xmax,ymin,ymax);
   planner_add_corner_intermediaries(b,n,i,owner,xmin,xmax,ymin,ymax);
   planner_add_corner_facets(b,n,i,owner,goalx,height,xmin,xmax,ymin,ymax);
   planner_add_endwall_goals(b,n,i,owner,goalx,height,xmin,xmax,ymin,ymax);
   planner_add_rail_contacts(b,n,i,owner,goalx,height,xmin,xmax,ymin,ymax);
  }
  if(!opening)for(int side=0;side<2;side++){
   Point target={goalx>0?min(goalx*0.94,xmax-1.5*b[0].r):pmax(goalx*0.94,xmin+1.5*b[0].r),
                 side?ymin+1.5*b[0].r:ymax-1.5*b[0].r};
   Point normal=punit(psub(target,b[0].p));
   Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,b[0].p.y-(b[i].r+b[0].r)*normal.y};
   Point approach=punit(psub(contact,b[i].p));double align=pdot(approach,normal);
   if(align>0.06){
    double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1)+plan_obstacles(b,n,b[0].p,target,b[0].r,0,i,-1);
    static const double placement_power[5]={0.50,0.66,0.34,0.82,1.0};
    for(int q=0;q<5;q++)plan_add(owner,PLAN_SETUP,(Shot){i,atan2(approach.y,approach.x),placement_power[q],11.0+align-penalty-0.025*q});
   }
  }

  /* Real bank paths must target the WHOLE legal goal mouth, not only five
   * center lanes. Trickshot mode gets 41 bank targets; ordinary Auto Goal
   * gets 41, with native physics deciding which indirect shots actually score. */
  int bank_lanes=fast_arena_mode==1?41:fast_arena_mode==2?5:41;
  for(int lane=0;lane<goal_lanes;lane++)for(int family=0;family<4;family++){
   if(family&&lane>=bank_lanes)continue;
   /* 0,-1,+1,...,-100,+100. Targets remain inside the mouth, with
    * a separate clearance for the actual ball radius. */
   int step=(lane<5)?search_lane(lane)-2:((lane&1)?-(lane+1)/2:lane/2);
   double bank_lane_max=(double)((bank_lanes-1)/2);
   double lane_fraction=(double)step/(family?bank_lane_max:lane_max);
   Point goal={goalx,lane_fraction*freeheight*0.94},bounce_ball;
   Point normal=punit(psub(goal,b[0].p));
   double goal_path=pdist(b[0].p,goal),penalty_ball=0;
   int kind=family==0?PLAN_DIRECT:family==3?PLAN_PUCK_BANK:PLAN_BALL_BANK;
   if(family==1||family==2){
    double wall=family==1?ymax-b[0].r:ymin+b[0].r;
    if(!plan_bounce(b[0].p,goal,wall,xmin+b[0].r,xmax-b[0].r,&bounce_ball))continue;
    normal=punit(psub(bounce_ball,b[0].p));
    goal_path=pdist(b[0].p,bounce_ball)+pdist(bounce_ball,goal);
    penalty_ball=plan_obstacles(b,n,b[0].p,bounce_ball,b[0].r,0,i,-1)+
                 plan_obstacles(b,n,bounce_ball,goal,b[0].r,0,i,-1);
   }else penalty_ball=plan_obstacles(b,n,b[0].p,goal,b[0].r,0,i,-1);
   Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,b[0].p.y-(b[i].r+b[0].r)*normal.y};
   int passes=family==3?2:1;
   for(int pass=0;pass<passes;pass++){
    Point from=b[i].p,first=contact;double shot_path=pdist(from,contact),penalty=penalty_ball;
    if(family==3){
     double wall=pass?ymin+b[i].r:ymax-b[i].r;
     if(!plan_bounce(from,contact,wall,xmin+b[i].r,xmax-b[i].r,&first))continue;
     shot_path=pdist(from,first)+pdist(first,contact);
     penalty+=plan_obstacles(b,n,from,first,b[i].r,i,0,-1)+plan_obstacles(b,n,first,contact,b[i].r,i,0,-1);
    }else penalty+=plan_obstacles(b,n,from,contact,b[i].r,i,0,-1);
    Point incoming=punit(psub(contact,family==3?first:from));double alignment=pdot(incoming,normal);
    if(alignment<=0.04)continue;
    Point direction=psub(first,from);
    double quality=3.0+2.1*alignment-penalty-0.28*(shot_path+goal_path)/(width+0.001)-0.12*absd(lane_fraction);
    double angle=atan2(direction.y,direction.x);
    plan_powers(owner,kind,i,angle,quality,shot_path+goal_path,width,opening);
    /* Fine-grained rotation for shooters located deep behind the ball.
     * Native re-simulation later rejects angles that do not actually score. */
    if(rear && family==0 && lane<5){
     static const double adjust[8]={-0.003,0.003,-0.008,0.008,-0.016,0.016,-0.029,0.029};
     for(int a=0;a<8;a++){
      double power=opening?(a<4?0.80:0.95):1.0;
      plan_add(owner,PLAN_REFINE,(Shot){i,angle+adjust[a],power,quality-0.05-0.009*a});
     }
    }
   }
  }
  if(!opening)for(int side=0;side<2;side++)for(int lane=0;lane<9;lane++){
   Point goal={goalx,((lane-4)/4.0)*freeheight*0.88},bounce;
   double first=side?ymin+b[0].r:ymax-b[0].r;
   double second=side?ymax-b[0].r:ymin+b[0].r;
   if(!plan_double_bounce(b[0].p,goal,first,second,xmin+b[0].r,xmax-b[0].r,&bounce))continue;
   Point normal=punit(psub(bounce,b[0].p));
   Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,b[0].p.y-(b[i].r+b[0].r)*normal.y};
   Point approach=punit(psub(contact,b[i].p));double align=pdot(approach,normal);
   if(align<=0.04)continue;
   double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1)+plan_obstacles(b,n,b[0].p,bounce,b[0].r,0,i,-1);
   plan_powers(owner,PLAN_BALL_BANK,i,atan2(contact.y-b[i].p.y,contact.x-b[i].p.x),
      3.0+align-penalty-0.02*absd(lane-4),pdist(b[i].p,contact)+pdist(b[0].p,bounce),width,0);
  }
  if(opening){
   /* A legal near-line landing can lie beside the goal mouth. Do not
    * restrict the opening to a blocked straight lane between the posts. */
   static const double offsets[5]={0,-0.35,0.35,-0.68,0.68};
   static const double opening_powers[5]={0.70,0.55,0.85,0.40,0.98};
   double line=goalx>0?xmax:xmin,direction=goalx>0?1:-1;
   for(int q=0;q<5;q++){
    Point target={line-direction*b[0].r*1.45,offsets[q]*(ymax-ymin)*0.5};
    Point normal=punit(psub(target,b[0].p));
    Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,b[0].p.y-(b[i].r+b[0].r)*normal.y};
    Point approach=punit(psub(contact,b[i].p));double align=pdot(approach,normal);
    if(align<=0.04)continue;
    double bearing=atan2(contact.y-b[i].p.y,contact.x-b[i].p.x);
    double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1)+plan_obstacles(b,n,b[0].p,target,b[0].r,0,i,-1);
    for(int k=0;k<5;k++)for(int a=-1;a<=1;a++)plan_add(owner,PLAN_SETUP,
     (Shot){i,bearing+a*0.004,opening_powers[k],4.0+align-penalty-0.04*q-0.03*k-0.01*absd(a)});
   }
  }
  /* Real-physics setup candidates are intentionally prioritized before
   * whole-circle sweeps. This solves the 'no verified goal -> no action'
   * starvation without blindly kicking the ball to the opponent. */
  if(!opening){
   static const double ahead[3]={0.48,0.65,0.81};
   static const double lanes[5]={0,-0.20,0.20,-0.42,0.42};
   static const double powers[3]={1.0,0.82,0.64};
   for(int m=0;m<3;m++)for(int k=0;k<5;k++){
    Point target={goalx*ahead[m], lanes[k]*(ymax-ymin)*0.48};
    Point normal=punit(psub(target,b[0].p));
    Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,
                   b[0].p.y-(b[i].r+b[0].r)*normal.y};
    Point approach=punit(psub(contact,b[i].p));
    double alignment=pdot(approach,normal);
    if(alignment<=0.09)continue;
    double obstacle=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1);
    double angle=atan2(contact.y-b[i].p.y,contact.x-b[i].p.x);
    for(int t=0;t<3;t++)plan_add(owner,PLAN_SETUP,(Shot){i,angle,powers[t],
          2.1+alignment-obstacle*0.4-m*0.04-k*0.02-t*0.03});
   }
  }
  /* Safe-clearance bank geometry. There are three wall families:
   * (1) ball via upper touchline, (2) ball via lower touchline,
   * (3) shooter via our own end-wall, outside our goal mouth.
   * All real interactions (walls/pucks/post/friction) still run through the
   * game's native Table.process and exact repeat-check. */
  if(!opening){
   static const double target_x[3]={0.15,0.42,0.70};
   static const double target_y[3]={0.0,-0.37,0.37};
   static const double bank_powers[4]={1.0,0.87,0.72,0.56};
   for(int a=0;a<3;a++)for(int v=0;v<3;v++){
    Point destination={goalx*target_x[a], (ymax-ymin)*0.5*target_y[v]};
    Point normal=punit(psub(destination,b[0].p));
    if(pdot(normal,normal)<0.5)continue;
    /* Top/bottom BANK BY THE BALL, not just a shooter straight at the ball. */
    for(int side=0;side<2;side++){
     double wall=side?ymin+b[0].r:ymax-b[0].r;
     Point impact;
     if(!plan_bounce(b[0].p,destination,wall,xmin+b[0].r,xmax-b[0].r,&impact))continue;
     Point impact_direction=punit(psub(impact,b[0].p));
     Point contact={b[0].p.x-(b[0].r+b[i].r)*impact_direction.x,
                    b[0].p.y-(b[0].r+b[i].r)*impact_direction.y};
     Point approach=punit(psub(contact,b[i].p));
     double align=pdot(approach,impact_direction);
     if(align<0.12)continue;
     double obstacles=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1);
     double bearing=atan2(contact.y-b[i].p.y,contact.x-b[i].p.x);
     double quality=3.20+0.7*align-obstacles-0.12*a-0.07*v;
     for(int k=0;k<4;k++)
      plan_add(owner,PLAN_SAFE_BANK,(Shot){i,bearing,bank_powers[k],quality-0.018*k});
    }
    /* Shooter BANKS OFF OUR OWN END-WALL to contact the ball. Do not use
     * the side boundary at our goal opening (no rail there). */
    double wall=goalx>0?xmin+b[i].r:xmax-b[i].r;
    Point contact={b[0].p.x-(b[0].r+b[i].r)*normal.x,
                   b[0].p.y-(b[0].r+b[i].r)*normal.y};
    Point bounced;
    if(plan_ownwall_bounce(b[i].p,contact,wall,ymin,ymax,height*0.5,
                           b[i].r,&bounced)){
     Point incoming=punit(psub(contact,bounced));
     if(pdot(incoming,normal)>=0.12){
      Point direction=psub(bounced,b[i].p);
      double bearing=atan2(direction.y,direction.x);
      double obstacles=plan_obstacles(b,n,b[i].p,bounced,b[i].r,i,0,-1)+
           plan_obstacles(b,n,bounced,contact,b[i].r,i,0,-1);
      double quality=3.0+0.8*pdot(incoming,normal)-obstacles-0.1*a;
      for(int k=0;k<4;k++)
       plan_add(owner,PLAN_SAFE_BANK,(Shot){i,bearing,bank_powers[k],quality-0.018*k});
     }
    }
   }
  }
  /* Emergency-only candidates: one own puck pushes the ball away from
   * our goal toward one of several safe field corridors. These must still
   * be completed and judged by the native solver before they may fire. */
  if(plan_emergency&&!opening){
   double save_lanes[5]={0,-0.45,0.45,-0.75,0.75};
   for(int q=0;q<5;q++){
    Point target={goalx*0.80,save_lanes[q]*(ymax-ymin)*0.5};
    Point normal=punit(psub(target,b[0].p));
    Point contact={b[0].p.x-(b[i].r+b[0].r)*normal.x,
                   b[0].p.y-(b[i].r+b[0].r)*normal.y};
    Point approach=punit(psub(contact,b[i].p));
    double alignment=pdot(approach,normal);
    if(alignment<=0.08)continue;
    double blocked=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,0,-1);
    double angle=atan2(contact.y-b[i].p.y,contact.x-b[i].p.x);
    double quality=3.0+1.4*alignment-1.4*blocked-0.08*q;
    double powers[4]={0.50,0.70,0.88,1.0};
    for(int k=0;k<4;k++)plan_add(owner,PLAN_SAVE,(Shot){i,angle,powers[k],quality-0.015*k});
   }
  }
  /* Target another player puck as an intermediary, including an opponent's
   * puck. Only the selected shooter belongs to us; actual collisions remain
   * entirely the native game's responsibility. */
  static const double combo_lanes[7]={0.0,-0.18,0.18,-0.40,0.40,-0.72,0.72};
  int combo_count=fast_arena_mode==1?7:5;
  for(int j=1;j<n;j++)if(j!=i)for(int lane=0;lane<combo_count;lane++){
   Point goal={goalx,combo_lanes[lane]*freeheight};Point normal=punit(psub(goal,b[0].p));
   Point second={b[0].p.x-(b[j].r+b[0].r)*normal.x,b[0].p.y-(b[j].r+b[0].r)*normal.y};
   Point middle=punit(psub(second,b[j].p));double align_middle=pdot(middle,normal);
   if(align_middle<=0.10)continue;
   Point contact={b[j].p.x-(b[i].r+b[j].r)*middle.x,b[j].p.y-(b[i].r+b[j].r)*middle.y};
   Point incoming=punit(psub(contact,b[i].p));double align_first=pdot(incoming,middle);
   if(align_first<=0.10)continue;
   double length=pdist(b[i].p,contact)+pdist(b[j].p,second)+pdist(b[0].p,goal);
   double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,j,0)+
                  plan_obstacles(b,n,b[j].p,second,b[j].r,i,j,0)+
                  plan_obstacles(b,n,b[0].p,goal,b[0].r,i,j,0);
   double quality=2.5+align_first+align_middle-penalty-0.3*length/(width+0.001)-lane*0.04;
   plan_powers(owner,PLAN_COMBO,i,atan2(incoming.y,incoming.x),quality,length,width,opening);
  }
  for(int j=1;j<n;j++)if(j!=i){
   for(int lane=0;lane<5;lane++){
    Point target={goalx*(lane<3?0.86:0.45),(lane==0?0:lane&1?-0.50:0.50)*(ymax-ymin)*0.5};
    Point last=punit(psub(target,b[0].p));
    Point second={b[0].p.x-(b[j].r+b[0].r)*last.x,b[0].p.y-(b[j].r+b[0].r)*last.y};
    Point middle=punit(psub(second,b[j].p));
    Point contact={b[j].p.x-(b[i].r+b[j].r)*middle.x,b[j].p.y-(b[i].r+b[j].r)*middle.y};
    Point first=punit(psub(contact,b[i].p));
    double align=pdot(middle,last),approach=pdot(first,middle);
    if(align<=0.02||approach<=0.02)continue;
    double penalty=plan_obstacles(b,n,b[i].p,contact,b[i].r,i,j,0)+plan_obstacles(b,n,b[j].p,second,b[j].r,i,j,0);
    double bearing=atan2(contact.y-b[i].p.y,contact.x-b[i].p.x);
    for(int k=0;k<4;k++)plan_add(owner,PLAN_COMBO,
     (Shot){i,bearing,1.0-k*0.19,2.6+align+approach-penalty-k*0.025});
   }
   /* Ball first, then a glancing rebound off another puck, then goal.
    * Reflection is a geometric proposal only; original masses and collision
    * physics perform the final checks, including post clearance. */
   for(int k=0;k<32;k++){
    double a=2*PI*k/32.0;Point normal={cos(a)*(goalx<0?-1:1),sin(a)};
    Point contact={b[j].p.x+(b[j].r+b[0].r)*normal.x,b[j].p.y+(b[j].r+b[0].r)*normal.y};
    Point incoming=punit(psub(contact,b[0].p));double hit=pdot(incoming,normal);
    if(hit>=-0.03)continue;
    Point rebound=punit((Point){incoming.x-1.6*hit*normal.x,incoming.y-1.6*hit*normal.y});
    Point goal={goalx,0};Point wanted=punit(psub(goal,contact));double align=pdot(rebound,wanted);
    if(align<0.92)continue;
    Point ball_contact={b[0].p.x-(b[i].r+b[0].r)*incoming.x,b[0].p.y-(b[i].r+b[0].r)*incoming.y};
    Point first=punit(psub(ball_contact,b[i].p));double approach=pdot(first,incoming);
    if(approach<=0.03)continue;
    double penalty=plan_obstacles(b,n,b[i].p,ball_contact,b[i].r,i,0,j);
    double bearing=atan2(ball_contact.y-b[i].p.y,ball_contact.x-b[i].p.x);
    for(int q=0;q<3;q++)plan_add(owner,PLAN_COMBO,(Shot){i,bearing,1-q*0.18,2.2+align+approach-penalty-q*0.03});
   }
  }
  /* Coarse full-circle exploration remains as a low-priority safety net. */
  for(int k=0;k<PLAN_PER_FAMILY;k++){
   /* 512 UNIQUE angular sectors with nine-bit reversal. Previously the
    * 256-sector sweep left every second half-step untested. */
   unsigned x=(unsigned)k,rev=0;for(int bit=0;bit<9;bit++){rev=(rev<<1)|(x&1);x>>=1;}
   double angle=2*PI*rev/512.0,power=search_power(k%5,opening);
   if(goalx<0)angle=PI-angle;
   plan_add(owner,PLAN_SWEEP,(Shot){i,angle,power,1.0-k*0.001});
  }
 }
 return 1;
}
/* An unavailable gap candidate must not let the next puck monopolize the
 * first 150 native tests. Try that SAME owner with the next best category.
 * Original table execution remains single-threaded / race-free. */
static int planner_take_owner(int who,int preferred,Shot*out){
 if(who<0||who>=plan_owner_count)return 0;
 if(plan_used[who][preferred]<plan_count[who][preferred]){
  *out=plans[who][preferred][plan_used[who][preferred]++];return 1;
 }
 static const int alternates[10]={PLAN_GAP,PLAN_DIRECT,PLAN_REFINE,
  PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_COMBO,PLAN_SAFE_BANK,
  PLAN_SAVE,PLAN_SETUP,PLAN_SWEEP};
 for(int k=0;k<10;k++){
  int f=alternates[k];
  if(f==preferred)continue;
  if(plan_used[who][f]<plan_count[who][f]){
   *out=plans[who][f][plan_used[who][f]++];return 1;
  }
 }
 return 0;
}
/* Interleave own shooters so every player gets direct checks early.
 * No indirect shots are searched before a substantial direct-goal pass.
 * Newly discovered narrow misses are refined before the remaining banks. */
static int planner_next(Shot*out,int*owner){
 if(plan_owner_count<=0)return 0;
 /* Normal play: examine one route for EVERY shooter before spending twelve
  * tests on the closest puck. Then compare a different family and a reduced
  * power/refinement for each shooter. At most three bounded owner passes;
  * all ordinary families and feedback remain available afterwards. */
 if(!plan_opening&&!fast_arena_mode){
  while(plan_attack_cursor<plan_owner_count*6){
   int slot=plan_attack_cursor++,who=plan_owner_order[slot%plan_owner_count];
   int pass=slot/plan_owner_count;
   if(!(plan_present_mask&((U)1<<who)))continue;
   static const int early85[6]={PLAN_GAP,PLAN_DIRECT,PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_COMBO,PLAN_REFINE};
   int family=early85[pass];
   if(!planner_take_owner(who,family,out))continue;
   *owner=who;return 1;
  }
  plan_coverage_cursor=plan_owner_count;
 }

 /* First a bounded complete family pass for the nearest puck: direct,
  * both bank types, contact chains and speed/angle refinements. The others
  * keep mandatory early coverage; exhausting every real angle is impossible. */
 static const int nearest_families[12]={PLAN_GAP,PLAN_DIRECT,PLAN_SETUP,PLAN_COMBO,
   PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_COMBO,PLAN_REFINE,PLAN_GAP,PLAN_DIRECT,PLAN_SAFE_BANK,PLAN_SETUP};
 static const int penalty_nearest[4]={PLAN_GAP,PLAN_DIRECT,PLAN_GAP,PLAN_REFINE};
 while(plan_nearest_cursor<(fast_arena_mode==2?4:12)){
  int f=fast_arena_mode==2?penalty_nearest[plan_nearest_cursor++]:nearest_families[plan_nearest_cursor++],who=plan_owner_order[0];
  if(plan_used[who][f]>=plan_count[who][f])continue;
  *out=plans[who][f][plan_used[who][f]++];*owner=who;return 1;
 }
 /* The original phase schedule was interleaved but did not guarantee every
  * owned puck got an actual physics check before a time cutoff. Start with
  * one direct-goal candidate per owned puck. If direct geometry is blocked,
  * still check one coarse shot for that puck before proceeding. */
 while(plan_coverage_cursor<plan_owner_count){
  int who=plan_owner_order[plan_coverage_cursor++];
  if(!(plan_present_mask & ((U)1<<who)))continue;
  int family=plan_count[who][PLAN_GAP]?PLAN_GAP:
             plan_count[who][PLAN_DIRECT]?PLAN_DIRECT:PLAN_SWEEP;
  if(plan_used[who][family]<plan_count[who][family]){
   *out=plans[who][family][plan_used[who][family]++];
   *owner=who;
   return 1;
  }
 }
 /* A bounded corner block precedes near-route feedback. Otherwise repeated
  * refinements can postpone the blocker-first route until the turn expires. */
 while(!plan_opening&&!fast_arena_mode&&planner_corner_cursor<plan_owner_count*PLAN_CORNER_EARLY){
  int slot=planner_corner_cursor++,who=plan_owner_order[slot%plan_owner_count],probe=slot/plan_owner_count;
  if(probe>=planner_corner_count[who])continue;
  *out=planner_corner_early[who][probe];*owner=who;
  static const double powers[3]={.82,1.0,.64};out->power=powers[probe%3];return 1;
 }
 /* Completed near-goal misses receive a bounded immediate fine search.
  * Four focused probes then one normal route prevents refinement starvation. */
 if(!plan_opening&&plan_focus_burst<4){
  int best=-1;double score=20.0;
  for(int k=0;k<plan_owner_count;k++){
   int used=plan_used[k][PLAN_REFINE];
   if(used<plan_count[k][PLAN_REFINE]&&plans[k][PLAN_REFINE][used].score>score){
    best=k;score=plans[k][PLAN_REFINE][used].score;
   }
  }
  if(best>=0){*out=plans[best][PLAN_REFINE][plan_used[best][PLAN_REFINE]++];
   *owner=best;plan_focus_burst++;return 1;}
 }
 plan_focus_burst=0;
 /* Opening overshoots teach lower-power probes. Spend two of three slots
  * on those refinements while continuing to sample every own shooter. */
 if(plan_opening){
  int slot=plan_opening_probe++,who=plan_owner_order[(slot/3)%plan_owner_count];
  if(slot%3!=2&&plan_used[who][PLAN_REFINE]<plan_count[who][PLAN_REFINE]){
   *out=plans[who][PLAN_REFINE][plan_used[who][PLAN_REFINE]++];*owner=who;return 1;
  }
 }
 /* In danger, check the fastest direct goals first, then genuine clearances.
  * Ordinary turns devote all early slots to direct goals and refinements. */
 /* The solver has a finite time budget. If all 41 direct goal lanes for
  * all own players were exhausted before checking any bank, the latter
  * would never get native-physics time. Prioritize direct shots, then let
  * a small selection of physically validated banks compete. */
 /* First sample the actual gaps with all five shooters; allow safe banks
  * and intermediate puck contacts within the same finite budget. */
 static const int normal_family[15]={PLAN_GAP,PLAN_DIRECT,PLAN_SAFE_BANK,PLAN_SETUP,
  PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_COMBO,PLAN_REFINE,PLAN_SETUP,
  PLAN_GAP,PLAN_DIRECT,PLAN_REFINE,PLAN_SAFE_BANK,PLAN_SWEEP,PLAN_DIRECT};
 static const int normal_rounds[15]={1,1,2,2,3,3,4,4,3,
  4,4,6,2,2,2};
 static const int danger_family[15]={PLAN_GAP,PLAN_DIRECT,PLAN_SAFE_BANK,
  PLAN_SAVE,PLAN_GAP,PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_COMBO,
  PLAN_REFINE,PLAN_SETUP,PLAN_GAP,PLAN_DIRECT,PLAN_SAFE_BANK,PLAN_SWEEP,PLAN_SAVE};
 static const int danger_rounds[15]={1,1,3,3,2,3,3,4,4,3,
  4,4,6,2,2};
 static const int trick_families[10]={PLAN_GAP,PLAN_DIRECT,PLAN_BALL_BANK,
   PLAN_PUCK_BANK,PLAN_COMBO,PLAN_GAP,PLAN_REFINE,PLAN_SAFE_BANK,
   PLAN_DIRECT,PLAN_SWEEP};
 static const int trick_rounds[10]={2,2,3,3,4,4,6,4,60,96};
 static const int penalty_families[11]={PLAN_GAP,PLAN_DIRECT,PLAN_COMBO,
   PLAN_BALL_BANK,PLAN_PUCK_BANK,PLAN_REFINE,PLAN_GAP,PLAN_DIRECT,
   PLAN_COMBO,PLAN_REFINE,PLAN_SWEEP};
 static const int penalty_rounds[11]={2,2,3,3,2,4,4,8,8,12,64};
 if(fast_arena_mode){
  const int *ff=fast_arena_mode==2?penalty_families:trick_families;
  const int *rr=fast_arena_mode==2?penalty_rounds:trick_rounds;
  int phases=fast_arena_mode==2?11:10;
  int total=0;for(int k=0;k<phases;k++)total+=rr[k]*plan_owner_count;
  while(plan_round<total){
   int slot=plan_round++,phase=0;
   while(phase<phases&&slot>=rr[phase]*plan_owner_count){slot-=rr[phase]*plan_owner_count;phase++;}
   if(phase>=phases)break;
   int who=plan_owner_order[slot%plan_owner_count],kind=ff[phase];
   if(!planner_take_owner(who,kind,out))continue;
   *owner=who;return 1;
  }
  return 0;
 }
 int remaining=0;
 for(int k=0;k<plan_owner_count;k++)for(int f=0;f<PLAN_FAMILIES;f++)
  remaining+=plan_count[k][f]-plan_used[k][f];
 if(!remaining)return 0;
 const int*families=plan_emergency?danger_family:normal_family;
 const int*rounds=plan_emergency?danger_rounds:normal_rounds;
 int phases=15;
 int total=0;for(int p=0;p<phases;p++)total+=rounds[p]*plan_owner_count;
 /* Revisit all families in short blocks. A 500-round gap block used to
  * starve useful banks and near-goal refinements until the turn expired. */
 while(plan_round<total*PLAN_PER_FAMILY){
  int slot=(plan_round++)%total,phase=0;
  while(phase<phases&&slot>=rounds[phase]*plan_owner_count){slot-=rounds[phase]*plan_owner_count;phase++;}
  if(phase>=phases)break;
  int who=plan_owner_order[slot%plan_owner_count],family=families[phase];
  if(!planner_take_owner(who,family,out))continue;
  *owner=who;return 1;
 }
 return 0;
}
static void planner_refine(int owner,Shot shot,double quality){
 /* Focus sub-degree probes around a near-goal miss. Coarse angles are
  * handled by mouth lanes and the full-circle sweep. Use one real 100%
  * control power for normal Auto Goal so geometry/verification agree. */
 const double angles[24]={-0.0001,0.0001,-0.0002,0.0002,-0.00035,0.00035,
 -0.0005,0.0005,-0.00075,0.00075,-0.001,0.001,-0.0015,0.0015,
 -0.002,0.002,-0.003,0.003,-0.004,0.004,-0.005,0.005,-0.007,0.007};
 if(!finite(shot.power))return;
 for(int k=0;k<24;k++)plan_add(owner,PLAN_REFINE,
     (Shot){shot.puck,shot.angle+angles[k],shot.power,quality-k*0.001});
 static const double powers[8]={-0.006,0.006,-0.015,0.015,-0.03,0.03,-0.06,0.06};
 for(int k=0;k<8;k++)if(shot.power+powers[k]>=0.08&&shot.power+powers[k]<=1)
  plan_add(owner,PLAN_REFINE,(Shot){shot.puck,shot.angle,shot.power+powers[k],quality-0.004-k*0.002});
}
/* Keep a small diverse beam of completed native near-goal routes. Do not
 * flood the queue again for the same unhelpful angle, nor let one shooter
 * consume all native work. Ordinary route sampling still gets every fifth slot. */
typedef struct {Shot shot;double miss;int valid;} AttackSeed;
static AttackSeed attack_seeds[MAX_BALLS][4];
static int attack_feedback_count;
static void attack_feedback_reset(void){memset(attack_seeds,0,sizeof(attack_seeds));attack_feedback_count=0;}
static int attack_feedback_offer(int owner,Shot shot,double miss,double radius){
 if(owner<0||owner>=plan_owner_count||!finite(shot.angle)||!finite(shot.power)||shot.power<0.08||shot.power>1||shot.puck<=0||!finite(miss)||!finite(radius)||radius<=0||miss<0||miss>radius*8)return 0;
 int slot=-1,worst=0;double worst_miss=-1;
 for(int k=0;k<4;k++){
  AttackSeed*seed=&attack_seeds[owner][k];
  if(!seed->valid){if(slot<0)slot=k;continue;}
  double angle=absd(atan2(sin(seed->shot.angle-shot.angle),cos(seed->shot.angle-shot.angle)));
  if(seed->shot.puck==shot.puck&&angle<0.025&&absd(seed->shot.power-shot.power)<0.12){
   if(miss>=seed->miss-radius*0.025)return 0;
   slot=k;break;
  }
  if(seed->miss>worst_miss){worst_miss=seed->miss;worst=k;}
 }
 if(slot<0){if(miss>=worst_miss)return 0;slot=worst;}
 attack_seeds[owner][slot]=(AttackSeed){shot,miss,1};attack_feedback_count++;
 double quality=26.0-miss/(radius+0.001);
 planner_refine(owner,shot,quality);
 static const double wider[8]={-0.012,0.012,-0.022,0.022,-0.04,0.04,-0.065,0.065};
 for(int k=0;k<8;k++)plan_add(owner,PLAN_REFINE,(Shot){shot.puck,shot.angle+wider[k],shot.power,quality-0.005-k*0.002});
 return 1;
}
static double opponent_threat(Point enemy,Point ball,double radii,double own_goal){
 Point d=psub(enemy,ball),goal={own_goal-ball.x,-ball.y};double gl=sqrt(pdot(goal,goal))+0.001;
 double separation=sqrt(pdot(d,d))/(radii+0.001);
 double cross=absd(d.x*goal.y-d.y*goal.x)/gl;
 double aligned=1.0/(1.0+cross/(radii+0.001));
 /* An attacker behind the ball relative to our net can hit towards it.
  * v0.26 weighted the opposite side as the greater danger. */
 double side=pdot(d,goal)<=0?1.0:0.35;
 return (0.7+1.6*aligned*side)/(0.6+separation);
}
