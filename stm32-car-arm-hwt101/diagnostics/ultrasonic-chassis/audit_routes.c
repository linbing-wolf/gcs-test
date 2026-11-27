/* 只读主机审计：使用当前测试版选路器，对所有合法封路/已走组合核对。 */
#define main audit_legacy_entry_main
#include "../../tests/verify_entry.c"
#undef main
#include <stdlib.h>

typedef struct {uint16_t mask,distance;uint8_t count,nodes[9];} AuditPath;
static AuditPath paths[81][128];
static unsigned path_count[81],bits[4096];
static uint8_t stack_nodes[9];
static struct {unsigned reachable,worse,max_extra,blocked,passed;double max_ratio;} stats[81];
static unsigned checks;
#define REQUIRE(x) do {if(!(x)){fprintf(stderr,"audit failure line %u: %s\n",__LINE__,#x);exit(1);}++checks;}while(0)

static unsigned map_distance(uint8_t from,uint8_t to)
{
    RoadPoint a,b,row;
    REQUIRE(road_point(from,&a) && road_point(to,&b));
    if(from==NODE_A || from==NODE_C){a.x-=60;REQUIRE(road_point(from==NODE_A?NODE_F:NODE_D,&row));a.y=row.y;}
    if(to==NODE_A || to==NODE_C){b.x-=60;REQUIRE(road_point(to==NODE_A?NODE_F:NODE_D,&row));b.y=row.y;}
    return (unsigned)(abs(a.x-b.x)+abs(a.y-b.y));
}

static void enumerate(uint8_t origin,uint8_t node,uint8_t target,uint16_t seen,uint16_t mask,unsigned distance,unsigned depth)
{
    unsigned i,pair=(unsigned)origin*9u+target;
    stack_nodes[depth]=node;
    if(node==target){
        AuditPath *p;
        REQUIRE(path_count[pair]<128u);p=&paths[pair][path_count[pair]++];
        p->mask=mask;p->distance=(uint16_t)distance;p->count=(uint8_t)(depth+1u);
        memcpy(p->nodes,stack_nodes,p->count);return;
    }
    for(i=0;i<ROAD_DIRECTION_COUNT;i++){
        const RoadPlan *road=&road_motion_table[i];
        if(road->from!=node || (seen&(1u<<road->to)))continue;
        enumerate(origin,road->to,target,(uint16_t)(seen|(1u<<road->to)),
                  (uint16_t)(mask|(1u<<road->road)),distance+map_distance(node,road->to),depth+1u);
    }
}

static unsigned route_mask(const RouteTablePath *path)
{
    unsigned i,mask=0;
    for(i=1;i<path->count;i++){int road=road_index(path->nodes[i-1u],path->nodes[i]);REQUIRE(road>=0);mask|=1u<<road;}
    return mask;
}

static void print_nodes(const uint8_t *nodes,unsigned count)
{
    unsigned i;putchar('"');for(i=0;i<count;i++){if(i)printf("->");putchar('A'+nodes[i]);}putchar('"');
}

static unsigned turn_quarters(const uint8_t *nodes,unsigned count,RoadHeading heading,int target,uint16_t passed,int initial,int final)
{
    unsigned i,j,total=0;
    for(i=1;i<count;i++){
        RoadPlan road;REQUIRE(road_plan(nodes[i-1u],nodes[i],&road));
        if(obstacle_start_node(road.from) && !(initial && i==1u))obstacle_adjust_inset(&road,road.from,OBSTACLE_ENTRY_BOTH,-1);
        if(obstacle_start_node(road.to) && !(final && i+1u==count))obstacle_adjust_inset(&road,road.to,OBSTACLE_ENTRY_BOTH,1);
        for(j=0;j<road.count;j++){
            RunLeg leg;unsigned delta;
            REQUIRE(run_resolve_leg(&road,(uint8_t)j,(uint8_t)(i-1u),heading,passed,&leg));
            delta=((unsigned)leg.heading+4u-(unsigned)heading)%4u;
            total+=delta>2u?4u-delta:delta;heading=leg.heading;
        }
    }
    if(target>=0){unsigned delta=((unsigned)target+4u-(unsigned)heading)%4u;total+=delta>2u?4u-delta:delta;}
    return total;
}

static void example(uint8_t start,unsigned visit,uint16_t passed)
{
    RouteTablePath current;unsigned i,pair=NODE_D*9u+NODE_H,best=65535u;
    uint16_t blocked=(uint16_t)(1u<<road_index(NODE_D,NODE_E));
    passed&=(uint16_t)~blocked;
    REQUIRE(obstacle_route_lookup(NODE_D,NODE_H,blocked,passed,&current));
    for(i=0;i<path_count[pair];i++)if(!(paths[pair][i].mask&blocked)
        && (best==65535u || paths[pair][i].distance<paths[pair][best].distance))best=i;
    printf("{\"start\":\"%c\",\"roughVisit\":%u,\"blocked\":%u,\"passed\":%u,\"current\":",'A'+start,visit,blocked,passed);
    print_nodes(current.nodes,current.count);
    printf(",\"currentMm\":%u,\"currentUnknown\":%u,\"currentTurnDeg\":%u,\"shortest\":",current.distance,bits[route_mask(&current)&~passed&4095u],90u*turn_quarters(current.nodes,current.count,HEAD_X_POS,HEAD_Y_POS,passed,0,0));
    print_nodes(paths[pair][best].nodes,paths[pair][best].count);
    printf(",\"shortestMm\":%u,\"shortestUnknown\":%u,\"shortestTurnDeg\":%u}",paths[pair][best].distance,bits[paths[pair][best].mask&~passed&4095u],90u*turn_quarters(paths[pair][best].nodes,paths[pair][best].count,HEAD_X_POS,HEAD_Y_POS,passed,0,0));
}

static uint16_t future_turn[8][4096],future_new[8][4096];
static uint8_t future_choice[8][4096],future_start;
static uint16_t future_blocked;
static int task_heading(uint8_t node)
{
    return node==NODE_B || node==NODE_F ? HEAD_X_NEG : node==NODE_D ? HEAD_X_POS : node==NODE_H ? HEAD_Y_POS : -1;
}

/* 审计参考：限定每阶段最短距离，在完整作业顺序中联合选择等距路线。 */
static unsigned future_turn_cost(unsigned stage,uint16_t passed)
{
    uint8_t from,target;
    RoadHeading heading;
    unsigned pair,k,minimum=65535u,best_turn=65535u,best_new=65535u;
    if(stage==8u)return 0;
    if(future_turn[stage][passed]!=65534u)return future_turn[stage][passed];
    from=stage ? full_route_jobs[future_start==NODE_C][stage-1u] : future_start;
    target=full_route_jobs[future_start==NODE_C][stage];pair=(unsigned)from*9u+target;
    heading=(RoadHeading)(stage ? task_heading(from) : future_start==NODE_A ? HEAD_X_NEG : HEAD_Y_NEG);
    for(k=0;k<path_count[pair];k++)if(!(paths[pair][k].mask&future_blocked) && paths[pair][k].distance<minimum)minimum=paths[pair][k].distance;
    for(k=0;k<path_count[pair];k++){
        AuditPath *p=&paths[pair][k];unsigned later,t,n;uint16_t next;
        if((p->mask&future_blocked) || p->distance!=minimum)continue;
        next=(uint16_t)(passed|p->mask);later=future_turn_cost(stage+1u,next);
        if(later==65535u)continue;
        t=turn_quarters(p->nodes,p->count,heading,task_heading(target),passed,stage==0u,stage==7u)+later;
        n=bits[p->mask&~passed&4095u]+(stage==7u ? 0u : future_new[stage+1u][next]);
        if(t<best_turn || (t==best_turn && n<best_new)){
            best_turn=t;best_new=n;future_choice[stage][passed]=(uint8_t)k;
        }
    }
    future_turn[stage][passed]=(uint16_t)best_turn;future_new[stage][passed]=(uint16_t)best_new;
    return best_turn;
}

/* 关闭道路事先已知的整圈反事实比较；不模拟途中发现障碍后的退回成本。 */
static void print_round(uint8_t start,uint16_t blocked,int shorter)
{
    uint8_t from=start,heading=start==NODE_A ? HEAD_X_NEG : HEAD_Y_NEG;
    uint16_t passed=0;
    unsigned stage,total=0,turns=0,completed=0;
    if(shorter==2){
        unsigned a,b;future_start=start;future_blocked=blocked;
        for(a=0;a<8u;a++)for(b=0;b<4096u;b++)future_turn[a][b]=65534u;
        (void)future_turn_cost(0,0);
    }
    printf("{\"start\":\"%c\",\"blocked\":%u,\"policy\":\"%s\",\"stages\":[",'A'+start,blocked,shorter==2?"distance_future_turn":shorter?"distance_then_turn":"current");
    for(stage=0;stage<FULL_ROUTE_JOBS;stage++){
        uint8_t target=full_route_jobs[start==NODE_C][stage];
        RouteTablePath path;
        int arrival=task_heading(target);
        unsigned quarter,mm,unknown;
        if(!obstacle_route_lookup(from,target,blocked,passed,&path))break;
        if(shorter==2){
            unsigned pair=(unsigned)from*9u+target,k=future_choice[stage][passed];
            REQUIRE(future_turn[stage][passed]!=65535u);
            path.count=paths[pair][k].count;path.distance=paths[pair][k].distance;
            memcpy(path.nodes,paths[pair][k].nodes,path.count);
        } else if(shorter){
            unsigned k,pair=(unsigned)from*9u+target,best=65535u,best_turn=65535u,best_unknown=65535u;
            for(k=0;k<path_count[pair];k++)if(!(paths[pair][k].mask&blocked)){
                AuditPath *p=&paths[pair][k];
                unsigned t=turn_quarters(p->nodes,p->count,(RoadHeading)heading,arrival,passed,stage==0u,stage==7u);
                unsigned u=bits[p->mask&~passed&4095u];
                if(best==65535u || p->distance<paths[pair][best].distance
                    || (p->distance==paths[pair][best].distance && (t<best_turn || (t==best_turn && u<best_unknown)))){
                    best=k;best_turn=t;best_unknown=u;
                }
            }
            REQUIRE(best!=65535u);path.count=paths[pair][best].count;path.distance=paths[pair][best].distance;
            memcpy(path.nodes,paths[pair][best].nodes,path.count);
        }
        quarter=turn_quarters(path.nodes,path.count,(RoadHeading)heading,arrival,passed,stage==0u,stage==7u);
        mm=path.distance;
        if(stage==0u || stage==7u)mm+=60u+(unsigned)abs(obstacle_entry_y(start));
        unknown=bits[route_mask(&path)&~passed&4095u];
        if(stage)putchar(',');
        printf("{\"job\":%u,\"from\":\"%c\",\"to\":\"%c\",\"route\":",stage,'A'+from,'A'+target);print_nodes(path.nodes,path.count);
        printf(",\"distanceMm\":%u,\"turnDeg\":%u,\"unknownRoads\":%u,\"passedBefore\":%u}",mm,90u*quarter,unknown,passed);
        total+=mm;turns+=quarter;passed|=(uint16_t)route_mask(&path);from=target;
        if(arrival>=0)heading=(uint8_t)arrival;
        completed++;
    }
    printf("],\"completedJobs\":%u,\"distanceMm\":%u,\"turnDeg\":%u}",completed,total,90u*turns);
}

static unsigned motion_case(int via_i)
{
    static const uint8_t around_c[]={NODE_D,NODE_C,NODE_B,NODE_E,NODE_H};
    static const uint8_t around_i[]={NODE_D,NODE_I,NODE_H};
    double actual[4]={0},enc[4],yaw=0;
    unsigned i;
    reset_fixture();heading_sensor_reset(&heading_sensor);
    can_rx_fault=imu_uart_fault=0;memset(motor_feedback,0,sizeof(motor_feedback));memset(motor_flags,1,sizeof(motor_flags));
    for(i=0;i<4u;i++)enc[i]=(i&1u)?-1000000.0:1000000.0;
    heading_sensor_publish(&heading_sensor,0,clock_ms);REQUIRE(start_run(NODE_C,0));
    run.initial_heading=run.heading=run.target_heading=HEAD_X_POS;
    run.yaw_zero_cdeg=heading_wrap_cdeg(-CAR_MAP_YAW_SIGN*27000);run.job=6;
    run.blocked_roads=(uint16_t)(1u<<road_index(NODE_D,NODE_E));run.completed_roads=3330u;
    obstacle_node_count=via_i?3u:5u;memcpy(obstacle_nodes,via_i?around_i:around_c,obstacle_node_count);
    while(clock_ms<120000u && run.job<7u){
        double turn=0;
        clock_ms++;
        for(i=0;i<4u;i++){
            double delta=simulated_wheel_rpm[i]-actual[i];
            actual[i]+=delta>0.18?0.18:delta< -0.18?-0.18:delta;
            enc[i]+=actual[i]*65536.0/60000.0;simulated_encoder[i]=(int64_t)llround(enc[i]);
            reported_wheel_rpm[i]=(int16_t)lround(actual[i]);turn+=actual[i]/4.0;
            REQUIRE(abs(simulated_wheel_rpm[i])<=60);
        }
        yaw+=turn*0.115;
        if(clock_ms%10u==0)heading_sensor_publish(&heading_sensor,heading_wrap_cdeg((int32_t)lround(yaw)),clock_ms);
        output_size=0;output[0]=0;firmware_step();
    }
    REQUIRE(run.job==7u && run.heading==HEAD_Y_POS && clock_ms<120000u);
    return clock_ms;
}

int main(void)
{
    unsigned from,to,blocked,passed,available,pair,i,states=0,max_paths=0;
    unsigned long long queries=0;
    for(i=1;i<4096u;i++)bits[i]=bits[i>>1]+(i&1u);
    for(from=0;from<9u;from++)for(to=0;to<9u;to++){
        enumerate((uint8_t)from,(uint8_t)from,(uint8_t)to,(uint16_t)(1u<<from),0,0,0);
        if(path_count[from*9u+to]>max_paths)max_paths=path_count[from*9u+to];
    }
    for(blocked=0;blocked<4096u;blocked++){
        available=4095u&~blocked;passed=available;
        for(;;){
            states++;
            for(pair=0;pair<81u;pair++){
                RouteTablePath current;unsigned unknown=65535u,lex_distance=65535u,shortest=65535u;
                int found=obstacle_route_lookup((uint8_t)(pair/9u),(uint8_t)(pair%9u),(uint16_t)blocked,(uint16_t)passed,&current);
                queries++;
                for(i=0;i<path_count[pair];i++){
                    AuditPath *p=&paths[pair][i];unsigned next;
                    if(p->mask&blocked)continue;
                    next=bits[p->mask&~passed&4095u];
                    if(next<unknown || (next==unknown && p->distance<lex_distance)){unknown=next;lex_distance=p->distance;}
                    if(p->distance<shortest)shortest=p->distance;
                }
                REQUIRE(found==(shortest!=65535u));
                if(!found)continue;
                REQUIRE(current.distance==lex_distance && bits[route_mask(&current)&~passed&4095u]==unknown);
                stats[pair].reachable++;
                if(current.distance>shortest){
                    unsigned extra=current.distance-shortest;double ratio=(double)current.distance/shortest;
                    stats[pair].worse++;
                    if(extra>stats[pair].max_extra){stats[pair].max_extra=extra;stats[pair].blocked=blocked;stats[pair].passed=passed;}
                    if(ratio>stats[pair].max_ratio)stats[pair].max_ratio=ratio;
                }
            }
            if(!passed)break;
            passed=(passed-1u)&available;
        }
    }
    printf("{\"states\":%u,\"queries\":%llu,\"assertions\":%u,\"maxSimplePathsPerPair\":%u,\"pairs\":[",states,queries,checks,max_paths);
    for(pair=0;pair<81u;pair++){
        if(pair)putchar(',');
        printf("{\"from\":\"%c\",\"to\":\"%c\",\"reachableStates\":%u,\"longerThanShortestStates\":%u,\"maxExtraMm\":%u,\"maxRatio\":%.6f,\"worstBlocked\":%u,\"worstPassed\":%u}",
               'A'+pair/9u,'A'+pair%9u,stats[pair].reachable,stats[pair].worse,stats[pair].max_extra,stats[pair].max_ratio,stats[pair].blocked,stats[pair].passed);
    }
    printf("],\"roughToStorageExamples\":[");
    for(i=0;i<4u;i++){
        uint8_t start=(i&1u)?NODE_C:NODE_A;
        uint16_t known=(uint16_t)((1u<<road_index(start,NODE_B))|(1u<<road_index(NODE_B,NODE_E))|(1u<<road_index(NODE_E,NODE_F)));
        if(i>=2u)known|=(uint16_t)(1u<<road_index(NODE_E,NODE_H));
        if(i)putchar(',');
        example(start,i>=2u?2u:1u,known);
    }
    printf("],\"rounds\":[");
    i=0;
    for(blocked=0;blocked<4096u;blocked++)if(bits[blocked]<=2u){
        unsigned start,policy;
        for(start=0;start<2u;start++)for(policy=0;policy<3u;policy++){
            if(i++)putchar(',');
            print_round(start?NODE_C:NODE_A,(uint16_t)blocked,(int)policy);
        }
    }
    printf("],\"motionModelMs\":{\"viaC\":%u,\"viaI\":%u}}\n",motion_case(0),motion_case(1));return 0;
}
