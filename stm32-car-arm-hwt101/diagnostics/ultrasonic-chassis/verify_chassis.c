#define main old_entry_tests
#include "../../tests/verify_entry.c"
#undef main

static double actual[4], encoder[4], field_x, field_y, yaw;
static int initial_field_heading;
static uint8_t checked_job;

static void reset_plant(void)
{
    unsigned i;
    reset_fixture();heading_sensor_reset(&heading_sensor);
    can_rx_fault=imu_uart_fault=0;
    memset(motor_feedback,0,sizeof(motor_feedback));
    memset(motor_flags,1,sizeof(motor_flags));
    memset(actual,0,sizeof(actual));
    for (i=0;i<4;i++) encoder[i]=(i&1u) ? -1000000.0 : 1000000.0;
    field_x=field_y=yaw=0.0;checked_job=0;
    heading_sensor_publish(&heading_sensor,0,clock_ms);
}

static void plant_step(void)
{
    static const uint8_t expected[FULL_ROUTE_JOBS]={NODE_B,NODE_F,NODE_D,NODE_H,NODE_F,NODE_D,NODE_H,NODE_A};
    double f,l,t,a;
    unsigned i;
    clock_ms++;
    for (i=0;i<4;i++) {
        double delta=simulated_wheel_rpm[i]-actual[i];
        actual[i]+=delta>0.18 ? 0.18 : delta< -0.18 ? -0.18 : delta;
        encoder[i]+=actual[i]*65536.0/60000.0;
        simulated_encoder[i]=(int64_t)llround(encoder[i]);
        reported_wheel_rpm[i]=(int16_t)lround(actual[i]);
    }
    f=(-actual[0]+actual[1]+actual[2]-actual[3])/4.0;
    l=(-actual[0]-actual[1]+actual[2]+actual[3])/4.0;
    t=(actual[0]+actual[1]+actual[2]+actual[3])/4.0;
    a=(initial_field_heading*9000.0-yaw)*3.141592653589793/18000.0;
    /* 当前道路已分开标定前进/后退；工位的通用Y系数不代表道路轮速换算。 */
    f*=3200.0/(60000.0*(f<0 ? CALIBRATED_PPM_BACKWARD : CALIBRATED_PPM_FORWARD));
    l*=3200.0/(60000.0*run_lateral_ppm());
    /* 地面模型包含配置所针对的机械侧偏；编码器仍记录原始轮转量。
     * 此假设仅验证控制/路线，不证明实际底盘标定已达到该精度。 */
    if(fabs(l)>fabs(f))
        f-=fabs(l)*(l>0 ? LEFT_FORWARD_MM_PER_M : RIGHT_FORWARD_MM_PER_M)/1000.0;
    else
        l-=f<0 ? -f*CAR_BACKWARD_LEFT_MM_PER_M/1000.0 : f*CAR_FORWARD_LEFT_MM_PER_M/1000.0;
    field_x+=-sin(a)*f+cos(a)*l;field_y+=cos(a)*f+sin(a)*l;
    yaw+=t*0.115;
    if (clock_ms%10u==0) heading_sensor_publish(&heading_sensor,heading_wrap_cdeg((int32_t)lround(yaw)),clock_ms);
    output_size=0;output[0]=0;firmware_step();
    if (run.job>checked_job) {
        uint8_t node=run.job==FULL_ROUTE_JOBS ? RUN_NODES[RUN_NODE_COUNT-1u] : expected[run.job-1u];
        if(run.job==FULL_ROUTE_JOBS)CHECK(node==NODE_A || node==NODE_C);
        CHECK(run.job==checked_job+1u && RUN_NODES[run.edge]==node);
        if (node==NODE_B || node==NODE_F) CHECK(run.heading==HEAD_X_NEG);
        if (node==NODE_D) CHECK(run.heading==HEAD_X_POS);
        if (node==NODE_H) CHECK(run.heading==HEAD_Y_POS);
        checked_job=run.job;
    }
}

static uint8_t test_sensor;
static uint32_t shot_ms;
static unsigned max_wheel_rpm;

static void start_at(uint8_t node)
{
    reset_plant();
    initial_field_heading=node==NODE_C ? HEAD_Y_NEG : HEAD_X_NEG;
    test_sensor=0;shot_ms=clock_ms;max_wheel_rpm=0;
    CHECK(start_run(node,0));
    CHECK(!run.station_enabled && (int)run.initial_heading==initial_field_heading);
}

static void step_with_sonar(int front,int left)
{
    unsigned i;
    if (clock_ms-shot_ms>=70u && obstacle_io_range_enabled()) {
        obstacle_io_shot(test_sensor);
        obstacle_io_result(test_sensor,test_sensor ? left : front);
        test_sensor^=1u;shot_ms=clock_ms;
    }
    plant_step();
    for (i=0;i<4;i++) {
        unsigned rpm=(unsigned)abs(simulated_wheel_rpm[i]);
        if (rpm>max_wheel_rpm) max_wheel_rpm=rpm;
    }
}

static void finish_route(void)
{
    RoadPoint origin,finish;
    uint32_t deadline=clock_ms+240000u;
    while (run_active && clock_ms<deadline) step_with_sonar(-1,-1);
    if(run.state!=RUN_DONE) fprintf(stderr,"state=%u job=%u edge=%u %u->%u segment=%u phase=%u obstacle=%u progress=%ld heading=%u goal=%u log=%s\n",
        run.state,run.job,run.edge,run.road.from,run.road.to,run.segment,run.velocity_phase,run.obstacle_phase,(long)run.progress_mm,run.heading,run.target_heading,output);
    CHECK(run.state==RUN_DONE && run.job==FULL_ROUTE_JOBS);
    CHECK(max_wheel_rpm>60u && max_wheel_rpm<=CAR_WHEEL_MAX_RPM);
    CHECK(road_point(run.plan ? NODE_C : NODE_A,&origin));
    CHECK(road_point(RUN_NODES[RUN_NODE_COUNT-1u],&finish));
    if(hypot(field_x-(finish.x-origin.x),field_y-(finish.y-origin.y))>=100.0)
        fprintf(stderr,"return endpoint plan=%u finish=%u x=%.2f y=%.2f\n",run.plan,RUN_NODES[RUN_NODE_COUNT-1u],field_x,field_y);
    CHECK(hypot(field_x-(finish.x-origin.x),field_y-(finish.y-origin.y))<100.0);
}

static void obstacle_on_launch(uint8_t node)
{
    int road=road_index(node,NODE_B);
    int sensor=node==NODE_C ? 0 : 1;
    uint32_t deadline;
    start_at(node);deadline=clock_ms+20000u;
    while (clock_ms<deadline && !(obstacle_sensor_for_leg()==sensor && run.segment==(CAR_START_INSET_MM>0)
            && run.progress_mm>=250)) step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && !run.blocked_roads);
    obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,180);
    CHECK(!run.blocked_roads);
    obstacle_io_shot((uint8_t)(sensor^1));obstacle_io_result((uint8_t)(sensor^1),50);
    CHECK(!run.blocked_roads);
    obstacle_io_shot((uint8_t)sensor);obstacle_io_result((uint8_t)sensor,-1);
    CHECK(!run.blocked_roads && obstacle_hits==0u);
    deadline=clock_ms+1000u;
    while (clock_ms<deadline && !run.blocked_roads)
        step_with_sonar(sensor ? 50 : 180,sensor ? 180 : 50);
    CHECK(run.blocked_roads==(1u<<road) && run.state==RUN_OBSTACLE_STOP);
    CHECK(!(run.completed_roads&(1u<<road)));
    CHECK(obstacle_last.valid && obstacle_last.from==node && obstacle_last.to==NODE_B);
    CHECK(obstacle_last.sensor==sensor && obstacle_last.mm==180 && obstacle_last.remaining_mm>0);
    deadline=clock_ms+35000u;
    while (clock_ms<deadline && run_active && run.obstacle_phase) step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run_active && !run.obstacle_phase && run.job==0u);
    CHECK(RUN_NODES[0]==node && RUN_NODES[1]==(node==NODE_C ? NODE_D : NODE_F));
    /* 去扫码线的 X 位移小于内侧连接量，只有实际走足的 Y 连接量可保留。 */
    CHECK(fabs(field_x+CAR_START_INSET_MM)<15.0
          && fabs(field_y-(node==NODE_A ? ROAD_A_TOP_LINK_MM : -ROAD_C_BOTTOM_LINK_MM))<15.0);
    CHECK(obstacle_inset_axes==OBSTACLE_ENTRY_BOTH);
    finish_route();
}

static void passed_route_preference_test(void)
{
    start_at(NODE_C);run.edge=1;run.job=1;run.obstacle_node=NODE_B;run.obstacle_phase=2;
    run.heading=run.target_heading=HEAD_X_NEG; /* B扫码后的实际离站朝向。 */
    run.completed_roads=(uint16_t)((1u<<road_index(NODE_B,NODE_C))
        |(1u<<road_index(NODE_C,NODE_D))|(1u<<road_index(NODE_D,NODE_E))
        |(1u<<road_index(NODE_E,NODE_F)));
    CHECK(obstacle_replan());
    /* 回归：已走道路的优势不能压过B-C-D-E-F的大幅绕远。 */
    CHECK(RUN_NODES[0]==NODE_B && RUN_NODES[1]==NODE_E);
    checked_job=run.job;queue_byte('!');plant_step();
}

/* 用独立的简单路径枚举核对优先级，避免只验证一条人工指定路线。 */
static unsigned reference_distance;
static uint32_t reference_cost;
static void reference_connection(RoadPoint *point,uint8_t node,unsigned axes)
{
    if(node==NODE_I)point->x+=CAR_MAP_STORAGE_INSET_MM;
    if(node==NODE_G){point->x+=CAR_MAP_STORAGE_INSET_MM;point->y+=CAR_MAP_RAW_EXTRA_INSET_MM;}
    if(node==NODE_A || node==NODE_C) {
        RoadPoint row;CHECK(road_point(node==NODE_A ? NODE_F : NODE_D,&row));
        if(axes&1u)point->x-=CAR_START_INSET_MM;
        if(axes&2u)point->y=row.y;
    }
}

static unsigned reference_transit_distance(uint8_t from,uint8_t to)
{
    RoadPoint a,b;
    CHECK(road_point(from,&a) && road_point(to,&b));
    reference_connection(&a,from,3u);reference_connection(&b,to,3u);
    return (unsigned)(abs(a.x-b.x)+abs(a.y-b.y));
}
/* 独立航向参考：逐个候选方向点积计算可见出路，不调用生产航向选择器。 */
static int reference_exit_direction(const RoadPlan *road)
{
    RoadPoint a,b;
    int dx,dy;
    CHECK(road_point(road->from,&a) && road_point(road->to,&b));
    reference_connection(&a,road->from,3u);reference_connection(&b,road->to,3u);
    dx=b.x-a.x;dy=b.y-a.y;
    /* B侧先接右侧通道，E侧先沿地图中线。 */
    if(dx && dy && road->from==NODE_B && road->to==NODE_E)return HEAD_Y_POS;
    if(dx && dy && road->from==NODE_E && road->to==NODE_B)return HEAD_X_POS;
    if(dx && dy && road->from==NODE_B)return dy>0 ? HEAD_Y_POS : HEAD_Y_NEG;
    if(dx && dy && road->to==NODE_B)return dx>0 ? HEAD_X_POS : HEAD_X_NEG;
    CHECK((dx!=0)!=(dy!=0));
    return dx ? (dx<0 ? 1 : 3) : (dy>0 ? 0 : 2);
}
static RoadHeading reference_branch_heading(const RoadPlan *road,unsigned segment,int dx,int dy,
                                             RoadHeading heading,uint16_t passed,uint16_t blocked)
{
    static const int fx[4]={0,-1,0,1},fy[4]={1,0,-1,0};
    static const int left_x[4]={1,0,-1,0},left_y[4]={0,1,0,-1};
    unsigned h,i,best_count=0,best_turn=5;
    RoadHeading best=heading;
    if((passed&(1u<<road->road)) || dx*fx[heading]+dy*fy[heading]>0
        || dx*left_x[heading]+dy*left_y[heading]>0)return heading;
    for(h=0;h<4;h++) {
        unsigned mask=0,count=0,turn=(h+4u-(unsigned)heading)%4u;
        int d=dx ? (dx<0 ? 1 : 3) : (dy>0 ? 0 : 2);
        if(dx*fx[h]+dy*fy[h]<0 || dx*left_x[h]+dy*left_y[h]<0)continue;
        mask=1u<<(unsigned)d;
        if(!segment)for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
            const RoadPlan *other=&road_motion_table[i];
            if(other->from!=road->from || ((passed|blocked)&(1u<<other->road)))continue;
            d=reference_exit_direction(other);
            if(d==(int)h || d==(int)((h+3u)%4u))mask|=1u<<(unsigned)d;
        }
        for(i=0;i<4;i++)count+=(mask>>i)&1u;
        if(turn>2u)turn=4u-turn;
        if(count>best_count || (count==best_count && turn<best_turn)) {
            best=(RoadHeading)h;best_count=count;best_turn=turn;
        }
    }
    return best;
}

/* 独立代价参考：用地图方向点积换算车身轴，不调用生产选路/动作解析器。 */
static uint32_t reference_path_cost(const uint8_t *nodes,unsigned count,uint16_t passed,
                                    RoadHeading heading,unsigned job,uint16_t blocked)
{
    static const int fx[4]={0,-1,0,1},fy[4]={1,0,-1,0};
    static const int lx[4]={1,0,-1,0},ly[4]={0,1,0,-1};
    uint32_t cost=0;
    unsigned k,j,actions=0;
    struct {unsigned axis,heading;int sign;uint16_t mm;unsigned entry;} groups[9u*ROAD_MAX_SEGMENTS];
    for(k=1;k<count;k++) {
        RoadPlan road;int trim_x=0,trim_y=0;
        CHECK(road_plan(nodes[k-1u],nodes[k],&road));
        if(road.from==NODE_I || road.to==NODE_I)trim_x+=CAR_MAP_STORAGE_INSET_MM;
        if(road.from==NODE_G || road.to==NODE_G) {
            trim_x+=CAR_MAP_STORAGE_INSET_MM;
            trim_y+=(road.from==NODE_F || road.to==NODE_F) ? abs(CAR_MAP_RAW_EXTRA_INSET_MM) : CAR_MAP_RAW_EXTRA_INSET_MM;
        }
        if(road.from==NODE_A || road.from==NODE_C) {
            unsigned axes=3u;
            if(k==1u && job==run.job)axes=road.from==obstacle_inset_node ? obstacle_inset_axes : 0u;
            trim_x=(axes&1u) ? CAR_START_INSET_MM : 0u;
            trim_y=(axes&2u) ? (road.from==NODE_A ? ROAD_A_TOP_LINK_MM : ROAD_C_BOTTOM_LINK_MM) : 0u;
        }
        if((road.to==NODE_A || road.to==NODE_C)
            && !(job==7u && road.to==nodes[count-1u])) {
            trim_x+=CAR_START_INSET_MM;trim_y+=road.to==NODE_A ? ROAD_A_TOP_LINK_MM : ROAD_C_BOTTOM_LINK_MM;
        }
        if(!(passed&(1u<<road.road)))cost+=OBSTACLE_UNKNOWN_COST_MS;
        {unsigned actual_segment=0;
        for(j=0;j<road.count;j++) {
            MotionParameters p=road.segments[j];
            unsigned mm=(unsigned)abs(p.mm),axis;
            int dx=p.axis==BODY_LEFT ? (p.mm>0 ? -1 : 1) : 0;
            int dy=p.axis==BODY_FORWARD ? (p.mm>0 ? 1 : -1) : 0;
            int f,l,sign;
            int *trim=p.axis==BODY_LEFT ? &trim_x : &trim_y;
            if(p.axis==BODY_LEFT && (road.from==NODE_B || road.to==NODE_B) && *trim>(int)mm) {
                mm=(unsigned)(*trim-(int)mm);dx=-dx;*trim=0;
            } else if((int)mm>=*trim){mm=(unsigned)((int)mm-*trim);*trim=0;}
            if(!mm)continue;
            f=dx*fx[heading]+dy*fy[heading];l=dx*lx[heading]+dy*ly[heading];
            if((f<0 || l<0) && !(passed&(1u<<road.road))) {
                unsigned old=(unsigned)heading;
                heading=reference_branch_heading(&road,actual_segment,dx,dy,heading,passed,blocked);
                if((unsigned)heading!=old)cost+=OBSTACLE_TURN_COST_MS;
                f=dx*fx[heading]+dy*fy[heading];l=dx*lx[heading]+dy*ly[heading];
            }
            axis=f ? 0u : 1u;sign=(f ? f : l)>0 ? 1 : -1;
            if(actions && groups[actions-1u].axis==axis && groups[actions-1u].sign==sign
                && groups[actions-1u].heading==(unsigned)heading)groups[actions-1u].mm+=(uint16_t)mm;
            else {
                CHECK(actions<9u*ROAD_MAX_SEGMENTS);
                groups[actions].axis=axis;groups[actions].sign=sign;groups[actions].heading=(unsigned)heading;
                groups[actions].mm=(uint16_t)mm;
                groups[actions].entry=!(passed&(1u<<road.road)) && road.to!=NODE_B && road.to!=NODE_F;
                actions++;
            }
            ++actual_segment;
        }}
        passed|=(uint16_t)(1u<<road.road);
    }
    /* 路径枚举、坐标变换和动作分组独立；单动作剖面另有1ms积分/控制器模型核验。 */
    for(k=0;k<actions;k++)cost+=obstacle_motion_time_ms((BodyAxis)groups[k].axis,groups[k].sign,groups[k].mm)
        +OBSTACLE_ACTION_COST_MS+(!CAR_RUN_CORNER_ENABLED ? CAR_ROUTE_NODE_STOP_MS : 0u)
        +(groups[k].entry ? OBSTACLE_ENTRY_COST_MS : 0u);
    {int arrival=nodes[count-1u]==NODE_B || nodes[count-1u]==NODE_F ? HEAD_X_NEG
        : nodes[count-1u]==NODE_D ? HEAD_X_POS : nodes[count-1u]==NODE_H ? HEAD_Y_POS : -1;
     if(arrival>=0 && (unsigned)heading!=(unsigned)arrival)cost+=OBSTACLE_TURN_COST_MS;}
    return cost;
}

static uint8_t reference_nodes[9];
static void enumerate_paths(uint8_t from,uint8_t to,uint16_t seen,uint16_t blocked,
                            uint16_t passed,unsigned distance,unsigned depth)
{
    unsigned i;
    reference_nodes[depth]=from;
    if(from==to || (to==OBSTACLE_ANY_START && (from==NODE_A || from==NODE_C))) {
        uint32_t cost=reference_path_cost(reference_nodes,depth+1u,passed,run.heading,run.job,blocked);
        if(cost<reference_cost || (cost==reference_cost && distance<reference_distance)) {
            reference_cost=cost;reference_distance=distance;
        }
        return;
    }
    for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
        const RoadPlan *road=&road_motion_table[i];
        if(road->from!=from || (blocked&(1u<<road->road)) || (seen&(1u<<road->to)))continue;
        enumerate_paths(road->to,to,(uint16_t)(seen|(1u<<road->to)),blocked,passed,
                        distance+reference_transit_distance(from,road->to),depth+1u);
    }
}

static void route_priority_tests(void)
{
    uint32_t random=20261120u;
    unsigned variant,from,to,i;
    start_at(NODE_C);
    for(variant=0;variant<64u;variant++) {
        uint16_t blocked,passed;
        random=random*1664525u+1013904223u;blocked=(uint16_t)((random>>12)&4095u);
        random=random*1664525u+1013904223u;passed=(uint16_t)((random>>12)&4095u);
        if(variant<2u){blocked=0;passed=variant ? 4095u : 0;}
        if(variant==2u){blocked=passed=4095u;}
        run.heading=(RoadHeading)(variant%4u);run.job=(uint8_t)(variant%8u);
        for(from=0;from<9u;from++)for(to=0;to<9u;to++) {
            RouteTablePath path;
            unsigned distance=0;
            uint16_t seen=0;
            int found;
            reference_cost=UINT32_MAX;reference_distance=65535u;
            enumerate_paths((uint8_t)from,(uint8_t)to,(uint16_t)(1u<<from),blocked,passed,0,0);
            found=obstacle_route_lookup((uint8_t)from,(uint8_t)to,blocked,passed,&path);
            CHECK(found==(reference_cost!=UINT32_MAX));
            if(!found)continue;
            CHECK(path.count>0 && path.count<=9u && path.nodes[0]==from && path.nodes[path.count-1u]==to);
            for(i=0;i<path.count;i++) {
                CHECK(!(seen&(1u<<path.nodes[i])));seen|=(uint16_t)(1u<<path.nodes[i]);
                if(i) {
                    RoadPlan road;CHECK(road_plan(path.nodes[i-1u],path.nodes[i],&road));
                    CHECK(!(blocked&(1u<<road.road)));
                    distance+=reference_transit_distance(road.from,road.to);
                }
            }
            CHECK(reference_path_cost(path.nodes,path.count,passed,run.heading,run.job,blocked)==reference_cost);
            CHECK(distance==reference_distance && path.distance==distance);
        }
    }
    run.job=0;checked_job=0;queue_byte('!');plant_step();
}

static void passed_road_skips_sonar_test(void)
{
    uint32_t deadline;
    unsigned i;
    start_at(NODE_C);deadline=clock_ms+20000u;
    while(clock_ms<deadline && !(obstacle_sensor_for_leg()==0 && run.segment==(CAR_START_INSET_MM>0)))step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && obstacle_io_range_enabled());
    run.completed_roads|=(uint16_t)(1u<<run.road.road);
    CHECK(obstacle_io_range_enabled() && obstacle_sensor_for_leg()==-1);
    for(i=0;i<4u;i++) {
        obstacle_io_shot((uint8_t)(i&1u));obstacle_io_result((uint8_t)(i&1u),20);
    }
    CHECK(!run.blocked_roads && !obstacle_hits);
    while(clock_ms<deadline && run.progress_mm<100)step_with_sonar(20,20);
    CHECK(clock_ms<deadline && run.progress_mm>=100 && !run.blocked_roads);
    queue_byte('!');plant_step();
}

static void incomplete_entry_returns_home_test(void)
{
    uint32_t deadline;
    start_at(NODE_C);deadline=clock_ms+20000u;
    while(clock_ms<deadline && !(obstacle_sensor_for_leg()==1 && run.segment==0u && run.progress_mm>=3))
        step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run.progress_mm<30 && obstacle_inset_node==255u);
    obstacle_io_shot(1);obstacle_io_result(1,20);
    obstacle_io_shot(1);obstacle_io_result(1,20);
    CHECK(run.state==RUN_OBSTACLE_STOP);
    deadline=clock_ms+35000u;
    while(clock_ms<deadline && run_active && run.obstacle_phase)step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run_active && !run.obstacle_phase);
    CHECK(obstacle_inset_node==255u && hypot(field_x,field_y)<15.0);
    CHECK(run.road.from==NODE_C && run.road.segments[0].mm==CAR_START_INSET_MM);
    finish_route();
}

static void start_node_geometry_tests(void)
{
    unsigned plan,mode,i,j;
    for(plan=0;plan<2u;plan++)for(mode=0;mode<6u;mode++) {
        start_at(plan ? NODE_C : NODE_A);
        for(i=0;i<ROAD_DIRECTION_COUNT;i++) {
            const RoadPlan *original=&road_motion_table[i];
            RoadPlan road;
            RoadPoint a,b;
            int dx=0,dy=0,length=0;
            uint8_t edge=mode==1u || mode==2u ? 1u : 0u;
            unsigned axes=mode==0u ? 0u : mode==4u ? 1u : mode==5u ? 2u : 3u;
            obstacle_nodes[edge]=original->from;obstacle_nodes[edge+1u]=original->to;
            obstacle_node_count=(uint8_t)(edge+2u);
            run.job=mode==2u ? FULL_ROUTE_JOBS-1u : 1u;
            obstacle_inset_node=mode>=3u ? original->from : 255u;
            obstacle_inset_axes=mode>=3u ? (uint8_t)axes : 0u;
            CHECK(run_plan_road(edge,&road));
            CHECK(road_point(road.from,&a) && road_point(road.to,&b));
            reference_connection(&a,road.from,axes);
            if(!(mode==2u && (road.to==NODE_A || road.to==NODE_C)))reference_connection(&b,road.to,3u);
            for(j=0;j<road.count;j++) {
                CHECK(road.segments[j].mm!=0);
                if(road.segments[j].axis==BODY_LEFT)dx-=road.segments[j].mm;
                else dy+=road.segments[j].mm;
                length+=abs(road.segments[j].mm);
            }
            CHECK(dx==b.x-a.x && dy==b.y-a.y && length==road.length);
        }
        checked_job=run.job;queue_byte('!');plant_step();
    }
}

static void transit_start_node_route_test(uint8_t node)
{
    uint32_t deadline;
    int crossed_connection=0;
    double connection_y=node==NODE_A ? ROAD_A_TOP_LINK_MM : -ROAD_C_BOTTOM_LINK_MM;
    start_at(node);
    /* 首次到B后必须沿已走道路回到A'/C'，再从外圈绕到F。 */
    run.blocked_roads=(uint16_t)((1u<<road_index(NODE_B,NODE_E))
        |(1u<<road_index(NODE_B,node==NODE_A ? NODE_C : NODE_A)));
    deadline=clock_ms+60000u;
    while(clock_ms<deadline && !(run.job==1u && run.road.from==node && obstacle_inset_node==node)) {
        step_with_sonar(-1,-1);
        /* 地面模型先跨越连接点，CAN 位置回读稍后才确认并切换道路。 */
        if(run.job==1u && fabs(field_x+CAR_START_INSET_MM)<2.0 && fabs(field_y-connection_y)<20.0)
            crossed_connection=1;
    }
    if(clock_ms>=deadline || !crossed_connection || fabs(field_y-connection_y)>=20.0)
        fprintf(stderr,"transit %c state=%u job=%u from=%u to=%u position=%.2f,%.2f inset=%u phase=%u\n",
            'A'+node,run.state,run.job,run.road.from,run.road.to,field_x,field_y,obstacle_inset_axes,run.obstacle_phase);
    CHECK(clock_ms<deadline && crossed_connection && fabs(field_y-connection_y)<20.0);
    CHECK(obstacle_inset_axes==OBSTACLE_ENTRY_BOTH && run.road.count==1u);
    CHECK(run.road.segments[run.road.count-1u].axis==BODY_LEFT
          && run.road.segments[run.road.count-1u].mm==ROAD_B_E_MM);
    finish_route();
}

static void scan_lane_obstacle_returns_home_test(void)
{
    uint32_t deadline;
    start_at(NODE_C);deadline=clock_ms+20000u;
    while(clock_ms<deadline && !(obstacle_sensor_for_leg()==0 && run.segment==(CAR_START_INSET_MM>0) && run.progress_mm>=15))
        step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run.progress_mm<CAR_START_INSET_MM && obstacle_inset_axes==OBSTACLE_ENTRY_X);
    obstacle_io_shot(0);obstacle_io_result(0,30);
    obstacle_io_shot(0);obstacle_io_result(0,30);
    CHECK(run.state==RUN_OBSTACLE_STOP);
    deadline=clock_ms+35000u;
    while(clock_ms<deadline && run_active && run.obstacle_phase)step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run_active && !run.obstacle_phase);
    CHECK(obstacle_inset_node==NODE_C && obstacle_inset_axes==OBSTACLE_ENTRY_X);
    CHECK(hypot(field_x+CAR_START_INSET_MM,field_y)<15.0);
    CHECK(run.road.segments[0].axis==BODY_FORWARD && run.road.segments[0].mm==-ROAD_C_BOTTOM_LINK_MM);
    finish_route();
}

static void incomplete_x_after_y_returns_y_only_test(void)
{
    uint32_t deadline;
    start_at(NODE_A);run.obstacle_node=NODE_A;run.obstacle_phase=2;
    run.blocked_roads=(uint16_t)(1u<<road_index(NODE_A,NODE_B));
    CHECK(obstacle_replan() && run.road.to==NODE_F);
    deadline=clock_ms+20000u;
    while(clock_ms<deadline && !(obstacle_sensor_for_leg()==0 && run.segment==1u && run.progress_mm>=3))
        step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run.progress_mm<30 && obstacle_inset_axes==OBSTACLE_ENTRY_Y);
    obstacle_io_shot(0);obstacle_io_result(0,20);
    obstacle_io_shot(0);obstacle_io_result(0,20);
    CHECK(run.state==RUN_OBSTACLE_STOP);
    deadline=clock_ms+35000u;
    while(clock_ms<deadline && run_active)step_with_sonar(-1,-1);
    CHECK(clock_ms<deadline && run.state==RUN_WAIT_OBSTACLE);
    CHECK(obstacle_inset_node==NODE_A && obstacle_inset_axes==OBSTACLE_ENTRY_Y);
    CHECK(fabs(field_x)<15.0 && fabs(field_y-ROAD_A_TOP_LINK_MM)<15.0);
}

static void move_then_close_obstacle_test(void)
{
    uint32_t deadline;
    unsigned i;
    start_at(NODE_C);deadline=clock_ms+20000u;
    /* 不注入任何测距结果，必须仍能完成入道并开始向B前进。 */
    while(clock_ms<deadline && !(obstacle_sensor_for_leg()==0 && run.segment==(CAR_START_INSET_MM>0) && run.progress_mm>=5))plant_step();
    CHECK(clock_ms<deadline && run.progress_mm>=5);
    for(i=0;i<3u;i++) {
        obstacle_io_shot(0);obstacle_io_result(0,300);
        obstacle_io_shot(0);obstacle_io_result(0,301);
        CHECK(!run.blocked_roads && run.state==RUN_MOVING);
    }
    obstacle_io_shot(0);obstacle_io_result(0,299);CHECK(!run.blocked_roads);
    obstacle_io_shot(0);obstacle_io_result(0,299);
    CHECK(run.state==RUN_OBSTACLE_STOP && run.blocked_roads==(1u<<road_index(NODE_C,NODE_B)));
    queue_byte('!');plant_step();
    {char last_command[]="obstacle";
     output_size=0;output[0]=0;command(last_command);
     CHECK(strstr(output,"[obstacle last] road=C-B sensor=FRONT mm=299")!=0);}
}

#include "verify_policy.h"
static void competition_speed_tests(void)
{
    start_at(NODE_C);
    CHECK(run.turn_rpm==CAR_TURN_CLOSED_LOOP_RPM);
    run.encoder_ready=ALL_MOTORS_MASK;run.turning=0;
    run.travel_axis=BODY_FORWARD;run.travel_sign=1;
    CHECK(run_forward_target(10000)==CAR_RUN_RPM);
    CHECK(run_wheel_limit_rpm()==CAR_RUN_RPM);
    run.travel_sign=-1;CHECK(run_forward_target(10000)==-CAR_RUN_RPM);
    run.travel_axis=BODY_LEFT;run.travel_sign=1;
    run.lateral_rpm=car_lateral_limit_rpm(1200);
    CHECK(run_forward_target(10000)==CAR_LATERAL_SHORT_RPM);
    CHECK(run_wheel_limit_rpm()==CAR_LATERAL_SHORT_RPM);
    run.lateral_rpm=car_lateral_limit_rpm(1201);
    CHECK(run_forward_target(10000)==CAR_LATERAL_LONG_RPM);
    CHECK(run_wheel_limit_rpm()==CAR_LATERAL_LONG_RPM);
    run.travel_axis=BODY_FORWARD;run.obstacle_phase=2;
    CHECK(run_forward_target(10000)==CAR_OBSTACLE_RETURN_RPM);
    run.velocity_phase=VELOCITY_FORWARD;
    CHECK(run_wheel_limit_rpm()==CAR_TURN_CLOSED_LOOP_RPM);
    run.velocity_sent=run.velocity_waiting=0;
    CHECK(run_velocity_send(CAR_OBSTACLE_RETURN_RPM,0,20));
    {unsigned i;int16_t expected[4];
     heading_control_translate_wheels(CAR_OBSTACLE_RETURN_RPM,0,20,expected);
     for(i=0;i<4u;i++)CHECK(simulated_wheel_rpm[i]==expected[i]);}
    run.travel_axis=BODY_LEFT;run.lateral_rpm=CAR_OBSTACLE_RETURN_RPM;
    CHECK(run_forward_target(10000)==CAR_OBSTACLE_RETURN_RPM);
    CHECK(run_wheel_limit_rpm()==CAR_TURN_CLOSED_LOOP_RPM);
    run.obstacle_phase=0;run.velocity_phase=VELOCITY_TURN;
    run.turning=1;CHECK(run_wheel_limit_rpm()==CAR_TURN_CLOSED_LOOP_RPM);
    run.obstacle_phase=0;queue_byte('!');plant_step();
    /* F-E-D连续横移总长超过1200mm，不能在经过E后重新加到短档。 */
    start_at(NODE_C);run.job=2;run.completed_roads=4095u;
    obstacle_nodes[0]=NODE_F;obstacle_nodes[1]=NODE_E;obstacle_nodes[2]=NODE_D;obstacle_node_count=3;
    run.heading=HEAD_X_NEG;CHECK(run_plan_road(0,&run.road));CHECK(run_segment());
    CHECK(run.travel_axis==BODY_LEFT && run.remaining_mm+run.continuation_mm>CAR_LATERAL_SHORT_MM
        && run.lateral_rpm==CAR_LATERAL_LONG_RPM);
    run.job=0;queue_byte('!');plant_step();
    puts("PASS competition speed: forward/reverse 250, lateral 250/200, turn 300, retreat60 RPM and shared mixed-wheel limit match TableNavigation");
}
int main(void)
{
    competition_speed_tests();
    full_mission_regression_test();remaining_root_state_tests();
    motion_cost_regression_test();
    turn_event_cost_tests();
    either_start_return_tests();
    g_connector_tests();g_connector_motion_tests();branch_heading_tests();i_connector_tests();i_connector_motion_tests();directional_threshold_tests();observation_tests();mission_route_tests();
    move_then_close_obstacle_test();
    /* B-C在途中应到C的入道连接点，不能包含进启停区的位移。 */
    {RoadPlan road;
     start_at(NODE_C);run.job=1;obstacle_nodes[0]=NODE_B;obstacle_nodes[1]=NODE_C;obstacle_nodes[2]=NODE_D;obstacle_node_count=3;
     CHECK(run_plan_road(0,&road));
     CHECK(road.count==1u && road.length==ROAD_START_TO_B_MM-ROAD_C_BOTTOM_LINK_MM);
     checked_job=run.job;queue_byte('!');plant_step();}
    start_node_geometry_tests();
    transit_start_node_route_test(NODE_A);transit_start_node_route_test(NODE_C);
    passed_route_preference_test();
    route_priority_tests();passed_road_skips_sonar_test();
    if(CAR_START_INSET_MM)incomplete_entry_returns_home_test();
    scan_lane_obstacle_returns_home_test();
    incomplete_x_after_y_returns_y_only_test();
    start_at(NODE_A);finish_route();CHECK(!run.blocked_roads);
    start_at(NODE_C);finish_route();CHECK(!run.blocked_roads);
    obstacle_on_launch(NODE_A);obstacle_on_launch(NODE_C);
    start_at(NODE_A);
    run.obstacle_phase=2;run.obstacle_node=NODE_A;
    run.blocked_roads=(uint16_t)((1u<<road_index(NODE_A,NODE_B))|(1u<<road_index(NODE_A,NODE_F)));
    run_return_wait();CHECK(!run_active && run.state==RUN_WAIT_OBSTACLE);
    puts("PASS chassis obstacle: A/C XY entry links retained after retreat; partial X/Y entry preserved accurately; 24 directed roads in six endpoint modes; forced A/C transit and final home runs; directional 300/550mm policy, bounded time cost, 5184 independent route references, continuous sonar and passed-road detection skip");
    return 0;
}
