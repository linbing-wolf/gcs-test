/* 旁观测只积累明确的近距离正回波，不把无回波当作道路已走通。 */
static void obstacle_capture_ray(uint8_t sensor)
{
    HeadingSample sample=heading_snapshot();
    RoadPoint point;
    float angle,fx,fy,lx,ly;
    int forward=sensor ? OBSTACLE_LEFT_FORWARD_MM : OBSTACLE_FRONT_FORWARD_MM;
    int left=sensor ? OBSTACLE_LEFT_LEFT_MM : OBSTACLE_FRONT_LEFT_MM;
    uint8_t i;
    obstacle_ray.valid=0;
    if(!run_active || stop_pending || run.state!=RUN_MOVING || run.turning || run.station_align_pending
        || run.obstacle_phase || run.translation_only || run.turn_only || !run.road.count
        || run.velocity_phase!=VELOCITY_FORWARD || run.encoder_ready!=ALL_MOTORS_MASK
        || !heading_sample_fresh(&sample,clock_ms,CAR_IMU_MAX_AGE_MS)
        || abs(run_heading_error(sample))>200) {
        obstacle_observations[sensor].hits=0;return;
    }
    if(run.station_return_mm)return; /* 回2环时尚未进入当前道路。 */
    if(run.corner.active) {
        static const int8_t forward_x[4]={0,-1,0,1},forward_y[4]={1,0,-1,0};
        RoadHeading heading=run.corner.start_heading;
        float x,y;
        if(!run.corner.polarity_ready || clock_ms-run.corner.integrated_ms>CAR_VELOCITY_FEEDBACK_MS)return;
        run_corner_position_now(&x,&y);
        obstacle_ray.x=run.corner.map_origin.x+x*forward_y[heading]-y*forward_x[heading];
        obstacle_ray.y=run.corner.map_origin.y-x*forward_x[heading]-y*forward_y[heading];
    } else {
        obstacle_road_endpoint(&run.road,run.edge,&point);
        obstacle_ray.x=point.x;obstacle_ray.y=point.y;
        /* 从本段真实终点倒推，保留A/C两轴连接点及已完成段，不靠墙面定位。 */
        for(i=run.segment;i<run.road.count;i++) {
            MotionParameters p=run.road.segments[i];
            float remaining=p.mm;
            if(i==run.segment)remaining=(p.mm>0 ? 1.0f : -1.0f)*((float)run.action_mm-run.progress_mm);
            if(p.axis==BODY_LEFT)obstacle_ray.x+=remaining;
            else obstacle_ray.y-=remaining;
        }
    }
    angle=heading_wrap_cdeg(sample.yaw_cdeg-run.yaw_zero_cdeg)
        *(3.14159265358979323846f/(18000.0f*CAR_MAP_YAW_SIGN));
    fx=-sinf(angle);fy=cosf(angle);lx=fy;ly=-fx;
    obstacle_ray.x+=forward*fx+left*lx;obstacle_ray.y+=forward*fy+left*ly;
    obstacle_ray.dx=sensor ? lx : fx;obstacle_ray.dy=sensor ? ly : fy;
    obstacle_ray.sensor=sensor;obstacle_ray.edge=run.edge;obstacle_ray.segment=run.segment;
    obstacle_ray.ms=clock_ms;obstacle_ray.valid=1;
}

static void obstacle_observe_result(int mm)
{
    float x,y;
    uint8_t i,j,candidate=255u,sensor,hits;
    if(!obstacle_ray.valid)return;
    obstacle_ray.valid=0;
    sensor=obstacle_ray.sensor;hits=obstacle_observations[sensor].hits;
    /* 每只探头独立要求连续正回波；另一只探头的未知结果不打断本探头。 */
    obstacle_observations[sensor].hits=0;
    if(mm<20 || mm>OBSTACLE_OBSERVE_MAX_MM || !run_active || stop_pending || run.obstacle_phase
        || run.turning || run.edge!=obstacle_ray.edge || run.segment!=obstacle_ray.segment
        || clock_ms-obstacle_ray.ms>100u)return;
    x=obstacle_ray.x+mm*obstacle_ray.dx;y=obstacle_ray.y+mm*obstacle_ray.dy;
    for(i=0;i<ROAD_DIRECTION_COUNT;i+=2u) {
        RoadPlan road=road_motion_table[i];
        RoadPoint point;
        if(road.road==run.road.road || ((run.completed_roads|run.blocked_roads)&(1u<<road.road)))continue;
        /* 固定设施附近仍由行进方向300mm策略处理，不用旁观测远距离封路。 */
        if(road.from==NODE_B || road.to==NODE_B || road.from==NODE_F || road.to==NODE_F)continue;
        (void)obstacle_map_point(road.from,&point);
        obstacle_adjust_corner(&road);
        if(obstacle_start_node(road.from)) {
            RoadPoint row;point.x-=OBSTACLE_START_INSET_MM;
            (void)road_point(road.from==NODE_A ? NODE_F : NODE_D,&row);point.y=row.y;
            obstacle_adjust_inset(&road,road.from,OBSTACLE_ENTRY_BOTH,-1);
        }
        if(obstacle_start_node(road.to))obstacle_adjust_inset(&road,road.to,OBSTACLE_ENTRY_BOTH,1);
        for(j=0;j<road.count;j++) {
            MotionParameters p=road.segments[j];
            float dx=p.axis==BODY_LEFT ? (p.mm>0 ? -1.0f : 1.0f) : 0.0f;
            float dy=p.axis==BODY_FORWARD ? (p.mm>0 ? 1.0f : -1.0f) : 0.0f;
            float length=(float)abs(p.mm),along=(x-point.x)*dx+(y-point.y)*dy;
            float side=fabsf((x-point.x)*dy-(y-point.y)*dx);
            float origin_side=fabsf((obstacle_ray.x-point.x)*dy-(obstacle_ray.y-point.y)*dx);
            float parallel=fabsf(obstacle_ray.dx*dx+obstacle_ray.dy*dy);
            if(parallel>0.999f && origin_side<=OBSTACLE_OBSERVE_LINE_MM
                && side<=OBSTACLE_OBSERVE_LINE_MM && along>OBSTACLE_OBSERVE_ENDPOINT_MM
                && along<length-OBSTACLE_OBSERVE_ENDPOINT_MM) {
                /* 探头至回波的线段必须沿候选道路，不能隔着其他路口猜测遮挡后的道路。 */
                float origin_along=(obstacle_ray.x-point.x)*dx+(obstacle_ray.y-point.y)*dy;
                if(origin_along>=-OBSTACLE_OBSERVE_LINE_MM && origin_along<=length+OBSTACLE_OBSERVE_LINE_MM) {
                    if(candidate!=255u && candidate!=road.road)return;
                    candidate=road.road;
                }
            }
            point.x+=(int16_t)(dx*length);point.y+=(int16_t)(dy*length);
        }
    }
    if(candidate!=255u) {
        int d=abs((int)x-obstacle_observations[sensor].x)+abs((int)y-obstacle_observations[sensor].y);
        if(obstacle_observations[sensor].road!=candidate
            || clock_ms-obstacle_observations[sensor].ms>OBSTACLE_OBSERVE_MAX_AGE_MS
            || d>OBSTACLE_OBSERVE_MATCH_MM)hits=0;
        obstacle_observations[sensor].x=(int16_t)x;obstacle_observations[sensor].y=(int16_t)y;
        obstacle_observations[sensor].ms=clock_ms;obstacle_observations[sensor].road=candidate;
        obstacle_observations[sensor].hits=(uint8_t)(hits+1u);
        if(obstacle_observations[sensor].hits>=OBSTACLE_REQUIRED_HITS) {
            const RoadPlan *road=&road_motion_table[(unsigned)candidate*2u];
            run.blocked_roads|=(uint16_t)(1u<<candidate);
            obstacle_observed_roads|=(uint16_t)(1u<<candidate);
            obstacle_map_changed=1;
            {
                RunLeg current;
                current.axis=run.travel_axis;current.sign=run.travel_sign;
                current.heading=run.target_heading;current.mm=run.action_mm;
                /* 撤销旧的跨节点直线预看，在下一节点交接重算，不能沿旧预看闯入新封路。 */
                run.continuation_mm=run_straight_remaining(current);
            }
            text("[road observed blocked] road=");send_byte((uint8_t)('A'+road->from));send_byte('-');
            send_byte((uint8_t)('A'+road->to));text(" mm=");number((uint32_t)mm);
            text("; current road continues; replan at next node\r\n");
        }
    }
}
