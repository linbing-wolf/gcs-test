/* 第二次暂存实际编排、视觉协议及独立车体积分；不连接硬件。 */
#define STATION_TEST_MAIN retained_second_storage_station_suite
#include "verify_station_vision.c"

static void second_storage_orders(void)
{
    static const char *orders[]={"123","132","213","231","312","321"};
    static const char *colors[]={"134","143","314","341","413","431"};
    unsigned plan,order,color;
    for(plan=0;plan<2;plan++)for(order=0;order<6;order++)for(color=0;color<6;color++) {
        char task[48];unsigned offset=0,placed=0,overlap=0,launched=0,moving_home=0,requests=0;
        uint32_t start,deadline,depart=0;float previous=ARM_CLAW_OPEN_DEG;RoadPoint h;
        unsigned index=colors[color][2]=='1' ? 0 : colors[color][2]=='3' ? 1 : 2;
        int last_ring=orders[order][index]-'0';
        int expected=(2-last_ring)*STN_STORAGE_INDEX_SIGN*STN_STORAGE_INDEX_MM;
        snprintf(task,sizeof(task),"task 134+%s+%s+231",orders[order],colors[color]);
        prepare_storage(1,task);run.plan=(uint8_t)plan;CHECK(road_point(NODE_H,&h));
        start=clock_ms;deadline=clock_ms+150000;
        while(clock_ms<deadline) {
            float yaw,claw;int16_t mm;
            serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(peer_stack_count!=requests) {
                CHECK(peer_stack_count==requests+1 && !station_chassis_busy() && !station_index_overlap_active() && !arm_busy());
                CHECK(peer_disc_count==0);requests=peer_stack_count;
            }
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG) {
                int lift;arm_pose_now(NULL,&lift,NULL,NULL);
                CHECK(lift==STN_STORAGE_LIFT_MM+STN_STORAGE_LAYER_LIFT_MM);placed++;
            }
            previous=claw;
            if(placed && placed<3 && run.station_shift && station_index_overlap_active() && arm_busy())overlap|=1u<<placed;
            if(run.job==7 && !launched) {
                launched=1;depart=clock_ms;CHECK(placed==3 && requests==3 && !peer_disc_count);
                if(STN_STORAGE_SECOND_THROUGH_ENABLED) {
                    CHECK(station_through_departure(&mm) && mm==expected && run.station_return_mm==expected);
                    CHECK(run.departure_overlap==2 && arm_busy());
                    CHECK(hypot(field_x-h.x,field_y-(h.y-expected))<35);
                } else CHECK(!station_through_departure(NULL) && !run.station_return_mm);
            }
            if(launched) {
                CHECK(peer_stack_count==3 && !peer_disc_count);
                if(run.departure_overlap==2 && arm_busy() && (run.forward_cmd_rpm || run.left_cmd_rpm))moving_home=1;
                if(STN_STORAGE_SECOND_THROUGH_ENABLED && run.corner.active)CHECK(run.corner.prefix_mm==expected);
                if(!run.departure_overlap)break;
            }
        }
        CHECK(clock_ms<deadline && launched && placed==3 && requests==3 && arm_parked());
        CHECK(CAR_ARM_TRANSLATION_OVERLAP_ENABLED ? overlap==6 : overlap==0);
        if(STN_STORAGE_SECOND_THROUGH_ENABLED)CHECK(moving_home);
        printf("STORAGE_SECOND plan=%u rings=%s colors=%s parallel=%u through=%u return_mm=%d depart_ms=%lu total_ms=%lu\n",
            plan,orders[order],colors[color],(unsigned)CAR_ARM_TRANSLATION_OVERLAP_ENABLED,
            (unsigned)STN_STORAGE_SECOND_THROUGH_ENABLED,expected,(unsigned long)(depart-start),(unsigned long)(clock_ms-start));
        stop_request();
    }
}

static void actual_lift_overlap(void)
{
    unsigned release,variant;
    if(!CAR_ARM_TRANSLATION_OVERLAP_ENABLED || !STN_STORAGE_SECOND_THROUGH_ENABLED)return;
    for(release=1;release<=3;release+=2)for(variant=0;variant<3;variant++) {
        uint32_t deadline;unsigned offset=0,placed=0,positions;int64_t base,counts,one;
        float previous=ARM_CLAW_OPEN_DEG;
        prepare_storage(1,"task 134+123+314+231");deadline=clock_ms+120000;
        while(placed<release && clock_ms<deadline) {
            float yaw,claw;serve_peer(&offset);plant_step();arm_pose_now(NULL,NULL,&yaw,&claw);
            if(claw!=previous && claw==ARM_CLAW_OPEN_DEG && yaw==STN_STORAGE_YAW_DEG)placed++;
            previous=claw;
        }
        CHECK(placed==release && run.job==6 && !run_active);
        base=arm_encoder[1];positions=arm_positions;arm_freeze=1;
        while(arm_positions==positions && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && arm_trace_address[positions]==ARM_LIFT_ADDRESS);
        counts=(int64_t)((ARM_CLEARANCE_MM-STN_STORAGE_LIFT_MM-STN_STORAGE_LAYER_LIFT_MM)*ARM_LIFT_PULSES_PER_MM)
            *ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV;
        if(ARM_LIFT_DIR_INVERT)counts=-counts;
        counts*=ARM_LIFT_ENCODER_SIGN;
        one=(int64_t)(ARM_LIFT_PULSES_PER_MM*ARM_ENCODER_COUNTS_PER_REV/ARM_PULSES_PER_REV);
        arm_encoder[1]=base+counts-(counts>0 ? one : -one);
        tick(600);CHECK(run.job==6 && !run_active && !station_through_departure(NULL));
        if(variant==2){stop_request();tick(100);CHECK(!run_active && !station_active() && arm_failed());continue;}
        if(variant==1) {
            arm_missing_motor=ARM_LIFT_ADDRESS;arm_encoder[1]=base+counts;
            tick(ARM_FEEDBACK_TIMEOUT_MS+200);CHECK(run.job==6 && !run_active);
            arm_missing_motor=0;
        }
        arm_encoder[1]=base+counts;
        while(!(run.forward_cmd_rpm || run.left_cmd_rpm) && clock_ms<deadline)plant_step();
        CHECK(clock_ms<deadline && run_active && arm_busy() && arm_encoder[1]==base+counts);
        CHECK(release==3 ? (run.job==7 && run.departure_overlap==2) : (run.job==6 && station_index_overlap_active()));
        stop_request();
    }
    puts("PASS actual lift overlap: first/last second-layer release;124mm or stale feedback waits,125mm moves with lift still below135mm; ! cancels");
}

static void second_storage_vision_moves(void)
{
    unsigned variant;
    for(variant=0;variant<3;variant++) {
        unsigned offset=0,positions;uint32_t deadline;int pixels=variant==0 ? 40 : variant==1 ? -40 : 0;
        char payload[32];RoadPoint h;
        prepare_storage(1,"task 134+231+314+123");CHECK(road_point(NODE_H,&h));field_y+=pixels;
        peer_stack_hold=1;deadline=clock_ms+30000;
        while(!peer_stack_count && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(peer_stack_count==1 && vision_stack_state()==VISION_PICK_WAITING && !run_active);
        positions=arm_positions;peer_reply("PICK,3,320,240",0);tick(500);
        CHECK(arm_positions==positions && peer_stack_count==1 && !run_active);
        snprintf(payload,sizeof(payload),"MOVE,%d",pixels);peer_reply(payload,0);peer_reply("STACK,600,450,620,1",0);
        deadline=clock_ms+60000;
        if(pixels) {
            while(!run.station_shift && clock_ms<deadline)plant_step();
            CHECK(clock_ms<deadline && run.remaining_mm==40 && run.travel_sign==(pixels>0 ? -1 : 1));
            while(station_chassis_busy() && clock_ms<deadline){check_correction_arm(positions);plant_step();}
        }
        CHECK(fabs(field_y-h.y+150)<5);
        peer_stack_hold=0;deadline=clock_ms+120000;
        while(run.job==6 && clock_ms<deadline){serve_peer(&offset);plant_step();}
        CHECK(clock_ms<deadline && run.job==7 && peer_stack_count==3 && !peer_disc_count);
        CHECK(!strstr(output,"[stack return]"));
        CHECK(fabs(field_y-h.y-(STN_STORAGE_SECOND_THROUGH_ENABLED ? 150 : 0))<5);
        stop_request();
    }
    puts("PASS second storage F: waiting/wrong terminal does not pick; signed40/-40/0 MOVE correction retained, no inverse return, exactly3F and no departure vision");
}

int main(void)
{
    second_storage_orders();actual_lift_overlap();second_storage_vision_moves();
    printf("PASS second storage parallel: %u assertions; fixed index/road overlap,3F retained; model only\n",assertions);
    return 0;
}
