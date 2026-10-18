/* 电脑端验证器：以现有C基准算法和当前道路核对离线表，不用于车辆固件。 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "route_table.h"
#include "reference_route.h"
#include "road_map.h"
static Road roads[12];

static uint32_t checks, states;
uint32_t route_table_storage_bytes(void);
static int fail(uint8_t from, uint8_t to, uint16_t blocked, uint16_t passed,
                const char *reason)
{
    fprintf(stderr,"FAIL from=%u to=%u blocked=%u passed=%u: %s\n",
            from,to,blocked,passed,reason);
    return 1;
}

int main(void)
{
    uint32_t blocked;
    uint16_t seen=0;
    unsigned edge;
    clock_t begin=clock();
    for(edge=0;edge<ROAD_DIRECTION_COUNT;edge++) {
        const RoadPlan *p=&road_motion_table[edge];
        if(seen & (1u<<p->road))continue;
        seen|=(uint16_t)(1u<<p->road);
        roads[p->road].a=p->from;roads[p->road].b=p->to;roads[p->road].length=p->length;
    }
    if(seen!=4095u)return fail(0,0,0,0,"missing current road");
    for(blocked=0;blocked<4096;blocked++) {
        uint16_t free_mask=(uint16_t)(4095u & ~blocked), passed=free_mask;
        for(;;) {
            uint16_t fd[9][9],fu[9][9];
            uint8_t from,to,i,j,k;
            states++;
            for(i=0;i<9;i++) for(j=0;j<9;j++)
                fd[i][j]=fu[i][j]=(i==j?0:65535);
            for(i=0;i<12;i++) if(!(blocked & (1u<<i))) {
                uint8_t a=roads[i].a,b=roads[i].b;
                fd[a][b]=fd[b][a]=roads[i].length;
                fu[a][b]=fu[b][a]=(passed & (1u<<i))?0:roads[i].length;
            }
            /* 独立 Floyd-Warshall 同时核对两个优化目标。 */
            for(k=0;k<9;k++) for(i=0;i<9;i++) for(j=0;j<9;j++) {
                uint32_t nd=(uint32_t)fd[i][k]+fd[k][j];
                uint32_t nu=(uint32_t)fu[i][k]+fu[k][j];
                if(nd<fd[i][j] || (nd==fd[i][j] && nu<fu[i][j])) {
                    fd[i][j]=(uint16_t)nd;fu[i][j]=(uint16_t)nu;
                }
            }
            for(from=0;from<9;from++) for(to=0;to<9;to++) {
                Route ref;
                RouteTablePath got;
                uint16_t distance=0,unknown=0;
                int expected=route_distance(from,to,(uint16_t)blocked,passed,roads,&ref);
                int actual=route_table_lookup(from,to,(uint16_t)blocked,passed,&got);
                checks++;
                if(expected!=actual || ref.distance!=got.distance || ref.count!=got.count ||
                   memcmp(ref.nodes,got.nodes,9))
                    return fail(from,to,(uint16_t)blocked,passed,"exact baseline route mismatch");
                if(!actual) {
                    if(fd[from][to]!=65535) return fail(from,to,(uint16_t)blocked,passed,"reachability");
                    continue;
                }
                if(!got.count || got.nodes[0]!=from || got.nodes[got.count-1]!=to)
                    return fail(from,to,(uint16_t)blocked,passed,"endpoints");
                for(i=1;i<got.count;i++) {
                    int edge=-1;
                    for(j=0;j<12;j++) if((roads[j].a==got.nodes[i-1] && roads[j].b==got.nodes[i]) ||
                        (roads[j].b==got.nodes[i-1] && roads[j].a==got.nodes[i])) {edge=j;break;}
                    if(edge<0 || (blocked & (1u<<edge)))
                        return fail(from,to,(uint16_t)blocked,passed,"blocked or missing edge");
                    distance=(uint16_t)(distance+roads[edge].length);
                    if(!(passed & (1u<<edge))) unknown=(uint16_t)(unknown+roads[edge].length);
                }
                if(distance!=got.distance || distance!=fd[from][to] || unknown!=fu[from][to] || unknown!=ref.unknown)
                    return fail(from,to,(uint16_t)blocked,passed,"independent distance/unknown optimum");
            }
            if(passed==0) break;
            passed=(uint16_t)((passed-1u)&free_mask);
        }
        if((blocked & 511u)==511u) {
            printf("verified blocked masks %u/4096, queries %u\n",blocked+1,checks);
            fflush(stdout);
        }
    }
    {
        RouteTablePath got,normal;
        if(route_table_lookup(9,0,0,0,&got) || got.count || got.distance!=65535 ||
           route_table_lookup(0,9,0,0,&got) || route_table_lookup(0,1,0,0,0))
            return fail(9,9,0,0,"invalid input contract");
        route_table_lookup(0,3,1,4094,&normal);
        route_table_lookup(0,3,0xf001u,0xffffu,&got);
        if(memcmp(&normal,&got,sizeof(got))) return fail(0,3,1,4095,"mask normalization");
    }
    if(states!=531441u || checks!=43046721u) return fail(0,0,0,0,"incomplete enumeration");
    {
        FILE *file=fopen("validation-result.json","w");
        if(!file) return fail(0,0,0,0,"cannot write validation record");
        fprintf(file,"{\n  \"states\": %u,\n  \"queries\": %u,\n  \"exactBaselineRoute\": true,\n  \"independentFloyd\": true,\n  \"blockedEdgesExcluded\": true,\n  \"invalidInputs\": true,\n  \"tableDataBytes\": %u,\n  \"seconds\": %.3f,\n  \"hardwareRun\": false\n}\n",states,checks,route_table_storage_bytes(),(double)(clock()-begin)/CLOCKS_PER_SEC);
        if(fclose(file)) return 1;
    }
    printf("PASS %u states, %u exact C route comparisons; independent Floyd passed; table bytes %u\n",states,checks,route_table_storage_bytes());
    return 0;
}
