#define HEADING_TEST_MAIN heading_suite_main
#include "../../tests/verify_heading.c"
int main(void) {
    unsigned i,j;const int angles[]={90,180};const double gains[]={0.65,1.0,1.4};
    for(i=0;i<3;i++)for(j=0;j<2;j++){uint32_t start;char cmd[32];reset_plant(17900,gains[i]);snprintf(cmd,sizeof(cmd),"left%d",angles[j]);
        send_line("turn");send_line("rpm 60");send_line(cmd);start=clock_ms;finish_plant(20000);
        printf("TURN_TIMING gain=%.2f angle=%d elapsed_ms=%lu error_cdeg=%ld reverse=%u\n",gains[i],angles[j],(unsigned long)(clock_ms-start),(long)run.heading_control.error_cdeg,reverse_seen);}
    return 0;
}
