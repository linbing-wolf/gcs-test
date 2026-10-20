/* 电脑验证基准：复用原route.c的节点扫描/严格松弛顺序，仅显式传入当前道路。 */
#include "reference_route.h"
#define INF 65535u
int route_distance(uint8_t from,uint8_t to,uint16_t blocked,uint16_t passed,const Road roads[12],Route *out) {
    uint16_t d[9],u[9];uint8_t prev[9],done[9],chain[9];uint32_t i,k;
    if(!out)return 0;
    out->distance=out->unknown=INF;out->count=0;
    for(i=0;i<9;i++){out->nodes[i]=255;d[i]=u[i]=INF;prev[i]=255;done[i]=0;}
    if(from>=9||to>=9)return 0;
    d[from]=u[from]=0;
    for(k=0;k<9;k++) {
        int best=-1;
        for(i=0;i<9;i++)if(!done[i]&&(best<0||d[i]<d[best]||(d[i]==d[best]&&u[i]<u[best])))best=(int)i;
        if(best<0||d[best]==INF)break;
        done[best]=1;if(best==to)break;
        for(i=0;i<12;i++) {
            const Road *r=&roads[i];uint8_t n;uint16_t nd,nu;
            if((blocked&(1u<<i))||(r->a!=best&&r->b!=best))continue;
            n=r->a==best?r->b:r->a;nd=(uint16_t)(d[best]+r->length);nu=(uint16_t)(u[best]+((passed&(1u<<i))?0:r->length));
            if(nd<d[n]||(nd==d[n]&&nu<u[n])){d[n]=nd;u[n]=nu;prev[n]=(uint8_t)best;}
        }
    }
    if(d[to]==INF)return 0;
    i=0;k=to;chain[i++]=(uint8_t)k;
    while(k!=from){k=prev[k];chain[i++]=(uint8_t)k;}
    out->distance=d[to];out->unknown=u[to];out->count=(uint8_t)i;
    for(k=0;k<i;k++)out->nodes[k]=chain[i-k-1];
    return 1;
}
