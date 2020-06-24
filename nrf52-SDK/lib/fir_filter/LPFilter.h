#ifndef LPFILTER_H_
#define LPFILTER_H_

#define LPFILTER_TAP_NUM 210

typedef struct {
  int history[LPFILTER_TAP_NUM];
  unsigned int last_index;
} LPFilter;

void LPFilter_init(LPFilter* f);
void LPFilter_put(LPFilter* f, int input);
int LPFilter_get(LPFilter* f);

#endif
