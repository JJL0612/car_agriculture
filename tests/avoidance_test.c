#include "avoidance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static AvoidanceInput sample(float left, float front, float right)
{
    AvoidanceInput in = {{left, front, right}, {1, 1, 1}, {100, 100, 100}};
    return in;
}

static void expect(AvoidanceInput in, uint32_t now, int left, int right)
{
    int l = -1, r = -1;
    avoidance_step(&in, now, &l, &r);
    assert(l == left && r == right);
}

int main(void)
{
    avoidance_reset();
    expect(sample(100, 100, 100), 100, 300, 300);
    expect(sample(100, 51, 100), 100, 300, 300); /* just outside new trigger */
    /* Front obstacle: choose the more open side, with correct wheel mapping. */
    expect(sample(90, 50, 40), 100, -300, 300);
    expect(sample(60, 65, 90), 100, -300, 300); /* keep turn despite noisy side ordering */
    expect(sample(40, 70, 40), 100, 300, 300); /* release thresholds */
    expect(sample(40, 45, 90), 100, 300, -300);
    expect(sample(90, 60, 30), 100, -300, 300); /* chosen side closes */
    avoidance_reset();
    expect(sample(30, 100, 90), 100, 300, -300); /* left-front obstacle */
    avoidance_reset();
    expect(sample(90, 100, 30), 100, -300, 300); /* right-front obstacle */
    expect(sample(30, 100, 30), 100, 0, 0); /* both sides blocked */
    expect(sample(100, 20, 100), 100, 0, 0); /* emergency stop */
    expect(sample(100, 21, 100), 100, -300, 300); /* deterministic tie */

    for (unsigned i = 0; i < 3; ++i) {
        AvoidanceInput in = sample(100, 100, 100);
        in.valid[i] = 0;
        expect(in, 100, 0, 0);
        in = sample(100, 100, 100);
        in.updated[i] = 0;
        expect(in, 301, 0, 0);
        in = sample(100, 100, 100);
        in.cm[i] = -1;
        expect(in, 100, 0, 0);
        in.cm[i] = NAN;
        expect(in, 100, 0, 0);
    }
    expect(sample(100, 100, 100), 400, 300, 300);
    expect(sample(100, 100, 100), 401, 0, 0);
    AvoidanceInput wrap = sample(100, 100, 100);
    for (unsigned i = 0; i < 3; ++i) wrap.updated[i] = UINT32_MAX - 49U;
    expect(wrap, 50, 300, 300);
    expect(sample(90, 45, 40), 100, -300, 300);
    avoidance_reset(); /* returning to manual clears the previous turn */
    expect(sample(45, 65, 45), 100, 300, 300);
    puts("avoidance tests passed");
    return 0;
}
