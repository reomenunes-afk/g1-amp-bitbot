#include "../adapters/HeadingFeedback.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
static bool near(double a,double b) { return std::abs(a-b)<1e-9; }
int main() {
    z::HeadingFeedback<double> h;
    // Disabled preserves arbitrary legacy command exactly.
    assert(near(h.Update(1.,.3,.01),.3));
    h.enabled=true;
    assert(near(h.Update(1.,0.,.01),0.));
    // Drift left -> negative (right) correction; raw command remains zero.
    assert(near(h.Update(1.1,0.,.01),-.05));
    assert(near(h.Update(2.,0.,.01),-.2));
    assert(near(h.Update(0.,0.,.01),.2));
    h.Reset(); h.Update(3.13,0.,.01);
    assert(std::abs(h.Update(-3.13,0.,.01))<.02); // wrap boundary
    // Desired heading advances only with operator rate, not feedback.
    h.Reset(); h.Update(0.,.1,.01);
    for(int n=1;n<100;++n) h.Update(0.,.1,.01);
    assert(near(h.target,.1));
    assert(near(h.Update(0.,0.,.01),.05)); // stop turning retains target
    h.Reset(); assert(near(h.Update(-1.,0.,.01),0.)); // re-enter
    // Ideal rate-tracking plant: heading disturbance converges without oscillation.
    h.Reset(); h.Update(0.,0.,.01); double yaw=.3;
    for(int n=0;n<2000;++n) yaw+=h.Update(yaw,0.,.01)*.01;
    assert(std::abs(yaw)<.0001);
    h.enabled=false; assert(near(h.Update(yaw,-.1,.01),-.1));
    h.enabled=true; assert(near(h.Update(2.,0.,.01),0.));
    bool caught=false;
    try { h.Update(NAN,0.,.01); } catch(const std::runtime_error&) { caught=true; }
    assert(caught);
    std::cout<<"heading_feedback_test PASSED\n";
}
