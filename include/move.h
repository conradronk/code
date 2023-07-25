#ifndef move_h
#define move_h

struct move {
    uint32_t startTime;
    uint32_t startSpeed;
    uint32_t endSpeed;
    float acceleration; //given in steps/s^2
    int16_t move;
};

#endif