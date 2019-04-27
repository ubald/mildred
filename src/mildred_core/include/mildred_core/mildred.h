#pragma once

#define LEG_COUNT 6
#define DOF 3

enum MildredCommand {
    Ragdoll = 0,
    Sit,
    Stand,
    Walk
};

enum class MildredState {
    Unknown = 0,
    Starting,
    Idle,
    Sitting,
    Standing,
    Walking
};