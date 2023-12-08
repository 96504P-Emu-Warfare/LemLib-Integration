#pragma once

#include <string>

//selector configuration
#define HUE 192
#define DEFAULT 1
#define AUTONS "SAFE/4", "RISKY/5-R", "RUSH/5-S"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
