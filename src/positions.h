#ifndef _POSITIONS_H_
#define _POSITIONS_H_

constexpr int lid = 8;
constexpr int base = 6;
constexpr int lid_on_base = 3;

struct coordinates
{
    int x;
    int y;
};

// sorting Box Position
coordinates sortBox;
sortBox.x = 380;
sortBox.y = 195;

// assembly Box Positions
coordinates assemBox[8];
assemBox[0].x = 35;
assemBox[0].y = 65;
assemBox[1].x = 35;
assemBox[1].y = 125;
assemBox[2].x = 35;
assemBox[2].y = 223;
assemBox[3].x = 35;
assemBox[3].y = 286;

assemBox[4].x = 90;
assemBox[4].y = 65;
assemBox[5].x = 90;
assemBox[5].y = 125;
assemBox[6].x = 90;
assemBox[6].y = 223;
assemBox[7].x = 90;
assemBox[7].y = 286;

// reserve Box Positions
coordinates reserveBox[8];
reserveBox[0].x = 144;
reserveBox[0].y = 65;
reserveBox[1].x = 144;
reserveBox[1].y = 125;
reserveBox[2].x = 144;
reserveBox[2].y = 223;
reserveBox[3].x = 144;
reserveBox[3].y = 286;

reserveBox[4].x = 199;
reserveBox[4].y = 65;
reserveBox[5].x = 199;
reserveBox[5].y = 125;
reserveBox[5].y = 223;
reserveBox[6].x = 199;
reserveBox[6].y = 286;

#endif