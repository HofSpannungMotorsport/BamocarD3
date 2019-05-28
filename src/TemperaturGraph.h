#ifndef TEMPERATURE_GRAPH_H
#define TEMPERATURE_GRAPH_H

/*
    The Temperature Graph is a 2-Dimensional array

    The first dimension represents the measurement points

    The second dimension is split into the value pair
        0 = Actual Value read from register
        1 = temperature in °C
    
    Everything in between can be scaled linear

    Because of the actual temperature will never reach the lower end,
    we will limit the graph for performance.
    No need to limmit the upper end, it only consumes storage, but not performance
*/

// Internal temperatur of the IGBT
const int16_t igbtTempGraph[][2] = {/*{16245, -35},
                                   {16308, -30},
                                   {16387, -25},
                                   {16487, -20},
                                   {16609, -15},
                                   {16759, -10},
                                   {16938, -5},
                                   {17151, 0},
                                   {17400, 5},*/
                                   {17688, 10},
                                   {18017, 15},
                                   {18387, 20},
                                   {18797, 25},
                                   {19247, 30},
                                   {19733, 35},
                                   {20250, 40},
                                   {20793, 45},
                                   {21357, 50},
                                   {21933, 55},
                                   {22515, 60},
                                   {23097, 65},
                                   {23671, 70},
                                   {24232, 75},
                                   {24775, 80},
                                   {25296, 85},
                                   {25792, 90},
                                   {26261, 95},
                                   {26702, 100},
                                   {27114, 105},
                                   {27497, 110},
                                   {27851, 115},
                                   {28179, 120},
                                   {28480, 125},
                                   {28757, 130},
                                   {29011, 135},
                                   {29243, 140},
                                   {29456, 145},
                                   {29650, 150},
                                   {29827, 155},
                                   {-1, 0}}; // To detect the end of the Array

// Graph for KTY81-2XX Sensor of Emrax 228
const int16_t motorTempGraph[][2] = {/*{7414, -35},
                                    {7687, -30},
                                    {7962, -25},
                                    {8240, -20},
                                    {8520, -15},
                                    {8802, -10},
                                    {9085, -5},
                                    {9369, 0},
                                    {9654, 5},*/
                                    {9939, 10},
                                    {10225, 15},
                                    {10510, 20},
                                    {10795, 25},
                                    {11080, 30},
                                    {11364, 35},
                                    {11646, 40},
                                    {11927, 45},
                                    {12207, 50},
                                    {12485, 55},
                                    {12762, 60},
                                    {13036, 65},
                                    {13308, 70},
                                    {13578, 75},
                                    {13846, 80},
                                    {14111, 85},
                                    {14373, 90},
                                    {14633, 95},
                                    {14890, 100},
                                    {15144, 105},
                                    {15391, 110},
                                    {15628, 115},
                                    {15852, 120},
                                    {16061, 125},
                                    {16251, 130},
                                    {16421, 135},
                                    {16569, 140},
                                    {16692, 145},
                                    {16789, 150},
                                    {16857, 155},
                                    {-1, 0}}; // End of array

// Graph for KTY82-2XX for Tair
const int16_t airTempGraph[][2] = {/*{8516, -35},
                                  {8785, -30},
                                  {9054, -25},
                                  {9322, -20},
                                  {9590, -15},
                                  {9857, -10},
                                  {10122, -5},
                                  {10386, 0},
                                  {10647, 5},*/
                                  {10907, 10},
                                  {11163, 15},
                                  {11418, 20},
                                  {11669, 25},
                                  {11917, 30},
                                  {12163, 35},
                                  {12404, 40},
                                  {12643, 45},
                                  {12878, 50},
                                  {13109, 55},
                                  {13337, 60},
                                  {13561, 65},
                                  {13781, 70},
                                  {13998, 75},
                                  {14210, 80},
                                  {14419, 85},
                                  {14624, 90},
                                  {14825, 95},
                                  {15022, 100},
                                  {15215, 105},
                                  {-1, 0}}; // Again, to detect end of array

#endif