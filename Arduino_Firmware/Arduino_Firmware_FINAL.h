#ifndef HEADER
#define HEADER

void G0_cmd();
void G1_cmd();
void G20_cmd();
void G21_cmd();
void G90_cmd();
void G91_cmd();
void M2_cmd();
void M6_cmd();
void M72_cmd();
void C1_cmd();
void C2_cmd();
int go_height(double z);
int go(double x, double y, double z, int f_speed);
double mm_to_in(double mm);
double in_to_mm(double in);
int calculate_number_steps(double current_, double new_);
int go_block(double x, double y, double z, int f_speed);
int go_len_block(double a, double b, double c, int f_speed);

#endif
