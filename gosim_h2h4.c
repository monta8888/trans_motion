#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME     0
#define HIP_Y_R  1
#define HIP_Y_L  2
#define HIP_R_R  3
#define HIP_R_L  4
#define HIP_P_R  5
#define HIP_P_L  6
#define KNE_P_R  7
#define KNE_P_L  8
#define ANK_P_R  9
#define ANK_P_L 10
#define ANK_R_R 11
#define ANK_R_L 12
#define DOU     13
#define DOU2    14
#define DOU3    15
#define HEAD    16
#define SHO_P_R 17
#define SHO_P_L 18
#define SHO_R_R 19
#define SHO_R_L 20
#define ARM_R   21
#define ARM_L   22
#define ELB_R   23
#define ELB_L   24
/* angle[][] =
   Time,HipY_R,HipY_L,HipR_R,HipR_L,HipP_R,HipP_L,KneP_R,KneP_L,AnkP_R,AnkP_L,AnkR_R,AnkR_L,Dou,Dou2,Dou3,Head,ShoP_R,ShoP_L,ShoR_R,ShoR_L,Arm_R,Arm_L,Elb_R,Elb_L
   -----------------------------------------------------------
               Head                                Head
        ShoP_R Dou  ShoP_L                  ShoP_R      ShoP_L
        ShoR_R Dou2 ShoR_L                  ShoR_R      ShoR_L
        Arm_R  Dou3 Arm_L
        Elb_R       Elb_L                   Elb_R       Elb_L
          HipY_R  HipY_L       GoSim=>H2H4
   HipR_R HipP_R  HipP_L HipR_L               HipR_R  HipR_L
          KneP_R  KneP_L                    HipP_R      HipP_L
   AnkR_R AnkP_R  AnkP_L AnkR_L             KneP_R      KneP_L
                                            AnkP_R      AnkP_L
                                             AnkR_R    AnkR_L
   -----------------------------------------------------------
   GoSim[Time msec]            -> Time / 15msec -> H2H4[1frame]
   GoSim[Angle(-25000<>25000)] -> Angle/100 * 30-> H2H4[Angle(-7500<>7500)]
                                  *SingInversion -> SHO_P_R/SHO_R_R/ELB_L/HIP_R_R/HIP_R_L/HIP_P_R/KNE_P_R/ANK_P_L/ANK_R_R/ANK_R_L

  ex. xml command =
    2B 10 3D F3 3F 00 00 0F 64 19 58 1B 4C 1D 04 29 2C 1A 4C 1D 4C 1D E2 1D 4C 1D 34 21 64 19 1C 25 7C 15 64 19 34 21 E2 1D 1A 1D 6B
    2B = Size
    10 = Command
    3D F3 3F 00 00
       = Select Servo
    0F = Time / 15msec
    1964 =  6500 = ( 6500-7500) = -1000 (Head)
    1B58 =  7000 = ( 7000-7500) =  -500 (ShoP_L)
    1D4C =  7500 = ( 7500-7500) =     0 (ShoP_R)
    2904 = 10500 = (10500-7500) =  3000 (ShoR_L)
    1A2C =  6700 = ( 6700-7500) =  -800 (ShoR_R)
    1D4C =  7500 = ( 7500-7500) =     0 (Elb_L)
    1D4C =  7500 = ( 7500-7500) =     0 (Elb_R)
    1DE2 =  7650 = ( 7650-7500) =   150 (HipR_L)
    1D4C =  7500 = ( 7500-7500) =     0 (HipR_R)
    2134 =  8500 = ( 8500-7500) =  1000 (HipP_L)
    1964 =  6500 = ( 6500-7500) = -1000 (HipP_R)
    251C =  9500 = ( 9500-7500) =  2000 (KneP_L)
    157C =  5500 = ( 5500-7500) = -2000 (KneP_R)
    1964 =  6500 = ( 6500-7500) = -1000 (AnkP_L)
    2134 =  8500 = ( 8500-7500) =  1000 (AnkP_R)
    1DE2 =  7650 = ( 7650-7500) =   150 (AnkR_L)
    1D1A =  7450 = ( 7450-7500) =   -50 (AnkR_R)
    6B = CheckSum
 */

void set_angle(FILE *fp_out, int angle_type, int angle_v, int *sum)
{
  char buf[16];
  int calc_angle = 0;

  if (angle_type == SHO_P_R || angle_type == SHO_R_R || angle_type == ELB_L ||
      angle_type == HIP_R_R || angle_type == HIP_R_L || angle_type == HIP_P_R || angle_type == KNE_P_R ||
      angle_type == ANK_P_L || angle_type == ANK_R_R || angle_type == ANK_R_L) {  // sign inversion
    calc_angle = ~angle_v / 100 * 30 + 7500;
  } else {
    calc_angle = angle_v / 100 * 30 + 7500;
  }
  *sum += calc_angle & 0xff;
  *sum += (calc_angle >> 8) & 0xff;
  sprintf(buf, "%02X %02X ", calc_angle & 0xff, (calc_angle >> 8) & 0xff);
// printf("%04x\n", calc_angle);
  fputs(buf, fp_out);
}

int main(int argc, char *argv[])
{
  FILE *fp_in;
  FILE *fp_out;
  char buf[256];
  char fin[256];
  char fout[256];
  char *tok;
  char str_1st[] = "=[";
  char str_2nd[] = "Time";
  char str_3rd[] = "]";
  int stat = 0;
  int h, i;
  int angle_x = 0;
  int angle_y = 0;
  int angle[128][25];

  int ls_x = 76; // next 76 + 72
  int ls_y = 96;
  int le_x = 90; // offset ls_x + 14
  int le_y = 96;
  int pos_x = 16; // next 16 + 72
  int pos_y = 72;
  int guid = 1;

  int calc_sum   = 0;

  /* check files */
  if (argc >= 2) {
    printf("usage: gosim_h2h4 # current file (input == src.rmo / output == dist.xml\n");
    return (0);
  }
  strcpy(fin,  "src.rmo");
  if ((fp_in = fopen(fin, "r")) == NULL) {
    printf("Can not open read file [%s]\n", fin);
    return (-1);
  }
  strcpy(fout, "dist.xml");
  if ((fp_out = fopen(fout, "w")) == NULL) {
    printf("Can not open write file [%s]\n", fout);
    return (-1);
  }

  /* read file */
  printf("PICKUP from .rmo\n");
  while (fgets(buf, 256, fp_in) != NULL) {
    // search "=["
    if      ( stat == 0 && (NULL != strstr(buf, str_1st)) ) { stat = 1; printf("hit: %s\n", buf ); }
    // search "Time"
    else if ( stat == 1 && (NULL != strstr(buf, str_2nd)) ) { stat = 2; printf("hit head\n"); }
    // get angle
    else if ( stat == 2 ) {
      if (strstr(buf, str_3rd)) { printf("hit end\n"); break; } // fin get angle
      angle_x = 0;
      tok = strtok( buf, " ," );
      while( tok != NULL ){
//        printf( "%s\n", tok );
        angle[angle_y][angle_x] = atoi(tok);
        angle_x++;
        tok = strtok( NULL, " ," );
      }
      angle_y++;
    }
  }
  printf("\nPICKUP RESULT\n");
  for (h=0; h < angle_y; h++) {
    for (i=0; i < angle_x; i++) {
      printf("%6d ", angle[h][i] );
    }
    printf("\n");
  }

  /* write file */
  printf("CREATE to .xml\n");
  // create header
  fputs("<?xml version=\"1.0\"?>\n", fp_out);
  fputs("<ArrayOfAnyType xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n", fp_out);
  // create line
  for (i=0; i < (angle_y-1); i++) {
    fputs("  <anyType xsi:type=\"LineData\">\n", fp_out);
    fputs("    <Name />\n",                      fp_out);
    fputs("    <StartPoint>\n",                  fp_out);
    sprintf(buf, "      <X>%d</X>\n", ls_x);
    fputs(buf,                                   fp_out);
    sprintf(buf, "      <Y>%d</Y>\n", ls_y);
    fputs(buf,                                   fp_out);
    fputs("    </StartPoint>\n",                 fp_out);
    fputs("    <EndPoint>\n",                    fp_out);
    sprintf(buf, "      <X>%d</X>\n", le_x);
    fputs(buf,                                   fp_out);
    sprintf(buf, "      <Y>%d</Y>\n", le_y);
    fputs(buf,                                   fp_out);
    fputs("    </EndPoint>\n",                   fp_out);
    sprintf(buf, "    <GUID>f%07d-f%03d-f%03d-f%03d-f%011d</GUID>\n", guid, guid, guid, guid, guid);
    fputs(buf,                                   fp_out);
    fputs("    <LineType>Straight</LineType>\n", fp_out);
    fputs("  </anyType>\n",                      fp_out);
    ls_x += 72;
    le_x += 72;
    guid ++;
  }
  // create position
  for (i=0; i < angle_y; i++) {
    fputs("  <anyType xsi:type=\"ActivityData\">\n",                        fp_out);
    sprintf(buf, "    <Name>Pos%d</Name>\n", i+1);
    fputs(buf,                                                            fp_out);
    sprintf(buf, "    <GUID>a%07d-1111-1111-1111-111111111111</GUID>\n", guid);
    fputs(buf,                                                            fp_out);
    fputs("    <Location>\n",                                             fp_out);
    sprintf(buf, "      <X>%d</X>\n", pos_x);
    fputs(buf,                                                            fp_out);
    fputs("      <Y>72</Y>\n",                                            fp_out);
    fputs("    </Location>\n",                                            fp_out);
    fputs("    <Size>\n",                                                 fp_out);
    fputs("      <Width>64</Width>\n",                                    fp_out);
    fputs("      <Height>48</Height>\n",                                  fp_out);
    fputs("    </Size>\n",                                                fp_out);
    sprintf(buf, "    <Text>Pos%d</Text>\n", i+1);
    fputs(buf,                                                            fp_out);
    fputs("    <BackColorText>buttonface</BackColorText>\n",              fp_out);
    if (i == 0) { // 1st connect
      fputs("    <MotionFlag>Start</MotionFlag>\n",                       fp_out);
    } else {
      fputs("    <MotionFlag>None</MotionFlag>\n",                        fp_out);
    }
    fputs("    <PriorityList>\n",                                         fp_out);
    fputs("      <anyType xsi:type=\"xsd:string\">00 00 00 00</anyType>\n", fp_out);
    fputs("    </PriorityList>\n",                                        fp_out);
    fputs("    <BaseName>Pos</BaseName>\n",                               fp_out);
    fputs("    <Description>set position</Description>\n",                 fp_out);
    fputs("    <Group>Position</Group>\n",                                fp_out);
    fputs("    <Path />\n",                                               fp_out);
    if (i == 0) { // 1st connect
      // ConnectedGuids: up, down, left, right
      sprintf(buf, "    <ConnectedGuids>00000000-0000-0000-0000-000000000000,00000000-0000-0000-0000-000000000000,00000000-0000-0000-0000-000000000000,f%07d-f%03d-f%03d-f%03d-f%011d</ConnectedGuids>\n", i+1, i+1, i+1, i+1, i+1);
      fputs(buf,                                                          fp_out);
      // ConnectTypes: up, down, left, right (BeginConnect/EndConnect/NotConnected)
      fputs("    <ConnectTypes>NotConnected,NotConnected,NotConnected,BeginConnect</ConnectTypes>\n", fp_out);
    } else if (i == (angle_y-1)) {
      sprintf(buf, "    <ConnectedGuids>00000000-0000-0000-0000-000000000000,00000000-0000-0000-0000-000000000000,f%07d-f%03d-f%03d-f%03d-f%011d,00000000-0000-0000-0000-000000000000</ConnectedGuids>\n", i, i, i, i, i);
      fputs(buf,                                                          fp_out);
      fputs("    <ConnectTypes>NotConnected,NotConnected,EndConnect,NotConnected</ConnectTypes>\n", fp_out);
    } else {
      sprintf(buf, "    <ConnectedGuids>00000000-0000-0000-0000-000000000000,00000000-0000-0000-0000-000000000000,f%07d-f%03d-f%03d-f%03d-f%011d,f%07d-f%03d-f%03d-f%03d-f%011d</ConnectedGuids>\n", i, i, i, i, i, i+1, i+1, i+1, i+1, i+1);
      fputs(buf,                                                          fp_out);
      fputs("    <ConnectTypes>NotConnected,NotConnected,EndConnect,BeginConnect</ConnectTypes>\n", fp_out);
    }
    fputs("    <ProgramCode>\n",                                          fp_out);
    fputs("      <anyType xsi:type=\"xsd:string\">2B 10 3D F3 3F 00 00 ", fp_out);
    calc_sum = 0x2b + 0x10 + 0x3d + 0xf3 + 0x3f;

    sprintf(buf, "%02X ", angle[i][TIME] / 15);
    fputs(buf,                                                            fp_out);
    calc_sum += (angle[i][TIME] / 15) & 0xff;

    set_angle(fp_out, HEAD,    angle[i][HEAD],    &calc_sum);
    set_angle(fp_out, SHO_P_L, angle[i][SHO_P_L], &calc_sum);
    set_angle(fp_out, SHO_P_R, angle[i][SHO_P_R], &calc_sum);
    set_angle(fp_out, SHO_R_L, angle[i][SHO_R_L], &calc_sum);
    set_angle(fp_out, SHO_R_R, angle[i][SHO_R_R], &calc_sum);
    set_angle(fp_out, ELB_L,   angle[i][ELB_L],   &calc_sum);
    set_angle(fp_out, ELB_R,   angle[i][ELB_R],   &calc_sum);
    set_angle(fp_out, HIP_R_L, angle[i][HIP_R_L], &calc_sum);
    set_angle(fp_out, HIP_R_R, angle[i][HIP_R_R], &calc_sum);
    set_angle(fp_out, HIP_P_L, angle[i][HIP_P_L], &calc_sum);
    set_angle(fp_out, HIP_P_R, angle[i][HIP_P_R], &calc_sum);
    set_angle(fp_out, KNE_P_L, angle[i][KNE_P_L], &calc_sum);
    set_angle(fp_out, KNE_P_R, angle[i][KNE_P_R], &calc_sum);
    set_angle(fp_out, ANK_P_L, angle[i][ANK_P_L], &calc_sum);
    set_angle(fp_out, ANK_P_R, angle[i][ANK_P_R], &calc_sum);
    set_angle(fp_out, ANK_R_L, angle[i][ANK_R_L], &calc_sum);
    set_angle(fp_out, ANK_R_R, angle[i][ANK_R_R], &calc_sum);

    sprintf(buf, "%02X</anyType>\n", calc_sum & 0xff);
    fputs(buf,                                                            fp_out);

    fputs("    </ProgramCode>\n",                                         fp_out);
    fputs("  </anyType>\n",                                               fp_out);
    pos_x += 72;
    guid ++;
  }
  // create footer
  fputs("</ArrayOfAnyType>\n", fp_out);

  /* fin */
  printf("FINISH!\n");

  fclose(fp_in);
  fclose(fp_out);
  return (0);
}