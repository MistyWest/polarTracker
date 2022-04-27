/*
 * argos_pass_prediction.c
 *
 *  Created on: Oct 29, 2019
 *      Author: klockwood
 */


/*    *******************************************************    */
/*    *******************************************************    */
/*               Copyright (c) 1995 CLS                          */
/*    All rights reserved including the right of reproduction    */
/*    in whole or in part in any form.                           */
/*    *******************************************************    */
/*    *******************************************************    */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "argos_pass_prediction.h"

#define TEST
#define RECETTE

#define TRACE     0

#if TRACE
FILE  *ul_trace;
#endif

/* +-------------------------------------------------------------------+*/
/* +                      variables                            +*/
/* +-------------------------------------------------------------------+*/


/* +-------------------------------------------------------------------+*/
/* +                      C O N S T A N T S                          +*/
/* +-------------------------------------------------------------------+*/

const float pi  = 3.1415926535;   /* Pi     value   */
const float demi_pi = 1.570796327;          /* Pi/2   value   */
const float two_pi  = 6.283185307;          /* 2*pi   value   */
const float deg_rad = 0.017453292;          /* pi/180 value   */
const float rad_deg = 57.29577951;          /* 180/pi value   */

const int pas = 30;                        /* en (sec)   */
const float rt  = 6378.137;               /* Earth radius   */
const float rs  = 7200;                   /* Orbit radius   */

/*Tables of the number of days before each month of the year  */
/*    ek_quant (1...12,1) for leap years    */
/*    ek_quant (1...12,2) for non-leap years   */
/*    --------------------------------------------------------------- */

const long  ek_quanti[13][2] = {  {0,0},
          {31,31},
          {60,59},
          {91,90},
          {121,120},
          {152,151},
          {182,181},
          {213,212},
          {244,243},
          {274,273},
          {305,304},
          {335,334},
          {0,0}};

void su_distance( long t1,   /* input */
                  float x_pf,
                  float y_pf,
                  float z_pf,
                  float ws,
                  float sin_i,
                  float cos_i,
                  float asc_node,
                  float wt,
                  float *d2 );   /* output */

char su_date_jmahms_stu90 ( long  ev_day    ,
                            long  ev_month    ,
                            long  ev_year   ,
                            long  ev_hour   ,
                            long  ev_minute  ,
                            long  ev_seconds ,
                            long  *dv_sec90  );

char su_date_stu90_jmahms  ( long  dv_sec90   ,
                             long  *ev_day    ,
                             long  *ev_month    ,
                             long  *ev_year   ,
                             long  *ev_hour   ,
                             long  *ev_minute  ,
                             long  *ev_seconds );

void add_element_to_end(llist* liste, pp_t new_value);



void su_distance( long  t1,   /* input */
                 float x_pf,
                 float y_pf,
                 float z_pf,
                 float ws,
                 float sin_i,
                 float cos_i,
                 float asc_node,
                 float wt,
                 float *d2 )    /* output */
{
  float lat_sat; /*  latitude of the satellite beetween the a.n  */
  float long_sat_an; /*  longitude of the satellite beetween the a.n */
  float long_sat; /*  longitude of the satellite      */
  float x, y, z; /*  satellite position        */

  /* ...  calculation of the satellite latitude */
  lat_sat = asin(sin(ws * (float) (t1)) * sin_i);

  /* ...  calculation of the satellite longitude */
  long_sat_an = atan(tan(ws * (float) (t1)) * cos_i);

  /*
   printf(" pi %lf \n", pi);
   */

  if ( cos(ws * t1) < 0. )
  {
    long_sat_an = long_sat_an + pi;
  }

  long_sat = asc_node + long_sat_an + wt * (float) (t1);
  long_sat = fmod(long_sat, two_pi);

  if ( long_sat < 0. )
  {
    long_sat = long_sat + two_pi;
  }

  /* ...  spheric satellite positions calculation in TR */
  x = cos(lat_sat) * cos(long_sat);
  y = cos(lat_sat) * sin(long_sat);
  z = sin(lat_sat);

  *d2 = (x - x_pf) * (x - x_pf) + (y - y_pf) * (y - y_pf) + (z - z_pf) * (z - z_pf);
}


/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | OPERATION : SU_DATE_JMAHMSM_STU90                                 |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | Role : Date conversion (day,month,year,hour,minute,second) to     |*/
/* |        (seconds since 01/01/1990 00:00:00)                        |*/
/* |                                                                   |*/
/* |                                                                   |*/
/* | Input parameters                                                  |*/
/* | ----------------                                                  |*/
/* |    ev_day   : Days           (1<= day <=31)                       |*/
/* |    ev_month  : Month         (1<= month <= 12)                    |*/
/* |    ev_year  : Year           ( > 1900)                            |*/
/* |    ev_hour  : Hour           (0 <= hour <= 23)                    |*/
/* |    ev_minute : Minute        (0 <= minute <= 59)                  |*/
/* |    ev_seconds: Secondes      (0 <= seconds <= 59)                 |*/
/* |                                                                   |*/
/* | Donnees en sortie                                                 |*/
/* | -----------------                                                 |*/
/* |                                                                   |*/
/* |    dv_sec_90 : Number of whole seconds since the                  |*/
/* |                01-Jan-1990 00:00:00                               |*/
/* |    jv_status : execution report                                   |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/
char su_date_jmahms_stu90 ( long  ev_day    ,
                            long  ev_month    ,
                            long  ev_year   ,
                            long  ev_hour   ,
                            long  ev_minute  ,
                            long  ev_seconds ,
                            long  *dv_sec90  )
{

  /* +-------------------------------------------------------------------+*/
  /* +                  V A R I A B L E S   L O C A L                    +*/
  /* +-------------------------------------------------------------------+*/

  long jv_nb_year; /* number of years since 1990 */
  long jv_nb_leap_years; /* number of leap years   */
  long jv_nb_day; /* number # of days since 1990 */
  long jv_ind_bis; /* current year indicator */

  /* +-------------------------------------------------------------------+*/
  /* +                            C O D E                                +*/
  /* +-------------------------------------------------------------------+*/

  /*    START SU_DATE_JMAHMSM_STU90 */
  /*    --------------------------- */

  /* Test if 1 <= ev_month <= 12 */
  if ( (ev_month > 0) && (ev_month < 13) )
  {

    /* CALCULATION OF THE NUMBER OF DAYS SINCE 1990 AND THE END OF  */
    /* THE PREVIOUS YEAR           */

    /* Number of years  */
    jv_nb_year = ev_year - 1990;

    /* Number of leap years since 1990 */
    jv_nb_leap_years = (ev_year - 1 - 1900) / 4 - 22;
    jv_nb_day = (jv_nb_year * 365) + jv_nb_leap_years;


    /* CALCULATION OF THE NUMBER OF DAYS SINCE THE BEGINNING OF THE YEAR */

    /* We look at whether the current year is a leap year   */
    jv_ind_bis = MIN_ARGOS((fmod(ev_year, 4) + 1), 2);
    jv_nb_day = jv_nb_day + ek_quanti[ev_month - 1][jv_ind_bis - 1] + ev_day - 1;

    /* CALCULATION OF THE NUMBER OF SECONDS SINCE 1990     */
    *dv_sec90 = jv_nb_day * 86400 + ev_hour * 3600 + ev_minute * 60 + ev_seconds;

    return (0);
  }
  else
  {
    return (1);
  }

  /*    END SU_DATE_JMAHMSM_STU90 */
}



/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | OPERATION : SU_DATE_STU90_JMAHMS                                  |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/
/* |                                                                   |*/
/* | Role : Date conversion  (seconds since 01/01/1990 00:00:00) to    |*/
/* |        (day,month,year,hour,minute,second)                        |*/
/* |                                                                   |*/
/* |                                                                   |*/
/* | Parametres en entree                                              |*/
/* | --------------------                                              |*/
/* |     dv_sec90 : Nombre de secondes ecoulees depuis le 01-Jan-1990  |*/
/* |                00:00:00                                           |*/
/* |                                                                   |*/
/* | Donnees en sortie                                                 |*/
/* | -----------------                                                 |*/
/* |    ev_day    : Day            (1<= day <=31)                      |*/
/* |    ev_month  : Month          (1<= month <= 12)                   |*/
/* |    ev_year  : Year            ( > 1900)                           |*/
/* |    ev_hour  : Heure           (0 <= hour <= 23)                   |*/
/* |    ev_minute : Minute         (0 <= minute <= 59)                 |*/
/* |    ev_seconds: Seconds        (0 <= seconde <= 59)                |*/
/* |    jv_status : execution report                                   |*/
/* |                                                                   |*/
/* +-------------------------------------------------------------------+*/

char su_date_stu90_jmahms( long dv_sec90,
                           long *ev_day,
                           long *ev_month,
                           long *ev_year,
                           long *ev_hour,
                           long *ev_minute,
                           long *ev_seconds )
{

  /* +-------------------------------------------------------------------+*/
  /* +                  V A R I A B L E S   L O C A L E S                +*/
  /* +-------------------------------------------------------------------+*/

  long jv_nb_day; /* number of days   */
  long jv_nb_year; /* number of years */

  long ev_nb_leap_years;/* number of leap years   */
  long ev_ind_leap_year; /* Indicate Leap year     */

  long iv_trav; /* working variable */
  float dv_trav; /* working variable  */

  /* +-------------------------------------------------------------------+*/
  /* +                            C O D E                                +*/
  /* +-------------------------------------------------------------------+*/

  if ( dv_sec90 >= 0 )
  {
    /*  CALCULATION OF THE NUMBER OF DAYS SINCE 1990  */

    jv_nb_day = dv_sec90 / 86400;

    /*    CALCULATION HOUR  */
    iv_trav = dv_sec90 - jv_nb_day * 86400;
    *ev_hour = iv_trav / 3600;

    /*    CALCULATION MINUTE */
    iv_trav = iv_trav - *ev_hour * 3600;
    *ev_minute = iv_trav / 60;

    /*    CALCULATION SECOND  */
    iv_trav = iv_trav - *ev_minute * 60;
    *ev_seconds = iv_trav;


    /* ADJUSTMENT NB_DAYS HOUR MINUTE SECOND IF THE NUMBER */
    /* OF MICROSECONDES CALCULATED EAST> = 1.D6      */
    if ( *ev_seconds >= 60 )
    {
      *ev_seconds = *ev_seconds - 60;
      *ev_minute = *ev_minute + 1;
      if ( *ev_minute >= 60 )
      {
        *ev_minute = *ev_minute - 60;
        *ev_hour = *ev_hour + 1;
        if ( *ev_hour >= 24 )
        {
          *ev_hour = *ev_hour - 24;
          jv_nb_day = jv_nb_day + 1;
        }
      }
    }


    /* CALCULATION NUMBER OF YEARS SINCE 1990 */
    dv_trav = (float) (jv_nb_day);
    dv_trav = (dv_trav + 0.5) / 365.25;
    jv_nb_year = (int) (dv_trav);


    /* CALCULATION OF YEAR */
    *ev_year = jv_nb_year + 1990;


    /* CALCULATION OF THE NUMBER OF LEAP YEARS SINCE 1990 */
    ev_nb_leap_years = (*ev_year - 1 - 1900) / 4 - 22;


    /* Test if current leap year   */
    ev_ind_leap_year = MIN_ARGOS((fmod(*ev_year, 4) + 1), 2);

    /* CALCULATION OF THE NUMBER OF DAYS REMAINING IN THE YEAR  */
    jv_nb_day = jv_nb_day - (jv_nb_year * 365) - ev_nb_leap_years + 1;


    /* CALCULATION OF THE REMAINING MONTHS IN THE YEAR */
    *ev_month = 1;
    while ( (jv_nb_day > ek_quanti[*ev_month - 1][ev_ind_leap_year - 1]) && (*ev_month <= 12) )
    {
      *ev_month = *ev_month + 1;
    }
    /* END WHILE  */


    *ev_month = *ev_month - 1;

    /* CALCULATION OF THE NUMBER OF DAYS IN THE MONTH */
    *ev_day = jv_nb_day - ek_quanti[*ev_month - 1][ev_ind_leap_year - 1];

    return (0);
  }
  else
  {
    return (1);
  }

  /*    END SU_DATE_STU90_JMAHMSM */
  /*    ------------------------- */
}


/**
 * @brief - Traverses Linked List and adds new_value to end
 */
void add_element_to_end( llist* liste, pp_t new_value )
{
  /* Create a new element_t */
  element_t* new_element = malloc(sizeof(element_t));

  /* We assign the value to the new element_t */
  new_element->val = new_value;
  new_element->next = NULL;

  /* If this is the first element */
  if(*liste == NULL)
  {
    *liste = new_element;
    return;
  }

  /* Go through the list using a temporary pointer and we
   indicates that the last item in the list is connected to the new item*/
  llist current = *liste;
  while ( current->next != NULL )
  {
    /* go to next item */
    current = (element_t*) current->next;
  }

  /* Create a new element_t */
  current->next = malloc(sizeof(element_t));

  /* We assign the value to the new element_t */
  current->next->val = new_value;
  current->next->next = NULL;

}




/**
 * @brief - Main Pass Prediction Function
 */
int prepas ( pc_t    * p_pc,
             po_t    * p_po,
             llist * listePP )
{
  /* -----------------------------------------------------------------
   C
   C Satellite passes prediction
   C Circular method taking into account drag coefficient (friction effect)
   C and J2 term of earth potential
   C
   C       ephemeris calculation
   C
   C
   C Author  : J.P. Malarde
   C Company : C.L.S.
   C Issue   : 1.0 25/09/97  first issue
   C             2.0   10/02/14    Sharc version
   C
   C -----------------------------------------------------------------*/

#if TRACE
  char nf_trace [] = "trace.dat";
#endif

  pp_t ppItem;

  long tpp;
  int duration;
  int duration_passage_MC; /* duration du passage Marges Comprises */

  long year_pp; /* date of next pass */
  long month_pp; /* date of next pass */
  long day_pp; /* date of next pass */
  long hour_pp; /* date of next pass */
  long min_pp; /* date of next pass */
  long sec_pp; /* date of next pass */

  char istatus;
  long s_start = 0; /* beginning of prediction (sec) */
  long s_end = 0; /* end of prediction (sec) */
  long k; /* number of revolution */
  long s_bul = 0; /* bulletin epoch (sec) */
  long t0; /* date (sec) */
  long t1; /* date (sec) */
  long t2; /* date (sec) */
  long tmil[8]; /* date milieu du prochain passage/satellite */
//long  t_sel;

  int isat;
  int step = 0;

  int site; /* min site */
  int passage;
  float d2;
  float d2_min;
  float d2_mem;
  float d2_mem_mem;
  float temp;
  float v;

  int duration_passage;
  long date_mid_pass;
//long date_debut_passage;
  int site_max_passage;

  int site_max; /* min site */
  int margin;
  int table_site_max[8]; /* DB */
  int table_duration[8]; /* DB */
  float delta_lon; /* asc.node drift during one revolution (deg) */
  float wt; /* earth rotation */

  float visi_min; /* visibility at site min*/
  float visi_max; /* visibility at site_max */

  float ws0; /* mean anomaly */
  float d_ws; /* friction effect */
  float ws; /* mean anomaly with friction effect */
  float sin_i, cos_i; /* sin, cos of orbit inclination */
  float asc_node; /* longitude of ascending node in terrestrial frame */
  float x_pf; /* beacon position */
  float y_pf;
  float z_pf;

  float v_lon;
  float v_lat;
  int v_differe;
  float v_site_max_requis;
  float v_ts;
  float v_margin_temporelle;
  float v_margin_geog_lat;
  float v_margin_geog_lon;
  float v_dgap;
  int Npass; /* number of passes */
  int Npass_max; /* max number of passes per sateliite */

  float v_site_min_requis;

  memset(tmil, 0, sizeof(tmil));
  memset(table_site_max, 0, sizeof(table_site_max));
  memset(table_duration, 0, sizeof(table_duration));

  /* ...  EEPROM --> RAM transfert  */
  /*  ------------------------  */

  v_lon = p_pc[0].pf_lon;
  v_lat = p_pc[0].pf_lat;
  v_differe = p_pc[0].s_differe;
  v_differe *= 1; /* keep compiler happy */

  v_site_max_requis = p_pc[0].site_max_requis;
  v_site_min_requis = p_pc[0].site_min_requis;
  v_margin_temporelle = p_pc[0].margin_temporelle;
  v_margin_geog_lat = p_pc[0].margin_geog_lat;
  v_margin_geog_lon = p_pc[0].margin_geog_lon;
  Npass_max = p_pc[0].Npass_max;

  /* ...  input parameter conversion  */
  /*  --------------------------- */

  v_lon = v_lon * deg_rad;
  v_lat = v_lat * deg_rad;

  x_pf = cos(v_lat) * cos(v_lon);
  y_pf = cos(v_lat) * sin(v_lon);
  z_pf = sin(v_lat);

#ifdef TEST
  printf(" Prepas : v_site_min_requis %lf \n", v_site_min_requis);
  printf(" Prepas : v_site_max_requis %lf \n", v_site_max_requis);
#endif

  v_site_min_requis = v_site_min_requis * deg_rad;
  visi_min = demi_pi - v_site_min_requis - asin(rt / rs * cos(v_site_min_requis));
  visi_min = 2. * sin(visi_min / 2.);
  visi_min = visi_min * visi_min;

  visi_max = v_site_max_requis * deg_rad;
  visi_max = demi_pi - visi_max - asin(rt / rs * cos(visi_max));
  visi_max = 2. * sin(visi_max / 2.);
  visi_max = visi_max * visi_max;

#ifdef TEST
  printf(" Prepas : visi_min %lf \n", visi_min);
  printf(" Prepas : visi_max %lf \n", visi_max);
#endif

#if TRACE
  ul_trace = fopen(nf_trace, "w");
#endif

  istatus = su_date_jmahms_stu90(p_pc[0].hour_start, p_pc[0].month_start, p_pc[0].year_start, p_pc[0].hour_start, p_pc[0].min_start, p_pc[0].sec_start, &s_start);

  istatus = su_date_jmahms_stu90(p_pc[0].day_end, p_pc[0].month_end, p_pc[0].year_end, p_pc[0].hour_end, p_pc[0].min_end, p_pc[0].sec_end, &s_end);

  /*  margin_temporelle(min/6mois) *60/(6*30*86400) */
  /*  margin_geog_lat(deg.) *110/(2*7)              */
  /*  margin_geog_lon(deg.) = 116*60                */

  v_margin_temporelle = v_margin_temporelle + 5.;
  v_margin_temporelle = v_margin_temporelle / 259200.;

  v_margin_geog_lat = v_margin_geog_lat * 7.857142;
  v_margin_geog_lon = 0;

  /* ...  reading of OP */

  isat = 1;
//  t_sel = 999999999;

  while ( ((isat - 1) < 8) && (strncmp(p_po[isat - 1].sat, "  ", 2) != 0) )
  {

    sin_i = sin(p_po[isat - 1].inc * deg_rad);
    cos_i = cos(p_po[isat - 1].inc * deg_rad);

    v_dgap = p_po[isat - 1].dgap / 1000; /*  conversion m/jr --> km/jr */

    istatus = su_date_jmahms_stu90( p_po[isat - 1].day_bul,
                                    p_po[isat - 1].month_bul,
                                    p_po[isat - 1].year_bul,
                                    p_po[isat - 1].hour_bul,
                                    p_po[isat - 1].min_bul,
                                    p_po[isat - 1].sec_bul, &s_bul );
    v_ts = p_po[isat - 1].ts * 60.;
    ws0 = two_pi / v_ts; /* tour/sec */
    delta_lon = p_po[isat - 1].d_node * deg_rad; /* distance between 2 asc nodes */
    wt = delta_lon / v_ts; /* tour/secondes */
    asc_node = p_po[isat - 1].lon_asc * deg_rad;

    /* ... search for the next pass whose max duration > max duration required */

    Npass = 0;
    passage = 0;
    site = 0;
    duration = 0;
    site_max = 0;
    d2_min = 999.;
    t1 = s_start - s_bul;
    t2 = s_end - s_bul;

#ifdef TEST
    printf(" Prepas : satellite %s ", p_po[isat - 1].sat);
    printf(" Prepas : date BO %ld ", s_bul);
    printf(" Prepas : s_start %ld ", s_start);
    printf(" s_end %ld ", s_end);
    printf(" t1 %ld ", t1);
    printf(" t2 %ld ", t2);
    printf(" Npass_max %d ", Npass_max);
    printf("\n");
    printf(" istatus is %c", istatus);
#endif

    t0 = t1;
    t0 *= 1; /* Keep Compiler Happy */

    k = (long) (t1 / v_ts);
    d_ws = -.75 * (v_dgap / p_po[isat - 1].dga / 86400.) * two_pi * k;
    ws = ws0 + d_ws;

    /* ... we skip the current passage */

    su_distance(t1, x_pf, y_pf, z_pf, ws, sin_i, cos_i, asc_node, wt, &d2);

    d2_mem = d2;
    if ( d2 < visi_min )
      t1 = t1 + 4500;

    while ( (t1 < t2) && (Npass < Npass_max) )
    {

#if TRACE
      fprintf(ul_trace," passage step date duration site d2 %d %d %ld %d %d %d %lf \n\n", passage, step, (t1-t0), duration, site, site_max, d2);
#endif

      d2_mem_mem = d2_mem;
      d2_mem_mem *= 1; /* Keep compiler happy */

      d2_mem = d2;

      su_distance(t1, x_pf, y_pf, z_pf, ws, sin_i, cos_i, asc_node, wt, &d2);

      if ( d2 < visi_min )
      {

        passage = 1;

//         printf("********* Visibility ***********\n");

        duration = duration + pas;
        v = 2. * asin(sqrt(d2) / 2.);
        temp = (rs * sin(v)) / sqrt(rt * rt + rs * rs - 2 * rt * rs * cos(v));
        temp = rad_deg * acos(temp);
        site = (int) (temp);
        if ( site > site_max )
          site_max = site;
        if ( d2 < d2_min )
          d2_min = d2;

        step = pas;
      }
      else
      {
//         printf("********* pass out of sight ******");
        if ( passage == 1 )
        {
//          printf("****** we just come out of a visibility ***********\n");

          if ( site_max < v_site_max_requis )
          {
            // store when leaving visibility
            duration_passage = duration;
            date_mid_pass = s_bul + t1 - duration / 2;
//                date_debut_passage = date_mid_pass - duration/2;
            site_max_passage = site_max;

            Npass += 1;

            istatus = su_date_stu90_jmahms(date_mid_pass, &day_pp, &month_pp, &year_pp, &hour_pp, &min_pp, &sec_pp);

            // print info about pass

#ifdef RECETTE
            printf("\n\n");
            printf(" Prepas : satellite number = %s\n", p_po[isat - 1].sat);
            printf(" Prepas : pass number %d\n", Npass);
            istatus = su_date_stu90_jmahms(date_mid_pass, &day_pp, &month_pp, &year_pp, &hour_pp, &min_pp, &sec_pp);
            printf(" Prepas : date of pass middle  %4ld %02ld %02ld %02ld:%02ld:%02ld \n", year_pp, month_pp, day_pp, hour_pp, min_pp, sec_pp);
            printf(" Prepas : site_max = %d \n", site_max_passage); /* DB */
            printf(" Prepas : duration (min.)= %d \n", duration_passage / 60); /* DB */
            printf("\n");

#endif
            /* ...  Output data :         */
            /*  - date of the beginning of the next pass (sec)   */
            /*  - duration of the next pass, including margin (sec)  */

            temp = (int) (v_margin_temporelle * t1 + v_margin_geog_lat + v_margin_geog_lon);
            margin = (int) (temp);
            tpp = date_mid_pass - duration_passage / 2 - margin;
            duration_passage_MC = duration_passage + 2 * margin;

#ifdef RECETTE
            printf(" Prepas : data output of the program (margins included) \n");
            printf(" Prepas : time margin            : %f  \n", v_margin_temporelle);
            printf(" Prepas : t1                          : %ld \n", t1);
            printf(" Prepas : next start date : %ld \n", tpp);
            printf(" Prepas : pass duration (min)      : %ld \n", (long) duration_passage_MC / 60);
            printf(" Prepas : of which one of margin (sec)     : %ld \n", (long) margin);
#endif

            istatus = su_date_stu90_jmahms(tpp, &day_pp, &month_pp, &year_pp, &hour_pp, &min_pp, &sec_pp);

            strcpy(ppItem.sat, p_po[isat - 1].sat);
            ppItem.tpp = tpp;
            ppItem.year_pp = year_pp;
            ppItem.month_pp = month_pp;
            ppItem.day_pp = day_pp;
            ppItem.hour_pp = hour_pp;
            ppItem.min_pp = min_pp;
            ppItem.sec_pp = sec_pp;
            ppItem.duration = duration_passage_MC;
            ppItem.site_max = site_max_passage;
            add_element_to_end(listePP, ppItem);

            passage = 2;
          }
          else
          {
//          printf("********* site max > site max required ***********\n");
            passage = 0;
          }

          site_max = 0;
          duration = 0;
          step = 4500;    // 75 min
          d2_min = 999999.;

        }
        else
        {
//            printf("********* we do not just come out of a visibility***********\n");
          /* we adjust the pace according to the distance */

          if ( d2 < (4 * visi_min) )
            step = pas;
          if ( (d2 >= (4 * visi_min)) && (d2 <= (16 * visi_min)) )
            step = 4 * pas;
          if ( d2 > (16 * visi_min) )
            step = 20 * pas;
        }
      }

      t1 = t1 + step;

    } /* END WHILE in the time */

    isat = isat + 1;

  } /* reading of each satellite bulletin */

#if TRACE
  fclose (ul_trace);
#endif

  return 0;
}



