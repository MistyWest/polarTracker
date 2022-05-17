/*
 * argos_pass_prediction.h
 *
 *  Created on: Oct 29, 2019
 *      Author: klockwood
 */

#ifndef ARTIC_ARGOS_PASS_PREDICTION_H_
#define ARTIC_ARGOS_PASS_PREDICTION_H_

#define MIN_ARGOS(A,B) ((A)<(B)?(A):(B))
#define MAX_ARGOS(A,B) ((A)>(B)?(A):(B))
#define abs(n)   ((n)>=0?(n):(-n))

#define MAXLU       132
#define exit_error  -1

typedef struct
{
float pf_lon;       /* geodetic position of the beacon */
float pf_lat;       /* geodetic position of the beacon */
int   year_start;   /* beginning of prediction */
int   month_start;  /* beginning of prediction */
int   jour_start;   /* beginning of prediction */
int   hour_start;   /* beginning of prediction */
int   min_start;    /* beginning of prediction */
int   sec_start;    /* beginning of prediction */
int   year_end;     /* end of prediction */
int   month_end;    /* end of prediction */
int   day_end;      /* end of prediction */
int   hour_end;     /* end of prediction */
int   min_end;      /* end of prediction */
int   sec_end;      /* end of prediction */
int   s_differe;    /* on differe (min.) */
float site_min_requis;    /* min site pour calcul de la duration (deg.) */
float site_max_requis;    /* min site (deg.) */
float margin_temporelle;  /* margin temporelle (min/6mois) */
float margin_geog_lat;    /* margin geographique (deg) */
float margin_geog_lon;    /* margin geographique (deg) */
int   Npass_max;          /* number of satellite passes */
}pc_t;

typedef struct
{
char  sat[2];
int   year_bul;  /* bulletin epoch */
int   month_bul; /* bulletin epoch */
int   day_bul;   /* bulletin epoch */
int   hour_bul;  /* bulletin epoch */
int   min_bul;   /* bulletin epoch */
int   sec_bul;   /* bulletin epoch */
float dga;       /* semi-major axis (km) */
float inc;       /* orbit inclination (deg) */
float lon_asc;   /* longitude of ascending node (deg) */
float d_node;    /* asc. node drift during one revolution (deg) */
float ts;        /* orbit period (min) */
float dgap;      /* drift of semi-major axis (m/jour) */
}po_t;

typedef struct
{
char  sat[2];
long  tpp;       /* date of the next pass (sec90) */
int   year_pp;   /* bulletin du proch. pass */
int   month_pp;  /* bulletin du proch. pass */
int   day_pp;    /* bulletin du proch. pass */
int   hour_pp;   /* bulletin du proch. pass */
int   min_pp;    /* bulletin du proch. pass */
int   sec_pp;    /* bulletin du proch. pass */
int   duration;  /* duration (sec) */
int   site_max;  /* site max dans le pass (deg) */
}pp_t;


typedef struct element
{
    pp_t val;
    struct element *next;
}element_t;

typedef element_t * llist;

/**
 * @brief - Traverses Linked List and adds new_value to end
 */
void add_element_to_end( llist* liste, pp_t new_value );


#endif /* ARTIC_ARGOS_PASS_PREDICTION_H_ */
