#include<math.h> 
#include<iostream> 
#include<stdio.h> 
#include<stdlib.h> 
#include <array>

using namespace std; 

namespace
{
#define PI 3.1415926 
 
 //定义全局变量 
int days_of_month_1[]={31,28,31,30,31,30,31,31,30,31,30,31}; 
int days_of_month_2[]={31,29,31,30,31,30,31,31,30,31,30,31}; 
long double h=-0.833; 

//  //输入日期 
// void input_date(int c[])
// { 
//      int i; 
//      cout<<"Enter the date (form: 2009 03 10):"<<endl; 
//      for(i=0;i<3;i++)
//      { 
//          cin>>c[i]; 
//      } 
//  } 
 
//  //输入纬度 
// void input_glat(int c[])
// { 
//      int i; 
//      cout<<"Enter the degree of latitude(range: 0°- 60°,form: 40 40 40 (means 40°40′40″)):"<<endl; 
//      for(i=0;i<3;i++)
//      { 
//          cin>>c[i]; 
//      } 
//  } 

//  //输入经度 
// void input_glong(int c[])
// { 
//      int i; 
//      cout<<"Enter the degree of longitude(west is negativ,form: 40 40 40 (means 40°40′40″)):"<<endl; 
//      for(i=0;i<3;i++)
//      { 
//          cin>>c[i]; 
 
//      } 
//  } 
 
 //判断是否为闰年:若为闰年，返回1；若非闰年，返回0 
int leap_year(int year)
{ 
     if(((year%400==0) || (year%100!=0) && (year%4==0))) return 1; 
     else return 0; 
 } 
 
 //求从格林威治时间公元2000年1月1日到计算日天数days 
 int days(int year, int month, int date)
 { 
     int i,a=0; 
     for(i=2000;i<year;i++)
     { 
         if(leap_year(i)) a=a+366; 
         else a=a+365; 
     } 
 
     if(leap_year(year))
     { 
         for(i=0;i<month-1;i++)
         { 
             a=a+days_of_month_2[i]; 
         } 
     } 
     else 
     { 
         for(i=0;i<month-1;i++)
         { 
             a=a+days_of_month_1[i]; 
         } 
     } 
     a=a+date; 
     return a; 
 } 
 
 //求格林威治时间公元2000年1月1日到计算日的世纪数t 
 long double t_century(int days, long double UTo){ 
 
     return ((long double)days+UTo/360)/36525; 
 } 
 
 //求太阳的平黄径 
 long double L_sun(long double t_century)
 { 
     return (280.460+36000.770*t_century); 
 } 
 
 //求太阳的平近点角 
long double G_sun(long double t_century)
{ 
     return (357.528+35999.050*t_century); 
 } 
 
 //求黄道经度 
long double ecliptic_longitude(long double L_sun,long double G_sun)
{ 
     return (L_sun+1.915*sin(G_sun*PI/180)+0.02*sin(2*G_sun*PI/180)); 
 } 
 
 //求地球倾角 
long double earth_tilt(long double t_century)
{ 
     return (23.4393-0.0130*t_century); 
 } 
 
 //求太阳偏差 
long double sun_deviation(long double earth_tilt, long double ecliptic_longitude)
{ 
     return (180/PI*asin(sin(PI/180*earth_tilt)*sin(PI/180*ecliptic_longitude))); 
 } 
 
 //求格林威治时间的太阳时间角GHA 
long double GHA(long double UTo, long double G_sun, long double ecliptic_longitude)
{ 
     return (UTo-180-1.915*sin(G_sun*PI/180)-0.02*sin(2*G_sun*PI/180)+2.466*sin(2*ecliptic_longitude*PI/180)-0.053*sin(4*ecliptic_longitude*PI/180)); 
 } 
 
 //求修正值e 
 long double e(long double h, long double glat, long double sun_deviation)
 { 
     return 180/PI*acos((sin(h*PI/180)-sin(glat*PI/180)*sin(sun_deviation*PI/180))/(cos(glat*PI/180)*cos(sun_deviation*PI/180))); 
 } 
 
 //求日出时间 
 long double UT_rise(long double UTo, long double GHA, long double glong, long double e)
 { 
     return (UTo-(GHA+glong+e)); 
 } 
 
 //求日落时间 
long double UT_set(long double UTo, long double GHA, long double glong, long double e)
{ 
     return (UTo-(GHA+glong-e)); 
 } 
 
 //判断并返回结果（日出） 
long double result_rise(long double UT, long double UTo, long double glong, long double glat, int year, int month, int date)
{ 
     long double d; 
     if(UT>=UTo) d=UT-UTo; 
     else d=UTo-UT; 
     if(d>=0.1) 
     { 
         UTo=UT; 
         UT=UT_rise(UTo,GHA(UTo,G_sun(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))))); 
         result_rise(UT,UTo,glong,glat,year,month,date); 
     } 
     return UT; 
 } 
 
 //判断并返回结果（日落） 
long double result_set(long double UT, long double UTo, long double glong, long double glat, int year, int month, int date)
{ 
     long double d; 
     if(UT>=UTo) d=UT-UTo; 
     else d=UTo-UT; 
     if(d>=0.1)
     { 
         UTo=UT; 
         UT=UT_set(UTo,GHA(UTo,G_sun(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))))); 
         result_set(UT,UTo,glong,glat,year,month,date); 
     } 
     return UT; 
 } 
 
 //求时区 
int Zone(long double glong)
{ 
     if(glong>=0) return (int)((int)(glong/15.0)+1); 
     else return (int)((int)(glong/15.0)-1); 
 } 

 //打印结果 
void output(long double rise, long double set, long double glong)
{ 
     if((int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))<10) 
         cout<<"The time at which the sun rises is "<<(int)(rise/15+Zone(glong))<<":0"<<(int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))<<" .\n"; 
     else cout<<"The time at which the sun rises is "<<(int)(rise/15+Zone(glong))<<":"<<(int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))))<<" .\n"; 

     if((int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))<10) 
         cout<<"The time at which the sun sets is "<<(int)(set/15+Zone(glong))<<": "<<(int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))<<" .\n"; 
     else cout<<"The time at which the sun sets is "<<(int)(set/15+Zone(glong))<<":"<<(int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))))<<" .\n"; 
 } 
 
// int main()
// { 
//      long double UTo=180.0; 
//      int year,month,date; 
//      long double glat,glong; 
//      int c[3]; 

//      input_date(c); 
//      year=c[0]; 
//      month=c[1]; 
//      date=c[2]; 

//      input_glat(c); 
//      glat=c[0]+c[1]/60+c[2]/3600; 

//      input_glong(c); 
//      glong=c[0]+c[1]/60+c[2]/3600; 

//      long double rise,set; 
//      rise=result_rise(UT_rise(UTo,GHA(UTo,G_sun(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))))),UTo,glong,glat,year,month,date); 
//      set=result_set(UT_set(UTo,GHA(UTo,G_sun(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,date),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,date),UTo)),G_sun(t_century(days(year,month,date),UTo)))))),UTo,glong,glat,year,month,date); 

//      output(rise,set,glong); 
//      system("pause"); 
//      return 0; 
//  } 
}

std::array<int, 2> getSunTime(long double glat, long double glong, int year, int month, int day)
{
    long double UTo = 180.0; 
    long double  rise = result_rise(UT_rise(UTo,GHA(UTo,G_sun(t_century(days(year,month,day),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,day),UTo)),G_sun(t_century(days(year,month,day),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,day),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,day),UTo)),G_sun(t_century(days(year,month,day),UTo)))))),UTo,glong,glat,year,month,day); 
    long double set = result_set(UT_set(UTo,GHA(UTo,G_sun(t_century(days(year,month,day),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,day),UTo)),G_sun(t_century(days(year,month,day),UTo)))),glong,e(h,glat,sun_deviation(earth_tilt(t_century(days(year,month,day),UTo)),ecliptic_longitude(L_sun(t_century(days(year,month,day),UTo)),G_sun(t_century(days(year,month,day),UTo)))))),UTo,glong,glat,year,month,day); 

    std::array<int, 2> ret;
    ret[0] = (int)(rise/15+Zone(glong)) * 60 + (int)(60*(rise/15+Zone(glong)-(int)(rise/15+Zone(glong))));
    ret[1] = (int)(set/15+Zone(glong)) * 60 + (int)(60*(set/15+Zone(glong)-(int)(set/15+Zone(glong))));
    return ret;
}

// int main()
// {
//     auto ret = getSunTime(31.815051, 120.0127376, 2019, 12, 12);
//     cout<<"The time at which the sun rises is "<< ret[0] <<":"<< ret[1] <<" .\n"; 
//     cout<<"The time at which the sun sets is "<< ret[2] <<":"<< ret[3] <<" .\n"; 
// }
