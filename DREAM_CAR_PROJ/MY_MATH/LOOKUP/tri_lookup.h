#ifndef __TRI_LOOKUP_H
#define __TRI_LOOKUP_H

//sin值查表
//提供0-90°的sin数值表值，间隔0.1°
//超出该范围的角度值由角度变换获得
//输入为角度
float Sin_Lookup(float angle);



//cos值查表
//提供0-90°的cos数值表值，间隔0.1°
//超出该范围的角度值由角度变换获得
//输入为角度
float Cos_Lookup(float angle);



#endif

