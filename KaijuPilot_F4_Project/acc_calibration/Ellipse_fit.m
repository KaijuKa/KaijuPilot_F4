%              X0 Y0 Z0    A B C
%椭圆中心点坐标x  y  z 参数a b c

%data_x    data_y    data_z
%x轴数据列 y轴数据列 z轴数据列

%普通形式：((x-x0)/A)^2 + ((y-y0)/B)^2 + ((z-z0)/C)^2 = 1
%一般形式：x^2 + a*y^2 + b*z^2 + c*x + d*y + e*z + f = 0

%a = A^2/B^2, b = A^2/C^2, c = -2*x0, d = -2*y0*A^2/B^2, e = -2*z0*A^2/B^2
%f = x0^2 + A^2/B^2*y0^2 + A^2/C^2*z0^2 - A^2

function [ X0,Y0,Z0,A,B,C ] = Ellipse_fit( data_x,data_y,data_z)
    len = length(data_x);%数据串长度
        
    %预计算
    x_avr = sum(data_x)/len;
    y_avr = sum(data_y)/len;
    z_avr = sum(data_z)/len;
    
    xy_avr = sum(data_x.*data_y)/len;
    xz_avr = sum(data_x.*data_z)/len;
    yz_avr = sum(data_z.*data_y)/len;
    
    xx_avr = sum(data_x.*data_x)/len;
    yy_avr = sum(data_y.*data_y)/len;
    zz_avr = sum(data_z.*data_z)/len;

    xxy_zvr = sum(data_x.*data_x.*data_y)/len;
    xxz_zvr = sum(data_x.*data_x.*data_z)/len;
    xyy_zvr = sum(data_x.*data_y.*data_y)/len;
    yyz_zvr = sum(data_y.*data_y.*data_z)/len;
    xzz_zvr = sum(data_x.*data_z.*data_z)/len;
    yzz_zvr = sum(data_y.*data_z.*data_z)/len;
    
    xxx_avr = sum(data_x.*data_x.*data_x)/len;
    yyy_avr = sum(data_y.*data_y.*data_y)/len;
    zzz_avr = sum(data_z.*data_z.*data_z)/len;
    
    yyyy_avr = sum(data_y.*data_y.*data_y.*data_y)/len;
    zzzz_avr = sum(data_z.*data_z.*data_z.*data_z)/len;
    xxyy_avr = sum(data_x.*data_x.*data_y.*data_y)/len;
    xxzz_avr = sum(data_x.*data_x.*data_z.*data_z)/len;
    yyzz_avr = sum(data_y.*data_y.*data_z.*data_z)/len;
    
    temp_X = [yyyy_avr yyzz_avr xyy_zvr yyy_avr yyz_zvr yy_avr;
          yyzz_avr zzzz_avr xzz_zvr yzz_zvr zzz_avr zz_avr;
          xyy_zvr  xzz_zvr  xx_avr  xy_avr  xz_avr  x_avr;
          yyy_avr  yzz_zvr  xy_avr  yy_avr  yz_avr  y_avr;
          yyz_zvr  zzz_avr  xz_avr  yz_avr  zz_avr  z_avr;
          yy_avr   zz_avr   x_avr   y_avr   z_avr   1];
      
    temp_Y = [-xxyy_avr;
              -xxzz_avr;
              -xxx_avr;
              -xxy_zvr;
              -xxz_zvr;
              -xx_avr];
          
    temp_result = inv(temp_X)*temp_Y;%temp_result = [a;b;c;d;e;f]
          
    X0 = -temp_result(3)/2;
    Y0 = -temp_result(4)/(2*temp_result(1));
    Z0 = -temp_result(5)/(2*temp_result(2));
    
    A = sqrt(X0*X0 + temp_result(1)*Y0*Y0 + temp_result(2)*Z0*Z0 - temp_result(6));
    B = A/sqrt(temp_result(1));
    C = A/sqrt(temp_result(2));
end

