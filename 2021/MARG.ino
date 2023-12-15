// https://nobita-rx7.hatenablog.com/entry/28660802
/*MARGセンサーの使い方！(Arduino+GY-80で三次元姿勢推定編)20180729NOBのArduino日記！ */
#include <GY80.h>
GY80 sensor = GY80(); //GY80インスタンスを作成する
void a_set_scale(uint8_t scale);

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;}

/*------------------------以下Madgwickフィルター--------------------------*/
//システム定数
float deltat = 0.001f;// 実測値、1秒間のサンプリング周期（秒単位）デフォルトは「0.001」
float lastt = 0;
float ct = 0;
float beta=2.5f;  //--------------------------初期値、デフォルトは「2.5」
//初期化終了後のbeta値
void Beta() {beta =0.5f;} //--------------------------デフォルト0.041f----------------------0.2でノイズ減を除去する！
//グローバルシステム変数
float q0 = 1.0f,q1 = 0.0f,q2 = 0.0f,q3 = 0.0f;// 初期条件付き推定方位四元数要素
void filterUpdate(float gg_x, float gg_y, float gg_z, float aa_x, float aa_y, float aa_z, float mm_x, float mm_y, float mm_z){ //w:角速度計、a:加速度計、m:地磁気計
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  // ジャイロスコープ度/秒をラジアン/秒に変換する
  gg_x *= 0.0174533f;
  gg_y *= 0.0174533f;
  gg_z *= 0.0174533f;
  // ジャイロスコープによる四元数の変化率
  qDot1 = 0.5f * (-q1 * gg_x - q2 * gg_y - q3 * gg_z);
  qDot2 = 0.5f * (q0 * gg_x + q2 * gg_z - q3 * gg_y);
  qDot3 = 0.5f * (q0 * gg_y - q1 * gg_z + q3 * gg_x);
  qDot4 = 0.5f * (q0 * gg_z + q1 * gg_y - q2 * gg_x);
  // 加速度計の測定が有効な場合にのみフィードバックを計算する（加速度計の正規化でNaNを回避する）
  if(!*1 > 3.0f || val.g_x > 20000 || val.g_x < -20000){;}else{gx = val.g_x;}
   if (abs*2 > 3.0f || val.g_y > 20000 || val.g_y < -20000){;}else{gy = val.g_y;}
   if (abs*3 > 3.0f || val.g_z > 20000 || val.g_z < -20000){;}else{gz = val.g_z;}
   if (val.a_x > 700 || val.a_x < -700){;}else{ax = val.a_x;}  //700
   if (val.a_y > 700 || val.a_y < -700){;}else{ay = val.a_y;} 
   if (val.a_z > 700 || val.a_z < -700){;}else{az = val.a_z;}
   if (val.m_x > 400 || val.m_x < -400){;}else{mx = val.m_x;}  //400
   if (val.m_y > 400 || val.m_y < -400){;}else{my = val.m_y;}
   if (val.m_z > 400 || val.m_z < -400){;}else{mz = val.m_z;}
 //ジャイロスコープ静止状態ゼロ点補正---------------------------------------------------------------------------------------------------------
   gxOld2=gxOld1,gyOld2=gyOld1,gzOld2=gzOld1,gxOld1=val.g_x,gyOld1=val.g_y,gzOld1=val.g_z;
  xG = convertRawGyro(gx - CoefficientGX); //静止状態のgx平均値でゼロ点補正
  yG = convertRawGyro(gy - CoefficientGY); //静止状態のgy平均値でゼロ点補正
  zG = convertRawGyro(gz - CoefficientGZ); //静止状態のgz平均値でゼロ点補正
 //Madgwickフィルタによる角速度・加速度・地磁気から四元数推定----------------------------------------------------------------------------------
   filterUpdate(xG,yG,zG,ax,ay,az,val.m_x,val.m_y,val.m_z);//g:角速度計、a:加速度計、m:地磁気計
 //四元数から姿勢(ピッチ・ヨウ・ロール)計算----------------------------------------------------------------------------------------------------
    pitch = asinf(-2.0f * (q1*q3 - q0*q2))* 57.29578f;
    yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* 57.29578f;
    roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)* 57.29578f;
//姿勢推定結果の出力----------------------------------------------------------------------------------------------------------------------------
   Serial.print("Orientation: ");
   Serial.print(-yaw+ 180.0f);Serial.print(" ");  // def:X * -1 + 180.0f
   Serial.print(-pitch+ 180.0f);Serial.print(" ");// def:X * -1 + 180.0f
   Serial.println(-roll);
   }

//補正係数計算
void Coefficient() {
//ジャイロスコープゼロ点補正係数
  int jx=0,jy=0,jz=0;
  for(int i=0;i<100;i++){   //------------------------------50～100回平均をする
    GY80_raw val = sensor.read_raw();
    //外れ値は除去して平均値を計算する
    if (val.g_x > 50 || val.g_x < -50){;}else{CoefficientGX += val.g_x;jx+=1;}
    if (val.g_y > 50 || val.g_y < -50){;}else{CoefficientGY += val.g_y;jy+=1;}
    if (val.g_z > 50 || val.g_z < -50){;}else{CoefficientGZ += val.g_z;jz+=1;}}
  CoefficientGX = -CoefficientGX/jx ;//x軸角速度補正値計算
  CoefficientGY = -CoefficientGY/jy ;//y軸角速度補正値計算
  CoefficientGZ = -CoefficientGZ/jz ;//z軸角速度補正値計算
  }

void setup(){  
  Serial.begin(115200);  // 毎秒9600,115200ビットでシリアル通信を初期化する
  sensor.begin();//センサーを初期化する
  for(int i=0;i<100;i++){Measurement();}
  Coefficient();
  for(int i=0;i<100;i++){Measurement();}
  Coefficient();
  for(int i=0;i<300;i++){Measurement();}
  Coefficient();
  Beta();}

void loop(){Measurement();}