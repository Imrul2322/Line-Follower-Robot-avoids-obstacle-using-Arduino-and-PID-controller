void sensorInit(){
  int i;
  for(i=0;i<4;i++){
    pinMode(sensorExPin[i],INPUT);
  }
  //sensorCalib(10);
  sensorCalibMan();
  for (i = 0; i < NUM_SENSOR; i++){
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (i = 0; i < NUM_SENSOR; i++){
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}
void sensorCalib(int sec){
  int i,j;
  pinMode(13,OUTPUT);
  int light=HIGH;
  for (i = 0; i < sec; i++) 
  {
    digitalWrite(13,light);
    light=(light==HIGH)?LOW:HIGH;
    for (j = 0; j < 40; j++) //40 loops for 1 sec apprx
    {
      qtrrc.calibrate();
    }
  }
}
void sensorCalibMan(){
  int i;
  qtrrc.calibrate();
  //int cmin[]={840,676,736,344,792,568,668,788};
  //int cmin[]={732,676,844,400,960,676,732,844};
  //int cmin[]={288,236,340,120,236,120,124,120};
  int cmin[]={348,284,292,120,412,236,292,348};
  int cmax[]={2500,2348,2500,1396,2500,2460,2500,2500};
  for (i = 0; i < NUM_SENSOR; i++){
    qtrrc.calibratedMinimumOn[i]=cmin[i];
  }
  for (i = 0; i < NUM_SENSOR; i++){
    qtrrc.calibratedMaximumOn[i]=cmax[i];
  }
}
int sensorRead(){
  unsigned int position = qtrrc.readLine(sensorA);
  unsigned int pos;
  int threshold=800;
  int sum=0,avg=0;
  static int prevpos;
  unsigned char i;
  
  
  for (i = 0; i < NUM_SENSOR; i++)
  {
    if((sensorA[i] * 10 / 1001)<=1){
      sensorD[i]=1;
    }
    else sensorD[i]=0;
    avg+=(sensorD[i]*threshold*(i+1));
    sum+=sensorD[i];
    #ifdef SENSOR_DEBUG
    //Serial.print(sensorA[i] * 10 / 1001);
    Serial.print(sensorD[i]);
    Serial.print('\t');
    #endif
  }
  if(sum==0){
    if(prevpos> ( (NUM_SENSOR+1)*threshold/2 ) ){
      pos=NUM_SENSOR*threshold;
    }
    else pos=threshold;
  }
  else pos=avg/sum;
  prevpos=pos;
  #ifdef SENSOR_DEBUG
  Serial.print("\t");
  Serial.print(pos);
  #endif
  return pos;
}

boolean sensorReadEx()
{
  /*for(int i=8,j=0;i<=11;i++,j++)
  {
    chke[j]=analogRead(i);
    Serial.print(chke[j]);
    Serial.print(" ");
  }
  */
  
  
  for(int j=0;j<4;j++)
  {
    chke[j]=analogRead(sensorExPin[j]);
    //chke[j]=analogRead(i);
    //Serial.print(chke[j]);
    //Serial.print(" ");
    if(chke[j]<200) sensorExD[j]=1;
    else sensorExD[j]=0;
    //Serial.print(sensorExD[j]);
    //Serial.print(" ");
  }
  if(sensorExD[1]&&sensorExD[2]) return true;
  else return false;
  //Serial.println(" ");
}
