void encoder1A();void encoder2A();
unsigned long previous_val_m1 = 0;
int period_e1 = 0;
unsigned char index_buff_1 = 0;
unsigned char index_buff_2 = 0;
unsigned long previous_val_m2 = 0;
int period_e2 = 0;
unsigned long encoder_val_m1=0;
unsigned long encoder_val_m2=0;
 bool m1=0;bool m2=0;
 int buff_1[10];
 int buff_2[10];




void encoder1A()
{
    unsigned long current_val=micros();
    encoder_val_m1++;

    period_e1 = abs(current_val - previous_val_m1);
    previous_val_m1 = current_val;
    buff_1[index_buff_1] = period_e1;
    index_buff_1=(index_buff_1+1)%10;;

    
}
void encoder2A()
{
    unsigned long current_val=micros();
    encoder_val_m2++;

    period_e2 = abs(current_val - previous_val_m2);
    previous_val_m2 = current_val;
    buff_2[index_buff_2] = period_e2;
    index_buff_2=(index_buff_2+1)%10;;
}
