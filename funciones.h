
/****************Send every data through the spi bus- if you wanna send other byte you have that modify here and controller skecth**************************/
void envia_por_tipo(uint16_t parametro_1,uint16_t parametro_2,uint16_t parametro_3,uint16_t parametro_4,uint16_t parametro_5,uint16_t parametro_6,uint16_t parametro_7){
    int ConfirmacionTipoFuncion_aux = 0;
    SPI.transfer16(parametro_1);
    Serial.print("parametro 1:  ");
    Serial.println(parametro_1);
    delay(30);
    SPI.transfer16(parametro_2);
    Serial.print("parametro 2:  ");
    Serial.println(parametro_2);
    delay(30);
    SPI.transfer16(parametro_3);
    Serial.print("parametro 3:  ");
    Serial.println(parametro_3);
    delay(30);
    SPI.transfer16(parametro_4);
    Serial.print("parametro 4:  ");
    Serial.println(parametro_4);
    delay(30);
    SPI.transfer16(parametro_5);
    Serial.print("parametro 5:  ");
    Serial.println(parametro_5);
    delay(30);
    SPI.transfer16(parametro_6);
    Serial.print("parametro 6:  ");
    Serial.println(parametro_6);
    delay(30);
    SPI.transfer16(parametro_7);
    Serial.print("parametro 7:  ");
    Serial.println(parametro_7);
    delay(30);

    }

