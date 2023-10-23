#define CRC_POL 50585
#define CRC_POL_BIT 15

static bool check_crc(const int arr_length, uint8_t* message_data)
{
    uint8_t data[1024] = {0};

    for(int i=0; i<arr_length; i++)
        data[i] = message_data[i];

    //compute received data CRC
    uint16_t received_crc = (data[arr_length-2]<<8) + data[arr_length-1];

    //remove received CRC for new calculation
    data[arr_length-2] = 0;
    data[arr_length-1] = 0;

    //create CRC mask based on CRC_POL
    uint8_t crc_arr[arr_length];
    for(int i=0; i<arr_length; i++)
        crc_arr[i] = 0;
    crc_arr[0] = CRC_POL>>8;
    crc_arr[1] = CRC_POL&255;

    //compute CRC
    for(int i=arr_length*8-1; i>=16; i--)
    {
        if(((data[arr_length-1-(i>>3)]>>(i&7))&1) == 1)
        {
            for(int j=0; j<arr_length; j++)
                data[j] = data[j]^crc_arr[j];
        }
        for(int j=arr_length-1; j>0; j--)
            crc_arr[j] = (crc_arr[j]>>1) | ((crc_arr[j-1]&1)<<7);
        crc_arr[0] = crc_arr[0]>>1;
    }
    uint16_t calc_crc = (data[arr_length-2]<<8) + data[arr_length-1];
    
    return calc_crc==received_crc;
}