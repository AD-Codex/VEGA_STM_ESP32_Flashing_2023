#ifndef FILE_HANDLE_H
#define FILE_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif

 char file_1[300];
 char file_2[300];
 int address1;
 int address2;
 int address3;
 int address4;
 int address5;
 int address6;
//  bool cardDetected;
//  int stateFile_1;
 //char transactionIDFile_1[18];
 //char userIDFile_1[10];
 //char userReferenceIDFile_1[25];

#define READFILE_OK           1
#define READFILE_ERROR        2
#define WRITEFILE_OK          1
#define WRITEFILE_ERROR       2

 float energySendFile_1;
 //float timeDiffFile_1;
 //float costCalcFile_1;
 //float timeWhenStoppingFile_1 ;
 //float energyWhenStoppingFile_1 ;
 //int doubleChargeFile_1 ;
 //float accountBalanceFile_1;


 int stateFile_2 ;
 float energySendFile_2 ;
 float timeDiffFile_2 ;
 float costCalcFile_2 ;
char transactionIDFile_2[18];
 char userReferenceIDFile_2[25];

 #ifdef __cplusplus
}
#endif

#endif