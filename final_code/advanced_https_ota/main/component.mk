#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_EMBED_TXTFILES :=  ${PROJECT_PATH}/server_certs/ca_cert_1.pem
COMPONENT_EMBED_TXTFILES :=  ${PROJECT_PATH}/server_certs/ca_cert_2.pem
#COMPONENT_EMBED_TXTFILES :=  ${PROJECT_PATH}/ca_cert_1.c

COMPONENT_ADD_INCLUDEDIRS += ./ble_helper/include
COMPONENT_SRCDIRS += ./ble_helper/
