idf_component_register(SRCS "main.c;\
gble.c;\
ftp.c;\
i2c-driver.c;\
jsn.c;\
base64url.c;\
Fisitron.c;\
gcpjwt.c;\
mqtt.c;\
utility.c;\
fsntp.c;\
snsmems.c;\
mems/mems.c;\
mems/mems_env_sensors.c;\
mems/hts221/hts221.c;\
mems/hts221/hts221_reg.c"  INCLUDE_DIRS "." "mems" "mems/hts221")