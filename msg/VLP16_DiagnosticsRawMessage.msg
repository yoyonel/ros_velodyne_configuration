# Velodyne LIDAR diagnostics messages.

# $ ? curl -s http://192.168.1.201/cgi/diag.json
# {
# "volt_temp":{
#     "bot":{
#         "i_out":2087,"pwr_1_2v":986,"lm20_temp":1037,"pwr_5v":2056,
#         "pwr_2_5v":2047,"pwr_3_3v":2706,"pwr_v_in":891,"pwr_1_25v":0
#     },
#     "top":{
#         "hv":2628,"ad_temp":602,"lm20_temp":1018,"pwr_5v":2078,
#         "pwr_2_5v":2047,"pwr_3_3v":2687,"pwr_5v_raw":2190,"pwr_vccint":976
#        }
#   },
#   "vhv":412,"adc_nf":14,"ixe":1
# }

uint16  bot_i_out       #
uint16  bot_pwr_1_2v    #
uint16  bot_lm20_temp   #
uint16  bot_pwr_5v      #
uint16  bot_pwr_2_5v    #
uint16  bot_pwr_3_3v    #
uint16  bot_pwr_v_in    #
uint16  bot_pwr_1_25v   #

uint16  top_hv          #
uint16  top_ad_temp     #
uint16  top_lm20_temp   #
uint16  top_pwr_5v      #
uint16  top_pwr_2_5v    #
uint16  top_pwr_3_3v    #
uint16  top_pwr_5v_raw  #
uint16  top_pwr_vccint  #

uint16  vhv             #
uint16  adc_nf          #
uint16  ixe             #
