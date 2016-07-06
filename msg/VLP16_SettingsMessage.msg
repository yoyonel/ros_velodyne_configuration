# Velodyne LIDAR settings messages.

# $ ? curl -s http://192.168.1.201/cgi/settings.json
# {"laser":"On","returns":"Strongest","rpm":300,"fov":{"start":0,"end":359},"phaselock":{"enabled":"Off","offset":"0"},
#  "host":{"addr":"172.20.0.215","dport":"2368","tport":"8308"},
#  "net":{"addr":"192.168.1.201","mask":"255.255.255.0","gateway":"192.168.1.1","dhcp":"Off"}
# }

bool    laser_state         #
string  returns             #
uint16  rpm                 #
uint16  fov_start           #
uint16  fov_end             #
bool    phaselock_enabled   #
uint16  phaselock_offset    #
string  host_addr           #
uint16  host_dport          #
uint16  host_tport          #
string  net_addr            #
string  net_mask            #
string  net_gateway         #
bool    net_dhcp            #
