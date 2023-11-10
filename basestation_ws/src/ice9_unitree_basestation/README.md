# Setup the base station PC
1. Join the Unitree_GO wifi, password is 888888888.
2. Set the ip address to 192.168.12.170.
3. add IP routes:
```
sudo route add -net 192.168.123.0 netmask 255.255.255.0 gw 192.168.12.1
```
4. Now the PC should be able to communicate with the Head Nano on Unitree. Test the connection:
```
ping 192.168.123.15
```

# Launch base station
```
roslaunch ice9_unitree_basestation basestation.launch
```