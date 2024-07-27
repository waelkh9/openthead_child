This is the code for the child device. It's run in combination with the leader code on a second microcontroller. 
To initiate connection and dataset use the following commands: 

To connect the child device to the active thread network:
– ”dataset set active” followed by the hex encoded key of the dataset from the leader. The rest is the same as on the leader device. 
-”dataset commit active”
-”ifconfig up”


