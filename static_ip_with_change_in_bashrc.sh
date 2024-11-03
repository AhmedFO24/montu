#!/bin/bash

# Variables
INTERFACE="wlp112s0"  # Your Wi-Fi interface
DNS1="8.8.8.8"  # Primary DNS server
DNS2="8.8.4.4"  # Secondary DNS server
NETMASK="24"  # Netmask (typically /24 for 255.255.255.0)

# Find the connection profile name associated with the interface
PROFILE_NAME=$(nmcli -t -f NAME,DEVICE con show | grep "$INTERFACE" | cut -d: -f1)

# Check if the profile name was found
if [ -z "$PROFILE_NAME" ]; then
    echo "No connection profile found for interface $INTERFACE. Exiting."
    exit 1
fi

# Step 1: Switch to DHCP (Automatic IP) to detect current IP range
echo "Switching to DHCP (Automatic IP) on $PROFILE_NAME..."
sudo nmcli con modify "$PROFILE_NAME" ipv4.method auto
sudo nmcli con down "$PROFILE_NAME"
sudo nmcli con up "$PROFILE_NAME"

# Restart the network service to apply DHCP
echo "Restarting network to apply DHCP..."
sudo systemctl restart NetworkManager

# Wait a moment to allow DHCP to assign an IP address
sleep 5

# Debugging Output
echo "Current network status:"
nmcli device status
echo "Current IP configuration:"
ip -4 addr show "$INTERFACE"
echo "Current route:"
ip route

# Step 2: Get the current dynamic IP and extract the network range
CURRENT_IP=$(ip -4 addr show "$INTERFACE" | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
CURRENT_GATEWAY=$(ip route | grep default | awk '{print $3}')

# Check if the current IP and gateway are found
if [ -z "$CURRENT_IP" ] || [ -z "$CURRENT_GATEWAY" ]; then
    echo "Failed to retrieve current IP or gateway. Please ensure you are connected to the Wi-Fi."
    exit 1
fi

# Extract the first three octets of the IP to define the subnet
IP_PREFIX=$(echo "$CURRENT_IP" | cut -d'.' -f1-3)

# Assign a new static IP within the same range
NEW_STATIC_IP="${IP_PREFIX}.101"  # Ensure this is a valid static IP

echo "Detected dynamic IP: $CURRENT_IP"
echo "Detected gateway: $CURRENT_GATEWAY"
echo "Assigning static IP: $NEW_STATIC_IP"

# Step 3: Set Static IP, Gateway, and Netmask
echo "Setting static IP $NEW_STATIC_IP with gateway $CURRENT_GATEWAY on $PROFILE_NAME..."
sudo nmcli con modify "$PROFILE_NAME" ipv4.addresses "$NEW_STATIC_IP/$NETMASK"
sudo nmcli con modify "$PROFILE_NAME" ipv4.gateway "$CURRENT_GATEWAY"
sudo nmcli con modify "$PROFILE_NAME" ipv4.dns "$DNS1 $DNS2"
sudo nmcli con modify "$PROFILE_NAME" ipv4.method manual

# Step 4: Restart the network service again to apply static settings
echo "Restarting network to apply static IP..."
sudo nmcli con down "$PROFILE_NAME"
sudo nmcli con up "$PROFILE_NAME"

# Verify if the new IP is set correctly
CURRENT_STATIC_IP=$(ip -4 addr show "$INTERFACE" | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
echo "Current Static IP after setting: $CURRENT_STATIC_IP"

# Update the MY_IP variable in .bashrc
echo "Updating MY_IP variable in .bashrc..."
if grep -q "export MY_IP=" ~/.bashrc; then
    # If the variable exists, replace it
    echo "Existing MY_IP found. Updating to $CURRENT_STATIC_IP."
    sed -i "s|^export MY_IP=.*|export MY_IP=$CURRENT_STATIC_IP|" ~/.bashrc
else
    # If the variable does not exist, add it
    echo "No existing MY_IP found. Adding it to .bashrc."
    echo "export MY_IP=$CURRENT_STATIC_IP" >> ~/.bashrc
fi


# Update the ROS_MASTER_URI variable in .bashrc
echo "Updating ROS_MASTER_URI variable in .bashrc..."
if grep -q "export ROS_MASTER_URI=" ~/.bashrc; then
    # If the variable exists, replace it
    echo "Existing ROS_MASTER_URI found. Updating to $CURRENT_STATIC_IP."
    sed -i "s|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://$CURRENT_STATIC_IP:11311|" ~/.bashrc
else
    # If the variable does not exist, add it
    echo "No existing ROS_MASTER_URI found. Adding it to .bashrc."
    echo "export ROS_MASTER_URI=http://$CURRENT_STATIC_IP:11311" >> ~/.bashrc
fi



# Verify that the change was made
echo "Current .bashrc contents after update:"
cat ~/.bashrc | grep "MY_IP"
cat ~/.bashrc | grep "ROS_MASTER_URI"

# Optional: Source .bashrc to apply changes immediately
echo "Sourcing .bashrc to apply changes..."
source ~/.bashrc

echo "Done! Static IP $CURRENT_STATIC_IP is set with gateway $CURRENT_GATEWAY and netmask $NETMASK."
