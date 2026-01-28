# Setting a Static IP on Ubuntu 22.04 LTS (Server) for LMD Drive Connection

This guide explains how to configure your Raspberry Pi to talk directly to an industrial LMD/Lexium drive using a static IPv4 address using the built-in Netplan.

---

## Prerequisite:
- [ ] rpi is connected to the LMD Drive via ethernet
- [ ] LMD Drive is powered with sufficient power

## Step 1: Identify your Ethernet Interface
Before editing configurations, you need the system name for your Ethernet port.

```bash
ip link show

```

Usually, this is `eth0`, but on some Ubuntu builds it may appear as `enp1s0` or similar.

---

## Step 2: Locate the Netplan Configuration

Netplan stores its settings in `.yaml` files. List them to find the one your system is using:

```bash
ls /etc/netplan/

```

Common names: `01-netcfg.yaml`, `50-cloud-init.yaml`, or `00-installer-config.yaml`.

---

## Step 3: Edit the Configuration

Open the file with `nano`. **Note:** YAML files are strictly sensitive to spaces. Do not use tabs.

```bash
sudo nano /etc/netplan/01-netcfg.yaml

```

Replace the content with the following block. Adjust `eth0` to your interface name and `192.168.33.10` to your desired Pi IP.

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.33.10/24

```

---

## Step 4: Validate and Apply

Netplan has a built-in safety feature to prevent you from locking yourself out of the system.

1. **Test the configuration:**

```bash
sudo netplan try

```

If you don't press 'Enter' within 120 seconds, it will roll back the changes.

2. **Apply the changes permanently:**

```bash
sudo netplan apply

```

---

## Step 5: Verify the Link

1. **Check that the IP is assigned:**

```bash
ip addr show eth0

```

2. **Ping the LMD Drive:**
Replace `192.168.33.1` with the actual IP address of your drive.

```bash
ping 192.168.33.1

```

---

## Troubleshooting Tips

* **No Link:** If `ip link` shows `NO-CARRIER`, check your physical Ethernet cable.
* **YAML Errors:** If `netplan apply` throws an error, double-check your indentation. Every nested line must be indented by exactly two spaces.
