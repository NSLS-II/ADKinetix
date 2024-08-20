# ADKinetix Documentation

This documentation outlines how to get started with `ADKinetix`, and some example performance results as collected at the HEX beamline at NSLS-II.

## Setup Guide

### Step 0 - Download PVCam

To start, go to the [Teledyne Photometrics software downloads site](https://www.photometrics.com/support/software-and-drivers), and download the latest `PVCAM` SDK and driver package for your platform of choice.
You will need to create a Teledyne account and explicitly request a download link via the webpage. After some time, you will recieve one in your email.

On windows, run the installer that comes with the download. On linux, unzip the archive, and enter the `pvcam` directory, and run the included `*.run` file as root, with bash. Next, navigate to the SDK directory and do the same thing. This should install all the pvcam software in the `/opt/pvcam` directory. Going forward, this guide will focus on setting up the driver on linux. On windows the steps should be more or less the same.

### Step 1 - Install Dolphin PCIE Card

Next, you will need to install the PCIE card that came with the detector in the PC or server that you are using. You will need at least an `x4` slot.
The camera should have come with two Mini-SAS HD 8644 copper cables. These are quite short, but are a good way of making sure that your PCIE card is installed correctly.
For situations where the server will be far away from the camera, you will need to purchase a set of longer cables from Dolphin Interconnect Solutions (other branded cables may work, but Teledyne does not guarantee this).
At HEX, we purchased 50 meter long cables w/ product code `MSFC50M` from [the Dolphib website](https://www.dolphinics.com/products/PCI_Express_SFF-8644_cables.html), and these have worked well.

### Step 2 - Configure the PCIE Kernel Drivers

Unfortunately, the scripts included with the distribution (at least as of version 3.9.11 on linux) do not build the PCIE kernel drivers correctly.
Instead, we must build the kernel driver manually. These steps must also be re-executed any time there is a kernel update on the machine, as the DKMS support is either broken, or outright removed.

Note all the following commands should be run as root. Navigate to the driver source directory, and compile the module.

```
cd /opt/pvcam/drivers/in-kernel/pcie/src
make
```

This should produce a kernel object file: `pvcam_pcie.ko`. Next, you must execute some commands to have the module load automatically on startup, and to allow non-root users to access the PCIE device.

First, install the module to permanently load on boot.

```
install -D -m 0664 pvcam_pcie.ko /lib/modules/`uname -r`/kernel/drivers/pvcam
depmod -a
```

Next, open a udev file `/etc/udev/rules.d/99-pvcam-drv-pcie.rules` with the CLI text editor of your choice, and write the following:

```
SUBSYSTEM=="pvcam_pcie", MODE="0660", USER="softioc-hex"
```

replacing `softioc-hex` with the username of whichever user will be running the `ADKinetix` IOC.

Next, reload the udev rules:

```
udevadm control --reload-rules
udevadm trigger -s pvcam_pcie -v
```

Finally, load the PCIE driver:

```
insmod pvcam_pcie.ko
```

To confirm that it is loaded, you can run:

```
lsmod | grep pvcam
```

and you should see a single entry. Also, you can run `lspci -v | grep Dolphin` to check to see if the PCI card is installed correctly.

If you look through the output of `lspci -v`, the entry for the Dolphin card should list `pvcam_pcie` as the kernel driver in-use.

### Step 2 - Connecting the Camera

Power off the server that has the PCI card installed, and connect the dual cables to the server and the camera.
Note that the order is important - port 1 on the server should go to port 1 on the camera.

Once both cables ae connected, power on the camera, and wait until the blinking LED turns off.
This means that the camera has initialized.

Finally, power on the server. If the cables and drivers are installed correctly, the LEDs on the PCI card should display as green.

### Step 3 - Testing camera connection

The simplest way to test if the camera connection is working, is to use the `PVCamTestCli` program included with the SDK.

Navigate to `/opt/pvcam/bin/x86_64`, and simply run the program:

```
# TODO: Add demo output from PVCamTestCli
```

You should see, as above, the camera connect and acquire a single frame.

### Step 4 - Setup IOC

From here, you may go ahead and compile the driver & IOC executable.
The only parameter you will likely need to adjust in the startup script is the device index if you are connecting more than one device at any time.

Upon starting, after connecting to the camera the IOC will print out information about all of the modes that it detects that the device can support.

### Performance Results

The Kinetix we tested this driver with was deployed at the HEX beamline at NSLS-II. It supported three modes in firmware, and we tested the driver up to the following performance levels in each of the three modes:

Mode | Bit Depth | Max Framerate Achieved | Matches Vendor Spec
-----|-----------|------------------------|---------------------
Sensitivity | 12 | 89 Hz | Yes
Speed | 8 | 500 Hz | Yes
Dynamic Range | 16 | 82 Hz | Yes

Lowering the dimentions of the region of interest (at full resolution, it is at the full 3200x3200 frame), or increasing the binning size increased framerates as expected.