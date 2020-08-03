# Building compiler from scratch for Mbed compilation for B-L475E-IOT01

## Command list

From an empty system, you first need to install python-3, git and mercurial:

	sudo apt install -y python3 python3-pip python3-venv git mercurial

Then you need the last version of the gcc-arm compiler to build the firmware and add it to your PATH:

	wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
	tar xvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
	export PATH=$PATH:${pwd}/gcc-arm-none-eabi-9-2020-q2-update/bin

Create a Python virtual environment:

	mkdir compilation
	cd compilation
	python3 -m venv env

Install mbed CLI:

	source env/bin/activate
	python3 -m pip install mbed-cli

Import the MBED OS Example LoRaWAN, which is at the base of our project:

	mbed import https://github.com/ARMmbed/mbed-os-example-lorawan

Go to your EdgeImpulse account and download the parameters (edgeimpulse-sdk, model-parameters and tflite-model) necessary to run your ML model (for consistency with EdgeImpulse, we will call it impulse in further documentation)

Extract the downloaded file in the mbed-os-example-lorawan folder. For me the downloaded file is ei-bernard-project-1-deployment-zip-123456789.zip. Yours probably has the same structure.

The following part consists in editing the main.cpp file in order to change the transmission function (send_message) and allow the algorithm to classify data before transmission.

My main.cpp is available [here](tests_lora_ei/main.cpp), and the modifications that were made to the send_message function are also available in a dedicated file [here](tests_lora_ei/send_message).

Once your main.cpp file is edited, your last task is to edit your LoRa configuration file for your device to send its decision on a LoRaWAN network.

Three parameters to change in this file :
- the main_stack_size parameter to set to 8192 instead of 4096.
- the LoRa parameters in the target_overrides section (lora.device-eui, lora.application-eui and lora.application-key) (Don't forget to set the lora.phy parameter, for example if you need to transmit on US territory)
- the target itself should be DISCO_L475VG_IOT01A. When building a project, mbed needs to know how the hardware is mapped. The easiest way to allow the project to build for DISCO_L475VG_IOT01A is to change K64F to DISCO_L475VG_IOT01A (first target card in the list and same hardware mapping as the DISCO_L475VG_IOT01A that we use)

Building the application

Building the application requires to set a last parameter:

	mbed compile -t GCC_ARM -m DISCO_L475VG_IOT01A -f

If you have already done all these steps and wish to keep working on your app after logging back onto the machine, there is a few parameters to set, all available here.

Else, you can follow us in our path to build a neural network for time series analysis.
