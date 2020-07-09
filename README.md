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


#mbed import https://github.com/edgeimpulse/example-standalone-inferencing-mbed
cd compilation


echo 'Add Inference Model & Features then run second script'
mbed config -G ARM_PATH "/home/ubuntu/gcc-arm-none-eabi-9-2020-q2-update/bin"
