#!/bin/bash

# Installation of Apptainer.
# Source: https://apptainer.org/docs/admin/main/installation.html#install-ubuntu-packages

sudo apt-get -y update
sudo apt -y install software-properties-common

sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer-suid