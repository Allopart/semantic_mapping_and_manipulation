#!/bin/bash
sudo bash -c 'echo -e "deb http://winnie.kuis.kyoto-u.ac.jp/HARK/harkrepos trusty non-free\ndeb-src http://winnie.kuis.kyoto-u.ac.jp/HARK/harkrepos trusty non-free" > /etc/apt/sources.list.d/hark.list'
wget -q -O - http://winnie.kuis.kyoto-u.ac.jp/HARK/harkrepos/public.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install harkfd hark-designer
sudo apt-get install julius-4.2.3-hark-plugin
sudo apt-get install harktool4
sudo apt-get install harktool5
