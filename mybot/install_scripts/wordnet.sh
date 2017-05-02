#!/bin/bash
sudo apt-get install -y perl wordnet tcl-dev tk-dev mesa-common-dev libjpeg-dev libtogl-dev libluajit-5.1-dev

#sudo cpan Digest::SHA1

cp /home/adllo/catkin_ws/src/mybot2/etc/wordnet/WordNet-3.0.tar.gz /tmp
cp /home/adllo/catkin_ws/src/mybot2/etc/wordnet/WordNet-QueryData-1.49.tar.gz /tmp
cp /home/adllo/catkin_ws/src/mybot2/etc/wordnet/Text-Similarity-0.10.tar.gz /tmp
cp /home/adllo/catkin_ws/src/mybot2/etc/wordnet/WordNet-Similarity-2.07.tar.gz /tmp

cd /tmp

tar -zxvf WordNet-3.0.tar.gz
tar -zxvf WordNet-QueryData-1.49.tar.gz
tar -zxvf Text-Similarity-0.10.tar.gz
tar -zxvf WordNet-Similarity-2.07.tar.gz

cd /tmp/WordNet-3.0
./configure
make
sudo make install

cd /tmp/WordNet-QueryData-1.49
perl Makefile.PL
make
make test
sudo make install

cd /tmp/Text-Similarity-0.10
perl Makefile.PL
make
make test
sudo make install


cd /tmp/WordNet-Similarity-2.07
perl Makefile.PL
make
make test
sudo make install

