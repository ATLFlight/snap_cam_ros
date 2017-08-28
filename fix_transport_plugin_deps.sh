#!/bin/bash
#****************************************************************************
# *   Copyright (c) 2017 Michael Shomin. All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# * 1. Redistributions of source code must retain the above copyright
# *    notice, this list of conditions and the following disclaimer.
# * 2. Redistributions in binary form must reproduce the above copyright
# *    notice, this list of conditions and the following disclaimer in
# *    the documentation and/or other materials provided with the
# *    distribution.
# * 3. Neither the name ATLFlight nor the names of its contributors may be
# *    used to endorse or promote products derived from this software
# *    without specific prior written permission.
# *
# * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file. 
# ****************************************************************************/

sudo apt-get -o Dpkg::Options::="--force-overwrite" install libglib2.0-dev -y

sudo apt-get remove libgtk-2.0 -y
sudo apt-get remove libharfbuzz0 -y
sudo apt-get remove pango -y
 
sudo apt-get -o Dpkg::Options::="--force-overwrite" install fontconfig-config=2.11.0-0ubuntu4 -y --force-yes
sudo apt-get -o Dpkg::Options::="--force-overwrite" install libfontconfig1=2.11.0-0ubuntu4 -y --force-yes
sudo apt-get install libfreetype6=2.5.2-1ubuntu2 -y --force-yes
sudo apt-get install libxcb-render0=1.10-2ubuntu1 -y --force-yes
sudo apt-get install libxcb1=1.10-2ubuntu1 -y --force-yes
sudo apt-get install libxdamage1=1:1.1.4-1ubuntu1 -y --force-yes
sudo apt-get install libxfixes3=1:5.0.1-1ubuntu1 -y --force-yes
sudo apt-get install libxft2=2.3.1-2 -y --force-yes
sudo apt-get install libxrender1=1:0.9.8-1 -y --force-yes
sudo apt-get install libxcomposite1=1:0.4.4-1 -y --force-yes
sudo apt-get install libpixman-1-0=0.30.2-2ubuntu1 -y --force-yes
sudo apt-get install libxcursor1=1:1.1.14-1 -y --force-yes
sudo apt-get install libxcb-shm0=1.10-2ubuntu1 -y --force-yes
sudo apt-get -o Dpkg::Options::="--force-overwrite" install libgdk-pixbuf2.0-dev -y --force-yes
sudo mkdir -p /usr/lib/gdk-pixbuf-2.0/2.10.0
sudo chown linaro /usr/lib/gdk-pixbuf-2.0/2.10.0

sudo apt-get install ros-indigo-image-transport -y
sudo apt-get install ros-indigo-image-transport-plugins -y
