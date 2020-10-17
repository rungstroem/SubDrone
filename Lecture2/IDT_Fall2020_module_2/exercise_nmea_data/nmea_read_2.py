#!/usr/bin/env python3
#/****************************************************************************
# nmea read function
# Copyright (c) 2018-2020, Kjeld Jensen <kj@kjen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
'''
2018-03-13 Kjeld First version, that reads in CSV data
2020-02-03 Kjeld Python 3 compatible
2020-09-17 Kjeld Changed first line to python3
'''

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker

class nmea_class:
  def __init__(self):
    self.data = []

  def import_file(self, file_name):
    file_ok = True
    try:
      # read all lines from the file and strip \n
      lines = [line.rstrip() for line in open(file_name)] 
    except:
      file_ok = False
    if file_ok == True:
      pt_num = 0
      for i in range(len(lines)): # for all lines
        if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
          csv = lines[i].split (',') # split into comma separated list
          self.data.append(csv)

  def print_data(self):
    for i in range(len(self.data)):
      print (self.data[i])


if __name__ == "__main__":
  print ('Importing file')
  nmea = nmea_class()
  nmea.import_file ('nmea_ublox_neo_24h_static.txt')
  #nmea.print_data()
  #print(nmea.data[1])

  nData = []
  for i in range( len(nmea.data)-1 ):
    if(nmea.data[i][0] == "$GPGGA"):
      nData.append(nmea.data[i])
  
  time = []
  lat = []
  lon = []
  qInd = []
  nSat = []
  HDOP = []
  z = []
  index = 0;
  for i in range( len(nData)-1 ):
    time.append(nData[i][1])
    lat.append(nData[i][2])
    lon.append(nData[i][4])
    qInd.append(nData[i][6])
    nSat.append(nData[i][7])
    HDOP.append(nData[i][8])
    z.append(nData[i][9])
    index = i;

## Plotting
  fig, ax = plt.subplots()
  ax.plot(time,nSat)

  x0, xEnd = ax.get_xlim()
  ax.xaxis.set_ticks(np.arange(x0, xEnd, 20000))
  ax.set_xlabel("Time")

  y0, yEnd = ax.get_ylim()
  #y0 = float(min(z))
  #yEnd = float(max(z))
  ax.yaxis.set_ticks(np.arange(y0, yEnd, 2))
  ax.set_ylabel("Satelites")

  ax.set_title("Number of satelites with respect to time")
  #plt.show()
  #print(y0,yEnd)
  plt.savefig("SatelitesVSTime.png")