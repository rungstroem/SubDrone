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
from exportkml import kmlclass

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
  nmea.import_file ('nmea_trimble_gnss_eduquad_flight.txt')
  #nmea.print_data()
  
  time = []
  lat = []
  lon = []
  qInd = []
  nSat = []
  HDOP = []
  z = []
  corLat = []
  corLon = []

  index = 0;
  for i in range( len(nmea.data)-1 ):
    time.append(nmea.data[i][1])
    lat.append(nmea.data[i][2])
    lon.append(nmea.data[i][4])
    qInd.append(nmea.data[i][6])
    nSat.append(nmea.data[i][7])
    HDOP.append(nmea.data[i][8])
    z.append(nmea.data[i][9])
    corLat.append(float(nmea.data[0][2])-float(nmea.data[i][2]))
    corLon.append(float(nmea.data[0][4])-float(nmea.data[i][4]))
    index = i;

  ddlat = []
  ddlon = []
  for i in range( len(nmea.data)-1 ):
    ddlat.append( ((float(lat[i])-(float(lat[i])%100))/100) + (((float(lat[i])%100))/60) )
    ddlon.append( ((float(lon[i])-(float(lon[i])%100))/100) + (((float(lon[i])%100))/60) )  
  
  kml = kmlclass();
  kml.begin("DroneTrack.kml", "Drone Track", "Track of drone", 0.7)
  kml.trksegbegin ('', '', 'red', 'absolute')
  for i in range( len(nmea.data)-1 ) :
    kml.pt(float(ddlat[i]), float(ddlon[i]), 0.0);
  kml.trksegend();
  kml.end;
  