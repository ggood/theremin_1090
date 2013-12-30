#!/usr/bin/env python

import io
import math
from optparse import OptionParser
import random
import socket
import sys
import time

import pygame.midi

def list_devices():
  pygame.midi.init()
  for i in range(pygame.midi.get_count()):
    (interf, name, input, output, opened) = pygame.midi.get_device_info(i)
    print "Device %s" % i
    print "  Interface: %s" % interf
    print "  Name: %s" % name
    io_info = []
    if input:
      io_info.append("input")
    if output:
      io_info.append("output")
    if opened:
      io_info.append("opened")
    if io_info:
      print "  Info: %s" % ", ".join(io_info)
    print

class Dump1090Endpoint(object):
  def __init__(self, host, port):
    self._host = host 
    self._port = port
    self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self._socket.connect((host, port))
    self._buf = io.open(self._socket.fileno(),  mode='r', buffering=1, encoding=None, errors=None, newline=None, closefd=True)

  def readline(self):
    return self._buf.readline()

  def close(self):
    self._buf.close()

def constrain(value, minimum, maximum):
  if value < minimum:
    return minimum
  elif value > maximum:
    return maximum
  else:
    return value


def map_range(value, in_min, in_max, out_min, out_max):
  in_range = in_max - in_min
  out_range = out_max - out_min
  scaled = float(value - in_min) / float(in_range)

  return out_min + (scaled * out_range)


def ground_distance(base_lat, base_long, target_lat, target_long):
  earth_radius = 6371;  # in km
  d_lat = deg2rad(target_lat - base_lat);
  d_lon = deg2rad(target_long - base_long); 
  a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
       math.cos(deg2rad(base_lat)) * math.cos(deg2rad(target_lat)) * 
       math.sin(d_lon / 2) * math.sin(d_lon / 2))
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) 
  d = earth_radius * c  # Distance in km
  return d
                                   

def deg2rad(deg):
  return deg * (math.pi / 180)


def clean_aircraft_map(aircraft_map):
  now = time.time()
  for aircraft_id, (last_heard, _) in aircraft_map.items():
    if now - last_heard > 60:
      print "Aged out %s" % aircraft_id
      del(aircraft_map[aircraft_id])


def loop(options):
  print "Using lat/long of %s %s" % (options.my_latitude, options.my_longitude)
  aircraft_map = {}  # Aircraft id -> (last_heard, channel)
  pygame.midi.init()
  if options.device == -1:
    device = pygame.midi.get_default_output_id()
  else:
    device = options.device
  m = pygame.midi.Output(device)
  m.set_instrument(12, 0)
  m.set_instrument(13, 1)
  m.set_instrument(24, 2)
  m.set_instrument(46, 3)
  m.set_instrument(12, 4)
  m.set_instrument(13, 5)
  m.set_instrument(24, 6)
  m.set_instrument(46, 7)
  dump1090 = Dump1090Endpoint("192.168.2.145", 30003)
  try:
      while True:
        line = dump1090.readline().strip()
        fields = line.split(",")
        if len(fields) < 22:
          continue
        msgtype = fields[1]
        if msgtype != "3":
          continue
        aircraft_id = fields[4]
        altitude = 0
        latitude = 0.0
        longitude = 0.0
        try:
          altitude = int(fields[11])
        except ValueError:
          pass
        try:
          latitude = float(fields[14])
        except ValueError:
          pass
        try:
          longitude = float(fields[15])
        except ValueError:
          pass
        if latitude == 0.0 or longitude == 0.0:
          continue
        note = int(map_range(constrain(altitude, 0, 32000), 0, 32000, 30, 100))
        dist = ground_distance(options.my_latitude, options.my_longitude, latitude, longitude)
        velocity = int(map_range(constrain(dist, 0, 100), 0, 100, 127, 0))
        if aircraft_id in aircraft_map:
          _, midi_channel = aircraft_map[aircraft_id]
        else:
          midi_channel = random.choice(range(8))
        aircraft_map[aircraft_id] = (time.time(), midi_channel)
        clean_aircraft_map(aircraft_map)
        print "%s %s %s %s (@%s km) -> %d (vel %s, channel %s)" % (aircraft_id, altitude, latitude, longitude, dist, note, velocity, midi_channel)
        m.note_on(note, velocity, midi_channel)
        time.sleep(0.025)
        m.note_off(note, velocity, midi_channel)
  finally:
    all_notes_off(m)

def all_notes_off(device):
  for channel in range(16):
    for note in range(128):
      device.note_off(note, 127, channel)

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("-d", "--device",
                      action="store", dest="device",
                      type=int, default=-1, help="device to use")
    parser.add_option("-n", "--latitude",
                      action="store", dest="my_latitude",
                      type=float, default=0.0, help="latitude of receiver")
    parser.add_option("-w", "--longitude",
                      action="store", dest="my_longitude",
                      type=float, default=0.0, help="longitude of receiver")
    parser.add_option("-l", "--list-devices",
                      action="store_true", dest="list_devices",
                      default=False, help="list available devices")
    (options, args) = parser.parse_args()
    if options.list_devices:
      list_devices()
      sys.exit(0)

    loop(options)
