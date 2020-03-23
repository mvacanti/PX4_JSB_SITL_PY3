#!/usr/bin/env python

# Classes for aircraft model

import numpy
import struct
import time
from math import sin, cos, sqrt

from modules.constants import *


class Controls(object):

    def __init__(self, motor_0, motor_1, motor_2, motor_3, aux1, aux2, aux3, aux4,
                 mode, nav_mode):
        self.motor_0 = motor_0
        self.motor_1 = motor_1
        self.motor_2 = motor_2
        self.motor_3 = motor_3
        self.aux1 = aux1
        self.aux2 = aux2
        self.aux3 = aux3
        self.aux4 = aux4
        self.mode = mode
        self.nav_mode = nav_mode
        self.C_nb = numpy.identity(3)

    @classmethod
    def default(cls):
        return cls(time.time(), 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def send_to_jsbsim(self, jsb_console):
        jsb_console.send('set %s %s\n' % ('fcs/esc-cmd-norm[0]', self.motor_0))

        jsb_console.send('set %s %s\n' % ('fcs/esc-cmd-norm[1]', self.motor_1))

        jsb_console.send('set %s %s\n' % ('fcs/esc-cmd-norm[2]', self.motor_2))

        jsb_console.send('set %s %s\n' % ('fcs/esc-cmd-norm[3]', self.motor_3))

    @classmethod
    def from_mavlink(cls, msg):
        return cls(
            motor_0=float(msg.controls[0]),
            motor_1=float(msg.controls[1]),
            motor_2=float(msg.controls[2]),
            motor_3=float(msg.controls[3]),
            aux1=msg.controls[4],
            aux2=msg.controls[5],
            aux3=msg.controls[6],
            aux4=msg.controls[7],
            mode=msg.mode,
            nav_mode=msg.flags)


class State(object):

    def __init__(self, time,
                 phi, theta, psi,
                 p, q, r,
                 lat, lon, alt,
                 vN, vE, vD,
                 xacc, yacc, zacc):
        self.time = time
        self.set_attitude(phi, theta, psi)
        self.p = p
        self.q = q
        self.r = r
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vN = vN
        self.vE = vE
        self.vD = vD
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc

    def set_attitude(self, phi, theta, psi):
        self.phi = phi
        self.theta = theta
        self.psi = psi
        cosPhi = cos(phi)
        sinPhi = sin(phi)
        cosThe = cos(theta)
        sinThe = sin(theta)
        cosPsi = cos(psi)
        sinPsi = sin(psi)
        self.C_nb = numpy.matrix([
            [cosThe * cosPsi,
             -cosPhi * sinPsi + sinPhi * sinThe * cosPsi,
             sinPhi * sinPsi + cosPhi * sinThe * cosPsi],
            [cosThe * sinPsi,
             cosPhi * cosPsi + sinPhi * sinThe * sinPsi,
             -sinPhi * cosPsi + cosPhi * sinThe * sinPsi],
            [-sinThe,
             sinPhi * cosThe,
             cosPhi * cosThe]])
        self.quat = numpy.matrix([
            [cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2)],
            [sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi / 2)],
            [cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2)],
            [cos(phi / 2) * cos(theta / 2) * sin(psi / 2) - sin(phi / 2) * sin(theta / 2) * cos(psi / 2)]
        ])

    @classmethod
    def default(cls):
        return cls(time.time(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def send_to_mav(self, mav):
        try:
            v = sqrt(self.vN ** 2 + self.vE ** 2 + self.vD ** 2)
            mav.hil_state_quaternion_send(
                self.time * sec2usec, self.quat,
                self.p, self.q, self.r,
                int(self.lat * rad2degE7), int(self.lon * rad2degE7), int(self.alt * m2mm),
                int(self.vN * m2cm), int(self.vE * m2cm), int(self.vD * m2cm),
                int(v * m2cm), int(v * m2cm),
                int(self.xacc * mpss2mg), int(self.yacc * mpss2mg), int(self.zacc * mpss2mg))
        except struct.error as e:
            print(e)
            print('mav hil packet data exceeds int bounds?')

    @classmethod
    def from_fdm(cls, fdm):

        # position
        lat = fdm.get('latitude', units='radians')
        lon = fdm.get('longitude', units='radians')
        alt = fdm.get('altitude', units='meters')

        # attitude
        phi = fdm.get('phi', units='radians')
        theta = fdm.get('theta', units='radians')
        psi = fdm.get('psi', units='radians')

        # rotation rates
        phidot = fdm.get('phidot', units='rps')
        thetadot = fdm.get('thetadot', units='rps')
        psidot = fdm.get('psidot', units='rps')

        p = phidot - psidot * sin(theta)
        q = cos(phi) * thetadot + sin(phi) * cos(theta) * psidot
        r = -sin(phi) * thetadot + cos(phi) * cos(theta) * psidot

        # acceleration
        xacc = fdm.get('A_X_pilot', units='mpss')
        yacc = fdm.get('A_Y_pilot', units='mpss')
        zacc = fdm.get('A_Z_pilot', units='mpss')

        # velocitiystate
        vN = fdm.get('v_north', units='mps')
        vE = fdm.get('v_east', units='mps')
        vD = fdm.get('v_down', units='mps')

        return cls(time=time.time(),
                   phi=phi, theta=theta, psi=psi,
                   p=p, q=q, r=r,
                   lat=lat, lon=lon, alt=alt,
                   vN=vN, vE=vE, vD=vD,
                   xacc=xacc, yacc=yacc, zacc=zacc)
