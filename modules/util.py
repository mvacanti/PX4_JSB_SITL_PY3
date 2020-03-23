import math
import os
import pexpect
import random
import sys
import time
from math import sqrt, acos, cos, pi, atan2
from subprocess import call, check_call, Popen, PIPE


def m2ft(x):
    # meters to feet
    return float(x) / 0.3048


def ft2m(x):
    # feet to meters
    return float(x) * 0.3048


def kt2mps(x):
    return x * 0.514444444


def mps2kt(x):
    return x / 0.514444444


def topdir():
    # return top of git tree where hil is running from
    d = os.path.dirname(os.path.realpath(__file__))
    assert(os.path.basename(d)=='pysim')
    d = os.path.dirname(d)
    assert(os.path.basename(d)=='hil')
    d = os.path.dirname(d)
    assert(os.path.basename(d)=='Tools')
    d = os.path.dirname(d)
    return d


def reltopdir(path):
    # return a path relative to topdir()
    return os.path.normpath(os.path.join(topdir(), path))


def run_cmd(cmd, dir=".", show=False, output=False, checkfail=True):
    # run a shell command
    if show:
        print("Running: '%s' in '%s'" % (cmd, dir))
    if output:
        return Popen([cmd], shell=True, stdout=PIPE, cwd=dir).communicate()[0]
    elif checkfail:
        return check_call(cmd, shell=True, cwd=dir)
    else:
        return call(cmd, shell=True, cwd=dir)


def rmfile(path):
    # remove a file if it exists
    try:
        os.unlink(path)
    except Exception:
        pass


def deltree(path):
    # delete a tree of files
    run_cmd('rm -rf %s' % path)


def build_SIL(atype, target='sitl'):
    # build desktop SIL
    run_cmd("make clean %s" % target,
            dir=reltopdir(atype),
            checkfail=True)
    return True


def build_AVR(atype, board='mega2560'):
    # build AVR binaries
    config = open(reltopdir('config.mk'), mode='w')
    config.write('''BOARD=%s PORT=/dev/null''' % board)
    config.close()
    run_cmd("make clean", dir=reltopdir(atype),  checkfail=True)
    run_cmd("make", dir=reltopdir(atype),  checkfail=True)
    return True

# list of pexpect children to close on exit
close_list = []

def pexpect_autoclose(p):
    '''mark for autoclosing'''
    global close_list
    close_list.append(p)

def pexpect_close(p):
    '''close a pexpect child'''
    global close_list

    try:
        p.close()
    except Exception:
        pass
    try:
        p.close(force=True)
    except Exception:
        pass
    if p in close_list:
        close_list.remove(p)

def pexpect_close_all():
    '''close all pexpect children'''
    global close_list
    for p in close_list[:]:
        pexpect_close(p)

def pexpect_drain(p):
    '''drain any pending input'''
    try:
        p.read_nonblocking(1000, timeout=0)
    #except pexpect.EOF:
    #except pexpect.TIMEOUT:
    except:
        pass

def start_SIL(atype, valgrind=False, wipe=False, CLI=False, height=None):
    '''launch a SIL instance'''
    cmd=""
    if valgrind and os.path.exists('/usr/bin/valgrind'):
        cmd += 'valgrind -q --log-file=%s-valgrind.log ' % atype
    cmd += reltopdir('tmp/%s.build/%s.elf' % (atype, atype))
    if wipe:
        cmd += ' -w'
    if CLI:
        cmd += ' -s'
    if height is not None:
        cmd += ' -H %u' % height
    ret = pexpect.spawn(cmd, logfile=sys.stdout, timeout=5)
    ret.delaybeforesend = 0
    pexpect_autoclose(ret)
    ret.expect('Waiting for connection')
    return ret

def start_MAVProxy_SIL(atype, aircraft=None, setup=False, master='tcp:127.0.0.1:5760',
                       options=None, logfile=sys.stdout):
    '''launch mavproxy connected to a SIL instance'''
    global close_list
    MAVPROXY = os.getenv('MAVPROXY_CMD', reltopdir('../MAVProxy/mavproxy.py'))
    cmd = MAVPROXY + ' --master=%s --out=127.0.0.1:14550' % master
    if setup:
        cmd += ' --setup'
    if aircraft is None:
        aircraft = 'test.%s' % atype
    cmd += ' --aircraft=%s' % aircraft
    if options is not None:
        cmd += ' ' + options
    ret = pexpect.spawn(cmd, logfile=logfile, timeout=60)
    ret.delaybeforesend = 0
    pexpect_autoclose(ret)
    return ret


def expect_setup_callback(e, callback):
    '''setup a callback that is called once a second while waiting for
       patterns'''
    def _expect_callback(pattern, timeout=e.timeout):
        tstart = time.time()
        while time.time() < tstart + timeout:
            try:
                ret = e.expect_saved(pattern, timeout=1)
                return ret
            except pexpect.TIMEOUT:
                e.expect_user_callback(e)
                pass
        print("Timed out looking for %s" % pattern)
        raise pexpect.TIMEOUT(timeout)

    e.expect_user_callback = callback
    e.expect_saved = e.expect
    e.expect = _expect_callback

def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def loadfile(fname):
    '''load a file as a string'''
    f = open(fname, mode='r')
    r = f.read()
    f.close()
    return r

def lock_file(fname):
    '''lock a file'''
    import fcntl
    f = open(fname, mode='w')
    try:
        fcntl.lockf(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        return None
    return f

def check_parent(parent_pid=os.getppid()):
    '''check our parent process is still alive'''
    try:
        os.kill(parent_pid, 0)
    except Exception:
        print("Parent had finished - exiting")
        sys.exit(1)


def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance 
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    '''
    from math import sin, asin, cos, atan2, radians, degrees
    radius_of_earth = 6378100.0 # in meters
    
    lat1 = radians(lat)
    lon1 = radians(lon)
    brng = radians(bearing)
    dr = distance/radius_of_earth
    
    lat2 = asin(sin(lat1)*cos(dr) +
                cos(lat1)*sin(dr)*cos(brng))
    lon2 = lon1 + atan2(sin(brng)*sin(dr)*cos(lat1), 
                        cos(dr)-sin(lat1)*sin(lat2))
    return (degrees(lat2), degrees(lon2))


class Wind(object):
    '''a wind generation object'''
    def __init__(self, windstring, cross_section=0.1):
        a = windstring.split(',')
        if len(a) != 3:
            raise RuntimeError("Expected wind in speed,direction,turbulance form, not %s" % windstring)
        self.speed     = float(a[0]) # m/s
        self.direction = float(a[1]) # direction the wind is going in
        self.turbulance= float(a[2]) # turbulance factor (standard deviation)

        # the cross-section of the aircraft to wind. This is multiplied by the
        # difference in the wind and the velocity of the aircraft to give the acceleration
        self.cross_section = cross_section

        # the time constant for the turbulance - the average period of the
        # changes over time
        self.turbulance_time_constant = 5.0

        # wind time record
        self.tlast = time.time()

        # initial turbulance multiplier
        self.turbulance_mul = 1.0

    def current(self, deltat=None):
        '''return current wind speed and direction as a tuple
        speed is in m/s, direction in degrees
        '''
        if deltat is None:
            tnow = time.time()
            deltat = tnow - self.tlast
            self.tlast = tnow

        # update turbulance random walk
        w_delta = math.sqrt(deltat)*(1.0-random.gauss(1.0, self.turbulance))
        w_delta -= (self.turbulance_mul-1.0)*(deltat/self.turbulance_time_constant)
        self.turbulance_mul += w_delta
        speed = self.speed * math.fabs(self.turbulance_mul)
        return (speed, self.direction)

def apparent_wind(wind_sp, obj_speed, alpha):
    delta = wind_sp * cos(alpha)
    x = wind_sp**2 + obj_speed**2 + 2 * obj_speed * delta
    rel_speed = sqrt(x)
    if rel_speed == 0:
        beta = pi
    else:
        beta = acos((delta + obj_speed) / rel_speed)

    return (rel_speed, beta)

# See http://en.wikipedia.org/wiki/Drag_equation
#
# Drag equation is F(a) = cl * p/2 * v^2 * a, where cl : drag coefficient
# (let's assume it's low, .e.g., 0.2), p : density of air (assume about 1
# kg/m^3, the density just over 1500m elevation), v : relative speed of wind
# (to the body), a : area acted on (this is captured by the cross_section
# paramter).
# 
# So then we have 
# F(a) = 0.2 * 1/2 * v^2 * cross_section = 0.1 * v^2 * cross_section
def drag_force(wind, sp): 
    return (sp**2.0) * 0.1 * wind.cross_section

# Function to make the force vector.  relWindVec is the direction the apparent
# wind comes *from*.  We want to compute the accleration vector in the direction
# the wind blows to.
def acc(val, mag):
    if val == 0:
        return mag
    else:
        return (val / abs(val)) * (0 - mag)

if __name__ == "__main__":
    import doctest
    doctest.testmod()
