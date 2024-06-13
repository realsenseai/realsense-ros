# Copyright 2024 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys, os, subprocess, re, getopt, time

start_time = time.time()
running_on_ci = False
if 'WORKSPACE' in os.environ:
    #Path for ROS-CI on Jenkins
    ws_rosci = os.environ['WORKSPACE']
    sys.path.append( os.path.join( ws_rosci, 'lrs/unit-tests/py' ))
    running_on_ci = True
else:
    #For running this script locally
    #Extract the root where both realsense-ros and librealsense are cloned
    ws_local = '/'.join(os.path.abspath( __file__ ).split( os.path.sep )[0:-5])
    #expected to have 'librealsense' repo in parallel to 'realsense-ros'
    assert os.path.exists( os.path.join(ws_local, 'librealsense')), f" 'librealsense' doesn't exist at {ws_local} "
    sys.path.append( os.path.join( ws_local, 'librealsense/unit-tests/py' ))

#logs are stored @ ./realsense2_camera/test/logs
logdir = os.path.join( '/'.join(os.path.abspath( __file__ ).split( os.path.sep )[0:-2]), 'logs')
dir_live_tests = os.path.dirname(__file__)

from rspy import log, file
regex = None
hub_reset = False
handle = None
test_ran = False
device_set = list()

def usage():
    ourname = os.path.basename( sys.argv[0] )
    print( 'Syntax: ' + ourname + ' [options] ' )
    print( 'Options:' )
    print( '        -h, --help      Usage help' )
    print( '        -r, --regex     Run all tests whose name matches the following regular expression' )
    print( '                        e.g.: --regex test_camera_imu; -r d415_basic')
    print( '        --device <>     Run only on the specified device(s); list of devices separated by ',', No white spaces' )
    print( '                        e.g.: --device=d455,d435i,d585 (or) --device d455,d435i,d585 ')
    print( '                        Note: if --device option not used, tests run on all connected devices ')
        
    sys.exit( 2 )

def command(dev_name, test=None):
    cmd =  ['pytest-3']
    cmd += ['-s']
    cmd += ['-m', ''.join(dev_name)]
    if test:
        cmd += ['-k', f'{test}'] 
    cmd += [''.join(dir_live_tests)]
    cmd += ['--debug']
    cmd += [f'--junit-xml={logdir}/{dev_name.upper()}_pytest.xml']
    return cmd

def run_test(cmd, test=None, dev_name=None, stdout=None, append =False):
    handle = None
    try:
        if test:
            stdout = stdout + os.sep + str(dev_name.upper()) + '_' + test + '.log'
        else:
            stdout = stdout + os.sep + str(dev_name.upper()) + '_' + 'full.log' 
        if stdout is None:
            sys.stdout.flush()
        elif stdout and stdout != subprocess.PIPE:
            if append:
                handle = open( stdout, "a" )
                handle.write(
                    "\n----------TEST-SEPARATOR----------\n\n" )
                handle.flush()
            else:
                handle = open( stdout, "w" )

        result = subprocess.run( cmd,
                stdout=handle,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                timeout=200,
                check=True )
        if not result.returncode:
                log.i("---Test Passed---")
    except Exception as e:
            log.e("---Test Failed---")
            log.w( "Error Exception:\n ",e )
                        
    finally:
        if handle:
            handle.close()      
        junit_xml_parsing(f'{dev_name.upper()}_pytest.xml')

def junit_xml_parsing(xml_file):
    '''
    - remove redundant hierarchy from testcase 'classname', and 'name' attributes \
    - running pytest-3 w/ --junit-xml={logdir}/{dev_name}_pytest.xml results in classname w/ \
        too long path wich is redundant. \
    - this helps in better reporting structure of test results in jenkins
    '''
    import xml.etree.ElementTree as ET
    global logdir

    if not os.path.isfile( os.path.join(logdir, f'{xml_file}' )):
        log.e(f'{xml_file} not found, test resutls can\'t be generated')
    else:
        tree = ET.parse(os.path.join(logdir,xml_file))
        root = tree.getroot()
        for testsuite in root.findall('testsuite'):
            for testcase in testsuite.findall('testcase'):
                testcase.set('classname', testcase.attrib['classname'].split('.')[-2])
                testcase.set('name', re.sub('launch_.*parameters','',testcase.attrib['name']))
        new_xml = xml_file.split('.')[0]
        tree.write(f'{logdir}/{new_xml}_refined.xml')

def find_devices_run_tests():     
    global logdir, device_set

    try:
        os.makedirs( logdir, exist_ok=True )
        
        from rspy import devices
        #Update dict '_device_by_sn' from devices module of rspy
        devices.query( hub_reset = hub_reset )
        global _device_by_sn
        if not devices._device_by_sn:
            assert False, 'No Camera device detected!'
        else:
            connected_devices = [device.name for device in devices._device_by_sn.values()]         
            log.i('Connected devices:', connected_devices)        
        
        testname = regex if regex else None
        
        if device_set:
            #Loop in for user specified devices and run tests only on them
            devices_not_found = []
            for device in device_set:
                if device.upper() in connected_devices:
                    log.i('Running tests on device:', device)
                    cmd = command(device.lower(), testname)
                    run_test(cmd, testname, device, stdout=logdir, append =False)
                else:
                   log.e('Skipping test run on device:', device, ', -- NOT found' )
                   devices_not_found.append(device)
            assert len(devices_not_found) == 0, f'Devices not found:{devices_not_found}'
        else:
            #Loop in for all connected devices and run all tests
            for device in connected_devices:
                    log.i('Running tests on device:', device)
                    cmd = command(device.lower(), testname)
                    run_test(cmd, testname, device, stdout=logdir, append =False)
    finally:
        if devices.hub and devices.hub.is_connected():
            devices.hub.disable_ports()
            devices.wait_until_all_ports_disabled()
            devices.hub.disconnect()
        if running_on_ci:
            log.i("Log path- \"Build Artifacts\":/ros2/realsense_camera/test/logs ")
        else:
            log.i("log path:", logdir)
        run_time = time.time() - start_time
        log.d( "server took", run_time, "seconds" )

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt( sys.argv[1:], 'hr:', longopts=['help', 'regex=', 'device=' ] )
    except getopt.GetoptError as err:
        log.e( err )
        usage()

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
        elif opt in ('-r', '--regex'):
            regex = arg
        elif opt == '--device':
            device_set = arg.split(',')
    
    find_devices_run_tests()

sys.exit( 0 )
