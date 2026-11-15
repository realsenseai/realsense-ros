# Copyright 2024 RealSense, Inc. All Rights Reserved.
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
handle = None
test_ran = False
device_set = list()
failed_devices = list()   # devices whose test run failed or timed out; drives the exit code

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

def command(dev_name, test_file, junit_name, test=None):
    cmd =  ['pytest-3']
    cmd += ['-s']
    cmd += ['-m', ''.join(dev_name)]
    if test:
        cmd += ['-k', f'{test}']
    cmd += [test_file]
    cmd += ['--debug']
    cmd += [f'--junit-xml={logdir}/{junit_name}']
    return cmd

def run_test(cmd, log_name, junit_name, dev_name):
    """
    Run one pytest invocation (one test file on one device). Exit code 5 (no tests
    collected) is not a failure; anything else non-zero, or a timeout, is.
    """
    handle = None
    try:
        handle = open( os.path.join( logdir, log_name ), "w" )
        result = subprocess.run( cmd,
                stdout=handle,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                timeout=200 )
        if result.returncode in (0, 5):
            log.i("---Test Passed---")
        else:
            raise RuntimeError( f"pytest exited with status {result.returncode}" )
    except Exception as e:
            log.e("---Test Failed---")
            log.w( "Error Exception:\n ",e )
            if dev_name not in failed_devices:
                failed_devices.append( dev_name )
    finally:
        if handle:
            handle.close()
        junit_xml_parsing( junit_name )

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

def build_device_port_mapping():
    """
    Map device-name -> YKUSH hub port from rspy's current enumeration.

    rspy resolves each device's hub port from its USB location during query()
    (done in find_devices_run_tests), so the mapping is read straight from the
    enumerated devices -- no need to power-cycle ports one at a time, and no
    direct ykushcmd calls.
    """
    from rspy import devices
    mapping = {}
    for device in devices._device_by_sn.values():
        if device.port is None:
            log.w(f"Could not resolve YKUSH port for {device.name} ({device._sn})")
            continue
        key = device.name.upper()
        if key not in mapping:
            mapping[key] = device.port
            log.i(f"Detected device: {device.name} ({device._sn}) on port {device.port}")

    return mapping


def device_test_files(device, testname):
    """
    Test files under the live-camera folder that hold tests for the given device marker.
    """
    cmd = ['pytest-3', '--collect-only', '-q', '-m', device.lower(), dir_live_tests]
    if testname:
        cmd += ['-k', testname]
    result = subprocess.run( cmd, capture_output=True, universal_newlines=True, timeout=120 )   # may raise TimeoutExpired
    # 0 = collected, 5 = nothing collected for this device; anything else (import/syntax errors,
    # bad markers, ...) is a real collection failure and must not look like "no tests".
    if result.returncode not in (0, 5):
        raise RuntimeError( f"test collection exited with status {result.returncode}:\n{result.stdout[-2000:]}{result.stderr[-2000:]}" )
    out = result.stdout
    files = sorted( { line.split('::')[0] for line in out.splitlines() if '::' in line } )
    return [ f if os.path.isabs(f) else os.path.join( os.getcwd(), f ) for f in files ]


def run_tests_for_device(device, port, testname):
    """
    Run the device's tests one test file at a time, each on a freshly powered camera --
    like LibCI, which power-cycles the device (rspy enable_only(recycle=True)) per test
    file -- so a file never inherits the state (e.g. D585S safety mode) the previous one
    left behind. rspy owns the hub, so there are no direct ykushcmd calls.
    """
    from rspy import devices
    if port is None:
        log.e(f"No port mapping found for device {device.upper()}")
        return

    def fail( msg ):
        log.e( msg )
        if device not in failed_devices:
            failed_devices.append( device )

    serials = [ sn for sn in devices.all() if devices.get( sn ).name.upper() == device.upper() ]
    if devices.hub and not serials:
        fail( f"No serial number found for {device}; cannot power-cycle it between test files" )
        return
    try:
        test_files = device_test_files( device, testname )
    except Exception as e:
        fail( f"Collecting tests for {device} failed: {e}" )
        return
    if not test_files:
        log.w( f"No tests found for {device}" )
        return

    for test_file in test_files:
        stem = os.path.splitext( os.path.basename( test_file ) )[0]
        log.i( f"Running {stem} on {device} (fresh power cycle)" )
        if devices.hub and serials:
            devices.enable_only( serials, recycle=True, disable_other_ports=True )
            time.sleep( 5 )   # let the FW settle after enumeration before the node talks to it
        junit_name = f'{device.upper()}_{stem}_pytest.xml'
        cmd = command( device.lower(), test_file, junit_name, testname )
        run_test( cmd, f'{device.upper()}_{stem}.log', junit_name, device )


def find_devices_run_tests():
    """
    Main function to find devices and run tests on them. 
    """
    from rspy import devices
    global logdir, device_set, _device_by_sn
    max_retry = 3

    try:
        os.makedirs(logdir, exist_ok=True)

        # Let rspy own the YKUSH hub: discover it, enumerate the connected
        # devices and resolve each device's port. Skip the hub reset on the first
        # attempt -- the 'ykushcmd --reset' it runs prints 'cannot claim
        # interface' to the console while the board re-enumerates. Only reset as a
        # recovery step if the first enumeration comes up empty.
        first_attempt = True
        while max_retry and not devices._device_by_sn:
            devices.query(hub_reset=not first_attempt)
            first_attempt = False
            max_retry -= 1

        if not devices._device_by_sn:
            assert False, 'No Camera device detected!'

        connected_devices = [device.name for device in devices._device_by_sn.values()]
        log.i('Connected devices:', connected_devices)
        device_port_mapping = build_device_port_mapping()
        log.i('Device to port mapping:', device_port_mapping)

        testname = regex if regex else None

        if device_set:
            # Loop through user-specified devices and run tests only on them
            devices_not_found = []
            for device in device_set:
                port = device_port_mapping.get(device.upper())
                if port is not None:
                    log.i('Running tests on device:', device)
                    run_tests_for_device(device, port, testname)
                else:
                    log.e('Skipping test run on device:', device, ', -- NOT found')
                    devices_not_found.append(device)
            assert len(devices_not_found) == 0, f'Devices not found: {devices_not_found}'
        else:
            # Loop through all connected devices and run all tests
            for device in connected_devices:
                log.i('Running tests on device:', device)
                run_tests_for_device(device, device_port_mapping.get(device.upper()), testname)
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
        log.d("server took", run_time, "seconds")

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

# Like LibCI's pytest stage: a non-zero exit tells the pipeline that tests failed,
# including a device whose run timed out and so produced no JUnit XML.
if failed_devices:
    log.e( "Test failures on:", failed_devices )
    sys.exit( 1 )
sys.exit( 0 )
