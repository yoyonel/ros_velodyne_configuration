#!/usr/bin/python

# url: http://docs.python-requests.org/en/master/api/
import requests
import time
import sys
import getopt


def build_cmd(
    network_sensor_ip,
    service_name
):
    return "http://" + network_sensor_ip + "/cgi/" + service_name + ".json"


def httprequest_get(
    network_sensor_ip,
    service_name
):
    cmd = build_cmd(network_sensor_ip, service_name)
    print "cmd for request: ", cmd

    try:
        r = requests.get(cmd)
    except:
        print "Unexpected error:", sys.exc_info()[0]
        sys.exit()

    return r


def print_request(r):
    try:
        print "- headers['content-type]: ", r.headers['content-type']
        print "- encoding: ", r.encoding
        print "- text: ", r.text
        print "- json(): ", r.json()
    except:
        print "Unexpected error:", sys.exc_info()[0]


def version_getopt(argv):

    NETWORK_SENSOR_IP = "172.20.0.191"
    SERVICE = "status"

    try:
        opts, args = getopt.getopt(argv, "hi:s:", ["ip_network_sensor=", "service_name="])
    except getopt.GetoptError:
        print 'python_request_vlp16.py -i <ip_network_sensor> -s <service_name>'
        sys.exit()

    for opt, arg in opts:
        print 'ici'
        if opt == '-h':
            print 'python_request_vlp16.py -i <ip_network_sensor> -s <service_name>'
            sys.exit()
        elif opt in ("-i", "--ip_network_sensor"):
            NETWORK_SENSOR_IP = arg
        elif opt in ("-s", "--service_name"):
            SERVICE = arg

    r = httprequest_get(NETWORK_SENSOR_IP, SERVICE)
    print '- status code: ', r.status_code
    if r.status_code == 200:
        print_request(r)


def main(argv):
    version_getopt(argv)


if __name__ == "__main__":
    main(sys.argv[1:])
