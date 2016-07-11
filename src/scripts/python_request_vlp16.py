#!/usr/bin/python
# exemple: ./python_request_vlp16.py --params '{"rpm":"311"}'

# urls:
# - https://docs.python.org/2/howto/argparse.html#the-basics
# - http://stackoverflow.com/questions/18608812/accepting-a-dictionary-as-an-argument-with-argparse-and-python
# - http://docs.python-requests.org/en/master/user/advanced/#timeouts
# - http://docs.python-requests.org/en/master/user/quickstart/#passing-parameters-in-urls

# url: http://docs.python-requests.org/en/master/api/
import requests
import sys
import argparse
import json


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


def httprequest_post(
    network_sensor_ip,
    service_name,
    dict_params={}
):
    cmd = "http://" + network_sensor_ip + "/cgi/setting"
    print "cmd for request: ", cmd

    try:
        r = requests.post(cmd, data=dict_params)
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
    PARAMS = {}

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--ip", type=str, help="ip network sensor")
    parser.add_argument("-s", "--service", type=str, help="service name")
    parser.add_argument("-p", "--params", type=json.loads, help="params for post request")

    args = parser.parse_args()

    if args.ip:
        NETWORK_SENSOR_IP = args.ip
    if args.service:
        SERVICE = args.service
    if args.params:
        PARAMS = args.params

    # si on passe des parametres, c'est qu'on souhaite generer un post request
    if(PARAMS):
        r = httprequest_post(NETWORK_SENSOR_IP, SERVICE, PARAMS)
    else:
        # sinon un get request
        r = httprequest_get(NETWORK_SENSOR_IP, SERVICE)

    print '- status code: ', r.status_code

    if r.status_code == 200:
        print_request(r)


def main(argv):
    version_getopt(argv)


if __name__ == "__main__":
    main(sys.argv[1:])
