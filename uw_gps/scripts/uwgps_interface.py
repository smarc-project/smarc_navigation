#!/usr/bin/env python3

import requests

class UWGPSInterface():

    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print("Exception occured {}".format(exc))
            return None

        if r.status_code != requests.codes.ok:
            print("Got error {}: {}".format(r.status_code, r.text))
            return None

        return r.json()

    def get_antenna_position(self, base_url):
        return self.get_data("{}/api/v1/config/antenna".format(base_url))

    def get_acoustic_position(self, base_url):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(base_url))

    def get_global_position(self, base_url, acoustic_depth=None):
        return self.get_data("{}/api/v1/position/global".format(base_url))

    def get_master_position(self, base_url):
        return self.get_data("{}/api/v1/position/master".format(base_url))

    def get_master_imu(self, base_url):
        return self.get_data("{}/api/v1/imu/calibrate".format(base_url))

    def set_position_master(self, url, latitude, longitude, orientation):
        payload = dict(lat=latitude, lon=longitude, orientation=orientation)
        # Keep loop running even if for some reason there is no connection.
        try:
            requests.put(url, json=payload, timeout=1)
        except requests.exceptions.RequestException as err:
            print("Serial connection error: {}".format(err))
